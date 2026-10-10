"""用线性化轮廓预测控制器跟踪已规划轨迹。"""

from dataclasses import dataclass, replace
from bisect import bisect_right
import math

import numpy as np
import osqp
from scipy import sparse

from .models import KinematicLimits, PlannedTrajectory


# 规划用的膨胀半径包含较大的跟踪余量；巡航时允许飞机进入这段余量，
# 只在接近手绘障碍物的实际边界时触发硬停止。单位为米，量到飞机中心。
HARD_OBSTACLE_CLEARANCE = 0.10


@dataclass(frozen=True)
class ControlResult:
    """单次速度指令；warning 只记录，fault 仅用于真实硬安全边界。"""

    vx: float
    vy: float
    z: float
    progress: float
    cross_track: float
    finished: bool = False
    fault: str = ""
    warning: str = ""


class PredictivePathController:
    """跟踪一条固定轨迹，不在飞行中创建或修改航线。

    状态为 XY 位置与速度，控制量为 AirSim 的 XY 速度命令。预测模型将
    飞机响应近似为一阶滞后；代价同时惩罚参考线偏差、偏离计划速度和命令变化。
    """

    def __init__(
        self,
        trajectory: PlannedTrajectory,
        limits: KinematicLimits,
        obstacles=(),
        dt: float = 0.05,
        horizon: int = 10,
        max_cross_track: float = 1.0,
    ):
        if len(trajectory.samples) < 2:
            raise ValueError("巡航轨迹至少需要两个采样点")
        self.trajectory = trajectory
        self.limits = limits
        self.obstacles = tuple(obstacles)
        self.dt = dt
        self.horizon = horizon
        self.max_cross_track = max_cross_track
        self._times = [sample.time for sample in trajectory.samples]
        self._distances = [sample.distance for sample in trajectory.samples]
        self._progress_distance = 0.0
        self._previous_position = None
        self._next_waypoint = 0
        self._last_warning = ""
        self._previous_command = np.zeros(2, dtype=float)
        self._solver, self._control_matrix = self._build_solver()

    def check_state(self, position):
        """检查偏航和实际障碍边界；规划膨胀余量不是碰撞边界。"""
        x, y, z = (float(value) for value in position)
        clearance = min(
            (math.hypot(x - obstacle.x, y - obstacle.y) - obstacle.radius
             for obstacle in self.obstacles),
            default=float("inf"),
        )
        if clearance < HARD_OBSTACLE_CLEARANCE:
            return 0.0, clearance, (
                "距障碍物实际边缘仅 %.2f m，低于 %.2f m 硬保护间距"
                % (clearance, HARD_OBSTACLE_CLEARANCE)
            )

        previous_progress = self._progress_distance
        distance, deviation, expected_z = self._project_progress(x, y, z)
        self._progress_distance = max(self._progress_distance, distance)
        warnings = []
        if deviation > 0.25:
            warnings.append("横向偏差 %.2f m，正在向前回归航线" % deviation)
        if deviation > self.max_cross_track:
            warnings.append(
                "偏离规划轨迹 %.2f m，超过 %.2f m 提醒阈值"
                % (deviation, self.max_cross_track)
            )
        if abs(z - expected_z) > 0.25:
            warnings.append("高度偏差 %.2f m，继续沿线修正" % abs(z - expected_z))
        waypoint_fault = self._check_waypoint_passage(
            (x, y, z), previous_progress, self._progress_distance
        )
        if waypoint_fault:
            warnings.append(waypoint_fault)
        self._previous_position = (x, y, z)
        self._last_warning = "；".join(warnings)
        return self._progress_distance, deviation, ""

    def step(self, position, velocity, elapsed: float) -> ControlResult:
        """读取一次遥测并返回下一拍速度命令。"""
        x, y, z = (float(value) for value in position)
        vx, vy = float(velocity[0]), float(velocity[1])
        progress, deviation, fault = self.check_state((x, y, z))
        if fault:
            return ControlResult(0.0, 0.0, z, progress, deviation, fault=fault)

        references, holding = self._references_from_progress(elapsed, deviation)
        q = self._objective_vector(x, y, vx, vy, references)
        self._solver.update(q=q)
        solution = self._solver.solve()
        if solution.info.status_val not in (1, 2) or solution.x is None:
            command = (np.zeros(2, dtype=float) if holding else
                       self._forward_recovery_command((x, y), deviation))
            self._previous_command = command
            return ControlResult(
                float(command[0]), float(command[1]), references[0].z,
                progress, deviation,
                warning="预测控制器本拍未能及时求解，使用沿线前进控制继续巡航",
            )

        command = np.asarray(solution.x[:2], dtype=float)
        magnitude = float(np.linalg.norm(command))
        if magnitude > self.limits.max_speed:
            command *= self.limits.max_speed / magnitude
        if not holding and deviation > 0.25:
            command = self._enforce_forward_recovery(command, progress)
        self._previous_command = command

        final = self.trajectory.samples[-1]
        finished = (
            elapsed >= self.trajectory.estimated_time
            and math.hypot(x - final.x, y - final.y) <= 0.25
            and abs(float(position[2]) - final.z) <= 0.25
            and math.hypot(vx, vy) <= 0.35
        )
        return ControlResult(
            float(command[0]), float(command[1]), references[0].z,
            progress, deviation, finished=finished, warning=self._last_warning,
        )

    def _check_waypoint_passage(self, position, previous_progress, progress):
        """按顺序核对航点；漏点只告警并向前推进，不反向追赶。"""
        warnings = []
        while self._next_waypoint < len(self.trajectory.waypoint_distances):
            target_distance = self.trajectory.waypoint_distances[self._next_waypoint]
            if progress < target_distance:
                return "；".join(warnings)
            target = self._sample_at_distance(target_distance).position
            previous = self._previous_position or position
            if progress > previous_progress + 1e-9:
                fraction = max(0.0, min(1.0,
                    (target_distance - previous_progress) / (progress - previous_progress)))
            else:
                fraction = 1.0
            crossing = tuple(a + fraction * (b - a)
                             for a, b in zip(previous, position))
            error = math.dist(crossing, target)
            if error > 0.25:
                warnings.append(
                    "第 %d 个航点经过误差 %.2f m；不回头，继续前飞并回归后续航线"
                    % (self._next_waypoint + 1, error)
                )
            self._next_waypoint += 1
        return "；".join(warnings)

    def _project_progress(self, x, y, z):
        """仅在当前弧长附近向前搜索，避免自交路线跳到较远的未来分支。"""
        samples = self.trajectory.samples
        distances = self._distances
        first = max(0, bisect_right(distances, self._progress_distance) - 2)
        if self._previous_position is None:
            observed_step = 0.0
        else:
            observed_step = math.dist(self._previous_position, (x, y, z))
        # 按相邻遥测实际移动距离限制前向搜索，防止自交航线突然跳到远端分支。
        max_advance = max(1.0, observed_step + max(0.35, self.limits.max_speed * self.dt * 2.0))
        last = min(len(samples) - 1,
                   bisect_right(distances, self._progress_distance + max_advance) + 2)
        best_distance = self._progress_distance
        best_score = float("inf")
        best_error = float("inf")
        closest_z = samples[first].z
        for index in range(first, last):
            a, b = samples[index], samples[index + 1]
            dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
            length_sq = dx * dx + dy * dy + dz * dz
            if length_sq <= 1e-12:
                fraction = 0.0
            else:
                fraction = max(0.0, min(1.0,
                    ((x - a.x) * dx + (y - a.y) * dy + (z - a.z) * dz) / length_sq))
            px, py = a.x + fraction * dx, a.y + fraction * dy
            pz = a.z + fraction * dz
            error = math.hypot(x - px, y - py)
            score = error * error + (z - pz) ** 2
            if score < best_score:
                best_score = score
                best_error = error
                best_distance = a.distance + fraction * (b.distance - a.distance)
                closest_z = pz
        return best_distance, best_error, closest_z

    def _sample_at_distance(self, distance):
        """按弧长定位规划采样，重复距离对应同一悬停点。"""
        samples = self.trajectory.samples
        distances = self._distances
        if distance <= distances[0]:
            return samples[0]
        if distance >= distances[-1]:
            return samples[-1]
        right = bisect_right(distances, distance)
        first, second = samples[right - 1], samples[right]
        span = second.distance - first.distance
        fraction = (distance - first.distance) / span if span > 1e-12 else 0.0
        return _blend(first, second, fraction)

    def _references_from_progress(self, elapsed, deviation):
        """以实测弧长推进 MPC 参考，飞机落后时参考点不会按墙上时钟跑远。"""
        timed = self._sample_at_time(elapsed)
        progress = self._progress_distance
        holding = self._is_holding_at(timed, elapsed, progress)
        if holding:
            return ([self._sample_at_time(elapsed + (index + 1) * self.dt)
                     for index in range(self.horizon)], True)

        # 偏差越大越保守，给飞机留出向前贴回曲线的时间。
        recovery_scale = max(0.45, 1.0 - 0.55 * max(0.0, deviation - 0.15))
        reference_distance = progress
        references = []
        total_distance = self._distances[-1]
        for _ in range(self.horizon):
            sample = self._sample_at_distance(reference_distance)
            next_sample = self._sample_at_distance(
                min(total_distance, reference_distance + max(0.1, self.limits.max_speed * self.dt))
            )
            scheduled_speed = math.sqrt(sample.vx ** 2 + sample.vy ** 2 + sample.vz ** 2)
            # 到达停车航点后，从后续速度曲线取启动速度，避免零速采样卡住进度。
            next_speed = math.sqrt(
                next_sample.vx ** 2 + next_sample.vy ** 2 + next_sample.vz ** 2
            )
            scheduled_speed = max(scheduled_speed, 0.5 * next_speed)
            scheduled_speed *= recovery_scale
            reference_distance = min(total_distance, reference_distance + scheduled_speed * self.dt)
            sample = self._sample_at_distance(reference_distance)
            references.append(replace(
                sample,
                vx=sample.vx * recovery_scale,
                vy=sample.vy * recovery_scale,
                vz=sample.vz * recovery_scale,
            ))
        return references, False

    def _is_holding_at(self, timed_sample, elapsed, progress):
        """只有任务确实处于航点悬停窗口时才继续按时间参考保持静止。"""
        if math.sqrt(timed_sample.vx ** 2 + timed_sample.vy ** 2 + timed_sample.vz ** 2) > 0.05:
            return False
        if abs(timed_sample.distance - progress) > 0.15:
            return False
        left = bisect_right(self._distances, timed_sample.distance - 1e-5)
        right = bisect_right(self._distances, timed_sample.distance + 1e-5)
        if right - left < 2:
            return False
        first_time = self._times[left]
        last_time = self._times[right - 1]
        return first_time - 1e-6 <= elapsed <= last_time + 1e-6

    def _tangent_at_distance(self, distance):
        """返回航线前进切向；停车点使用其后续线段作为前进方向。"""
        sample = self._sample_at_distance(distance)
        speed = math.hypot(sample.vx, sample.vy)
        if speed > 1e-4:
            return np.asarray((sample.vx / speed, sample.vy / speed), dtype=float)
        samples = self.trajectory.samples
        index = min(len(samples) - 1, max(0, bisect_right(self._distances, distance) - 1))
        for offset in range(1, len(samples)):
            if index + offset < len(samples):
                first, second = samples[index + offset - 1], samples[index + offset]
                direction = np.asarray((second.x - first.x, second.y - first.y), dtype=float)
                length = float(np.linalg.norm(direction))
                if length > 1e-8:
                    return direction / length
        return np.asarray((1.0, 0.0), dtype=float)

    def _enforce_forward_recovery(self, command, progress):
        """偏航恢复时确保仍沿当前曲线正向飞行，不倒着追刚刚错过的位置。"""
        if progress >= self._distances[-1] - 0.15:
            return command
        tangent = self._tangent_at_distance(progress)
        forward_speed = float(np.dot(command, tangent))
        minimum_forward = min(
            self.limits.max_speed,
            0.45,
            max(0.2, self.limits.max_speed * 0.15),
        )
        lateral = command - forward_speed * tangent
        forward_speed = min(self.limits.max_speed, max(forward_speed, minimum_forward))
        lateral_cap = math.sqrt(max(0.0, self.limits.max_speed ** 2 - forward_speed ** 2))
        lateral_magnitude = float(np.linalg.norm(lateral))
        if lateral_magnitude > lateral_cap > 0.0:
            lateral *= lateral_cap / lateral_magnitude
        elif lateral_cap <= 0.0:
            lateral[:] = 0.0
        return forward_speed * tangent + lateral

    def _forward_recovery_command(self, position, deviation):
        """MPC 偶发超时时，用切向前进加横向回归作为单拍替代指令。"""
        progress = self._progress_distance
        tangent = self._tangent_at_distance(progress)
        sample = self._sample_at_distance(progress)
        speed = math.hypot(sample.vx, sample.vy)
        if speed <= 1e-4:
            ahead = self._sample_at_distance(min(self._distances[-1], progress + 0.25))
            speed = math.hypot(ahead.vx, ahead.vy)
        if speed <= 1e-4:
            return np.zeros(2, dtype=float)
        speed = min(
            self.limits.max_speed,
            max(0.0, speed * max(0.45, 1.0 - deviation * 0.4)),
        )
        if progress >= self._distances[-1] - 0.15:
            return np.zeros(2, dtype=float)
        speed = max(min(0.25, self.limits.max_speed), speed)
        target = self._sample_at_distance(min(self._distances[-1], progress + max(0.35, speed * 0.35)))
        correction = np.asarray((target.x - position[0], target.y - position[1]), dtype=float)
        command = speed * tangent + 1.2 * correction
        return self._enforce_forward_recovery(command, progress)

    def _sample_at_time(self, when):
        samples = self.trajectory.samples
        if when <= samples[0].time:
            return samples[0]
        if when >= samples[-1].time:
            return samples[-1]
        right = bisect_right(self._times, when)
        first, second = samples[right - 1], samples[right]
        span = second.time - first.time
        fraction = (when - first.time) / span if span > 1e-12 else 0.0
        return _blend(first, second, fraction)

    def _build_solver(self):
        """生成固定 Hessian 与速度多边形约束；每拍只更新参考项 q。"""
        horizon = self.horizon
        variables = horizon * 2
        response = max(0.05, self.limits.response_time + self.limits.command_delay)
        alpha = math.exp(-self.dt / response)
        prediction = np.zeros((horizon * 2, variables), dtype=float)
        free_motion = np.zeros((horizon * 2, 2), dtype=float)
        for step in range(1, horizon + 1):
            row = (step - 1) * 2
            free_motion[row:row + 2, :] = (
                self.dt * alpha * (1.0 - alpha ** step) / (1.0 - alpha)
            ) * np.eye(2)
            for command_index in range(step):
                coefficient = self.dt * (1.0 - alpha ** (step - command_index))
                col = command_index * 2
                prediction[row:row + 2, col:col + 2] = coefficient * np.eye(2)

        position_weight = 12.0
        speed_weight = 0.25
        smooth_weight = 2.5
        hessian = 2.0 * position_weight * prediction.T @ prediction
        hessian += 2.0 * speed_weight * np.eye(variables)
        for step in range(horizon):
            for axis in range(2):
                row = np.zeros(variables)
                row[step * 2 + axis] = 1.0
                if step:
                    row[(step - 1) * 2 + axis] = -1.0
                hessian += 2.0 * smooth_weight * np.outer(row, row)

        constraint_rows = []
        angles = [index * math.pi / 4.0 for index in range(8)]
        for step in range(horizon):
            for angle in angles:
                row = np.zeros(variables)
                row[step * 2] = math.cos(angle)
                row[step * 2 + 1] = math.sin(angle)
                constraint_rows.append(row)
        constraint = np.asarray(constraint_rows)
        cap = self.limits.max_speed * math.cos(math.pi / 8.0)
        lower = np.full(constraint.shape[0], -np.inf)
        upper = np.full(constraint.shape[0], cap)

        solver = osqp.OSQP()
        solver.setup(
            P=sparse.csc_matrix((hessian + hessian.T) * 0.5),
            q=np.zeros(variables),
            A=sparse.csc_matrix(constraint),
            l=lower,
            u=upper,
            verbose=False,
            polishing=False,
            warm_starting=True,
            adaptive_rho=False,
            max_iter=60,
            eps_abs=0.015,
            eps_rel=0.015,
            time_limit=0.035,
        )
        return solver, (prediction, free_motion, position_weight, speed_weight,
                        smooth_weight, alpha)

    def _objective_vector(self, x, y, vx, vy, references):
        prediction, free_motion, position_weight, speed_weight, smooth_weight, _ = self._control_matrix
        variables = self.horizon * 2
        reference_positions = np.empty(self.horizon * 2, dtype=float)
        reference_velocities = np.empty(variables, dtype=float)
        for index, sample in enumerate(references):
            reference_positions[index * 2:index * 2 + 2] = (sample.x, sample.y)
            reference_velocities[index * 2:index * 2 + 2] = (sample.vx, sample.vy)
        current_position = np.tile(np.asarray((x, y)), self.horizon)
        current_velocity = np.tile(np.asarray((vx, vy)), self.horizon)
        affine = current_position + free_motion @ np.asarray((vx, vy))
        q = 2.0 * position_weight * prediction.T @ (affine - reference_positions)
        q -= 2.0 * speed_weight * reference_velocities

        previous = self._previous_command
        q[:2] -= 2.0 * smooth_weight * previous
        return q


def _blend(first, second, fraction):
    """在轨迹采样之间插值，保持字段含义一致。"""
    from .models import TrajectorySample

    def mix(a, b):
        return a + fraction * (b - a)

    return TrajectorySample(
        time=mix(first.time, second.time),
        distance=mix(first.distance, second.distance),
        x=mix(first.x, second.x), y=mix(first.y, second.y), z=mix(first.z, second.z),
        vx=mix(first.vx, second.vx), vy=mix(first.vy, second.vy), vz=mix(first.vz, second.vz),
        ax=mix(first.ax, second.ax), ay=mix(first.ay, second.ay), az=mix(first.az, second.az),
        curvature=mix(first.curvature, second.curvature),
        waypoint_index=first.waypoint_index if fraction < 0.5 else second.waypoint_index,
    )
