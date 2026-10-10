"""用前视目标跟踪已规划曲线，并保留曲线上的竞速速度计划。"""

from bisect import bisect_right
from dataclasses import dataclass
import math

from .models import KinematicLimits, PlannedTrajectory, TrajectorySample


# 只有接近手绘障碍的实体边界才触发硬悬停。规划曲线另有安全间距和跟踪余量。
HARD_OBSTACLE_CLEARANCE = 0.10


@dataclass(frozen=True)
class ControlResult:
    """单拍 AirSim 速度指令及其跟踪诊断。"""

    vx: float
    vy: float
    z: float
    progress: float
    cross_track: float
    finished: bool = False
    fault: str = ""
    warning: str = ""
    reference_x: float = 0.0
    reference_y: float = 0.0
    reference_z: float = 0.0
    reference_distance: float = 0.0
    sample_period: float = 0.0
    controller_mode: str = ""
    waypoint_passages: tuple = ()
    reference_curvature: float = 0.0
    planned_speed: float = 0.0


class CarrotPathController:
    """沿固定规划曲线前视跟踪，不在飞行中改线或向后追点。

    目标点始终取自原规划曲线的前方。控制器直接朝前视点给出速度方向，
    因而飞机过弯时可以自然走弦线、向弯内收；它不会把这种合理偏差硬拉回
    曲线中心。速度大小读取规划器的逐点曲线，并在连续加速段补偿响应滞后；
    弯道限速和制动点仍由原速度曲线约束。
    """

    def __init__(
        self,
        trajectory: PlannedTrajectory,
        limits: KinematicLimits,
        obstacles=(),
        dt: float = 0.05,
        max_cross_track: float = 1.0,
    ):
        if len(trajectory.samples) < 2:
            raise ValueError("巡航轨迹至少需要两个采样点")
        self.trajectory = trajectory
        self.limits = limits
        self.obstacles = tuple(obstacles)
        self.dt = dt
        self.max_cross_track = max_cross_track
        self._times = [sample.time for sample in trajectory.samples]
        self._distances = [sample.distance for sample in trajectory.samples]
        self._progress_distance = 0.0
        self._previous_position = None
        self._next_waypoint = 0
        self._last_warning = ""
        self._last_waypoint_passages = ()
        self._previous_command = (0.0, 0.0)

    def update_period(self, observed_period: float):
        """平滑采用真实遥测周期，忽略明显异常的停顿。"""
        if not math.isfinite(observed_period) or not 0.03 <= observed_period <= 0.15:
            return
        observed_period = min(0.10, max(0.035, observed_period))
        self.dt = 0.8 * self.dt + 0.2 * observed_period

    def check_state(self, position):
        """报告偏航；只有逼近实体障碍边缘才触发安全悬停。"""
        self._last_waypoint_passages = ()
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
        self._progress_distance = max(previous_progress, distance)
        warnings = []
        if deviation > 0.25:
            warnings.append("横向偏差 %.2f m；前视跟踪继续向前" % deviation)
        if deviation > self.max_cross_track:
            warnings.append(
                "偏离规划曲线 %.2f m，超过 %.2f m 提醒阈值"
                % (deviation, self.max_cross_track)
            )
        if abs(z - expected_z) > 0.25:
            warnings.append("高度偏差 %.2f m，继续沿线修正" % abs(z - expected_z))
        waypoint_warning = self._check_waypoint_passage(
            (x, y, z), previous_progress, self._progress_distance
        )
        if waypoint_warning:
            warnings.append(waypoint_warning)
        self._previous_position = (x, y, z)
        self._last_warning = "；".join(warnings)
        return self._progress_distance, deviation, ""

    def step(self, position, velocity, elapsed: float) -> ControlResult:
        """按当前进度选前视点，再输出受速度计划与加速度限制的指令。"""
        x, y, z = (float(value) for value in position)
        vx, vy = float(velocity[0]), float(velocity[1])
        progress, deviation, fault = self.check_state((x, y, z))
        if fault:
            return ControlResult(
                0.0, 0.0, z, progress, deviation,
                fault=fault, sample_period=self.dt, controller_mode="safety-stop",
            )

        final = self.trajectory.samples[-1]
        if self._next_waypoint >= len(self.trajectory.waypoint_distances):
            return self._terminal_result(x, y, z, vx, vy, progress, deviation)

        timed_sample = self._sample_at_time(elapsed)
        if self._is_holding_at(timed_sample, elapsed, progress):
            command = self._slew_command((0.0, 0.0))
            return self._result(
                command, timed_sample.z, progress, deviation, timed_sample,
                mode="waypoint-hold",
            )

        # 前视距离随实际速度增长，弯中指向弦线上的目标点，形成自然内切。
        actual_speed = math.hypot(vx, vy)
        path_sample = self._sample_at_distance(progress)
        lookahead = self._lookahead_distance(
            actual_speed, deviation, abs(path_sample.curvature)
        )
        target_distance = min(self._distances[-1], progress + lookahead)
        target = self._sample_at_distance(target_distance)
        tangent = self._tangent_at_distance(progress)
        direction = self._forward_carrot_direction(
            (x, y), (target.x, target.y), tangent, lookahead
        )

        # 速度曲线已按可用加速和制动距离排好时序。连续加速段会向前预读少量，
        # 只补偿控制响应滞后，不因横向偏差降速，避免偏航时减速后再摆动修正。
        scheduled_speed = self._speed_with_acceleration_preview(progress, actual_speed)
        if self._next_waypoint == len(self.trajectory.waypoint_distances) - 1:
            scheduled_speed = min(scheduled_speed, self._final_braking_cap(progress))
        desired = (direction[0] * scheduled_speed,
                   direction[1] * scheduled_speed)
        command = self._slew_command(desired)

        reference = path_sample
        # 高度目标用前方短距离样点，平顺接入爬升段，不把整段 lookahead 当作升降跳变。
        z_sample = self._sample_at_distance(
            min(self._distances[-1], progress + min(0.5, lookahead))
        )
        finished = self._finish_if_at_last_waypoint(x, y, z, vx, vy)
        if finished:
            command = self._slew_command((0.0, 0.0))
        return ControlResult(
            command[0], command[1], z_sample.z, progress, deviation,
            finished=finished,
            warning=self._last_warning,
            reference_x=target.x,
            reference_y=target.y,
            reference_z=target.z,
            reference_distance=target.distance,
            sample_period=self.dt,
            controller_mode="carrot-forward",
            waypoint_passages=self._last_waypoint_passages,
            reference_curvature=reference.curvature,
            planned_speed=scheduled_speed,
        )

    def _terminal_result(self, x, y, z, vx, vy, progress, deviation):
        """按顺序点都已报告后制动；末点偏差写日志，不倒飞找点。"""
        final = self.trajectory.samples[-1]
        command = self._slew_command((0.0, 0.0))
        endpoint_error = math.sqrt((x - final.x) ** 2 + (y - final.y) ** 2
                                    + (z - final.z) ** 2)
        warning = self._last_warning
        if endpoint_error > 0.25:
            warning = "；".join(filter(None, (warning, "末点误差 %.2f m；不倒飞追点" % endpoint_error)))
        return ControlResult(
            command[0], command[1], final.z, progress, deviation,
            finished=math.hypot(vx, vy) <= 0.35,
            warning=warning,
            reference_x=final.x,
            reference_y=final.y,
            reference_z=final.z,
            reference_distance=final.distance,
            sample_period=self.dt,
            controller_mode="terminal-braking",
            waypoint_passages=self._last_waypoint_passages,
            reference_curvature=final.curvature,
        )

    def _finish_if_at_last_waypoint(self, x, y, z, vx, vy):
        """在末点容差内停车时补记经过误差，预计时间不阻挡提前完成。"""
        if self._next_waypoint != len(self.trajectory.waypoint_distances) - 1:
            return False
        target_distance = self.trajectory.waypoint_distances[-1]
        target = self._sample_at_distance(target_distance)
        error = math.hypot(x - target.x, y - target.y)
        if error <= 0.25 and abs(z - target.z) <= 0.25 and math.hypot(vx, vy) <= 0.35:
            self._last_waypoint_passages += ((self._next_waypoint, error),)
            self._next_waypoint += 1
            return True
        return False

    def _check_waypoint_passage(self, position, previous_progress, progress):
        """按输入顺序记录航点；漏点只告警并继续前飞。"""
        warnings = []
        passages = []
        while self._next_waypoint < len(self.trajectory.waypoint_distances):
            target_distance = self.trajectory.waypoint_distances[self._next_waypoint]
            if progress < target_distance:
                self._last_waypoint_passages = tuple(passages)
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
            error = math.hypot(crossing[0] - target[0], crossing[1] - target[1])
            if error > 0.25:
                warnings.append(
                    "第 %d 个航点经过误差 %.2f m；继续前飞，不回头" %
                    (self._next_waypoint + 1, error)
                )
            passages.append((self._next_waypoint, error))
            self._next_waypoint += 1
        self._last_waypoint_passages = tuple(passages)
        return "；".join(warnings)

    def _project_progress(self, x, y, z):
        """在当前弧长附近单调投影，避免自交路线跳到远端分支。"""
        samples = self.trajectory.samples
        distances = self._distances
        first = max(0, bisect_right(distances, self._progress_distance) - 2)
        if self._previous_position is None:
            observed_step = 0.0
        else:
            observed_step = math.dist(self._previous_position, (x, y, z))
        max_advance = max(
            1.0, observed_step + max(0.35, self.limits.max_speed * self.dt * 2.0)
        )
        last = min(
            len(samples) - 1,
            bisect_right(distances, self._progress_distance + max_advance) + 2,
        )
        best_distance = self._progress_distance
        best_score = float("inf")
        best_error = float("inf")
        closest_z = samples[first].z
        for index in range(first, last):
            a, b = samples[index], samples[index + 1]
            delta = (b.x - a.x, b.y - a.y, b.z - a.z)
            length_squared = sum(value * value for value in delta)
            if length_squared <= 1e-12:
                fraction = 0.0
            else:
                offset = (x - a.x, y - a.y, z - a.z)
                fraction = max(0.0, min(1.0,
                    sum(a * b for a, b in zip(offset, delta)) / length_squared))
            px, py, pz = (a.x + fraction * delta[0],
                          a.y + fraction * delta[1],
                          a.z + fraction * delta[2])
            error = math.hypot(x - px, y - py)
            score = error * error + (z - pz) ** 2
            if score < best_score:
                best_score = score
                best_error = error
                best_distance = a.distance + fraction * (b.distance - a.distance)
                closest_z = pz
        return best_distance, best_error, closest_z

    def _sample_at_distance(self, distance):
        """按弧长读取规划点；航点悬停的重复弧长仍映射到同一位置。"""
        samples = self.trajectory.samples
        if distance <= self._distances[0]:
            return samples[0]
        if distance >= self._distances[-1]:
            return samples[-1]
        right = bisect_right(self._distances, distance)
        first, second = samples[right - 1], samples[right]
        span = second.distance - first.distance
        fraction = (distance - first.distance) / span if span > 1e-12 else 0.0
        return _blend(first, second, fraction)

    def _sample_at_time(self, when):
        """读取速度计划上的时间点，仅用于识别明确的航点悬停窗口。"""
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

    def _scheduled_speed(self, progress):
        """读取规划器的平面速度；只在零速采样处探看很短的前方距离以起步。"""
        sample = self._sample_at_distance(progress)
        speed = math.hypot(sample.vx, sample.vy)
        if speed > 0.12 or progress >= self._distances[-1] - 1e-5:
            return min(self.limits.max_speed, speed)
        probe_distance = min(self._distances[-1], progress + 0.25)
        probe = self._sample_at_distance(probe_distance)
        probe_speed = math.hypot(probe.vx, probe.vy)
        # 探测只用于从静止开始和越过无悬停的短暂停车点，不越过局部速度上限。
        return min(self.limits.max_speed, speed + 0.30 * probe_speed)

    def _speed_with_acceleration_preview(self, progress, actual_speed):
        """在连续加速段提前给油，补偿 AirSim 速度响应滞后。

        只读取当前路线速度曲线前方一个响应距离，并限制增量和当前曲率速度上限。
        如果这段距离里遇到减速、弯道速度谷或停车点，就保留原速度计划，避免跨过
        已规划的制动区。这样能更早利用长直线的加速距离，却不会把下一段的速度
        提前带进弯道。
        """
        current_speed = self._scheduled_speed(progress)
        response_horizon = max(
            0.0,
            max(self.limits.response_time, self.limits.velocity_time_constant)
            + self.limits.command_delay,
        )
        if response_horizon <= 1e-6 or progress >= self._distances[-1] - 1e-5:
            return current_speed

        lead_distance = max(
            0.25,
            actual_speed * response_horizon
            + 0.5 * self.limits.max_acceleration * response_horizon ** 2,
        )
        target_distance = min(self._distances[-1], progress + lead_distance)
        future_speed = self._scheduled_speed(target_distance)
        if future_speed <= current_speed + 0.02:
            return current_speed
        if self._has_speed_drop(progress, target_distance, current_speed):
            return current_speed

        # 不因前视加速超过当前位置的曲率约束；速度下降仍完全由原计划决定。
        curvature = abs(self._sample_at_distance(progress).curvature)
        curve_cap = (
            math.sqrt(max(0.01, self.limits.max_lateral_acceleration) / curvature)
            if curvature > 1e-8 else self.limits.max_speed
        )
        increase_cap = self.limits.max_acceleration * response_horizon
        return max(
            current_speed,
            min(future_speed, current_speed + increase_cap, curve_cap,
                self.limits.max_speed),
        )

    def _has_speed_drop(self, first_distance, last_distance, current_speed):
        """检查前视范围内是否先遇到减速或停车，防止加速跨过速度谷。"""
        first_index = bisect_right(self._distances, first_distance + 1e-6)
        for index in range(first_index, len(self.trajectory.samples)):
            sample = self.trajectory.samples[index]
            if sample.distance > last_distance + 1e-6:
                break
            planned_speed = math.hypot(sample.vx, sample.vy)
            if planned_speed < current_speed - 0.05:
                return True
        return False

    @staticmethod
    def _lookahead_distance(actual_speed, deviation, curvature):
        """直线缩短前视以快速收敛，弯道增加前视以顺着弦线自然内切。"""
        straight = (1.0 + 0.15 * actual_speed) / (1.0 + 0.30 * deviation)
        straight = max(0.75, min(2.1, straight))
        curved = min(3.5, max(1.5, 0.9 + 0.42 * actual_speed))
        # 平滑混合直线和弯道设置，避免曲率刚过阈值时前视距离跳变。
        curve_weight = max(0.0, min(1.0, (curvature - 0.01) / 0.025))
        return straight + curve_weight * (curved - straight)

    def _final_braking_cap(self, progress):
        """把响应延迟计入末点制动，防止速度曲线误差导致明显冲过终点。"""
        target_distance = self.trajectory.waypoint_distances[-1]
        remaining = max(0.0, target_distance - progress - 0.10)
        delay = (max(self.limits.response_time, self.limits.velocity_time_constant)
                 + self.limits.command_delay)
        delayed_distance = self.limits.max_braking * delay
        return max(
            0.0,
            math.sqrt(delayed_distance ** 2
                      + 2.0 * self.limits.max_braking * remaining)
            - delayed_distance,
        )

    def _tangent_at_distance(self, distance):
        """取曲线切向；零速停车点使用其前方的第一个非零线段。"""
        sample = self._sample_at_distance(distance)
        speed = math.hypot(sample.vx, sample.vy)
        if speed > 1e-4:
            return sample.vx / speed, sample.vy / speed
        index = min(len(self.trajectory.samples) - 1,
                    max(0, bisect_right(self._distances, distance) - 1))
        for offset in range(1, len(self.trajectory.samples)):
            if index + offset < len(self.trajectory.samples):
                first = self.trajectory.samples[index + offset - 1]
                second = self.trajectory.samples[index + offset]
                dx, dy = second.x - first.x, second.y - first.y
                length = math.hypot(dx, dy)
                if length > 1e-8:
                    return dx / length, dy / length
        return 1.0, 0.0

    @staticmethod
    def _forward_carrot_direction(position, target, tangent, lookahead):
        """朝前视点走弦线；目标落到侧后方时也保留横向回归分量。"""
        dx, dy = target[0] - position[0], target[1] - position[1]
        forward = dx * tangent[0] + dy * tangent[1]
        lateral = dx * -tangent[1] + dy * tangent[0]
        # 偏差较大时，前视点可能投影到当前切向后方。丢掉横向分量会让飞机
        # 继续平行于航线飞，无法收敛；给它一个小的正向分量，同时向前视点靠拢。
        forward = max(forward, max(0.15, 0.20 * lookahead))

        # 限制从航向的最大夹角，避免极端偏差时横飞或朝反方向修线。
        max_lateral = forward * math.tan(math.radians(65.0))
        lateral = max(-max_lateral, min(max_lateral, lateral))
        vx = forward * tangent[0] - lateral * tangent[1]
        vy = forward * tangent[1] + lateral * tangent[0]
        length = math.hypot(vx, vy)
        if length <= 1e-8:
            return tangent
        return vx / length, vy / length

    def _slew_command(self, desired):
        """只限制速度大小变化；方向直接跟随前视目标，避免弯后拖着横向指令。"""
        previous = self._previous_command
        previous_speed = math.hypot(*previous)
        target_speed = math.hypot(*desired)
        if target_speed > 1e-8:
            direction = (desired[0] / target_speed, desired[1] / target_speed)
        elif previous_speed > 1e-8:
            # 刹车时沿当前运动方向减速；不要让零速目标引入新的横向转向。
            direction = (previous[0] / previous_speed, previous[1] / previous_speed)
        else:
            direction = (0.0, 0.0)

        increasing = target_speed >= previous_speed
        acceleration = self.limits.max_acceleration if increasing else self.limits.max_braking
        maximum_change = acceleration * self.dt
        speed_change = max(-maximum_change, min(maximum_change, target_speed - previous_speed))
        command_speed = max(0.0, previous_speed + speed_change)
        command = (direction[0] * command_speed, direction[1] * command_speed)
        self._previous_command = command
        return command

    def _is_holding_at(self, timed_sample, elapsed, progress):
        """仅在规划的航点悬停时段保持零速度。"""
        if math.hypot(timed_sample.vx, timed_sample.vy) > 0.05:
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

    def _result(self, command, z, progress, deviation, reference, mode,
                planned_speed=0.0):
        return ControlResult(
            command[0], command[1], z, progress, deviation,
            warning=self._last_warning,
            reference_x=reference.x,
            reference_y=reference.y,
            reference_z=reference.z,
            reference_distance=reference.distance,
            sample_period=self.dt,
            controller_mode=mode,
            waypoint_passages=self._last_waypoint_passages,
            reference_curvature=reference.curvature,
            planned_speed=planned_speed,
        )


def _blend(first: TrajectorySample, second: TrajectorySample, fraction: float):
    """对相邻采样插值，保证日志和控制使用一致的曲线值。"""
    names = (
        "time", "distance", "x", "y", "z", "vx", "vy", "vz",
        "ax", "ay", "az", "curvature",
    )
    values = {
        name: getattr(first, name) + fraction * (getattr(second, name) - getattr(first, name))
        for name in names
    }
    values["waypoint_index"] = first.waypoint_index
    return TrajectorySample(**values)
