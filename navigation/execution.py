"""闭环预测 AirSim 飞行器对一条规划轨迹的实际跟踪结果。"""

from collections import deque
import math
from typing import Sequence

from .controller import CarrotPathController
from .models import (
    CircleObstacle,
    ExecutionPrediction,
    KinematicLimits,
    PlannedTrajectory,
    TrajectorySample,
)


def predict_execution(
    trajectory: PlannedTrajectory,
    limits: KinematicLimits,
    obstacles: Sequence[CircleObstacle] = (),
    dt: float = 0.05,
) -> ExecutionPrediction:
    """用与自动巡航相同的控制器模拟飞机响应、净空和航点经过误差。

    AirSim 的 `moveByVelocityZAsync` 给出的是目标水平速度。实飞响应不是瞬时的，
    因此这里用标定的指令延迟、一阶速度响应和矢量加速度上限推进飞机状态。
    该模拟用于比较候选路线，不会改变实际巡航时消费的引导曲线。
    """
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("预测步长必须大于 0")
    if not trajectory.samples:
        raise ValueError("不能预测空轨迹")
    if limits.velocity_time_constant <= 0.0 or limits.velocity_acceleration <= 0.0:
        raise ValueError("速度响应时间常数和加速度必须大于 0")

    if len(trajectory.samples) == 1:
        # 起点与所有航点重合时没有水平飞行段，保留既有的到点/悬停语义。
        sample = trajectory.samples[0]
        clearance = _point_clearance(sample.position, obstacles)
        complete = all(distance <= 1e-6 for distance in trajectory.waypoint_distances)
        errors = tuple(0.0 if complete else float("inf")
                       for _ in trajectory.waypoint_distances)
        required_clearance = limits.safety_clearance + limits.tracking_reserve
        return ExecutionPrediction(
            samples=trajectory.samples,
            estimated_time=trajectory.estimated_time,
            minimum_clearance=clearance,
            waypoint_errors=errors,
            complete=complete,
            safe=complete and clearance >= required_clearance - 1e-6,
        )

    controller = CarrotPathController(trajectory, limits, obstacles, dt=dt)
    first = trajectory.samples[0]
    position = (first.x, first.y, first.z)
    velocity = (0.0, 0.0)
    elapsed = 0.0
    distance = 0.0
    pending_commands = deque(
        [(0.0, 0.0)] * int(math.ceil(limits.command_delay / dt - 1e-9))
    )
    errors = [float("inf")] * len(trajectory.waypoint_distances)
    states = [
        (0.0, distance, position, velocity, (0.0, 0.0), 0.0)
    ]
    minimum_clearance = _point_clearance(position, obstacles)
    complete = False

    # 有控制器故障时停止评估；未完成的路线不参与候选比较。
    max_duration = max(30.0, trajectory.estimated_time * 2.5 + 10.0)
    max_steps = int(math.ceil(max_duration / dt))
    for _ in range(max_steps):
        result = controller.step(position, velocity, elapsed)
        for waypoint_index, error in result.waypoint_passages:
            errors[waypoint_index] = error
        if result.fault:
            break
        if result.finished:
            complete = True
            break

        command = (result.vx, result.vy)
        pending_commands.append(command)
        applied = pending_commands.popleft() if pending_commands else command
        next_velocity = _advance_velocity(
            velocity, applied, dt,
            limits.velocity_time_constant,
            limits.velocity_acceleration,
        )
        next_position = (
            position[0] + 0.5 * (velocity[0] + next_velocity[0]) * dt,
            position[1] + 0.5 * (velocity[1] + next_velocity[1]) * dt,
            position[2] + max(
                -limits.max_vertical_speed * dt,
                min(limits.max_vertical_speed * dt, result.z - position[2]),
            ),
        )
        distance += math.dist(position, next_position)
        minimum_clearance = min(
            minimum_clearance,
            _segment_clearance(position, next_position, obstacles),
        )
        elapsed += dt
        states.append((
            elapsed,
            distance,
            next_position,
            next_velocity,
            ((next_velocity[0] - velocity[0]) / dt,
             (next_velocity[1] - velocity[1]) / dt),
            (next_position[2] - position[2]) / dt,
        ))
        position, velocity = next_position, next_velocity

    samples = _to_samples(states)
    waypoint_errors = tuple(errors)
    required_clearance = limits.safety_clearance + limits.tracking_reserve
    safe = (
        complete
        and all(math.isfinite(error) for error in waypoint_errors)
        and minimum_clearance >= required_clearance - 1e-6
    )
    return ExecutionPrediction(
        samples=samples,
        estimated_time=elapsed,
        minimum_clearance=minimum_clearance,
        waypoint_errors=waypoint_errors,
        complete=complete,
        safe=safe,
    )


def _advance_velocity(current, target, dt, time_constant, acceleration):
    """按一阶速度响应推进，并限制每秒可改变的速度矢量。"""
    response = 1.0 - math.exp(-dt / time_constant)
    change = ((target[0] - current[0]) * response,
              (target[1] - current[1]) * response)
    change_length = math.hypot(*change)
    maximum_change = acceleration * dt
    if change_length > maximum_change:
        scale = maximum_change / change_length
        change = (change[0] * scale, change[1] * scale)
    return current[0] + change[0], current[1] + change[1]


def _point_clearance(position, obstacles):
    return min(
        (math.hypot(position[0] - obstacle.x, position[1] - obstacle.y)
         - obstacle.radius for obstacle in obstacles),
        default=float("inf"),
    )


def _segment_clearance(first, second, obstacles):
    """按线段到障碍圆的最近点计算净空，避免高速采样漏掉擦碰。"""
    dx, dy = second[0] - first[0], second[1] - first[1]
    length_squared = dx * dx + dy * dy
    clearances = []
    for obstacle in obstacles:
        if length_squared <= 1e-12:
            fraction = 0.0
        else:
            fraction = ((obstacle.x - first[0]) * dx
                        + (obstacle.y - first[1]) * dy) / length_squared
            fraction = max(0.0, min(1.0, fraction))
        x = first[0] + fraction * dx
        y = first[1] + fraction * dy
        clearances.append(math.hypot(x - obstacle.x, y - obstacle.y) - obstacle.radius)
    return min(clearances, default=float("inf"))


def _to_samples(states):
    samples = []
    for index, (when, distance, position, velocity, acceleration, vertical_speed) in enumerate(states):
        curvature = 0.0
        if 0 < index < len(states) - 1:
            before = states[index - 1][2]
            after = states[index + 1][2]
            curvature = _curvature(before, position, after)
        samples.append(TrajectorySample(
            time=when,
            distance=distance,
            x=position[0], y=position[1], z=position[2],
            vx=velocity[0], vy=velocity[1], vz=vertical_speed,
            ax=acceleration[0], ay=acceleration[1], az=0.0,
            curvature=curvature,
        ))
    return tuple(samples)


def _curvature(first, middle, last):
    """用相邻三点估算实际模拟 XY 轨迹的曲率。"""
    a = math.hypot(middle[0] - first[0], middle[1] - first[1])
    b = math.hypot(last[0] - middle[0], last[1] - middle[1])
    c = math.hypot(last[0] - first[0], last[1] - first[1])
    denominator = a * b * c
    if denominator <= 1e-10:
        return 0.0
    twice_area = abs((middle[0] - first[0]) * (last[1] - first[1])
                     - (middle[1] - first[1]) * (last[0] - first[0]))
    return 2.0 * twice_area / denominator
