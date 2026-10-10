"""满足飞行器速度和加速度约束的近似最短时间定时。

对固定几何曲线离散出弧长采样，根据速度上限、垂直速度和曲率生成局部速度上界，
再以前向加速、反向制动可达性传播得到速度平方曲线。该方法优化的是已选曲线上的
时间，不会为了追求速度越过障碍，也不会假设飞机能瞬间改变速度。
"""

from bisect import bisect_left
import math
from typing import Sequence, Tuple

from .curve import CurveSamples
from .models import KinematicLimits, PlannedTrajectory, TrajectorySample, Waypoint


def time_parameterize(
    curve: CurveSamples,
    waypoints: Sequence[Waypoint],
    limits: KinematicLimits,
    mission_signature: str,
) -> PlannedTrajectory:
    """计算固定曲线上的速度、加速度、预计时间和到点悬停采样。"""
    positions = curve.positions
    distances = curve.distances
    if not positions:
        raise ValueError("轨迹没有位置采样")
    if len(positions) == 1:
        # 起点与唯一航点重合时仍可执行到点悬停，不制造零长度飞行段。
        x, y, z = positions[0]
        output = [TrajectorySample(
            time=0.0, distance=0.0, x=x, y=y, z=z,
            vx=0.0, vy=0.0, vz=0.0,
            ax=0.0, ay=0.0, az=0.0, curvature=0.0,
            waypoint_index=0,
        )]
        elapsed = 0.0
        for waypoint_index, waypoint in enumerate(waypoints):
            if waypoint.hold <= 0.0:
                continue
            elapsed += waypoint.hold
            output.append(TrajectorySample(
                time=elapsed, distance=0.0, x=x, y=y, z=z,
                vx=0.0, vy=0.0, vz=0.0,
                ax=0.0, ay=0.0, az=0.0, curvature=0.0,
                waypoint_index=waypoint_index,
            ))
        return PlannedTrajectory(
            samples=tuple(output),
            waypoint_distances=tuple(0.0 for _ in waypoints),
            estimated_time=elapsed,
            length=0.0,
            minimum_clearance=0.0,
            mission_signature=mission_signature,
        )

    count = len(positions)
    speed_cap = [limits.max_speed] * count
    for index, position in enumerate(positions):
        curvature = abs(curve.curvatures[index])
        if curvature > 1e-8:
            speed_cap[index] = min(
                speed_cap[index],
                math.sqrt(max(0.01, limits.max_lateral_acceleration) / curvature),
            )
        if index > 0:
            ds = distances[index] - distances[index - 1]
            if ds > 1e-9:
                vertical_slope = abs(position[2] - positions[index - 1][2]) / ds
                if vertical_slope > 1e-8:
                    speed_cap[index] = min(
                        speed_cap[index], limits.max_vertical_speed / vertical_slope
                    )

    waypoint_sample_indices = [
        min(range(count), key=lambda i: abs(distances[i] - waypoint_distance))
        for waypoint_distance in curve.waypoint_distances
    ]
    stop_indices = {0, count - 1}
    for waypoint_index, sample_index in enumerate(waypoint_sample_indices):
        waypoint = waypoints[waypoint_index]
        if waypoint.hold > 0.0 or _sharp_turn(positions, sample_index):
            stop_indices.add(sample_index)
    for stop_distance in curve.stop_distances:
        stop_indices.add(min(
            range(count),
            key=lambda index: abs(distances[index] - stop_distance),
        ))
    for index in stop_indices:
        speed_cap[index] = 0.0

    # 将响应延迟作为一次制动事件处理，而不是每个 10 cm 采样重复扣除一次。
    # 用实测机体时间常数安排提前制动；仅使用通信延迟会让速度曲线晚刹。
    braking_delay = max(
        0.0,
        max(limits.response_time, limits.velocity_time_constant)
        + limits.command_delay,
    )
    speed_cap = _apply_delayed_braking_caps(
        speed_cap, distances, limits.max_braking, braking_delay
    )

    # 在速度平方域做可达性传播，v² 的变化与距离成线性关系。
    speed_squared = [cap * cap for cap in speed_cap]
    for index in range(1, count):
        ds = distances[index] - distances[index - 1]
        reachable = speed_squared[index - 1] + 2.0 * limits.max_acceleration * ds
        speed_squared[index] = min(speed_squared[index], reachable)
    for index in range(count - 2, -1, -1):
        ds = distances[index + 1] - distances[index]
        reachable = speed_squared[index + 1] + 2.0 * limits.max_braking * ds
        speed_squared[index] = min(speed_squared[index], reachable)
    speeds = [math.sqrt(max(0.0, value)) for value in speed_squared]

    base_times = [0.0]
    for index in range(count - 1):
        ds = distances[index + 1] - distances[index]
        speed_sum = speeds[index] + speeds[index + 1]
        if ds <= 1e-10:
            dt = 0.0
        elif speed_sum <= 1e-8:
            raise ValueError("速度约束无法让飞机通过该曲线采样段")
        else:
            dt = 2.0 * ds / speed_sum
        base_times.append(base_times[-1] + dt)

    # 趋势切换处补上实测响应延迟，使曲线定时、航点悬停和总 T 使用同一时钟。
    # 时间戳只表示实测指令响应延迟；机体时间常数已用于提前制动和闭环仿真，
    # 不要在每次离散速度趋势切换时重复加上一整段时间常数。
    schedule_delay = max(0.0, limits.response_time + limits.command_delay)
    base_times = _add_response_delays(base_times, speeds, schedule_delay)

    waypoint_by_sample = {}
    for waypoint_index, sample_index in enumerate(waypoint_sample_indices):
        waypoint_by_sample.setdefault(sample_index, []).append(waypoint_index)

    # 把悬停展开为同位置、同弧长但时间更晚的采样，控制器自然会在此等待。
    output = []
    elapsed_hold = 0.0
    for index, position in enumerate(positions):
        waypoint_ids = waypoint_by_sample.get(index, [])
        marker = waypoint_ids[0] if waypoint_ids else None
        velocity = _velocity_at(positions, speeds, index)
        acceleration = _acceleration_at(positions, speeds, base_times, index)
        output.append(TrajectorySample(
            time=base_times[index] + elapsed_hold,
            distance=distances[index],
            x=position[0], y=position[1], z=position[2],
            vx=velocity[0], vy=velocity[1], vz=velocity[2],
            ax=acceleration[0], ay=acceleration[1], az=acceleration[2],
            curvature=curve.curvatures[index],
            waypoint_index=marker,
        ))
        for waypoint_index in waypoint_ids:
            hold = waypoints[waypoint_index].hold
            if hold <= 0.0:
                continue
            elapsed_hold += hold
            output.append(TrajectorySample(
                time=base_times[index] + elapsed_hold,
                distance=distances[index],
                x=position[0], y=position[1], z=position[2],
                vx=0.0, vy=0.0, vz=0.0,
                ax=0.0, ay=0.0, az=0.0,
                curvature=curve.curvatures[index],
                waypoint_index=waypoint_index,
            ))

    total_length = distances[-1]
    hold_time = sum(waypoint.hold for waypoint in waypoints)
    # 每段连续的加速或减速趋势都可能触发一次响应延迟；同一趋势中的细采样点只计一次。
    estimated_time = base_times[-1] + hold_time
    minimum_clearance = 0.0
    return PlannedTrajectory(
        samples=tuple(output),
        waypoint_distances=curve.waypoint_distances,
        estimated_time=estimated_time,
        length=total_length,
        minimum_clearance=minimum_clearance,
        mission_signature=mission_signature,
    )


def _sharp_turn(positions, index, threshold_degrees=145.0):
    """对几乎掉头的必经点停车，避免样条在狭小空间绕出危险回环。"""
    if index <= 0 or index >= len(positions) - 1:
        return False
    before = _unit(tuple(positions[index][axis] - positions[index - 1][axis]
                         for axis in range(3)))
    after = _unit(tuple(positions[index + 1][axis] - positions[index][axis]
                        for axis in range(3)))
    if before is None or after is None:
        return True
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(before, after))))
    angle = math.degrees(math.acos(dot))
    return angle >= threshold_degrees


def _add_response_delays(times, speeds, response_delay):
    """在每个持续加减速趋势开始处加入一次响应延迟。"""
    if response_delay <= 0.0:
        return times
    events = set()
    previous_sign = 0
    for index, (first, second) in enumerate(zip(speeds, speeds[1:]), start=1):
        delta = second - first
        sign = 1 if delta > 0.05 else (-1 if delta < -0.05 else 0)
        if sign and sign != previous_sign:
            events.add(index)
        if sign:
            previous_sign = sign
    shifted = []
    delay = 0.0
    for index, value in enumerate(times):
        if index in events:
            delay += response_delay
        shifted.append(value + delay)
    return shifted


def _apply_delayed_braking_caps(speed_cap, distances, braking, response_delay):
    """把制动延迟距离计入每个局部速度谷值，且每个谷值只计算一次。"""
    if response_delay <= 0.0 or len(speed_cap) < 3:
        return speed_cap
    critical = [0]
    for index in range(1, len(speed_cap) - 1):
        if (speed_cap[index] < speed_cap[index - 1] - 1e-4
                and speed_cap[index] <= speed_cap[index + 1] + 1e-4):
            critical.append(index)
    critical.append(len(speed_cap) - 1)
    result = list(speed_cap)
    delayed = braking * response_delay
    for target_index in critical:
        target_squared = speed_cap[target_index] ** 2
        for index in range(target_index):
            distance = max(0.0, distances[target_index] - distances[index])
            reachable = -delayed + math.sqrt(
                delayed * delayed + target_squared + 2.0 * braking * distance
            )
            result[index] = min(result[index], max(0.0, reachable))
    return result


def _velocity_at(positions, speeds, index):
    if index == 0:
        direction = _unit(tuple(positions[1][axis] - positions[0][axis] for axis in range(3)))
    else:
        direction = _unit(tuple(positions[index][axis] - positions[index - 1][axis]
                                for axis in range(3)))
    if direction is None:
        return (0.0, 0.0, 0.0)
    return tuple(value * speeds[index] for value in direction)


def _acceleration_at(positions, speeds, times, index):
    if index <= 0:
        first, second = 0, 1
    elif index >= len(positions) - 1:
        first, second = index - 1, index
    else:
        first, second = index - 1, index + 1
    dt = times[second] - times[first]
    if dt <= 1e-9:
        return (0.0, 0.0, 0.0)
    first_velocity = _velocity_at(positions, speeds, first)
    second_velocity = _velocity_at(positions, speeds, second)
    return tuple((b - a) / dt for a, b in zip(first_velocity, second_velocity))


def _unit(vector):
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-10:
        return None
    return tuple(value / length for value in vector)
