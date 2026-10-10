"""按固定顺序规划必经航点，并在安全候选中选择预计用时最短者。"""

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Sequence, Tuple

from .curve import CurveSamples, fit_safe_curve, splice_straight_leg
from .geometry import (
    GeometricPath,
    InflatedCircle,
    inflate_obstacles,
    plan_leg_candidates,
    segment_clear,
)
from .models import (
    CircleObstacle,
    KinematicLimits,
    PlannedTrajectory,
    Waypoint,
)
from .time_parameterization import time_parameterize


class PlanningError(RuntimeError):
    """路线因安全、可达或动力学限制而无法规划。"""


@dataclass(frozen=True)
class PlannerSettings:
    """单次规划参数；障碍间距按飞机中心到圆柱边缘计算。"""

    max_candidates_per_leg: int = 8
    beam_width: int = 24
    curve_margin: float = 0.35
    sample_spacing: float = 0.10


def plan_ordered_route(
    start: Tuple[float, float, float],
    waypoints: Sequence[Waypoint],
    obstacles: Sequence[CircleObstacle],
    limits: KinematicLimits,
    settings: PlannerSettings = PlannerSettings(),
) -> PlannedTrajectory:
    """规划一条按输入顺序经过所有航点的、避障且带时间参数的航迹。"""
    if not waypoints:
        raise PlanningError("还没有航点")
    numeric_limits = (
        limits.max_speed, limits.max_vertical_speed, limits.max_acceleration,
        limits.max_braking, limits.max_lateral_acceleration, limits.response_time,
        limits.command_delay, limits.safety_clearance, limits.tracking_reserve,
    )
    if not all(math.isfinite(value) for value in numeric_limits):
        raise PlanningError("飞行约束必须是有限数值")
    if len(start) != 3 or not all(math.isfinite(float(value)) for value in start):
        raise PlanningError("规划起点必须是有限坐标")
    if limits.max_speed <= 0.0:
        raise PlanningError("最高速度必须大于 0 m/s")
    if min(limits.max_vertical_speed, limits.max_acceleration,
           limits.max_braking, limits.max_lateral_acceleration) <= 0.0:
        raise PlanningError("垂直速度、加速度、制动能力和横向加速度必须大于 0")
    if (limits.safety_clearance < 0.0 or limits.tracking_reserve < 0.0
            or limits.response_time < 0.0 or limits.command_delay < 0.0):
        raise PlanningError("安全间距、跟踪余量和响应延迟不能为负数")
    if (settings.max_candidates_per_leg < 1 or settings.beam_width < 1
            or not math.isfinite(settings.curve_margin) or settings.curve_margin < 0.0
            or not math.isfinite(settings.sample_spacing) or settings.sample_spacing <= 0.0):
        raise PlanningError("候选数量、曲线余量或采样间距无效")

    targets = [(float(start[0]), float(start[1]), float(start[2]))]
    targets.extend(waypoint.position for waypoint in waypoints)
    protected_circles = tuple(
        InflatedCircle(
            obstacle.x,
            obstacle.y,
            obstacle.radius + limits.safety_clearance + limits.tracking_reserve,
            index,
        )
        for index, obstacle in enumerate(obstacles)
    )
    search_circles = inflate_obstacles(
        obstacles,
        limits.safety_clearance,
        limits.tracking_reserve,
        settings.curve_margin,
    )

    for index, target in enumerate(targets):
        if not _point_clear((target[0], target[1]), protected_circles):
            label = "起点" if index == 0 else "第 %d 个航点" % index
            raise PlanningError("%s 位于障碍物安全区内，请调整点位或安全间距" % label)

    signature = _mission_signature(start, waypoints, obstacles, limits, settings)
    states = [([], [], (), 0.0)]
    infeasible_leg = None

    for leg_index, (leg_start, leg_goal) in enumerate(zip(targets, targets[1:])):
        leg_candidates = plan_leg_candidates(
            (leg_start[0], leg_start[1]),
            (leg_goal[0], leg_goal[1]),
            search_circles,
            max_candidates=settings.max_candidates_per_leg,
        )
        if not leg_candidates:
            infeasible_leg = leg_index + 1
            break

        expanded = []
        for points_3d, anchors, topology, _score in states:
            for candidate in leg_candidates:
                leg_points = _lift_leg(candidate, leg_start[2], leg_goal[2])
                combined = list(points_3d)
                if not combined:
                    combined.extend(leg_points)
                else:
                    combined.extend(leg_points[1:])
                next_anchors = list(anchors)
                next_anchors.append(len(combined) - 1)
                next_topology = topology + (candidate.signature,)
                score = _polyline_time_estimate(combined, limits)
                expanded.append((combined, next_anchors, next_topology, score))
        expanded.sort(key=lambda state: state[3])
        states = expanded[:settings.beam_width]

    if infeasible_leg is not None:
        raise PlanningError("第 %d 段没有满足安全间距的可行绕障路线" % infeasible_leg)

    feasible = []
    for route_points, anchors, topology, _score in states:
        base_curve = fit_safe_curve(
            route_points,
            anchors,
            protected_circles,
            sample_spacing=settings.sample_spacing,
        )
        curve_candidates = []
        if base_curve is not None:
            curve_candidates.append(base_curve)

        # 如果可见图本来给出一条直线，但全局样条把它弯开，增加两种真实可飞的
        # 版本：曲线切向平滑接入；若安全校验失败，则在航点停车后沿直线飞行。
        for leg_index, anchor_index in enumerate(anchors):
            first_index = 0 if leg_index == 0 else anchors[leg_index - 1]
            if anchor_index != first_index + 1:
                continue
            start_xy = route_points[first_index][:2]
            end_xy = route_points[anchor_index][:2]
            if not segment_clear(start_xy, end_xy, search_circles):
                continue
            try:
                smooth_straight = fit_safe_curve(
                    route_points,
                    anchors,
                    protected_circles,
                    sample_spacing=settings.sample_spacing,
                    preserve_straight_leg=leg_index,
                )
            except ValueError:
                smooth_straight = None
            if smooth_straight is not None:
                curve_candidates.append(smooth_straight)
            elif base_curve is not None:
                # 两段外侧仍采用已认证曲线，直线只连接原样条经过的精确航点。
                curve_candidates.append(splice_straight_leg(
                    route_points,
                    anchors,
                    base_curve,
                    leg_index,
                    sample_spacing=settings.sample_spacing,
                ))

        for curve in curve_candidates:
            try:
                trajectory = time_parameterize(curve, waypoints, limits, signature)
            except ValueError:
                continue
            clearance = _minimum_center_clearance(curve.positions, obstacles)
            trajectory = PlannedTrajectory(
                samples=trajectory.samples,
                waypoint_distances=trajectory.waypoint_distances,
                estimated_time=trajectory.estimated_time,
                length=trajectory.length,
                minimum_clearance=clearance,
                mission_signature=trajectory.mission_signature,
            )
            feasible.append((trajectory.estimated_time, topology, trajectory))

    if not feasible:
        raise PlanningError(
            "切线候选无法在保持安全间距时平滑通过所有航点；"
            "请放宽障碍间距或调整航点，系统不会改用直线穿过障碍"
        )
    feasible.sort(key=lambda item: (item[0], item[1]))
    return feasible[0][2]


def _lift_leg(candidate: GeometricPath, start_z: float, goal_z: float):
    """沿每段路线弧长平滑分配高度端点，保持用户航点高度精确。"""
    points = candidate.points
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.dist(first, second))
    total = cumulative[-1]
    result = []
    for index, (x, y) in enumerate(points):
        fraction = cumulative[index] / total if total > 1e-10 else index / max(1, len(points) - 1)
        result.append((x, y, start_z + (goal_z - start_z) * fraction))
    # 保护精确端点，避免数值运算改变用户输入的航点位置。
    result[0] = (points[0][0], points[0][1], start_z)
    result[-1] = (points[-1][0], points[-1][1], goal_z)
    return result


def _polyline_time_estimate(points, limits):
    """供有限宽度候选保留使用的快速近似时间。"""
    if len(points) < 2:
        return 0.0
    segment_lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    speed = [limits.max_speed] * len(points)
    for index in range(1, len(points) - 1):
        curvature = _three_point_curvature(points[index - 1], points[index], points[index + 1])
        if curvature > 1e-8:
            speed[index] = min(
                speed[index],
                math.sqrt(max(0.01, limits.max_lateral_acceleration) / curvature),
            )
    speed[0] = 0.0
    speed[-1] = 0.0
    squared = [value * value for value in speed]
    for index, length in enumerate(segment_lengths, start=1):
        squared[index] = min(
            squared[index],
            squared[index - 1] + 2.0 * limits.max_acceleration * length,
        )
    for index in range(len(points) - 2, -1, -1):
        squared[index] = min(
            squared[index],
            squared[index + 1] + 2.0 * limits.max_braking * segment_lengths[index],
        )
    speed = [math.sqrt(value) for value in squared]
    total = 0.0
    for index, length in enumerate(segment_lengths):
        if length <= 1e-9:
            continue
        denominator = speed[index] + speed[index + 1]
        if denominator <= 1e-9:
            return float("inf")
        total += 2.0 * length / denominator
    return total


def _three_point_curvature(a, b, c):
    """由三点外接圆估算离散曲率；共线时曲率为零。"""
    ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
    twice_area = abs((b[0] - a[0]) * (c[1] - a[1])
                     - (b[1] - a[1]) * (c[0] - a[0]))
    denominator = ab * bc * ca
    return 2.0 * twice_area / denominator if denominator > 1e-10 else 0.0


def _minimum_center_clearance(points, obstacles):
    """记录轨迹采样到真实障碍圆边缘的最小中心距离。"""
    if not obstacles:
        return float("inf")
    return min(
        math.hypot(point[0] - obstacle.x, point[1] - obstacle.y) - obstacle.radius
        for point in points
        for obstacle in obstacles
    )


def _point_clear(point, circles):
    return all(math.hypot(point[0] - circle.x, point[1] - circle.y)
               >= circle.radius - 1e-7 for circle in circles)


def _mission_signature(start, waypoints, obstacles, limits, settings):
    """形成稳定的任务哈希，供自动巡航检查规划是否仍对应当前任务。"""
    payload = {
        "start": [round(float(value), 5) for value in start],
        "waypoints": [[round(w.x, 5), round(w.y, 5), round(w.z, 5), round(w.hold, 4)]
                      for w in waypoints],
        "obstacles": [[round(o.x, 5), round(o.y, 5), round(o.radius, 5)]
                      for o in obstacles],
        "limits": [round(value, 5) for value in (
            limits.max_speed, limits.max_vertical_speed, limits.max_acceleration,
            limits.max_braking, limits.max_lateral_acceleration,
            limits.response_time, limits.command_delay,
            limits.safety_clearance, limits.tracking_reserve,
        )],
        "settings": [settings.max_candidates_per_leg, settings.beam_width,
                     round(settings.curve_margin, 5), round(settings.sample_spacing, 5)],
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()
