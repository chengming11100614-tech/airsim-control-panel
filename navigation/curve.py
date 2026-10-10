"""平滑曲线拟合与障碍物安全校验。

切线图给出路线的绕障拓扑。这里用三次样条在图路线周围拟合 C2 连续曲线，
并把每段样条转换为 Bézier 控制多边形做递归安全检查。曲线控制点的凸包与
安全圆不相交时，Bézier 凸包性质可证明整段曲线都在安全圆之外。
"""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

import numpy as np
from scipy.interpolate import BPoly, CubicSpline

from .geometry import InflatedCircle


@dataclass(frozen=True)
class CurveSamples:
    """平滑曲线的弧长采样和原始参数位置。"""

    positions: Tuple[Tuple[float, float, float], ...]
    distances: Tuple[float, ...]
    curvatures: Tuple[float, ...]
    waypoint_distances: Tuple[float, ...]
    # 必须在这些弧长位置停下的折线转向点；用于平直候选的安全接入失败时。
    stop_distances: Tuple[float, ...] = ()


def fit_safe_curve(
    route_points: Sequence[Tuple[float, float, float]],
    waypoint_anchor_indices: Sequence[int],
    protected_circles: Sequence[InflatedCircle],
    sample_spacing: float = 0.10,
    preserve_straight_leg: int = -1,
) -> CurveSamples:
    """拟合经过所有航点的 C2 曲线；无法证明避障时返回 None。

    preserve_straight_leg 指定按顺序编号的航点段（0 表示起点到第一个航点）。
    对该段在两端约束直线切向和零曲率，使前后曲线平滑接入后，段内仍保持直线。
    """
    points = _remove_consecutive_duplicates(route_points)
    if not points:
        raise ValueError("规划路线没有位置点")

    # 原始点可能在航点处重复；只做索引映射，不改变航点的精确坐标约束。
    anchor_points = [route_points[index] for index in waypoint_anchor_indices]
    anchor_params = [_point_parameter(points, target) for target in anchor_points]

    if len(points) == 1:
        position = points[0]
        if any(math.hypot(position[0] - c.x, position[1] - c.y)
               < c.radius - 1e-7 for c in protected_circles):
            return None
        return CurveSamples((position,), (0.0,), (0.0,), tuple(0.0 for _ in anchor_params))

    parameter = _cumulative_distance(points)
    if parameter[-1] <= 1e-8:
        raise ValueError("路线长度接近零，无法拟合曲线")
    values = np.asarray(points, dtype=float)
    baseline = CubicSpline(parameter, values, axis=0, bc_type="natural")
    first_derivatives = np.asarray(baseline(parameter, 1), dtype=float)
    second_derivatives = np.asarray(baseline(parameter, 2), dtype=float)

    if preserve_straight_leg >= 0:
        if preserve_straight_leg >= len(waypoint_anchor_indices):
            raise ValueError("直线候选的航点段编号无效")
        first_index = (0 if preserve_straight_leg == 0
                       else waypoint_anchor_indices[preserve_straight_leg - 1])
        last_index = waypoint_anchor_indices[preserve_straight_leg]
        if last_index != first_index + 1:
            raise ValueError("指定的航点段不是无遮挡直线候选")
        delta = values[last_index] - values[first_index]
        length = float(np.linalg.norm(delta))
        if length <= 1e-8:
            raise ValueError("直线候选长度接近零")
        direction = delta / length
        first_derivatives[first_index] = direction
        first_derivatives[last_index] = direction
        second_derivatives[first_index] = 0.0
        second_derivatives[last_index] = 0.0

    # 每个结点给出位置、一阶和二阶导数。没有改约束的区间仍严格等于原三次样条；
    # 直线候选区间则由两端相同切向和零曲率约束成精确直线。
    derivatives = [
        [values[index], first_derivatives[index], second_derivatives[index]]
        for index in range(len(parameter))
    ]
    spline = BPoly.from_derivatives(parameter, derivatives, extrapolate=False)

    # 每个原始节点都是样条结点，故曲线会精确通过用户航点。
    if not _spline_is_clear(spline, parameter, protected_circles):
        return None

    sample_parameters = set(np.arange(0.0, parameter[-1], sample_spacing).tolist())
    sample_parameters.add(0.0)
    sample_parameters.add(float(parameter[-1]))
    sample_parameters.update(float(value) for value in anchor_params)
    sample_parameters = np.asarray(sorted(sample_parameters), dtype=float)

    sampled = spline(sample_parameters)
    first = spline(sample_parameters, 1)
    second = spline(sample_parameters, 2)
    distance = [0.0]
    for a, b in zip(sampled, sampled[1:]):
        distance.append(distance[-1] + float(np.linalg.norm(b - a)))

    curvature = []
    for derivative, acceleration in zip(first, second):
        horizontal_speed = math.hypot(float(derivative[0]), float(derivative[1]))
        denominator = horizontal_speed ** 3
        if denominator <= 1e-10:
            curvature.append(0.0)
        else:
            cross = derivative[0] * acceleration[1] - derivative[1] * acceleration[0]
            curvature.append(float(abs(cross) / denominator))

    # 把样条参数 u 映射为实际曲线弧长，供速度定时器定位到航点。
    waypoint_distances = tuple(
        float(np.interp(value, sample_parameters, distance))
        for value in anchor_params
    )
    positions = tuple(tuple(float(component) for component in row) for row in sampled)
    return CurveSamples(positions, tuple(distance), tuple(curvature), waypoint_distances)


def splice_straight_leg(
    route_points: Sequence[Tuple[float, float, float]],
    waypoint_anchor_indices: Sequence[int],
    curve: CurveSamples,
    leg_index: int,
    sample_spacing: float = 0.10,
) -> CurveSamples:
    """把已验证曲线的一段替换为已通过可见性检查的直线，并在折角处停车。

    该候选用于平滑切向约束会碰到障碍物时。替换范围之外沿用已认证曲线，
    新线段的端点作为航点，并在需要改变方向的位置加入停车弧长。
    """
    if leg_index < 0 or leg_index >= len(waypoint_anchor_indices):
        raise ValueError("直线候选的航点段编号无效")
    if sample_spacing <= 0.0:
        raise ValueError("曲线采样间距必须大于零")

    first_index = (0 if leg_index == 0
                   else waypoint_anchor_indices[leg_index - 1])
    last_index = waypoint_anchor_indices[leg_index]
    if last_index != first_index + 1:
        raise ValueError("指定的航点段不是无遮挡直线候选")

    start_distance = (0.0 if leg_index == 0
                      else curve.waypoint_distances[leg_index - 1])
    end_distance = curve.waypoint_distances[leg_index]
    start = tuple(float(value) for value in route_points[first_index])
    end = tuple(float(value) for value in route_points[last_index])
    line_length = math.dist(start, end)
    if line_length <= 1e-8:
        raise ValueError("直线候选长度接近零")

    prefix = [index for index, value in enumerate(curve.distances)
              if value <= start_distance + 1e-7]
    suffix = [index for index, value in enumerate(curve.distances)
              if value > end_distance + 1e-7]
    if not prefix or not suffix and end_distance < curve.distances[-1] - 1e-7:
        raise ValueError("无法在曲线上定位直线候选的端点")

    positions = [curve.positions[index] for index in prefix]
    curvatures = [curve.curvatures[index] for index in prefix]
    pieces = max(1, int(math.ceil(line_length / sample_spacing)))
    for piece in range(1, pieces + 1):
        fraction = piece / pieces
        positions.append(tuple(a + fraction * (b - a) for a, b in zip(start, end)))
        curvatures.append(0.0)
    for index in suffix:
        positions.append(curve.positions[index])
        curvatures.append(curve.curvatures[index])

    distances = [0.0]
    for first, second in zip(positions, positions[1:]):
        distances.append(distances[-1] + math.dist(first, second))
    new_start_distance = distances[len(prefix) - 1]
    new_end_distance = new_start_distance + line_length
    shift = line_length - (end_distance - start_distance)
    waypoint_distances = tuple(
        value if index < leg_index else value + shift
        for index, value in enumerate(curve.waypoint_distances)
    )

    stop_distances = []
    if leg_index > 0:
        stop_distances.append(new_start_distance)
    if leg_index + 1 < len(waypoint_anchor_indices):
        stop_distances.append(new_end_distance)

    return CurveSamples(
        positions=tuple(positions),
        distances=tuple(distances),
        curvatures=tuple(curvatures),
        waypoint_distances=waypoint_distances,
        stop_distances=tuple(stop_distances),
    )


def _spline_is_clear(spline, parameter, circles):
    """通过每个样条区间的 Bézier 凸包递归验证整段曲线。"""
    for interval in range(len(parameter) - 1):
        u0, u1 = float(parameter[interval]), float(parameter[interval + 1])
        width = u1 - u0
        # BPoly 系数本身就是 Bézier 控制点，可直接用凸包性质证明整段安全。
        control = np.asarray(spline.c[:, interval, :2], dtype=float)
        for circle in circles:
            if not _bezier_clear(control, (circle.x, circle.y), circle.radius, 0):
                return False
    return True


def _bezier_clear(control, center, radius, depth):
    """凸包不相交即通过，否则递归细分；无法证明安全就拒绝该曲线。"""
    if _hull_distance(control[:, :2], center) >= radius - 1e-7:
        return True
    if depth >= 14:
        return False
    left, right = _split_bezier(control)
    return (_bezier_clear(left, center, radius, depth + 1)
            and _bezier_clear(right, center, radius, depth + 1))


def _split_bezier(control):
    """在参数 0.5 处用 de Casteljau 算法精确二分任意阶 Bézier 曲线。"""
    level = [np.asarray(control, dtype=float)]
    while len(level[-1]) > 1:
        level.append((level[-1][:-1] + level[-1][1:]) * 0.5)
    left = np.asarray([row[0] for row in level])
    right = np.asarray([row[-1] for row in reversed(level)])
    return left, right


def _hull_distance(points, center):
    """计算圆心到控制点凸包的最短距离。"""
    hull = _convex_hull(points)
    if len(hull) == 1:
        return math.dist(hull[0], center)
    if len(hull) >= 3 and _inside_convex_polygon(center, hull):
        return 0.0
    edges = zip(hull, hull[1:] + hull[:1])
    return min(_point_segment_distance(center, a, b) for a, b in edges)


def _convex_hull(points):
    """用单调链得到最多四个 Bézier 控制点的二维凸包。"""
    unique = sorted(set((float(point[0]), float(point[1])) for point in points))
    if len(unique) <= 1:
        return unique

    def cross(origin, a, b):
        return ((a[0] - origin[0]) * (b[1] - origin[1])
                - (a[1] - origin[1]) * (b[0] - origin[0]))

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _inside_convex_polygon(point, polygon):
    signs = []
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        signs.append((b[0] - a[0]) * (point[1] - a[1])
                     - (b[1] - a[1]) * (point[0] - a[0]))
    return all(value >= -1e-10 for value in signs) or all(value <= 1e-10 for value in signs)


def _point_segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 <= 1e-12:
        return math.dist(point, a)
    t = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2))
    return math.hypot(point[0] - a[0] - t * dx,
                      point[1] - a[1] - t * dy)


def _remove_consecutive_duplicates(points):
    output = []
    for point in points:
        value = tuple(float(component) for component in point)
        if not output or math.dist(output[-1], value) > 1e-7:
            output.append(value)
    return output


def _cumulative_distance(points):
    result = [0.0]
    for first, second in zip(points, points[1:]):
        result.append(result[-1] + math.dist(first, second))
    return result


def _point_parameter(points, target):
    """取曲线锚点处的累计距离；路线构造时保证航点坐标是结点。"""
    cumulative = _cumulative_distance(points)
    closest = min(range(len(points)), key=lambda index: math.dist(points[index], target))
    if math.dist(points[closest], target) > 1e-5:
        raise ValueError("曲线路线中缺少精确航点坐标")
    return cumulative[closest]
