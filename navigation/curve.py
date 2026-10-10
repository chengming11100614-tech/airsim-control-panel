"""平滑曲线拟合与障碍物安全校验。

切线图给出路线的绕障拓扑。这里用三次样条在图路线周围拟合 C2 连续曲线，
并把每段样条转换为 Bézier 控制多边形做递归安全检查。曲线控制点的凸包与
安全圆不相交时，Bézier 凸包性质可证明整段曲线都在安全圆之外。
"""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

import numpy as np
from scipy.interpolate import CubicSpline

from .geometry import InflatedCircle


@dataclass(frozen=True)
class CurveSamples:
    """平滑曲线的弧长采样和原始参数位置。"""

    positions: Tuple[Tuple[float, float, float], ...]
    distances: Tuple[float, ...]
    curvatures: Tuple[float, ...]
    waypoint_distances: Tuple[float, ...]


def fit_safe_curve(
    route_points: Sequence[Tuple[float, float, float]],
    waypoint_anchor_indices: Sequence[int],
    protected_circles: Sequence[InflatedCircle],
    sample_spacing: float = 0.10,
) -> CurveSamples:
    """拟合经过所有航点的 C2 曲线；无法证明避障时返回 None。"""
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
    spline = CubicSpline(parameter, values, axis=0, bc_type="natural")

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


def _spline_is_clear(spline, parameter, circles):
    """通过每个样条区间的 Bézier 凸包递归验证整段曲线。"""
    for interval in range(len(parameter) - 1):
        u0, u1 = float(parameter[interval]), float(parameter[interval + 1])
        width = u1 - u0
        # scipy CubicSpline 的多项式按区间局部变量展开，转换为等价 Bézier 控制点。
        coeff = spline.c[:, interval, :2]
        p0 = coeff[3]
        p1 = coeff[3] + coeff[2] * width / 3.0
        p2 = coeff[3] + 2.0 * coeff[2] * width / 3.0 + coeff[1] * width * width / 3.0
        p3 = coeff[3] + coeff[2] * width + coeff[1] * width * width + coeff[0] * width ** 3
        control = np.asarray((p0, p1, p2, p3), dtype=float)
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
    """在参数 0.5 处用 de Casteljau 算法精确二分 Bézier 曲线。"""
    p01 = (control[0] + control[1]) * 0.5
    p12 = (control[1] + control[2]) * 0.5
    p23 = (control[2] + control[3]) * 0.5
    p012 = (p01 + p12) * 0.5
    p123 = (p12 + p23) * 0.5
    middle = (p012 + p123) * 0.5
    return (np.asarray((control[0], p01, p012, middle)),
            np.asarray((middle, p123, p23, control[3])))


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
