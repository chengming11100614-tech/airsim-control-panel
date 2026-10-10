"""圆柱障碍物的连续几何搜索。

在 XY 平面将每个圆形障碍膨胀成安全圆，再用圆的切点、圆间公切线和可见圆弧
构造图。与方格 A* 不同，图上的直线段和圆弧直接表示连续空间中的绕障路线。
"""

from dataclasses import dataclass
import heapq
import math
from typing import Iterable, List, Optional, Sequence, Tuple

from .models import CircleObstacle


Point2 = Tuple[float, float]
_EPS = 1e-8


@dataclass(frozen=True)
class InflatedCircle:
    """用于几何搜索的膨胀圆。"""

    x: float
    y: float
    radius: float
    source_index: int


@dataclass(frozen=True)
class PathEdge:
    """可见图的一条有向边，保存原语以便按真实圆弧展开。"""

    edge_id: int
    start: int
    end: int
    length: float
    kind: str
    circle_index: int = -1
    angle_start: float = 0.0
    angle_delta: float = 0.0


@dataclass(frozen=True)
class GeometricPath:
    """一条连续空间候选路线的离散绘图点与拓扑签名。"""

    points: Tuple[Point2, ...]
    length: float
    signature: Tuple[int, ...]


@dataclass
class _Vertex:
    point: Point2
    circles: set


def inflate_obstacles(
    obstacles: Sequence[CircleObstacle],
    clearance: float,
    tracking_reserve: float,
    curve_margin: float,
) -> Tuple[InflatedCircle, ...]:
    """按飞机中心安全距离、跟踪误差余量与曲线拟合余量扩张圆障碍。"""
    extra = clearance + tracking_reserve + curve_margin
    return tuple(
        InflatedCircle(obstacle.x, obstacle.y, obstacle.radius + extra, index)
        for index, obstacle in enumerate(obstacles)
    )


def segment_clear(a: Point2, b: Point2, circles: Sequence[InflatedCircle]) -> bool:
    """判断线段是否完整位于所有膨胀圆之外；边界接触视为可行。"""
    for circle in circles:
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        if length2 <= _EPS:
            distance = math.hypot(a[0] - circle.x, a[1] - circle.y)
        else:
            t = ((circle.x - a[0]) * dx + (circle.y - a[1]) * dy) / length2
            t = max(0.0, min(1.0, t))
            px, py = a[0] + t * dx, a[1] + t * dy
            distance = math.hypot(px - circle.x, py - circle.y)
        if distance < circle.radius - 1e-7:
            return False
    return True


def plan_leg_candidates(
    start: Point2,
    goal: Point2,
    circles: Sequence[InflatedCircle],
    max_candidates: int = 12,
) -> Tuple[GeometricPath, ...]:
    """为一段有序航点连线生成不同绕行侧的无碰撞候选路线。"""
    if math.dist(start, goal) <= 1e-8:
        return (GeometricPath((start, goal), 0.0, ()),)
    if not _point_clear(start, circles) or not _point_clear(goal, circles):
        return ()
    if segment_clear(start, goal, circles):
        return (GeometricPath((start, goal), math.dist(start, goal), ()),)

    vertices, circle_vertices = _make_vertices(start, goal, circles)
    edges, adjacency = _make_edges(vertices, circle_vertices, circles)
    if not adjacency:
        return ()

    paths = _k_shortest_edge_paths(
        start=0,
        goal=1,
        edges=edges,
        adjacency=adjacency,
        max_paths=max_candidates * 4,
    )
    candidates = []
    signatures = set()
    for edge_path in paths:
        points = _expand_edges(start, vertices, edge_path, circles)
        if len(points) < 2:
            continue
        signature = _winding_signature(start, goal, points, circles)
        if signature in signatures:
            continue
        signatures.add(signature)
        candidates.append(GeometricPath(
            points=tuple(points),
            length=sum(edge.length for edge in edge_path),
            signature=signature,
        ))
        if len(candidates) >= max_candidates:
            break
    return tuple(candidates)


def _make_vertices(start, goal, circles):
    """建立所有外露切点和相交圆边界点，并补齐圆弧采样节点。"""
    vertices: List[_Vertex] = [_Vertex(start, set()), _Vertex(goal, set())]
    circle_vertices: List[set] = [set() for _ in circles]

    def add(point, circle_ids=()):
        # 几何构造可能从多条公切线得到同一个切点，合并可避免零长边。
        for index, vertex in enumerate(vertices):
            if math.dist(vertex.point, point) <= 1e-7:
                vertex.circles.update(circle_ids)
                for circle_id in circle_ids:
                    circle_vertices[circle_id].add(index)
                return index
        index = len(vertices)
        vertices.append(_Vertex(point, set(circle_ids)))
        for circle_id in circle_ids:
            circle_vertices[circle_id].add(index)
        return index

    for point in (start, goal):
        for circle_id, circle in enumerate(circles):
            delta = (point[0] - circle.x, point[1] - circle.y)
            distance = math.hypot(*delta)
            if distance <= circle.radius + 1e-7:
                continue
            center_angle = math.atan2(delta[1], delta[0])
            offset = math.acos(min(1.0, circle.radius / distance))
            for angle in (center_angle - offset, center_angle + offset):
                add(_on_circle(circle, angle), (circle_id,))

    for first in range(len(circles)):
        for second in range(first + 1, len(circles)):
            _add_common_tangents(first, second, circles, add)
            _add_intersections(first, second, circles, add)

    # 仅保留安全圆并集边界上可见的点，隐藏在其他圆内部的切点不能用于绕障。
    for circle_id, indices in enumerate(circle_vertices):
        circle_vertices[circle_id] = {
            index for index in indices
            if all(
                math.dist(vertices[index].point, other_center) >= other.radius - 1e-7
                for other_id, other_center, other in (
                    (i, (c.x, c.y), c) for i, c in enumerate(circles)
                    if i != circle_id
                )
            )
        }
    vertices = [vertices[0], vertices[1]] + [
        vertex for index, vertex in enumerate(vertices[2:], start=2)
        if all(_point_on_or_outside(vertex.point, circle) for circle in circles)
    ]
    # 上面的过滤重排了索引；重新生成按圆归属的节点表。
    circle_vertices = [set() for _ in circles]
    for index, vertex in enumerate(vertices):
        for circle_id in vertex.circles:
            if _point_on_or_outside(vertex.point, circles[circle_id]) and all(
                math.dist(vertex.point, (other.x, other.y)) >= other.radius - 1e-7
                for other_id, other in enumerate(circles)
                if other_id != circle_id
            ):
                circle_vertices[circle_id].add(index)

    # 分割圆弧，既让搜索显式保留绕行侧，也避免用一条含糊的长弧边表达两种路线。
    for circle_id, circle in enumerate(circles):
        original = list(circle_vertices[circle_id])
        if len(original) < 2:
            continue
        ordered = sorted(original, key=lambda i: math.atan2(
            vertices[i].point[1] - circle.y,
            vertices[i].point[0] - circle.x,
        ))
        angles = [math.atan2(vertices[i].point[1] - circle.y,
                             vertices[i].point[0] - circle.x) % (2.0 * math.pi)
                  for i in ordered]
        for index, angle0 in enumerate(angles):
            angle1 = angles[(index + 1) % len(angles)]
            delta = (angle1 - angle0) % (2.0 * math.pi)
            pieces = max(1, int(math.ceil(delta / math.radians(30.0))))
            for piece in range(1, pieces):
                angle = angle0 + delta * piece / pieces
                point = _on_circle(circle, angle)
                if all(
                    math.dist(point, (other.x, other.y)) >= other.radius - 1e-7
                    for other_id, other in enumerate(circles)
                    if other_id != circle_id
                ):
                    # 这里不会与已存在的点重合：原有端点只在 piece=0 或 pieces。
                    vertex_id = len(vertices)
                    vertices.append(_Vertex(point, {circle_id}))
                    circle_vertices[circle_id].add(vertex_id)

    return vertices, circle_vertices


def _make_edges(vertices, circle_vertices, circles):
    """连接安全直视线段与圆并集边界上可见的相邻圆弧。"""
    edges: List[PathEdge] = []
    adjacency: List[List[int]] = [[] for _ in vertices]

    def add(start, end, length, kind, circle_index=-1,
            angle_start=0.0, angle_delta=0.0):
        edge_id = len(edges)
        edges.append(PathEdge(edge_id, start, end, length, kind,
                              circle_index, angle_start, angle_delta))
        adjacency[start].append(edge_id)

    # 对少量手绘障碍，完整可见图足够简单；精确线段检测比栅格分辨率更可靠。
    for first in range(len(vertices)):
        for second in range(first + 1, len(vertices)):
            a, b = vertices[first].point, vertices[second].point
            length = math.dist(a, b)
            if length > 1e-8 and segment_clear(a, b, circles):
                add(first, second, length, "line")
                add(second, first, length, "line")

    for circle_id, indices in enumerate(circle_vertices):
        circle = circles[circle_id]
        if len(indices) < 2:
            continue
        ordered = sorted(indices, key=lambda i: math.atan2(
            vertices[i].point[1] - circle.y,
            vertices[i].point[0] - circle.x,
        ) % (2.0 * math.pi))
        for index, first in enumerate(ordered):
            second = ordered[(index + 1) % len(ordered)]
            theta0 = math.atan2(vertices[first].point[1] - circle.y,
                                vertices[first].point[0] - circle.x) % (2.0 * math.pi)
            theta1 = math.atan2(vertices[second].point[1] - circle.y,
                                vertices[second].point[0] - circle.x) % (2.0 * math.pi)
            delta = (theta1 - theta0) % (2.0 * math.pi)
            if delta <= 1e-8 or not _arc_clear(circle_id, theta0, delta, circles):
                continue
            length = circle.radius * delta
            add(first, second, length, "arc", circle_id, theta0, delta)
            add(second, first, length, "arc", circle_id, theta1, -delta)

    return edges, adjacency


def _arc_clear(circle_index, angle_start, angle_delta, circles):
    """精确检查圆弧到其他圆心的最小角距离，避免采样漏掉细小碰撞。"""
    circle = circles[circle_index]
    low, high = sorted((angle_start, angle_start + angle_delta))
    for other_index, other in enumerate(circles):
        if other_index == circle_index:
            continue
        dx, dy = other.x - circle.x, other.y - circle.y
        center_angle = math.atan2(dy, dx)
        candidates = [angle_start, angle_start + angle_delta]
        for shift in range(-2, 3):
            angle = center_angle + shift * 2.0 * math.pi
            if low - 1e-9 <= angle <= high + 1e-9:
                candidates.append(angle)
        minimum = min(
            math.hypot(circle.x + circle.radius * math.cos(angle) - other.x,
                       circle.y + circle.radius * math.sin(angle) - other.y)
            for angle in candidates
        )
        if minimum < other.radius - 1e-7:
            return False
    return True


def _k_shortest_edge_paths(start, goal, edges, adjacency, max_paths):
    """Yen 算法：枚举有限条无环最短路线，供后续按预计飞行时间比较。"""
    first = _shortest_path(start, goal, edges, adjacency, set(), set())
    if first is None:
        return ()
    accepted = [first]
    accepted_keys = {tuple(edge.edge_id for edge in first)}
    candidates = []
    candidate_keys = set()

    while len(accepted) < max_paths:
        previous = accepted[-1]
        previous_nodes = [start] + [edge.end for edge in previous]
        for spur_index in range(len(previous)):
            root_edges = previous[:spur_index]
            root_nodes = previous_nodes[:spur_index + 1]
            banned_nodes = set(root_nodes[:-1])
            banned_edges = {
                path[spur_index].edge_id
                for path in accepted
                if len(path) > spur_index
                and tuple(edge.edge_id for edge in path[:spur_index])
                == tuple(edge.edge_id for edge in root_edges)
            }
            spur = _shortest_path(root_nodes[-1], goal, edges, adjacency,
                                  banned_nodes, banned_edges)
            if spur is None:
                continue
            combined = root_edges + spur
            key = tuple(edge.edge_id for edge in combined)
            if key in accepted_keys or key in candidate_keys:
                continue
            cost = sum(edge.length for edge in combined)
            heapq.heappush(candidates, (cost, key, combined))
            candidate_keys.add(key)
        if not candidates:
            break
        _cost, key, next_path = heapq.heappop(candidates)
        candidate_keys.discard(key)
        accepted.append(next_path)
        accepted_keys.add(key)
    return tuple(accepted)


def _shortest_path(start, goal, edges, adjacency, banned_nodes, banned_edges):
    """Dijkstra 搜索；障碍物边界圆弧与直线边拥有相同的长度单位（米）。"""
    costs = {start: 0.0}
    parent = {}
    queue = [(0.0, start)]
    visited = set()
    while queue:
        cost, vertex = heapq.heappop(queue)
        if vertex in visited:
            continue
        visited.add(vertex)
        if vertex == goal:
            route = []
            while vertex != start:
                edge = parent[vertex]
                route.append(edge)
                vertex = edge.start
            return list(reversed(route))
        for edge_id in adjacency[vertex]:
            if edge_id in banned_edges:
                continue
            edge = edges[edge_id]
            if edge.end in banned_nodes or edge.end in visited:
                continue
            next_cost = cost + edge.length
            if next_cost < costs.get(edge.end, float("inf")):
                costs[edge.end] = next_cost
                parent[edge.end] = edge
                heapq.heappush(queue, (next_cost, edge.end))
    return None


def _expand_edges(start, vertices, edge_path, circles):
    """将直线和圆弧边展开成供曲线拟合使用的点列。"""
    points = [start]
    for edge in edge_path:
        if edge.kind == "line":
            point = vertices[edge.end].point
            if math.dist(points[-1], point) > 1e-8:
                points.append(point)
            continue
        circle = circles[edge.circle_index]
        count = max(2, int(math.ceil(abs(edge.angle_delta) * circle.radius / 0.35)))
        for index in range(1, count + 1):
            angle = edge.angle_start + edge.angle_delta * index / count
            point = _on_circle(circle, angle)
            if math.dist(points[-1], point) > 1e-8:
                points.append(point)
    return points


def _winding_signature(start, goal, points, circles):
    """用各圆心的绕行数区分同一段的左右绕行拓扑。"""
    signature = []
    for circle in circles:
        angles = [math.atan2(point[1] - circle.y, point[0] - circle.x)
                  for point in points]
        change = sum(
            math.atan2(
                math.sin(second - first),
                math.cos(second - first),
            )
            for first, second in zip(angles, angles[1:])
        )
        direct = math.atan2(
            math.sin(math.atan2(goal[1] - circle.y, goal[0] - circle.x)
                     - math.atan2(start[1] - circle.y, start[0] - circle.x)),
            math.cos(math.atan2(goal[1] - circle.y, goal[0] - circle.x)
                     - math.atan2(start[1] - circle.y, start[0] - circle.x)),
        )
        signature.append(int(round((change - direct) / (2.0 * math.pi))))
    return tuple(signature)


def _add_common_tangents(first, second, circles, add_vertex):
    """生成两个不相交圆之间的内、外公切线接触点。"""
    a, b = circles[first], circles[second]
    dx, dy = b.x - a.x, b.y - a.y
    distance = math.hypot(dx, dy)
    if distance <= 1e-8:
        return
    ux, uy = dx / distance, dy / distance
    px, py = -uy, ux
    for sign_a in (-1, 1):
        for sign_b in (-1, 1):
            ratio = (sign_a * a.radius - sign_b * b.radius) / distance
            if abs(ratio) > 1.0 + 1e-10:
                continue
            ratio = max(-1.0, min(1.0, ratio))
            side = math.sqrt(max(0.0, 1.0 - ratio * ratio))
            for side_sign in (-1, 1):
                nx = ratio * ux + side_sign * side * px
                ny = ratio * uy + side_sign * side * py
                pa = (a.x + sign_a * a.radius * nx,
                      a.y + sign_a * a.radius * ny)
                pb = (b.x + sign_b * b.radius * nx,
                      b.y + sign_b * b.radius * ny)
                add_vertex(pa, (first,))
                add_vertex(pb, (second,))


def _add_intersections(first, second, circles, add_vertex):
    """为重叠圆的并集边界增加交点，使可见圆弧能在两圆间切换。"""
    a, b = circles[first], circles[second]
    dx, dy = b.x - a.x, b.y - a.y
    distance = math.hypot(dx, dy)
    if distance <= 1e-8 or distance > a.radius + b.radius + 1e-8:
        return
    if distance < abs(a.radius - b.radius) - 1e-8:
        return
    along = (a.radius * a.radius - b.radius * b.radius + distance * distance) / (2.0 * distance)
    height2 = a.radius * a.radius - along * along
    if height2 < -1e-8:
        return
    height = math.sqrt(max(0.0, height2))
    ux, uy = dx / distance, dy / distance
    midpoint = (a.x + along * ux, a.y + along * uy)
    for side in (-1, 1):
        point = (midpoint[0] - side * height * uy,
                 midpoint[1] + side * height * ux)
        add_vertex(point, (first, second))


def _point_clear(point, circles):
    """判断点是否在所有膨胀圆之外。"""
    return all(math.hypot(point[0] - circle.x, point[1] - circle.y)
               >= circle.radius - 1e-7 for circle in circles)


def _point_on_or_outside(point, circle):
    return math.hypot(point[0] - circle.x, point[1] - circle.y) >= circle.radius - 1e-7


def _on_circle(circle, angle):
    return (circle.x + circle.radius * math.cos(angle),
            circle.y + circle.radius * math.sin(angle))
