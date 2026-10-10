"""沿离散规划曲线计算遥测偏差、进度和摆动方向。"""

import math


def cumulative_lengths(points):
    """计算三维路径点的累计弧长；遥测显示沿线进度时复用。"""
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.sqrt(
            sum((second[axis] - first[axis]) ** 2 for axis in range(len(first)))
        ))
    return cumulative


def deviation_to_path(x, y, path, cumulative=None):
    """返回点到 XY 曲线的最近横向偏差、弧长进度和左右方向。"""
    if not path or len(path) < 2:
        return {"dist": 0.0, "seg": 0, "s": 0.0,
                "total": 0.0, "progress": 0.0, "side": ""}
    if cumulative is None or len(cumulative) != len(path):
        cumulative = cumulative_lengths(path)

    best_squared, best_segment, best_fraction = float("inf"), 0, 0.0
    for index, (first, second) in enumerate(zip(path, path[1:])):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            fraction = 0.0
        else:
            fraction = max(0.0, min(1.0,
                ((x - first[0]) * dx + (y - first[1]) * dy) / length_squared))
        px, py = first[0] + fraction * dx, first[1] + fraction * dy
        squared = (x - px) ** 2 + (y - py) ** 2
        if squared < best_squared:
            best_squared, best_segment, best_fraction = squared, index, fraction

    first, second = path[best_segment], path[best_segment + 1]
    cross = ((second[0] - first[0]) * (y - first[1])
             - (second[1] - first[1]) * (x - first[0]))
    side = "左" if cross > 0 else ("右" if cross < 0 else "")
    segment_length = cumulative[best_segment + 1] - cumulative[best_segment]
    distance = cumulative[best_segment] + best_fraction * segment_length
    total = cumulative[-1] if cumulative else 0.0
    return {
        "dist": math.sqrt(best_squared),
        "seg": best_segment,
        "s": distance,
        "total": total,
        "progress": distance / total if total > 1e-9 else 0.0,
        "side": side,
    }
