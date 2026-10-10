"""演示地图数据；航点顺序就是任务访问顺序。"""

from .models import CircleObstacle, Waypoint


def demo_waypoints(height=5.0):
    """绕场一圈并在最后一个航点悬停。"""
    z = -abs(height)
    return [
        Waypoint(10.0, -10.0, z),
        Waypoint(25.0, -10.0, z),
        Waypoint(25.0, 10.0, z),
        Waypoint(10.0, 10.0, z),
        Waypoint(0.0, 0.0, z, hold=3.0),
    ]


def demo_obstacles():
    """配套的手绘圆柱障碍，能展示绕障曲线与安全膨胀边界。"""
    return [
        CircleObstacle(17.5, -10.0, 4.0, "障碍 A"),
        CircleObstacle(25.0, 2.0, 3.5, "障碍 B"),
    ]
