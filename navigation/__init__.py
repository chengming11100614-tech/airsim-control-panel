"""导航算法包。

几何规划、轨迹定时和飞行跟踪分别放在独立模块中；本包只公开稳定的数据类型。
"""

from .models import (
    CalibrationProfile,
    CircleObstacle,
    KinematicLimits,
    PlannedTrajectory,
    TrajectorySample,
    Waypoint,
)

__all__ = [
    "CalibrationProfile",
    "CircleObstacle",
    "KinematicLimits",
    "PlannedTrajectory",
    "TrajectorySample",
    "Waypoint",
]
