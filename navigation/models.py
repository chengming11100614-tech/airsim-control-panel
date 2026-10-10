"""导航规划与自动跟踪共用的数据契约。

坐标均为 AirSim 的 NED 坐标，距离用米、速度用米每秒、时间用秒。
规划器输出不可变的离散轨迹，跟踪器只消费该轨迹和实时状态，不重新规划。
"""

from dataclasses import dataclass
from typing import Optional, Tuple


Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class CircleObstacle:
    """XY 平面内的竖直圆柱障碍；高度方向视为无限延伸。"""

    x: float
    y: float
    radius: float
    name: str = ""

    def __post_init__(self):
        if self.radius < 0.0:
            raise ValueError("障碍物半径不能为负数")


@dataclass(frozen=True)
class Waypoint:
    """必须按输入顺序经过的航点。

    每点可指定到点后悬停时间。竞速模式下速度由全局上限、曲率和飞行器能力决定，
    因此这里不包含每航点速度限制。
    """

    x: float
    y: float
    z: float
    hold: float = 0.0

    def __post_init__(self):
        if self.hold < 0.0:
            raise ValueError("航点悬停时间不能为负数")

    @property
    def position(self) -> Vector3:
        """返回 AirSim NED 下的三维位置。"""
        return (self.x, self.y, self.z)


@dataclass(frozen=True)
class CalibrationProfile:
    """沿用户选定的安全直线实测到的速度控制特性。"""

    calibrated: bool = False
    safe_max_speed: float = 2.0
    acceleration: float = 1.0
    braking: float = 1.0
    lateral_acceleration: float = 1.0
    response_time: float = 0.25
    command_delay: float = 0.10
    cross_track_p95: float = 0.0
    tested_speeds: Tuple[float, ...] = ()


@dataclass(frozen=True)
class KinematicLimits:
    """一次规划使用的速度、加速度及安全边界。"""

    max_speed: float = 15.0
    max_vertical_speed: float = 2.0
    max_acceleration: float = 1.0
    max_braking: float = 1.0
    max_lateral_acceleration: float = 1.0
    response_time: float = 0.25
    command_delay: float = 0.10
    safety_clearance: float = 0.8
    tracking_reserve: float = 0.25


@dataclass(frozen=True)
class TrajectorySample:
    """轨迹上的一个带时间戳采样，供绘图、评估和控制器共同使用。"""

    time: float
    distance: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    ax: float
    ay: float
    az: float
    curvature: float
    waypoint_index: Optional[int] = None

    @property
    def position(self) -> Vector3:
        """返回该采样位置。"""
        return (self.x, self.y, self.z)


@dataclass(frozen=True)
class PlannedTrajectory:
    """规划结果及其任务身份信息。

    `mission_signature` 用于确保自动巡航消费的是当前航点、障碍物和参数所对应的
    轨迹；地图或航点改变后必须重新规划。
    """

    samples: Tuple[TrajectorySample, ...]
    waypoint_distances: Tuple[float, ...]
    estimated_time: float
    length: float
    minimum_clearance: float
    mission_signature: str

    @property
    def duration(self) -> float:
        """轨迹的飞行时长（不含到点悬停）。"""
        return self.samples[-1].time if self.samples else 0.0
