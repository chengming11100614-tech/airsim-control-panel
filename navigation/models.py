"""导航规划与自动跟踪共用的数据契约。

坐标均为 AirSim 的 NED 坐标，距离用米、速度用米每秒、时间用秒。
规划器输出不可变的离散轨迹，跟踪器只消费该轨迹和实时状态，不重新规划。
"""

from dataclasses import dataclass
import math
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
        if not all(math.isfinite(float(value)) for value in (self.x, self.y, self.radius)):
            raise ValueError("障碍物位置和半径必须是有限数值")
        if self.radius < 0.0:
            raise ValueError("障碍物半径不能为负数")

    @property
    def r(self) -> float:
        """障碍物地图上使用的半径简称。"""
        return self.radius

    def to_dict(self):
        """保存成兼容旧航线文件的圆柱字段。"""
        return {"x": self.x, "y": self.y, "r": self.radius, "name": self.name}

    @classmethod
    def from_dict(cls, data):
        """读取旧文件中的 r 字段或新文件中的 radius 字段。"""
        return cls(data["x"], data["y"], data.get("radius", data.get("r")),
                   data.get("name", ""))


@dataclass
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
        if not all(math.isfinite(float(value)) for value in (self.x, self.y, self.z, self.hold)):
            raise ValueError("航点位置和悬停时间必须是有限数值")
        if self.hold < 0.0:
            raise ValueError("航点悬停时间不能为负数")

    @property
    def position(self) -> Vector3:
        """返回 AirSim NED 下的三维位置。"""
        return (self.x, self.y, self.z)

    @property
    def altitude(self) -> float:
        """返回界面使用的正向高度值。"""
        return -self.z

    def to_dict(self):
        """保存位置和悬停时长；不会再写入无效的每点速度。"""
        return {"x": self.x, "y": self.y, "z": self.z, "hold": self.hold}

    @classmethod
    def from_dict(cls, data):
        """读取航点；旧文件中的 speed 字段被忽略。"""
        return cls(data["x"], data["y"], data["z"], data.get("hold", 0.0))


@dataclass(frozen=True)
class CalibrationProfile:
    """沿用户选定的安全直线实测到的速度控制特性。"""

    calibrated: bool = False
    safe_max_speed: float = 2.0
    # 本轮 AirSim 实飞的有效加速度/制动中位数约为 0.48/0.54 m/s²；
    # 未完成独立标定前用略偏保守的模型安排提速和弯前制动。
    acceleration: float = 0.65
    braking: float = 0.55
    lateral_acceleration: float = 1.0
    response_time: float = 0.25
    command_delay: float = 0.05
    cross_track_p95: float = 0.0
    tested_speeds: Tuple[float, ...] = ()
    # 速度指令到实际速度的响应模型。它与规划速度曲线的加速度上限不同。
    velocity_time_constant: float = 1.3
    velocity_acceleration: float = 1.3
    # 实飞日志中 AirSim 遥测与控制循环的中位周期约为 62.5 ms。
    control_period: float = 0.0625


@dataclass(frozen=True)
class KinematicLimits:
    """一次规划使用的速度、加速度及安全边界。"""

    max_speed: float = 15.0
    max_vertical_speed: float = 2.0
    max_acceleration: float = 0.65
    max_braking: float = 0.55
    max_lateral_acceleration: float = 1.0
    response_time: float = 0.25
    command_delay: float = 0.05
    safety_clearance: float = 0.8
    tracking_reserve: float = 0.25
    # AirSim 实测速度响应：一阶时间常数和速度矢量最大变化率。
    velocity_time_constant: float = 1.3
    velocity_acceleration: float = 1.3
    control_period: float = 0.0625


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
class ExecutionPrediction:
    """用巡航控制器和 AirSim 响应模型闭环仿真的实际飞行轨迹。"""

    samples: Tuple[TrajectorySample, ...]
    estimated_time: float
    minimum_clearance: float
    waypoint_errors: Tuple[float, ...]
    complete: bool
    safe: bool


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
    execution_prediction: Optional[ExecutionPrediction] = None
    nominal_time: Optional[float] = None

    @property
    def duration(self) -> float:
        """返回模拟飞行时长；尚未模拟时使用理想速度曲线时长。"""
        if self.execution_prediction is not None:
            return self.execution_prediction.estimated_time
        return self.samples[-1].time if self.samples else 0.0
