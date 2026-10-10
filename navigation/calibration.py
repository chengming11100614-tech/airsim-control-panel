"""从安全直线的分级飞行记录中估计巡航参数。"""

from dataclasses import dataclass
import math
from statistics import median
from typing import Sequence, Tuple

from .models import CalibrationProfile


SPEED_STEPS = (2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 15.0)


@dataclass(frozen=True)
class CalibrationSample:
    """一拍标定遥测；速度和误差均沿选定直线计算。"""

    time: float
    requested_speed: float
    along_speed: float
    cross_track: float
    phase: str = "speed"


@dataclass(frozen=True)
class SpeedTrial:
    """单个速度台阶的实测精度与响应数据。"""

    speed: float
    average_speed: float
    cross_track_p95: float
    max_cross_track: float
    response_time: float


@dataclass(frozen=True)
class CalibrationResult:
    """标定结果和各速度台阶的可读报告。"""

    profile: CalibrationProfile
    trials: Tuple[SpeedTrial, ...]


def analyze_calibration(
    samples: Sequence[CalibrationSample],
    lateral_acceleration: float = 1.0,
    clearance_target: float = 0.25,
) -> CalibrationResult:
    """只接受速度、偏差都达标的最高台阶，精度不达标时不靠提速掩盖。"""
    groups = {}
    for sample in samples:
        if sample.phase == "speed" and sample.requested_speed > 0.0:
            groups.setdefault(sample.requested_speed, []).append(sample)
    trials = []
    for speed in SPEED_STEPS:
        group = groups.get(speed, [])
        if not group:
            continue
        ordered = sorted(group, key=lambda row: row.time)
        steady = ordered[-min(4, len(ordered)):]
        errors = sorted(abs(row.cross_track) for row in ordered)
        p95_index = max(0, math.ceil(0.95 * len(errors)) - 1)
        t0 = ordered[0].time
        response = next(
            (row.time - t0 for row in ordered if row.along_speed >= 0.9 * speed),
            float("inf"),
        )
        trials.append(SpeedTrial(
            speed=speed,
            average_speed=sum(row.along_speed for row in steady) / len(steady),
            cross_track_p95=errors[p95_index],
            max_cross_track=max(errors),
            response_time=response,
        ))

    qualified = [trial for trial in trials
                 if trial.cross_track_p95 <= clearance_target
                 and trial.max_cross_track <= max(0.5, clearance_target * 2.0)
                 and trial.average_speed >= max(0.8 * trial.speed, trial.speed - 0.5)]
    safe_max_speed = max((trial.speed for trial in qualified), default=2.0)
    calibrated = bool(qualified)

    # 直线试验不激发转弯载荷；横向加速度继续用保守值，不伪装成实测。
    speed_samples = sorted(
        (sample for sample in samples if sample.phase == "speed"),
        key=lambda row: row.time,
    )
    accelerations = []
    for first, second in zip(speed_samples, speed_samples[1:]):
        dt = second.time - first.time
        if dt > 1e-4 and second.along_speed > first.along_speed:
            accelerations.append((second.along_speed - first.along_speed) / dt)
    acceleration = min(5.0, max(0.5, median(accelerations))) if accelerations else 1.0

    braking_samples = sorted(
        (sample for sample in samples if sample.phase == "braking"),
        key=lambda row: row.time,
    )
    braking_values = []
    for first, second in zip(braking_samples, braking_samples[1:]):
        dt = second.time - first.time
        if dt > 1e-4 and second.along_speed < first.along_speed:
            braking_values.append((first.along_speed - second.along_speed) / dt)
    braking = min(5.0, max(0.5, median(braking_values))) if braking_values else 1.0

    responses = [trial.response_time for trial in qualified if math.isfinite(trial.response_time)]
    command_delay = 0.10  # 20 Hz 指令周期折算为最多两个采样周期。
    response_time = max(0.05, median(responses) - command_delay) if responses else 0.25
    worst_error = max((trial.cross_track_p95 for trial in qualified), default=0.0)
    profile = CalibrationProfile(
        calibrated=calibrated,
        safe_max_speed=min(15.0, safe_max_speed),
        acceleration=acceleration,
        braking=braking,
        lateral_acceleration=max(0.5, lateral_acceleration),
        response_time=max(0.05, response_time),
        command_delay=command_delay,
        cross_track_p95=worst_error,
        tested_speeds=tuple(trial.speed for trial in trials),
    )
    return CalibrationResult(profile, tuple(trials))
