"""基础飞行动作库 —— 功能一「无人机普通飞行训练」动作层。

按任务书要求，这里提供三种移动方式与三组基础航线：

三种移动方式
    move_by_position()   按「位置」移动：给定 NED 坐标点飞过去
    move_by_velocity()   按「速度」移动：给定三个方向的速度，飞一段时间
    move_by_altitude()   按「高度」移动：只改高度，水平位置不动

三组基础航线
    fly_line()           直线航线：A 点 -> B 点
    fly_rectangle()      矩形航线：四个航点走一个闭环
    fly_circle()         绕圈：用速度指令连续画圆

另外提供 draw_path() 把飞过的轨迹直接画在仿真器里，方便现场演示。

所有坐标均为 NED（米），z 负值表示在地面以上。
"""

import math
import time

import airsim

# 主题紫色，用于在仿真器中绘制轨迹（RGBA，0-1）
TRAJ_COLOR = [0.325, 0.110, 0.518, 1.0]      # #531C84
WAYPOINT_COLOR = [0.573, 0.153, 0.561, 1.0]  # #92278F


# ---------------------------------------------------------------------------
# 三种移动方式
# ---------------------------------------------------------------------------

def move_by_position(drone, x, y, z, speed=2.0):
    """按位置移动：飞到 NED 点 (x, y, z)，speed 单位米每秒。"""
    print("[动作] 按位置移动 -> (%.1f, %.1f, %.1f) 速度 %.1f" % (x, y, z, speed))
    drone.client.moveToPositionAsync(
        x, y, z, speed,
        timeout_sec=60,
        vehicle_name=drone.vehicle_name,
    ).join()


def move_by_velocity(drone, vx, vy, vz, duration=3.0):
    """按速度移动：以 (vx, vy, vz) 飞 duration 秒后自动悬停。"""
    print("[动作] 按速度移动 -> v(%.1f, %.1f, %.1f) 持续 %.1f 秒" % (vx, vy, vz, duration))
    drone.client.moveByVelocityAsync(
        vx, vy, vz, duration,
        vehicle_name=drone.vehicle_name,
    ).join()


def move_by_altitude(drone, altitude, speed=1.5):
    """按高度移动：只改变离地高度（传正值，内部转成 NED 的负 z）。"""
    z = -abs(altitude)
    print("[动作] 按高度移动 -> 离地 %.1f 米" % abs(altitude))
    drone.client.moveToZAsync(z, speed, vehicle_name=drone.vehicle_name).join()


def hold_altitude(drone, altitude, seconds=5.0):
    """保持指定高度若干秒，用于定高科目的高度波动考核。"""
    move_by_altitude(drone, altitude)
    print("[动作] 保持高度 %.1f 米，持续 %.1f 秒" % (abs(altitude), seconds))
    time.sleep(seconds)


def rotate_to_yaw(drone, yaw_deg, timeout=20):
    """把机头转到指定偏航角（角度制），用于姿态保持练习。"""
    print("[动作] 转向偏航角 %.0f 度" % yaw_deg)
    drone.client.rotateToYawAsync(
        math.radians(yaw_deg), timeout_sec=timeout, vehicle_name=drone.vehicle_name,
    ).join()


# ---------------------------------------------------------------------------
# 三组基础航线
# ---------------------------------------------------------------------------

def fly_line(drone, start, end, speed=2.0):
    """直线航线：先从 start 起飞到位，再平飞到 end。

    start / end 均为 (x, y, z) 的 NED 三元组。
    """
    print("[航线] 直线：%s -> %s" % (start, end))
    move_by_position(drone, *start, speed=speed)
    move_by_position(drone, *end, speed=speed)


def fly_rectangle(drone, origin=(0.0, 0.0, -5.0), side=8.0, speed=2.0, repeats=1):
    """矩形航线：以 origin 为一个角点，边长 side，走闭合矩形。

    默认绕 X-Y 平面的正方形，高度保持不变。验收时主要看四个直角是否走准。
    """
    ox, oy, oz = origin
    corners = [
        (ox, oy, oz),
        (ox + side, oy, oz),
        (ox + side, oy + side, oz),
        (ox, oy + side, oz),
        (ox, oy, oz),           # 回到起点，闭合成环
    ]
    print("[航线] 矩形：边长 %.1f 米，共 %d 圈" % (side, repeats))
    for i in range(repeats):
        for idx, corner in enumerate(corners):
            print("[航线] 矩形第 %d 圈，第 %d/4 个航点" % (i + 1, idx if idx < 4 else 1))
            move_by_position(drone, *corner, speed=speed)


def fly_circle(drone, center=(0.0, 0.0), radius=4.0, altitude=5.0,
               speed=2.0, laps=1, segments=36):
    """绕圈航线：用速度指令沿圆周连续飞行。

    实现思路：把圆周等分成 segments 段，每一段用切线方向的速度飞一小段时间。
    用 moveByVelocityZAsync 让高度在这一段里保持恒定，画出来的圆更规整。
    """
    cx, cy = center
    z = -abs(altitude)
    omega = speed / radius                      # 角速度，弧度每秒
    dt = (2 * math.pi / segments) / omega       # 每一小段的时长

    print("[航线] 绕圈：半径 %.1f 米，高度 %.1f 米，共 %d 圈" % (radius, altitude, laps))
    # 先飞到圆周的起点上
    move_by_position(drone, cx + radius, cy, z, speed=speed)

    theta = 0.0
    total_steps = int(segments * laps)
    for step in range(total_steps):
        # 切线方向即速度方向；同时让机头朝向切线，飞行姿态更自然
        vx = -speed * math.sin(theta)
        vy = speed * math.cos(theta)
        yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=math.degrees(theta) + 90.0)
        drone.client.moveByVelocityZAsync(
            vx, vy, z, dt,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=yaw_mode,
            vehicle_name=drone.vehicle_name,
        ).join()
        theta += 2 * math.pi / segments

    print("[航线] 绕圈完成")


# ---------------------------------------------------------------------------
# 轨迹可视化
# ---------------------------------------------------------------------------

def draw_path(drone, path, persistent=True, thickness=6.0):
    """把一串 NED 点连成线画在仿真器里，现场演示非常直观。

    path 为 [(x, y, z), ...]；persistent=True 表示一直保留到下次清空。
    """
    if not path:
        print("[可视化] 轨迹为空，跳过绘制")
        return
    points = [airsim.Vector3r(x, y, z) for (x, y, z) in path]
    print("[可视化] 绘制轨迹，共 %d 个点" % len(points))
    drone.client.simPlotLineStrip(
        points,
        color_rgba=TRAJ_COLOR,
        thickness=thickness,
        duration=-1.0,          # -1 表示由 is_persistent 决定
        is_persistent=persistent,
    )


def mark_waypoints(drone, path, size=3.0):
    """在每个航点上画一个点，用来对照「计划航线」与「实际轨迹」的偏差。"""
    if not path:
        return
    points = [airsim.Vector3r(x, y, z) for (x, y, z) in path]
    drone.client.simPlotPoints(
        points,
        color_rgba=WAYPOINT_COLOR,
        size=size * 10,
        duration=-1.0,
        is_persistent=True,
    )


def clear_all_markers(drone):
    """清空之前绘制的所有标记。"""
    drone.client.simFlushPersistentMarkers()
    print("[可视化] 已清空轨迹标记")


def path_from_logger(logger):
    """把 TrackLogger 记录的采样点转成可绘制的 NED 点列。"""
    return [(s["x"], s["y"], s["z"]) for s in logger.samples]
