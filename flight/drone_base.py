"""连接与基础飞行封装 —— 功能一「无人机普通飞行训练」公共层。

这一层把每个训练科目都要做的动作收拢到一起：

    connect()  -> 连上仿真器
    arm()      -> 解锁电机（不调用飞机不会动）
    takeoff()  -> 起飞到指定高度
    hover()    -> 定点悬停
    land()     -> 降落

坐标系提醒：AirSim 全部使用 NED 坐标系，单位是米。
    X 轴 -> 正北（机头前方）
    Y 轴 -> 正东（机身右方）
    Z 轴 -> 地心方向，**向下为正**
所以「离地 5 米」写作 z = -5.0，「向上飞」写作 z 变小（更负）。

运行前请先双击 Africa_001.exe 启动仿真器，否则连接会超时。
"""

import time

import airsim

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

DEFAULT_PORT = 41451          # AirSim 的 RPC 端口，被防火墙拦截时连不上
DEFAULT_HEIGHT = -5.0         # 默认训练高度：离地 5 米（NED 下为负）
ARM_TIMEOUT = 10.0            # 解锁后等待电机就绪的秒数
POSITION_EPS = 0.5            # 判定「到点」的容差，单位米


class DroneBase:
    """对 airsim.MultirotorClient 的薄封装，供各训练科目复用。"""

    def __init__(self, ip="", port=DEFAULT_PORT, vehicle_name=""):
        self.ip = ip
        self.port = port
        self.vehicle_name = vehicle_name
        self.client = airsim.MultirotorClient(ip=ip, port=port)
        self._armed = False
        self._flying = False

    # -- 连接 ---------------------------------------------------------------

    def connect(self):
        """确认仿真器在线，并打开 API 控制权。"""
        print("[DroneBase] 正在连接仿真器 ...")
        self.client.confirmConnection()          # 连不上会在这里阻塞并超时
        print("[DroneBase] 已连接")
        print("[DroneBase] 客户端版本: %d" % self.client.getClientVersion())
        print("[DroneBase] 服务端版本: %d" % self.client.getServerVersion())
        return self

    def arm(self):
        """打开 API 控制并解锁电机。起飞前必须调用，否则飞机不动。"""
        self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.client.armDisarm(True, vehicle_name=self.vehicle_name)
        self._armed = True
        time.sleep(1.0)                          # 给电机一点起转时间
        print("[DroneBase] 已解锁电机")
        return self

    # -- 起降与悬停 ---------------------------------------------------------

    def takeoff(self, height=DEFAULT_HEIGHT, timeout=20.0):
        """起飞并爬升到 height（NED，负值表示离地高度）。"""
        if not self._armed:
            self.arm()
        print("[DroneBase] 起飞，目标高度 %.1f 米" % abs(height))
        self.client.takeoffAsync(timeout_sec=timeout, vehicle_name=self.vehicle_name).join()
        # takeoffAsync 只负责离地，这里再稳稳地爬到训练高度
        self.client.moveToZAsync(height, 2.0, vehicle_name=self.vehicle_name).join()
        self._flying = True
        print("[DroneBase] 已到达训练高度")
        return self

    def hover(self, seconds=10.0):
        """在当前位置定点悬停指定秒数，用于考核位置保持精度。"""
        print("[DroneBase] 定点悬停 %.1f 秒" % seconds)
        self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
        time.sleep(seconds)
        return self

    def land(self):
        """降落到地面并锁桨。"""
        print("[DroneBase] 降落中 ...")
        self.client.landAsync(timeout_sec=60, vehicle_name=self.vehicle_name).join()
        self.client.armDisarm(False, vehicle_name=self.vehicle_name)
        self.client.enableApiControl(False, vehicle_name=self.vehicle_name)
        self._armed = False
        self._flying = False
        print("[DroneBase] 已落地并锁桨")
        return self

    # -- 状态读取 -----------------------------------------------------------

    def state(self):
        """返回当前位置、姿态角、速度与时间戳，供记录与验收使用。"""
        s = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
        k = s.kinematics_estimated
        roll, pitch, yaw = airsim.to_eularian_angles(k.orientation)
        return {
            "t": s.timestamp / 1e9,               # 纳秒 -> 秒
            "x": k.position.x_val,
            "y": k.position.y_val,
            "z": k.position.z_val,                # 注意：负值表示在地面以上
            "alt": -k.position.z_val,             # 换算成「离地高度」，读起来更直观
            "vx": k.linear_velocity.x_val,
            "vy": k.linear_velocity.y_val,
            "vz": k.linear_velocity.z_val,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }

    def position(self):
        """快捷方法：只取 NED 位置三元组。"""
        s = self.state()
        return s["x"], s["y"], s["z"]

    # -- 收尾 ---------------------------------------------------------------

    def close(self):
        """确保飞机落地并交还控制权；出异常时也会被调用。"""
        try:
            if self._flying:
                self.land()
            elif self._armed:
                self.client.armDisarm(False, vehicle_name=self.vehicle_name)
                self.client.enableApiControl(False, vehicle_name=self.vehicle_name)
        except Exception as exc:                  # 收尾失败不应掩盖原始异常
            print("[DroneBase] 收尾时出现问题: %s" % exc)

    def __enter__(self):
        return self.connect().arm()

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
