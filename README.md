# AirSim Mission Control Panel

Python 3.10 control panel for the Africa AirSim scene. It includes route planning, ordered waypoint flight, manual controls, telemetry logging, and flight training tools.

The route planner and automatic cruise controller support safe, time-aware racing over a fixed waypoint order. Obstacles are manually drawn circular cylinders in the XY plane; waypoint altitude uses AirSim NED coordinates.

## Run

```powershell
python -m pip install -r requirements.txt
python -m pip install --no-build-isolation --no-deps airsim==1.8.1
python main.py
```

AirSim 1.8.1 uses a legacy `setup.py` that imports NumPy and `msgpackrpc` while
pip is preparing package metadata. Install the requirements first, then install
AirSim separately with `--no-build-isolation --no-deps`; this also prevents its
old OpenCV dependency from replacing the installed NumPy version. The panel can
still open for offline route planning when AirSim is unavailable.

Start the AirSim scene first, then connect from the panel. The simulator runs on the configured AirSim RPC endpoint (default `127.0.0.1:41451`).

## Structure

- `main.py`: Tkinter application entry and panel wiring.
- `navigation/`: inflated-circle route search, smooth curve certification,
  time parameterization, speed calibration, and carrot path following.
- `flight/`: standalone flight training exercises and CSV recording.

## 导航规划与巡航

地图使用 AirSim NED 坐标，距离为米，速度为米每秒，障碍物是 XY 平面圆形截面的竖直圆柱。
输入航点按给定顺序全部经过；每点可设置到点悬停时间，不能从航点位置穿过障碍物。

`navigation/geometry.py` 将障碍半径膨胀为“障碍半径 + 安全间距 + 跟踪余量”，并用切线、圆弧和可见图搜索有限条不同绕行路线；这种多拓扑候选思路参考了 [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)。`navigation/curve.py` 为候选路线拟合通过航点的连续三次样条，并用 Bézier 凸包递归检查曲线全程安全。规划线保持普通安全曲线，不另造一条内切赛道。`navigation/time_parameterization.py` 根据速度、加速度、制动、曲率、爬升和响应延迟生成逐点速度曲线：曲率上限给出弯中速度，反向可达性传播安排弯前制动，正向传播安排出弯加速；方法采用了 [TOPP-RA](https://github.com/hungpham2511/toppra) 一类的路径速度可达性思路，并按完整预计时间 T 比较候选。只在安全且动力学可行的有限候选中选最快路线，不宣称全局数学最优；找不到可行曲线时会报错，不会退回直线。

`navigation/controller.py` 每拍将目标放在已规划曲线前方，再沿弦线给出速度方向，采用了 [AirSim 自带前视跟踪](https://github.com/microsoft/AirSim/blob/main/docs/apis.md)的 carrot-following 思路。飞机过弯时可自然向弯内收，不会被横向误差项拉回曲线中心；速度大小仍读取上面的逐点计划。控制器按实测弧长推进，偏航或漏过航点只记告警并保持正向飞行，不倒飞追点；只有逼近手绘障碍物实体边缘才触发硬悬停。它保留了“沿路径前进，同时约束横向偏差”的思想，但不再用导致抖动的 OSQP 短时 MPC；偏差和沿线进度以 CSV 记录，可用于后续按实际飞行数据调参。这与 [MPCC 的路径进度和轮廓误差目标](https://arxiv.org/abs/2108.13205)相呼应。

面板的速度字段是最高允许速度，上限 15 m/s。速度按路径弧长动态计算：前向加速传播和反向制动传播共同决定当前能达到的速度；短直线来不及到上限时会形成加速后制动的速度曲线，长直线才有机会接近上限。未完成独立标定时不再把速度锁死在 2 m/s；本轮 AirSim 实飞测得的有效加速/制动中位数约为 0.48/0.54 m/s²，因此暂用 0.65/0.55 m/s²作为规划约束，并保留保守横向加速度。完成直线标定后，直线速度仍受实测合格速度约束；横向加速度不会从直线测试中伪装成实测。飞行 CSV 同时记录计划速度和参考曲率，便于检查弯前何时减速、出弯何时加速。面板消息写入 `logs/panel_日期.log`，20 Hz 飞行遥测仍写入对应 CSV。规划时间 T 是有限候选路线上的估计值，实飞速度和飞行时间仍需用日志校准。

控制面板会在飞行 CSV 中记录横向偏差、航点误差、最小障碍边缘净空、直线左右摆动次数和任务时间。纯离线测试覆盖直线、弯道自然内切、单个/重叠障碍、封闭障碍环、障碍内航点、变高航点、按 T 选择比最短距离更快的候选，以及前视跟踪器在直线和绕障路线上的表现：

```powershell
python -m unittest discover -s tests -v
```

Flight logs, local simulator settings, and Python environments are excluded from Git.
