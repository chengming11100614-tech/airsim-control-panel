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
  speed parameterization, AirSim response simulation, calibration, and path following.
- `flight/`: standalone flight training exercises and CSV recording.

## 导航规划与巡航

地图使用 AirSim NED 坐标，距离为米，速度为米每秒，障碍物是 XY 平面圆形截面的竖直圆柱。
输入航点按给定顺序全部经过；每点可设置到点悬停时间，不能从航点位置穿过障碍物。

`navigation/geometry.py` 将障碍半径膨胀为“障碍半径 + 安全间距 + 跟踪余量”，并用切线、圆弧和可见图搜索有限条不同绕行路线；这种多拓扑候选思路参考了 [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)。`navigation/curve.py` 为候选路线拟合通过航点的连续三次样条，并用 Bézier 凸包递归检查曲线全程安全。`navigation/time_parameterization.py` 根据速度、加速度、制动、曲率、爬升和响应延迟生成逐点速度曲线：曲率上限给出弯中速度，反向可达性传播安排弯前制动，正向传播安排出弯加速；方法采用了 [TOPP-RA](https://github.com/hungpham2511/toppra) 一类的路径速度可达性思路。

`navigation/execution.py` 将这条引导曲线交给实际使用的前视控制器，再按 AirSim 的控制周期、速度指令延迟、一阶响应时间常数和加速度上限模拟飞机运动。最近实飞日志测得中位控制周期约 0.063 秒；速度响应拟合约为 τ=1.3 秒、矢量加速度 1.3 m/s²、指令延迟 0.05 秒。对同一五航点任务进行两次重复试飞后，预测 T 分别比实飞短 0.73/0.83 秒；按两次数据校准 1.015 倍的预测时间轴，并同步修正预测速度和加速度，航点悬停时间保持原值。将修正后的预测与两次实际轨迹按时间对齐，位置误差 P95 为 0.132/0.136 米，预测 T 与实飞相差 0.05/0.04 秒；路线几何和实际控制指令不受时钟修正影响。该修正目前依据同一任务的重复数据，仍需在其他航线复核。预测在转弯处允许至 1.25 米的航点经过误差，依据是历史竞速转弯实飞最大误差约 1.18 米；直线航点仍限制在 0.6 米。所有路线必须按顺序完成，且预测障碍净空必须达标。规划器按闭环预计用时 T 选择最快可行候选。界面绘制和上传的是预测飞行轨迹，自动巡航仍沿已验证的引导曲线执行，并把弯内切线作为合理的实际路线。每次巡航 CSV 记录预测位置、同一时刻的位置误差及到预测轨迹的横向偏差，便于用后续实飞复核；当前模型仍是低阶近似，不是全局最优或精确飞行保证。

`navigation/controller.py` 每拍将目标放在已规划曲线前方，再沿弦线给出速度方向，采用了 [AirSim 自带前视跟踪](https://github.com/microsoft/AirSim/blob/main/docs/apis.md)的 carrot-following 思路。飞机过弯时可自然向弯内收，不会被横向误差项拉回曲线中心。控制器在速度曲线持续上升且前方没有制动谷或停车点时，按响应距离提前给油，并受当前位置曲率速度上限约束；遇到减速段仍服从原有制动计划。控制器按实测弧长推进，偏航或漏过航点只记告警并保持正向飞行，不倒飞追点；只有逼近手绘障碍物实体边缘才触发硬悬停。它保留了“沿路径前进，同时约束横向偏差”的思想，但不再用导致抖动的 OSQP 短时 MPC；偏差和沿线进度以 CSV 记录，可用于后续按实际飞行数据调参。这与 [MPCC 的路径进度和轮廓误差目标](https://arxiv.org/abs/2108.13205)相呼应。

面板的速度字段是最高允许速度，上限 15 m/s。速度按路径弧长动态计算：前向加速传播根据可用加速距离限制速度，反向制动传播从弯道和航点速度上限反推减速起点，曲率再约束弯中速度；短直线来不及到上限时会形成加速后制动的速度曲线，长直线才有机会接近上限。当前默认规划加速度/制动上限为 2.4/0.75 m/s²；AirSim 的实测速度响应模型仍约为 τ=1.3 秒、速度矢量加速度 1.3 m/s²，规划上限不会绕过闭环预测。

在同一起点、同五航点和三障碍任务上，旧纵向参数 1.8/0.55 m/s² 的两次实飞用时为 53.17/53.20 秒；新参数 2.4/0.75 的三次实飞用时为 52.484/52.500/52.563 秒，平均快约 0.67 秒（1.3%），最高实测速度从 3.86 提到 4.27 m/s。新参数三次最低实体障碍净空均约 1.12 m，预测安全门槛为 1.05 m，飞行中碰撞记录为零；经过误差最大 1.111 m，出现在转弯处。直线横向偏差 P95 约 0.33 m，弯道 P95 约 1.07 m；弯道允许自然内切/外绕，安全净空仍是硬约束。预测 T 为 52.526 秒，与三次实飞平均值相差约 0.01 秒；同时间位置误差 P95 为 0.167–0.184 m。落地接触报告的对象是 `Base_Terrain`，不计为飞行中碰撞。

这组加减速参数只在该赛道完成验证，不代表所有航线和机体状态都达到相同表现；横向加速度仍保持 1.0 m/s²，因为离线提高横向上限会把净空压近安全门槛。独立直线标定仍可限制直线速度，横向能力不会从直线测试中伪装成实测。飞行 CSV 同时记录计划速度和参考曲率，便于检查弯前何时减速、出弯何时加速。面板消息写入 `logs/panel_日期.log`，20 Hz 飞行遥测写入对应 CSV；其它航线仍需通过实飞日志复核预测时间和路径偏差。

控制面板会在飞行 CSV 中记录横向偏差、航点误差、最小障碍边缘净空、直线左右摆动次数和任务时间。纯离线测试覆盖直线、弯道自然内切、单个/重叠障碍、封闭障碍环、障碍内航点、变高航点、按 T 选择比最短距离更快的候选，以及前视跟踪器在直线和绕障路线上的表现：

```powershell
python -m unittest discover -s tests -v
```

Flight logs, local simulator settings, and Python environments are excluded from Git.
