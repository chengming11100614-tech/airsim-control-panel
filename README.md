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

`navigation/execution.py` 将这条引导曲线交给实际使用的前视控制器，再按 AirSim 的控制周期、速度指令延迟、一阶响应时间常数和加速度上限模拟飞机运动。最近实飞日志测得中位控制周期约 0.063 秒；速度响应拟合约为 τ=1.3 秒、矢量加速度 1.3 m/s²、指令延迟 0.05 秒。当前竞速配置在同一五航点任务上重复飞行三次后，将预测时间轴校准为 1.017 倍，并同步修正预测速度和加速度，航点悬停时间保持原值。校准后的预计 T 为 50.977 秒，三次实飞平均为 50.943 秒；按时间对齐的位置误差 P95 为 0.165–0.183 米，到预测轨迹的最近点距离 P95 为 0.056–0.093 米。路线几何和实际控制指令不受时钟修正影响。该修正仅由同一任务的重复数据拟合，仍需在其他航线复核。预测在转弯处允许至 1.25 米的航点经过误差，依据是近期实飞最大约 1.12 米；直线航点仍限制在 0.6 米。所有路线必须按顺序完成，且预测障碍净空必须达标。规划器按闭环预计用时 T 选择最快可行候选。界面绘制和上传的是预测飞行轨迹，自动巡航仍沿已验证的引导曲线执行，并把弯内切线作为合理的实际路线。每次巡航 CSV 记录预测位置、同一时刻的位置误差及到预测轨迹的横向偏差，便于用后续实飞复核；当前模型仍是低阶近似，不是全局最优或精确飞行保证。

`navigation/controller.py` 每拍将目标放在已规划曲线前方，再沿弦线给出速度方向，采用了 [AirSim 自带前视跟踪](https://github.com/microsoft/AirSim/blob/main/docs/apis.md)的 carrot-following 思路。飞机过弯时可自然向弯内收，不会被横向误差项拉回曲线中心。控制器在速度曲线持续上升且前方没有制动谷或停车点时，按响应距离提前给油，并受当前位置曲率速度上限约束；遇到减速段仍服从原有制动计划。控制器按实测弧长推进，偏航或漏过航点只记告警并保持正向飞行，不倒飞追点；只有逼近手绘障碍物实体边缘才触发硬悬停。它保留了“沿路径前进，同时约束横向偏差”的思想，但不再用导致抖动的 OSQP 短时 MPC；偏差和沿线进度以 CSV 记录，可用于后续按实际飞行数据调参。这与 [MPCC 的路径进度和轮廓误差目标](https://arxiv.org/abs/2108.13205)相呼应。

面板的速度字段是最高允许速度，上限 15 m/s。速度按路径弧长动态计算：前向加速传播根据可用加速距离限制速度，反向制动传播从弯道和航点速度上限反推减速起点，曲率再约束弯中速度；短直线来不及到上限时会形成加速后制动的速度曲线，长直线才有机会接近上限。当前默认规划加速度/制动上限为 2.4/0.75 m/s²；AirSim 的实测速度响应模型仍约为 τ=1.3 秒、速度矢量加速度 1.3 m/s²，规划上限不会绕过闭环预测。

在同一起点、同五航点和三障碍任务上，旧纵向参数 1.8/0.55 m/s² 的两次实飞用时为 53.17/53.20 秒；纵向参数 2.4/0.75、横向上限 1.0 m/s² 的三次实飞平均用时 52.516 秒；将横向上限调整为 1.15 m/s²、跟踪预留调整为 0.35 m 后，最新三次用时为 50.891/50.937/51.000 秒，平均再快 1.57 秒（约 3.0%）。最新配置最高实测速度约 4.28 m/s，最低实体障碍净空 1.174–1.175 m（规划安全门槛 1.15 m），飞行中碰撞记录为零；转弯航点经过误差最大约 1.124 m。沿几何引导线测得直线横向偏差 P95 约 0.38 m、弯道约 1.10 m；转弯可自然内切/外绕，安全净空仍是硬约束。校准后预测 T 为 50.977 秒，与三次实飞平均相差约 0.034 秒。落地接触报告的对象是 `Base_Terrain`，不计为飞行中碰撞。

这组加减速和横向参数只在该赛道完成验证，不代表所有航线和机体状态都达到相同表现。实体净空比规划门槛多约 0.024 m，余量不大；其它地图应先检查规划净空和预测轨迹，再试飞。直线相对几何引导线的偏差 P95 仍约 0.38 m，而相对预测实际轨迹的最近距离 P95 小于 0.10 m；两项指标口径不同，后者说明实飞轨迹模型贴合较好，前者仍是直线循线精度的改进空间。独立直线标定仍可限制直线速度，横向能力不会从直线测试中伪装成实测。飞行 CSV 同时记录计划速度和参考曲率，便于检查弯前何时减速、出弯何时加速。面板消息写入 `logs/panel_日期.log`，20 Hz 飞行遥测写入对应 CSV；其它航线仍需通过实飞日志复核预测时间和路径偏差。

控制面板会在飞行 CSV 中记录横向偏差、航点误差、最小障碍边缘净空、直线左右摆动次数和任务时间。纯离线测试覆盖直线、弯道自然内切、单个/重叠障碍、封闭障碍环、障碍内航点、变高航点、按 T 选择比最短距离更快的候选，以及前视跟踪器在直线和绕障路线上的表现：

```powershell
python -m unittest discover -s tests -v
```

## 参数优化记录

参数对比应固定起点、航点顺序、障碍物地图和速度上限；每个候选至少重复试飞三次，并结合平均用时、航点误差和最小障碍净空评估效果。

已整理的实飞日志和参数迭代汇总见 [experiments/README.md](experiments/README.md)。归档仅保留飞行区间，空闲遥测和本地模拟器文件不纳入 Git。

Flight logs, local simulator settings, and Python environments are excluded from Git.
