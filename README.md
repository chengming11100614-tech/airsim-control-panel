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
  time parameterization, speed calibration, and predictive path following.
- `flight/`: standalone flight training exercises and CSV recording.

## 导航规划与巡航

地图使用 AirSim NED 坐标，距离为米，速度为米每秒，障碍物是 XY 平面圆形截面的竖直圆柱。
输入航点按给定顺序全部经过；每点可设置到点悬停时间，不能从航点位置穿过障碍物。

`navigation/geometry.py` 将障碍半径膨胀为“障碍半径 + 安全间距 + 标定跟踪余量”，并用切线、圆弧和可见图搜索有限条不同绕行路线。`navigation/curve.py` 为候选路线拟合通过航点的连续三次样条，并用 Bézier 凸包递归检查曲线全程安全。`navigation/time_parameterization.py` 根据速度、加速度、制动、曲率、爬升和响应延迟生成速度曲线，再按完整预计时间 T 比较候选。只在安全且动力学可行的有限候选中选最快路线，不宣称全局数学最优；找不到可行曲线时会报错，不会退回直线。

`navigation/controller.py` 通过 OSQP 每 50 ms 求一次短时速度控制，跟踪已验证轨迹并限制速度和命令变化。跟踪参考按实测弧长推进，避免飞机落后时参考点仍按计划时钟跑到前方；发生横向偏差或漏过航点时记录告警，保持航线正向并尝试回归，不倒飞追赶。规划膨胀半径是绕障缓冲区，不是碰撞边界；巡航允许跟踪误差进入缓冲区，只有飞机中心距手绘障碍边缘小于 0.10 m 或 AirSim 报告碰撞时才触发障碍硬停止。遥测长时间失效、用户停止或航线被编辑时仍会安全悬停。面板消息写入 `logs/panel_日期.log`，20 Hz 飞行遥测仍写入对应 CSV。`navigation/calibration.py` 汇总安全直线上的实测数据。未标定时规划速度上限为 2 m/s；标定后最高不超过实测合格速度和 15 m/s。横向加速度目前保留保守值，因为直线加速测试本身不能测出转弯能力。

控制面板会在飞行 CSV 中记录横向偏差、航点误差、最小障碍边缘净空、直线左右摆动次数和任务时间。纯离线测试覆盖直线、单个/重叠障碍、封闭障碍环、障碍内航点、变高航点、按 T 选择比最短距离更快的候选，以及预测跟踪器在直线和绕障路线上的表现：

```powershell
python -m unittest discover -s tests -v
```

Flight logs, local simulator settings, and Python environments are excluded from Git.
