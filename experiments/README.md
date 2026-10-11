# AirSim 实飞迭代记录

本页记录控制面板在 AirSim 中进行的路线、动态速度和循线参数迭代。它把被 Git 忽略的运行日志整理为可审阅的数据；原日志仍保留在本机运行目录。

## 数据范围与提取规则

- 本机共有 **30 份** `route_*.csv`，原始合计 **124.59 MiB**，其中包含应用空闲期和长时间遥测。
- 本目录归档 **29 段完整试飞记录**，包括面板日志中的飞行区间和专用试飞 CSV；计时器重置处分段，专用试飞器文件保留收尾制动样本。
- 未归档的空闲遥测或未成功起飞数据列于文末。提取保留原 CSV 列和行内容；不补造缺失参数。
- 表中试飞时长优先用 `flight_elapsed`；没有该字段时，若 `t` 从零附近起算则用结束值，否则用飞行区间跨度。峰值速度为水平速度 `sqrt(vx²+vy²)`。P95 使用 nearest-rank 95th percentile。
- `P95路径偏差`保留日志原字段口径：`cross_track_pred` 表示到预测轨迹的偏差；`dev` 和 `cross_track` 是当时记录的路线偏差。不同字段的 P95 不直接横向比较。
- `—` 表示日志未记录该指标；碰撞列的 `0` 只表示 CSV 有碰撞字段且没有碰撞采样，`—` 表示 CSV 根本没有记录碰撞字段。若 CSV 文件名没有参数标签，实验参数仍标记为未知，不根据速度或用时反推。

## 可复核的参数迭代

下表汇总同一五航点、三障碍竞速路线的关键阶段。每组用时来自原始 CSV；只有明确标注重复次数的组才计算均值。

| 阶段 | 参数/变化 | 实飞用时 | 重复数 | 观测 |
|---|---|---:|---:|---|
| 速度曲线探索 | 早期速度试飞；CSV 未完整记录每次参数 | 55.406 / 54.172 / 53.171 s | 3 | 后续改为按弧长安排加速与制动 |
| 前视与路线对比 | route compare、直线/曲线 lookahead | 53.203 / 53.172 / 53.297 s | 各 1 | 仅作探索记录，不作为重复统计 |
| 动态速度基线 | 纵向加速度/制动 2.4/0.75 m/s²，横向上限 1.0 m/s² | 52.484 / 52.500 / 52.563 s；均值 **52.516 s** | 3 | 峰值速度约 4.27 m/s |
| 横向与跟踪余量 | 横向上限 1.1 m/s²，跟踪余量 0.30 m | 51.235 / 51.219 / 51.187 s；均值 **51.214 s** | 3 | 峰值约 4.29 m/s |
| 横向与跟踪余量 | 横向上限 1.15 m/s²，跟踪余量 0.35 m | 50.891 / 50.937 / 51.000 s；均值 **50.943 s** | 3 | 实测最小障碍净空约 1.174–1.175 m |
| 新候选 | 横向上限 1.25 m/s²，跟踪余量 0.40 m | 49.953 s | 1 | 实测最小净空约 1.196 m |
| 新候选 | 横向上限 1.25 m/s²，跟踪余量 0.45 m | **49.796 s** | 1 | 预测用时 49.833 s；同一时刻位置误差 P95 0.197 m；到预测轨迹偏差 P95 0.044 m；实测净空 1.2415 m，略低于 1.25 m 规划门槛；无飞行碰撞记录 |

新候选比 1.15/0.35 组三次均值快约 1.15 s（约 2.3%），但它只飞了一次，且横向上限和跟踪余量同时改变；该差值不能归因于单一参数。

## 单次试飞明细

“净空”是飞机中心到手绘障碍圆柱边缘的实测距离。P95误差与路径偏差沿用各 CSV 原字段口径。

| 归档日志 | 时长 (s) | 峰值水平速度 (m/s) | 最小净空 (m) | P95预测误差 (m) | P95路径偏差 (m；字段) | 碰撞记录 |
|---|---:|---:|---:|---:|---:|---:|
| [`route_20260926_182607.csv`](flight-logs/route_20260926_182607.csv) | 74.885 | 1.983 | — | — | 0.427；dev | — |
| [`route_20261010_121942.csv`](flight-logs/route_20261010_121942.csv) | 53.950 | 1.983 | — | — | 0.550；dev | — |
| [`route_20261010_140524.csv`](flight-logs/route_20261010_140524.csv) | 13.390 | 1.921 | 0.796 | — | 0.426；dev | — |
| [`route_20261010_141759.csv`](flight-logs/route_20261010_141759.csv) | 7.438 | 1.777 | 4.010 | — | 0.697；dev | — |
| [`route_20261010_143341_run2.csv`](flight-logs/route_20261010_143341_run2.csv) | 89.656 | 1.957 | 1.446 | — | 0.308；dev | — |
| [`route_20261010_201717.csv`](flight-logs/route_20261010_201717.csv) | 31.125 | 1.931 | 0.067 | — | 1.877；dev | — |
| [`route_20261010_214901.csv`](flight-logs/route_20261010_214901.csv) | 45.266 | 3.332 | 0.955 | — | 1.045；dev | — |
| [`route_20261010_232710.csv`](flight-logs/route_20261010_232710.csv) | 57.062 | 3.502 | 1.156 | 1.585 | 0.033；dev | — |
| [`route_speed_trial_20261010_233831.csv`](flight-logs/route_speed_trial_20261010_233831.csv) | 55.406 | 4.292 | 1.117 | — | 1.189；cross_track | 0 |
| [`route_speed_trial_20261010_234827.csv`](flight-logs/route_speed_trial_20261010_234827.csv) | 54.172 | 3.754 | 1.146 | — | 0.865；cross_track | 0 |
| [`route_speed_trial_20261010_235638.csv`](flight-logs/route_speed_trial_20261010_235638.csv) | 53.171 | 3.858 | 1.144 | — | 0.875；cross_track | 0 |
| [`route_compare_20261011_000419.csv`](flight-logs/route_compare_20261011_000419.csv) | 53.203 | 3.857 | 1.144 | 1.693 | 0.881；cross_track | 0 |
| [`route_lookahead_trial_20261011_002102.csv`](flight-logs/route_lookahead_trial_20261011_002102.csv) | 53.172 | 3.857 | 1.148 | 0.174 | 0.875；cross_track | 0 |
| [`route_curve_lookahead_trial_20261011_002536.csv`](flight-logs/route_curve_lookahead_trial_20261011_002536.csv) | 53.297 | 3.857 | 1.203 | 0.149 | 0.852；cross_track | 0 |
| [`route_dynamic_speed_trial_20261011_003245.csv`](flight-logs/route_dynamic_speed_trial_20261011_003245.csv) | 52.484 | 4.271 | 1.120 | 0.182 | 1.019；cross_track | 0 |
| [`route_dynamic_speed_trial_2_20261011_003531.csv`](flight-logs/route_dynamic_speed_trial_2_20261011_003531.csv) | 52.500 | 4.269 | 1.120 | 0.185 | 1.013；cross_track | 0 |
| [`route_dynamic_speed_trial_3_20261011_003637.csv`](flight-logs/route_dynamic_speed_trial_3_20261011_003637.csv) | 52.563 | 4.269 | 1.121 | 0.169 | 1.027；cross_track | 0 |
| [`route_final_defaults_20261011_004310.csv`](flight-logs/route_final_defaults_20261011_004310.csv) | 52.484 | 4.272 | 1.121 | 0.165 | 1.020；cross_track | 0 |
| [`route_lateral_11_trial_20261011_004741.csv`](flight-logs/route_lateral_11_trial_20261011_004741.csv) | 51.297 | 4.282 | 1.095 | 0.228 | 1.033；cross_track | 0 |
| [`route_lateral_11_reserve_30_20261011_004955.csv`](flight-logs/route_lateral_11_reserve_30_20261011_004955.csv) | 51.235 | 4.286 | 1.136 | 0.170 | 1.040；cross_track | 0 |
| [`route_lateral_11_reserve_30_repeat2_20261011_005142.csv`](flight-logs/route_lateral_11_reserve_30_repeat2_20261011_005142.csv) | 51.219 | 4.285 | 1.135 | 0.191 | 1.041；cross_track | 0 |
| [`route_lateral_11_reserve_30_repeat3_20261011_005247.csv`](flight-logs/route_lateral_11_reserve_30_repeat3_20261011_005247.csv) | 51.187 | 4.285 | 1.134 | 0.174 | 1.044；cross_track | 0 |
| [`route_lateral_115_reserve_35_20261011_005457.csv`](flight-logs/route_lateral_115_reserve_35_20261011_005457.csv) | 50.891 | 4.276 | 1.175 | 0.206 | 1.038；cross_track | 0 |
| [`route_lateral_115_reserve_35_repeat2_20261011_005816.csv`](flight-logs/route_lateral_115_reserve_35_repeat2_20261011_005816.csv) | 50.937 | 4.280 | 1.174 | 0.208 | 1.053；cross_track | 0 |
| [`route_lateral_115_reserve_35_repeat3_20261011_005921.csv`](flight-logs/route_lateral_115_reserve_35_repeat3_20261011_005921.csv) | 51.000 | 4.276 | 1.175 | 0.332 | 1.055；cross_track | 0 |
| [`route_candidate_lat125_reserve40_20261011_011349.csv`](flight-logs/route_candidate_lat125_reserve40_20261011_011349.csv) | 49.953 | 4.298 | 1.196 | 0.193 | 0.043；cross_track_pred | 0 |
| [`route_candidate_lat125_reserve45_20261011_012113.csv`](flight-logs/route_candidate_lat125_reserve45_20261011_012113.csv) | 49.796 | 4.301 | 1.242 | 0.197 | 0.044；cross_track_pred | 0 |
| [`route_20261011_012753_run1.csv`](flight-logs/route_20261011_012753_run1.csv) | 38.359 | 4.215 | 1.370 | 0.759 | 0.058；dev | — |
| [`route_20261011_012753_run2.csv`](flight-logs/route_20261011_012753_run2.csv) | 37.516 | 3.872 | 1.368 | 0.460 | 0.059；dev | — |

## 未纳入对比的数据

- `route_20261010_135723.csv`：run1: no sustained flight (T=0.062s, vmax=0.001m/s)。
- `route_20261010_143341.csv`：run1: no sustained flight (T=0.936s, vmax=0.000m/s)；run2 已归档。
- `route_20261010_225919.csv`：全程 `flight_elapsed=0` 且水平速度为零，是约 17 分钟的空闲遥测。

## 复现实验与使用说明

- 竞速参数对比固定起点、航点顺序、障碍物地图和速度上限；每个候选至少重复 3 次，再比较平均用时和离散程度。
- 选最快方案前同时检查航点通过误差、到预测轨迹偏差、直线段偏摆和最小障碍净空。
- 当前历史数据覆盖单一主要赛道；不同地图和飞行器状态需要重新验证。数据量较少，优先做重复试飞与参数辨识，不用机器学习拟合飞行器。
- 原始运行日志与虚拟环境继续由 `.gitignore` 排除；本目录只保存已筛选的飞行区间。
