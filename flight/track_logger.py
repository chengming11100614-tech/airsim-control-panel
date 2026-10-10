"""轨迹与姿态数据记录 —— 功能一「无人机普通飞行训练」数据层。

以固定频率（默认 20 Hz）后台采样飞机状态，写成一个 CSV 文件：

    t, x, y, z, alt, vx, vy, vz, roll, pitch, yaw

其中 t 为相对本次记录开始的秒数，x / y / z 为 NED 坐标（米），
alt 为换算后的离地高度（米），姿态角为弧度。

这个 CSV 就是验收材料里的「位置偏差数据」，也是画轨迹曲线的原始输入。
用法：

    logger = TrackLogger(drone)
    logger.start()
    ... 执行训练动作 ...
    logger.stop()
    logger.plot()        # 可选：把轨迹画成 PNG
"""

import csv
import os
import threading
import time

# CSV 列顺序，改这里要同步改 _write_header
FIELDS = ["t", "x", "y", "z", "alt", "vx", "vy", "vz", "roll", "pitch", "yaw"]


class TrackLogger:
    """后台线程按固定频率采样 DroneBase，落盘为 CSV。"""

    def __init__(self, drone, filename=None, rate_hz=20.0, out_dir=None):
        self.drone = drone
        self.period = 1.0 / float(rate_hz)
        self.out_dir = out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        self.filename = filename or ("track_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        self.path = os.path.join(self.out_dir, self.filename)
        self.samples = []                 # 同时留在内存里，方便结束时统计
        self._thread = None
        self._stop = threading.Event()
        self._t0 = None

    # -- 生命周期 -----------------------------------------------------------

    def start(self):
        os.makedirs(self.out_dir, exist_ok=True)
        self._t0 = time.time()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[TrackLogger] 开始记录 -> %s" % self.path)
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        print("[TrackLogger] 记录结束，共 %d 个采样点" % len(self.samples))
        return self

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False

    # -- 采样循环 -----------------------------------------------------------

    def _loop(self):
        with open(self.path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=FIELDS)
            writer.writeheader()
            while not self._stop.is_set():
                try:
                    row = self.drone.state()
                    row["t"] = time.time() - self._t0
                except Exception:
                    # 仿真器短暂无响应时跳过这一拍，不要中断整段记录
                    time.sleep(self.period)
                    continue
                self.samples.append(row)
                writer.writerow({k: row[k] for k in FIELDS})
                fh.flush()                 # 崩溃也不至于丢掉全部数据
                time.sleep(self.period)

    # -- 统计与可视化 -------------------------------------------------------

    def summary(self):
        """返回本次记录的汇总指标，直接可用于验收报告。"""
        if not self.samples:
            return {}
        xs = [s["x"] for s in self.samples]
        ys = [s["y"] for s in self.samples]
        zs = [s["z"] for s in self.samples]
        alts = [s["alt"] for s in self.samples]
        span = max(self.samples[-1]["t"] - self.samples[0]["t"], 1e-6)

        def extent(vals):
            return max(vals) - min(vals)

        return {
            "duration_s": self.samples[-1]["t"] - self.samples[0]["t"],
            "samples": len(self.samples),
            "alt_mean": sum(alts) / len(alts),
            "alt_span": extent(alts),          # 高度波动范围
            "x_span": extent(xs),
            "y_span": extent(ys),
            "z_span": extent(zs),
            "path_len": self._path_length(),
            "avg_speed": self._path_length() / span,
        }

    def _path_length(self):
        total = 0.0
        for a, b in zip(self.samples, self.samples[1:]):
            total += ((b["x"] - a["x"]) ** 2 + (b["y"] - a["y"]) ** 2 + (b["z"] - a["z"]) ** 2) ** 0.5
        return total

    def print_summary(self):
        s = self.summary()
        if not s:
            print("[TrackLogger] 没有采样数据")
            return
        print("[TrackLogger] 记录时长 %.1f 秒 / %d 点" % (s["duration_s"], s["samples"]))
        print("[TrackLogger] 平均离地高度 %.2f 米，高度波动 %.2f 米" % (s["alt_mean"], s["alt_span"]))
        print("[TrackLogger] 水平位移 X %.2f 米 / Y %.2f 米" % (s["x_span"], s["y_span"]))
        print("[TrackLogger] 轨迹总长 %.2f 米，平均速度 %.2f 米每秒" % (s["path_len"], s["avg_speed"]))

    def plot(self, png_name=None, show=False):
        """把 XY 轨迹与高度曲线画成一张 PNG（需要 matplotlib）。"""
        try:
            import matplotlib
            if not show:
                matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("[TrackLogger] 未安装 matplotlib，跳过绘图（pip install matplotlib）")
            return None

        if not self.samples:
            return None

        xs = [s["x"] for s in self.samples]
        ys = [s["y"] for s in self.samples]
        ts = [s["t"] for s in self.samples]
        alts = [s["alt"] for s in self.samples]

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        axes[0].plot(xs, ys, color="#531C84", linewidth=2)
        axes[0].scatter([xs[0]], [ys[0]], color="#2E7D32", s=60, label="起点", zorder=3)
        axes[0].scatter([xs[-1]], [ys[-1]], color="#C62828", s=60, label="终点", zorder=3)
        axes[0].set_xlabel("X 正北 (m)")
        axes[0].set_ylabel("Y 正东 (m)")
        axes[0].set_title("Flight Path (NED top view)")
        axes[0].axis("equal")
        axes[0].grid(alpha=0.3)
        axes[0].legend()

        axes[1].plot(ts, alts, color="#7E55A3", linewidth=2)
        axes[1].set_xlabel("t (s)")
        axes[1].set_ylabel("altitude (m)")
        axes[1].set_title("Altitude vs Time")
        axes[1].grid(alpha=0.3)

        fig.tight_layout()
        png_name = png_name or (os.path.splitext(self.filename)[0] + ".png")
        out = os.path.join(self.out_dir, png_name)
        fig.savefig(out, dpi=150)
        if show:
            plt.show()
        plt.close(fig)
        print("[TrackLogger] 轨迹图已保存 -> %s" % out)
        return out
