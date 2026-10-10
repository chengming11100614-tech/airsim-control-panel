"""无人机普通飞行训练 —— 功能一 主程序。

用法（先启动 Africa_001.exe，再运行本脚本）：

    python flight_trainer.py --task takeoff     起降
    python flight_trainer.py --task hover       定点悬停
    python flight_trainer.py --task altitude    定高
    python flight_trainer.py --task line        直线航线
    python flight_trainer.py --task rect        矩形航线
    python flight_trainer.py --task circle      绕圈
    python flight_trainer.py --task all         依次跑完六组动作

每次运行都会在 logs/ 下生成一份 CSV 轨迹数据，--plot 还会额外输出一张轨迹图。
"""

import argparse
import sys
import traceback

import actions
from drone_base import DroneBase
from track_logger import TrackLogger

# 各科目的默认参数，命令行可以覆盖
DEFAULTS = {
    "height": 5.0,     # 训练高度（米，正值，内部转 NED）
    "speed": 2.0,      # 飞行速度（米每秒）
    "hover": 10.0,     # 悬停秒数
    "side": 8.0,       # 矩形边长（米）
    "radius": 4.0,     # 绕圈半径（米）
    "laps": 1,         # 绕圈圈数
}


# ---------------------------------------------------------------------------
# 各训练科目
# ---------------------------------------------------------------------------

def task_takeoff(drone, args):
    """起降：起飞 -> 悬停 -> 降落，验收平稳度。"""
    drone.takeoff(height=-args.height)
    drone.hover(args.hover)
    drone.land()


def task_hover(drone, args):
    """定点悬停：飞到一个指定点后悬停，验收位置偏差。"""
    drone.takeoff(height=-args.height)
    actions.move_by_position(drone, 5.0, 5.0, -args.height, speed=args.speed)
    drone.hover(args.hover)
    drone.land()


def task_altitude(drone, args):
    """定高：在两个高度之间切换，验收高度波动范围。"""
    drone.takeoff(height=-args.height)
    actions.hold_altitude(drone, args.height, seconds=args.hover / 2.0)
    actions.move_by_altitude(drone, args.height + 3.0)
    actions.hold_altitude(drone, args.height + 3.0, seconds=args.hover / 2.0)
    actions.move_by_altitude(drone, args.height)
    drone.hover(3)
    drone.land()


def task_line(drone, args):
    """直线航线：两点之间定速平飞。"""
    z = -args.height
    drone.takeoff(height=z)
    start = (0.0, 0.0, z)
    end = (args.side, 0.0, z)
    actions.fly_line(drone, start, end, speed=args.speed)
    drone.hover(3)
    drone.land()


def task_rect(drone, args):
    """矩形航线：四个航点走闭环，验收直角精度。"""
    drone.takeoff(height=-args.height)
    actions.fly_rectangle(
        drone, origin=(0.0, 0.0, -args.height),
        side=args.side, speed=args.speed, repeats=1,
    )
    drone.hover(3)
    drone.land()


def task_circle(drone, args):
    """绕圈：用速度指令连续画圆，验收半径一致性。"""
    drone.takeoff(height=-args.height)
    actions.fly_circle(
        drone, center=(0.0, 0.0), radius=args.radius,
        altitude=args.height, speed=args.speed, laps=args.laps,
    )
    drone.hover(3)
    drone.land()


ALL_TASKS = [
    ("起降", task_takeoff),
    ("定点悬停", task_hover),
    ("定高", task_altitude),
    ("直线航线", task_line),
    ("矩形航线", task_rect),
    ("绕圈", task_circle),
]

TASK_MAP = {
    "takeoff": task_takeoff,
    "hover": task_hover,
    "altitude": task_altitude,
    "line": task_line,
    "rect": task_rect,
    "circle": task_circle,
}


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def build_parser():
    p = argparse.ArgumentParser(
        description="无人机普通飞行训练（功能一）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--task", default="hover",
                   choices=sorted(list(TASK_MAP.keys()) + ["all"]),
                   help="要执行的训练科目")
    p.add_argument("--height", type=float, default=DEFAULTS["height"], help="训练高度（米）")
    p.add_argument("--speed", type=float, default=DEFAULTS["speed"], help="飞行速度（米每秒）")
    p.add_argument("--hover", type=float, default=DEFAULTS["hover"], help="悬停秒数")
    p.add_argument("--side", type=float, default=DEFAULTS["side"], help="矩形边长（米）")
    p.add_argument("--radius", type=float, default=DEFAULTS["radius"], help="绕圈半径（米）")
    p.add_argument("--laps", type=int, default=DEFAULTS["laps"], help="绕圈圈数")
    p.add_argument("--ip", default="", help="仿真器 IP，本机留空即可")
    p.add_argument("--port", type=int, default=41451, help="RPC 端口")
    p.add_argument("--csv", default=None, help="自定义 CSV 文件名")
    p.add_argument("--plot", action="store_true", help="结束后额外输出轨迹 PNG")
    p.add_argument("--draw", action="store_true", help="在仿真器中直接画出飞行轨迹")
    return p


def run_one(drone, args, name, func):
    """跑单个科目，并把它的轨迹单独记一份 CSV。"""
    print("\n" + "=" * 60)
    print("训练科目：%s" % name)
    print("=" * 60)

    filename = args.csv if args.csv else ("track_%s_%s.csv" % (func.__name__, name))
    logger = TrackLogger(drone, filename=filename)
    logger.start()
    try:
        func(drone, args)
    finally:
        logger.stop()

    logger.print_summary()
    if args.draw:
        actions.draw_path(drone, actions.path_from_logger(logger))
    if args.plot:
        logger.plot()
    return logger


def main(argv=None):
    args = build_parser().parse_args(argv)

    drone = DroneBase(ip=args.ip, port=args.port)
    try:
        drone.connect()
        drone.arm()

        if args.task == "all":
            for name, func in ALL_TASKS:
                run_one(drone, args, name, func)
        else:
            func = TASK_MAP[args.task]
            name = [n for n, f in ALL_TASKS if f is func]
            run_one(drone, args, name[0] if name else args.task, func)

        print("\n[训练] 全部科目完成")

    except KeyboardInterrupt:
        print("\n[训练] 被用户中断，正在安全降落 ...")
        return 130
    except Exception:
        print("\n[训练] 运行出错：")
        traceback.print_exc()
        return 1
    finally:
        drone.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
