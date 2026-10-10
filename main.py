# -*- coding: utf-8 -*-
"""航线规划与自主巡航上位机（地面站）—— 功能二。

这是一个 tkinter 写的单窗口地面站，不用装 PyQt，Python 自带库就够。
打开后是一张俯视图（NED 的 X 正北朝上、Y 正东朝右），你在地图上点几下
就能把航线画出来，剩下的交给程序：

    地图上点击       ->  加航点（拖拽可以改位置，右键删掉）
    画几个障碍物     ->  「规划航线」自动生成一条绕开障碍的可行航线
    「上传到仿真器」 ->  在 AirSim 场景里把航线画出来，现场演示很直观
    「自动巡航」     ->  无人机沿航线自主飞行，到点悬停
    「手动模式」     ->  切 WASD 操控，沿线飞，界面上实时显示偏了多少米

界面左边是地图，右边分四个区：航点表 / 规划参数 / 手动操控 / 执行按钮。
底部状态栏一直在刷位置、高度、速度和到规划航线的横向偏差。

运行前提：先双击 Africa_001.exe 把场景加载完，再启动本程序并点「连接」。
没有装 airsim 也能打开——只做离线规划，飞行相关的按钮会自动禁用。

坐标系：NED，X 正北、Y 正东、Z 向下为正；界面上「高度」一律显示正值。
"""

import ctypes
import json
import math
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from planner.route_planner import (Obstacle, Waypoint, cumulative_lengths,
                                   demo_obstacles, demo_waypoints,
                                   deviation_to_path, plan_route)

try:
    import airsim
    HAS_AIRSIM = True
except ImportError:            # 没装 airsim 也能用来看航线规划，飞行按钮会禁用
    airsim = None
    HAS_AIRSIM = False


# ---------------------------------------------------------------------------
# 外观常量（配色沿用任务书里那套紫色）
# ---------------------------------------------------------------------------

BG      = "#17141D"      # 窗口底色
PANEL   = "#211C29"      # 面板
PANEL2  = "#2B2436"      # 输入框 / 表格
LINE    = "#3A3147"      # 分隔线
TEXT    = "#E9E5F2"
MUTED   = "#948CA6"
ACCENT  = "#B06CF5"      # 主题紫
ACCENT2 = "#7C3AED"
OK      = "#34D399"
WARN    = "#FBBF24"
DANGER  = "#F87171"
CYAN    = "#22D3EE"
GRID    = "#241F2E"
GRID2   = "#2E2739"

FONT    = ("Microsoft YaHei UI", 9)
FONT_B  = ("Microsoft YaHei UI", 9, "bold")
FONT_S  = ("Microsoft YaHei UI", 8)
FONT_L  = ("Microsoft YaHei UI", 11, "bold")
FONT_XL = ("Consolas", 20, "bold")

DEFAULT_ALT = 5.0        # 默认巡航高度（离地，米）
DEFAULT_SPD = 2.0        # 默认巡航速度（米每秒）
POSITION_EPS = 0.6       # 到点判定容差（米）
DEV_WARN = 3.0           # 手动模式偏差告警阈值（米）

# 地图编辑模式
MODE_PAN, MODE_WP, MODE_OBS, MODE_DEL = "pan", "wp", "obs", "del"


# ---------------------------------------------------------------------------
# 手动模式的按键
# ---------------------------------------------------------------------------
# 这里不走 tkinter 的 <KeyPress> 事件，而是直接问系统要按键状态，原因是那套
# 事件有两个致命限制：
#   1. 只有 GCS 窗口位于前台时才收得到。手动飞行时人往往在盯 Africa_001.exe
#      的仿真画面，GCS 一进后台就一个按键都收不到；
#   2. 输入法停在中文时会把字母键收去组词，Tk 只会报一个 keysym='??'。
# GetAsyncKeyState 读的是系统级按键状态，前台后台一样有效，也绕开了输入法。
# 代价是手动模式期间这几个字母键被全局读走，别在飞的同时往输入框里打字。

_VK = {"w": 0x57, "a": 0x41, "s": 0x53, "d": 0x44,
       "r": 0x52, "f": 0x46, "q": 0x51, "e": 0x45}

try:
    _GetAsyncKeyState = ctypes.windll.user32.GetAsyncKeyState
except AttributeError:      # 非 Windows 平台没有 windll
    _GetAsyncKeyState = None


def key_down(name):
    """这个键此刻是不是按着的。与窗口焦点、输入法都无关。"""
    return (_GetAsyncKeyState is not None
            and bool(_GetAsyncKeyState(_VK[name]) & 0x8000))


# ---------------------------------------------------------------------------
# 数据落盘
# ---------------------------------------------------------------------------

class FlightRecorder:
    """把每一拍的飞行数据写成 CSV，作为验收材料里的原始数据。

    列比小组一的 track_logger 多了 dev / wp / mode 三列，
    这样「手动模式沿航线的偏差」可以直接在表里查，不用再事后对齐。
    """

    FIELDS = ["t", "x", "y", "z", "alt", "vx", "vy", "vz",
              "roll", "pitch", "yaw", "dev", "wp", "mode"]

    def __init__(self, out_dir=None):
        self.out_dir = out_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "logs")
        self.path = None
        self.count = 0
        self._fh = None
        self._t0 = None
        self._since_flush = 0

    def start(self):
        os.makedirs(self.out_dir, exist_ok=True)
        name = "route_%s.csv" % time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(self.out_dir, name)
        self._fh = open(self.path, "w", newline="", encoding="utf-8")
        self._fh.write(",".join(self.FIELDS) + "\n")
        self._t0 = time.time()
        self.count = 0
        return self.path

    def write(self, row):
        # 采样线程和收尾逻辑是并发的：这里可能正好撞上 stop() 关文件。
        # 定格一份句柄、且整段包在 try 里，宁可丢掉最后一两拍也不能让采样线程崩掉。
        fh = self._fh
        if fh is None:
            return
        row = dict(row)
        row["t"] = "%.3f" % (time.time() - self._t0)
        try:
            fh.write(",".join(str(row.get(k, "")) for k in self.FIELDS) + "\n")
            self.count += 1
            self._since_flush += 1
            if self._since_flush >= 20:    # 每秒刷一次，崩溃不至于全丢
                fh.flush()
                self._since_flush = 0
        except (ValueError, OSError):
            self._fh = None

    def stop(self):
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass
            self._fh = None


class LockedClient:
    """把 AirSim 调用串行化的代理。

    msgpackrpc 的连接不是线程安全的——它自己的 session.py 里就留着一句
    「# need lock?」的注释。传输层是 tornado 的 IOStream，写缓冲是个
    bytearray：写线程在 write() 里做完 `_write_buffer += data`，紧接着会
    内联调用 _handle_write() 把数据发出去，而那里是

        socket.send(memoryview(self._write_buffer)[start:start + size])

    发送期间这个 bytearray 一直处于「已导出」状态、GIL 也是放开的。此时
    另一个线程再来一次 `+=` 需要扩容，CPython 就抛

        BufferError: Existing exports of data: object cannot be re-sized

    本程序有三个线程会碰 client：指令线程（_runner_loop）、遥测线程
    （_telem_loop）、以及退出时的界面线程。手动模式下指令线程 5 Hz、遥测
    线程 20 Hz 一起写，几秒内必撞上；巡航时指令线程大多阻塞在 join() 上
    不写，所以这个竞争一直没暴露。

    注意 join() 是在返回的 Future 上调用、不经过这里，所以锁只覆盖一次
    发送，不会让降落那种几十秒的阻塞把遥测线程一起卡住。
    """

    def __init__(self, client, lock):
        self._client = client
        self._lock = lock

    def __getattr__(self, name):
        attr = getattr(self._client, name)
        if not callable(attr):
            return attr

        def locked(*args, **kwargs):
            with self._lock:
                return attr(*args, **kwargs)

        return locked


# ---------------------------------------------------------------------------
# 主界面
# ---------------------------------------------------------------------------

class GCSApp:

    def __init__(self, root):
        self.root = root
        self.root.title("无人机航线规划与自主巡航 · 地面站")
        self.root.geometry("1400x880")
        self.root.minsize(1120, 720)
        self.root.configure(bg=BG)

        # --- 业务状态 ---
        self.client = None
        self.connected = False
        self.armed = False
        self.flying = False
        self.mode = "未连接"
        self.waypoints = []          # [Waypoint, ...]
        self.obstacles = []          # [Obstacle, ...]
        self.route = []              # 规划后的稀疏航线 [(x, y, z), ...]
        self.route_xy = []           # 同上，只留 XY，算偏差用
        self.route_cum = []
        self.route_dense = []        # A* 原始路径，画在地图上对比
        self.info = {}
        self.home = None             # 起飞点 NED
        self.cruise_index = -1       # 当前正在飞的航点序号
        self.telem = {}
        self.trail = []              # 实际飞过的轨迹

        # --- 地图视图 ---
        self.scale = 12.0            # 像素 / 米
        self.view_cx = 0.0
        self.view_cy = 0.0
        self.edit_mode = MODE_PAN
        self.drag_wp = None          # 正在拖拽的航点序号
        self.pan_from = None
        self.obs_draft = None        # 拖出来的障碍物圆心

        # --- 手动模式 ---
        self.manual_z = -DEFAULT_ALT
        self.manual_yaw_rate = 0.0

        # --- 线程与队列 ---
        self._lock = threading.Lock()
        self._rpc_lock = threading.RLock()   # 串行化 AirSim 调用，理由见 LockedClient
        self.cmd_q = queue.Queue()
        self.ui_q = queue.Queue()
        self.stop_evt = threading.Event()
        self.pause_evt = threading.Event()
        self.manual_evt = threading.Event()
        self._runner_on = True
        self._telem_on = True
        self._last_dev_warn = 0.0
        self.recorder = None

        self._build_style()
        self._build_ui()

        self._runner = threading.Thread(target=self._runner_loop, daemon=True)
        self._runner.start()
        self._telem = threading.Thread(target=self._telem_loop, daemon=True)
        self._telem.start()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(80, self._tick)

        self._log("地面站已就绪。")
        if not HAS_AIRSIM:
            self._log("!! 未检测到 airsim 库，只能做离线航线规划。")
            self._log("   安装：pip install airsim==1.8.1（需 Python 3.10）")
            self._set_mode("仅规划")
        else:
            self._log("请先启动 Africa_001.exe，待场景加载完再点「连接」。")

    # =======================================================================
    # 界面搭建
    # =======================================================================

    def _build_style(self):
        st = ttk.Style()
        try:
            st.theme_use("clam")
        except tk.TclError:
            pass
        st.configure("Treeview", background=PANEL2, fieldbackground=PANEL2,
                     foreground=TEXT, borderwidth=0, rowheight=23, font=FONT_S)
        st.configure("Treeview.Heading", background=PANEL, foreground=MUTED,
                     relief="flat", font=FONT_S)
        st.map("Treeview", background=[("selected", ACCENT2)],
               foreground=[("selected", "#FFFFFF")])
        st.map("Treeview.Heading", background=[("active", PANEL2)])
        st.configure("TNotebook", background=PANEL, borderwidth=0, tabmargins=0)
        st.configure("TNotebook.Tab", background=PANEL2, foreground=MUTED,
                     padding=(14, 7), borderwidth=0, font=FONT)
        st.map("TNotebook.Tab", background=[("selected", PANEL)],
               foreground=[("selected", ACCENT)], expand=[("selected", [0, 0, 0, 0])])
        st.configure("TCombobox", fieldbackground=PANEL2, background=PANEL2,
                     foreground=TEXT, arrowcolor=TEXT)
        st.configure("Vertical.TScrollbar", background=PANEL2, troughcolor=PANEL,
                     borderwidth=0, arrowcolor=MUTED)

    def _btn(self, parent, text, cmd, bg=PANEL2, fg=TEXT, font=FONT, **kw):
        b = tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg, font=font,
                      activebackground=ACCENT2, activeforeground="#FFFFFF",
                      relief="flat", bd=0, highlightthickness=0, cursor="hand2",
                      padx=6, pady=5, **kw)
        return b

    def _label(self, parent, text, fg=MUTED, font=FONT, **kw):
        return tk.Label(parent, text=text, bg=parent["bg"], fg=fg, font=font, **kw)

    def _entry_var(self, parent, var, width=7):
        """带变量绑定的输入框。直接返回控件，调用方自己 pack/grid。"""
        return tk.Entry(parent, width=width, textvariable=var, bg=PANEL2, fg=TEXT,
                        font=FONT_S, relief="flat", insertbackground=ACCENT, bd=6,
                        highlightthickness=0)

    def _build_ui(self):
        # ---------------- 顶部工具条 ----------------
        top = tk.Frame(self.root, bg=PANEL, height=54)
        top.pack(side="top", fill="x")
        top.pack_propagate(False)

        self._label(top, "无人机地面站", ACCENT, ("Microsoft YaHei UI", 13, "bold")
                    ).pack(side="left", padx=(14, 4))
        self._label(top, "航线规划与自主巡航", MUTED, FONT_S).pack(side="left", padx=(0, 18))

        self._label(top, "地址").pack(side="left")
        self.var_ip = tk.StringVar(value="127.0.0.1")
        self._entry_var(top, self.var_ip, 13).pack(side="left", padx=(4, 8))

        self._label(top, "端口").pack(side="left")
        self.var_port = tk.StringVar(value="41451")
        self._entry_var(top, self.var_port, 7).pack(side="left", padx=(4, 10))

        self.btn_conn = self._btn(top, "连接", self.on_connect, ACCENT2, "#FFFFFF", FONT_B, width=8)
        self.btn_conn.pack(side="left", padx=(0, 6))
        self.btn_arming = self._btn(top, "解锁电机", self.on_arm, width=9)
        self.btn_arming.pack(side="left", padx=(0, 6))

        self.pill = tk.Label(top, text="未连接", bg=PANEL2, fg=MUTED, font=FONT_B,
                             padx=12, pady=5)
        self.pill.pack(side="right", padx=14)

        self.lbl_hint = self._label(top, "先启动 Africa_001.exe", MUTED, FONT_S)
        self.lbl_hint.pack(side="right", padx=10)

        # ---------------- 底部状态栏 ----------------
        bar = tk.Frame(self.root, bg=PANEL, height=30)
        bar.pack(side="bottom", fill="x")
        bar.pack_propagate(False)
        self.lbl_status = self._label(bar, "等待连接 ...", TEXT, ("Consolas", 9))
        self.lbl_status.pack(side="left", padx=14)

        # ---------------- 主体 ----------------
        body = tk.Frame(self.root, bg=BG)
        body.pack(side="top", fill="both", expand=True)

        # ---- 左侧：地图 ----
        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="both", expand=True)

        mbar = tk.Frame(left, bg=PANEL, height=40)
        mbar.pack(side="top", fill="x")
        mbar.pack_propagate(False)

        self._label(mbar, " 编辑模式").pack(side="left")
        self.var_mode = tk.StringVar(value=MODE_PAN)
        for val, txt in ((MODE_PAN, "选择/平移"), (MODE_WP, "加航点"),
                         (MODE_OBS, "加障碍"), (MODE_DEL, "删除")):
            tk.Radiobutton(mbar, text=txt, value=val, variable=self.var_mode,
                           command=self._on_mode_change, bg=PANEL, fg=TEXT,
                           selectcolor=PANEL2, activebackground=PANEL,
                           activeforeground=ACCENT, font=FONT_S, bd=0,
                           highlightthickness=0, cursor="hand2").pack(side="left", padx=3)

        self.btn_fit = self._btn(mbar, "适配窗口", self.fit_view, width=8)
        self.btn_fit.pack(side="right", padx=(4, 10))
        self.btn_zoom_in = self._btn(mbar, "＋", lambda: self.zoom(1.25), width=2)
        self.btn_zoom_in.pack(side="right", padx=2)
        self.btn_zoom_out = self._btn(mbar, "－", lambda: self.zoom(0.8), width=2)
        self.btn_zoom_out.pack(side="right", padx=2)

        self.var_show_route = tk.BooleanVar(value=True)
        self.var_show_trail = tk.BooleanVar(value=True)
        self.var_show_dense = tk.BooleanVar(value=False)
        tk.Checkbutton(mbar, text="规划航线", variable=self.var_show_route,
                       bg=PANEL, fg=TEXT, selectcolor=PANEL2, activebackground=PANEL,
                       activeforeground=ACCENT, font=FONT_S, bd=0,
                       highlightthickness=0, cursor="hand2").pack(side="right", padx=6)
        tk.Checkbutton(mbar, text="实际轨迹", variable=self.var_show_trail,
                       bg=PANEL, fg=TEXT, selectcolor=PANEL2, activebackground=PANEL,
                       activeforeground=ACCENT, font=FONT_S, bd=0,
                       highlightthickness=0, cursor="hand2").pack(side="right", padx=6)
        tk.Checkbutton(mbar, text="A* 原始路径", variable=self.var_show_dense,
                       bg=PANEL, fg=TEXT, selectcolor=PANEL2, activebackground=PANEL,
                       activeforeground=ACCENT, font=FONT_S, bd=0,
                       highlightthickness=0, cursor="hand2").pack(side="right", padx=6)

        self.canvas = tk.Canvas(left, bg="#121018", highlightthickness=0)
        self.canvas.pack(side="top", fill="both", expand=True)
        self.canvas.bind("<Button-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Button-3>", self.on_right_click)
        self.canvas.bind("<MouseWheel>", self.on_wheel)
        self.canvas.bind("<Motion>", self.on_motion)

        # ---- 右侧：控制面板 ----
        right = tk.Frame(body, bg=PANEL, width=398)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)
        right.grid_rowconfigure(0, weight=5, minsize=330)
        right.grid_rowconfigure(1, weight=0)
        right.grid_rowconfigure(2, weight=2, minsize=120)
        right.grid_columnconfigure(0, weight=1)

        nb = ttk.Notebook(right)
        nb.grid(row=0, column=0, sticky="nsew", padx=6, pady=(6, 4))
        self._tab_waypoints(nb)
        self._tab_planning(nb)
        self._tab_manual(nb)

        self._build_exec_panel(right)

        # ---- 日志 ----
        logf = tk.Frame(right, bg=PANEL)
        logf.grid(row=2, column=0, sticky="nsew", padx=6, pady=(0, 6))
        self._label(logf, "运行日志", MUTED, FONT_S).pack(anchor="w")
        wrap = tk.Frame(logf, bg=LINE, bd=0)
        wrap.pack(fill="both", expand=True)
        self.txt_log = tk.Text(wrap, bg="#100E16", fg="#B9B1C9", font=("Consolas", 8),
                               relief="flat", wrap="word", bd=6, height=6,
                               insertbackground=ACCENT, state="disabled")
        self.txt_log.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(wrap, orient="vertical", command=self.txt_log.yview)
        sb.pack(side="right", fill="y")
        self.txt_log.configure(yscrollcommand=sb.set)
        for tag, col in (("info", "#B9B1C9"), ("ok", OK), ("warn", WARN),
                         ("err", DANGER), ("hi", ACCENT)):
            self.txt_log.tag_configure(tag, foreground=col)

    # -----------------------------------------------------------------------
    # Tab 1：航点表
    # -----------------------------------------------------------------------

    def _tab_waypoints(self, nb):
        f = tk.Frame(nb, bg=PANEL)
        nb.add(f, text="  航点  ")

        self.wp_tree = ttk.Treeview(f, columns=("i", "x", "y", "alt", "spd", "hold"),
                                    show="headings", height=7, selectmode="browse")
        for col, txt, w in (("i", "#", 30), ("x", "X北 m", 60), ("y", "Y东 m", 60),
                            ("alt", "高度 m", 60), ("spd", "速度", 50), ("hold", "悬停 s", 55)):
            self.wp_tree.heading(col, text=txt)
            self.wp_tree.column(col, width=w, anchor="center")
        self.wp_tree.pack(fill="both", expand=True, padx=6, pady=(6, 4))
        self.wp_tree.bind("<<TreeviewSelect>>", self._on_wp_select)

        # 编辑表单
        form = tk.Frame(f, bg=PANEL)
        form.pack(fill="x", padx=6)
        self.wp_vars = {}
        for key, txt, dflt in (("x", "X北", "0"), ("y", "Y东", "0"),
                               ("alt", "高度", str(DEFAULT_ALT)),
                               ("spd", "速度", str(DEFAULT_SPD)), ("hold", "悬停", "0")):
            cell = tk.Frame(form, bg=PANEL)
            cell.pack(side="left", padx=(0, 5))
            self._label(cell, txt, MUTED, FONT_S).pack(anchor="w")
            v = tk.StringVar(value=dflt)
            self.wp_vars[key] = v
            self._entry_var(cell, v, 6).pack()

        btns = tk.Frame(f, bg=PANEL)
        btns.pack(fill="x", padx=6, pady=6)
        for txt, cmd in (("添加", self.wp_add), ("更新选中", self.wp_update),
                         ("删除", self.wp_delete)):
            self._btn(btns, txt, cmd, width=7).pack(side="left", padx=(0, 4))
        self._btn(btns, "清空", self.wp_clear, width=5).pack(side="left")

        btns2 = tk.Frame(f, bg=PANEL)
        btns2.pack(fill="x", padx=6, pady=(0, 6))
        for txt, cmd in (("上移", lambda: self.wp_move(-1)),
                         ("下移", lambda: self.wp_move(1)),
                         ("反向", self.wp_reverse),
                         ("示范航线", self.load_demo)):
            self._btn(btns2, txt, cmd, width=7).pack(side="left", padx=(0, 4))
        self._btn(btns2, "保存", self.save_route, width=5).pack(side="left", padx=(0, 4))
        self._btn(btns2, "载入", self.load_route, width=5).pack(side="left")

        self._label(f, "提示：地图上「加航点」模式直接点击即可落点；"
                       "点住航点可拖动；右键删除。", MUTED, FONT_S
                    ).pack(anchor="w", padx=8, pady=(0, 6))

    # -----------------------------------------------------------------------
    # Tab 2：规划参数
    # -----------------------------------------------------------------------

    def _tab_planning(self, nb):
        f = tk.Frame(nb, bg=PANEL)
        nb.add(f, text="  规划  ")

        self._label(f, "障碍物（圆柱，XY 平面）", TEXT, FONT_B).pack(anchor="w", padx=8, pady=(8, 2))
        self.obs_tree = ttk.Treeview(f, columns=("i", "x", "y", "r"),
                                     show="headings", height=5, selectmode="browse")
        for col, txt, w in (("i", "#", 30), ("x", "X北 m", 70), ("y", "Y东 m", 70),
                            ("r", "半径 m", 70)):
            self.obs_tree.heading(col, text=txt)
            self.obs_tree.column(col, width=w, anchor="center")
        self.obs_tree.pack(fill="x", padx=6)
        ob = tk.Frame(f, bg=PANEL)
        ob.pack(fill="x", padx=6, pady=6)
        self._btn(ob, "添加示例障碍", self.obs_add_demo, width=12).pack(side="left", padx=(0, 4))
        self._btn(ob, "删除选中", self.obs_delete, width=8).pack(side="left", padx=(0, 4))
        self._btn(ob, "清空", self.obs_clear, width=5).pack(side="left")

        par = tk.Frame(f, bg=PANEL)
        par.pack(fill="x", padx=8, pady=(2, 6))
        self._label(par, "栅格边长 m", MUTED, FONT_S).grid(row=0, column=0, sticky="w")
        self.var_res = tk.StringVar(value="0.5")
        self._entry_var(par, self.var_res, 5).grid(row=1, column=0, padx=(0, 10))
        self._label(par, "安全间距 m", MUTED, FONT_S).grid(row=0, column=1, sticky="w")
        self.var_clr = tk.StringVar(value="0.8")
        self._entry_var(par, self.var_clr, 5).grid(row=1, column=1)

        pb = tk.Frame(f, bg=PANEL)
        pb.pack(fill="x", padx=6, pady=(4, 6))
        self.btn_plan = self._btn(pb, "规划航线", self.on_plan, ACCENT2, "#FFFFFF", FONT_B, width=9)
        self.btn_plan.pack(side="left", padx=(0, 4))
        self._btn(pb, "上传到仿真器", self.on_upload, width=11).pack(side="left", padx=(0, 4))
        self._btn(pb, "清除标记", self.on_clear_markers, width=8).pack(side="left")

        self.lbl_plan = self._label(f, "尚未规划。地图上点几个航点，再按「规划航线」。",
                                    MUTED, FONT_S, justify="left", wraplength=350)
        self.lbl_plan.pack(anchor="w", padx=8, pady=(0, 6))

    # -----------------------------------------------------------------------
    # Tab 3：手动操控
    # -----------------------------------------------------------------------

    def _tab_manual(self, nb):
        f = tk.Frame(nb, bg=PANEL)
        nb.add(f, text="  手动  ")

        self.btn_manual = self._btn(f, "进入手动模式", self.on_manual_toggle,
                                    ACCENT2, "#FFFFFF", FONT_B)
        self.btn_manual.pack(fill="x", padx=8, pady=(10, 8))

        box = tk.Frame(f, bg=PANEL2)
        box.pack(fill="x", padx=8, pady=(0, 8))
        self._label(box, "距规划航线", MUTED, FONT_S).pack(pady=(8, 0))
        self.lbl_dev = tk.Label(box, text="--  m", bg=PANEL2, fg=OK, font=FONT_XL)
        self.lbl_dev.pack(pady=(0, 2))
        self.lbl_dev_side = tk.Label(box, text="尚未进入手动模式", bg=PANEL2,
                                     fg=MUTED, font=FONT_S)
        self.lbl_dev_side.pack(pady=(0, 8))

        keys = ("W 前进（北）    S 后退（南）\n"
                "A 左移（西）    D 右移（东）\n"
                "R 上升          F 下降\n"
                "Q 左转          E 右转\n"
                "空格 原地悬停（松开所有键也可）")
        self._label(f, keys, TEXT, ("Consolas", 9), justify="left"
                    ).pack(anchor="w", padx=12, pady=(0, 6))

        sp = tk.Frame(f, bg=PANEL)
        sp.pack(fill="x", padx=12)
        self._label(sp, "手动速度 m/s").pack(side="left")
        self.var_man_speed = tk.StringVar(value="2.0")
        self._entry_var(sp, self.var_man_speed, 5).pack(side="left", padx=6)

        self._label(f, "手动模式下飞机沿规划航线飞，\n偏差超过 %.0f 米会告警。"
                       % DEV_WARN, MUTED, FONT_S, justify="left"
                    ).pack(anchor="w", padx=12, pady=(8, 0))

    # -----------------------------------------------------------------------
    # 执行区（常驻，不藏在 Tab 里）
    # -----------------------------------------------------------------------

    def _build_exec_panel(self, parent):
        f = tk.Frame(parent, bg=PANEL)
        f.grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 4))

        tk.Frame(f, bg=LINE, height=1).pack(fill="x", pady=(0, 6))
        self._label(f, "飞行执行", TEXT, FONT_B).pack(anchor="w", padx=2)

        par = tk.Frame(f, bg=PANEL)
        par.pack(fill="x", pady=6)
        self._label(par, "巡航高度 m").pack(side="left")
        self.var_alt = tk.StringVar(value=str(DEFAULT_ALT))
        self._entry_var(par, self.var_alt, 5).pack(side="left", padx=(4, 12))
        self._label(par, "巡航速度 m/s").pack(side="left")
        self.var_speed = tk.StringVar(value=str(DEFAULT_SPD))
        self._entry_var(par, self.var_speed, 5).pack(side="left", padx=4)

        self.var_use_path = tk.BooleanVar(value=False)
        tk.Checkbutton(f, text="整段用 moveOnPathAsync 飞（任务书演示模式）",
                       variable=self.var_use_path, bg=PANEL, fg=MUTED,
                       selectcolor=PANEL2, activebackground=PANEL,
                       activeforeground=ACCENT, font=FONT_S, bd=0,
                       highlightthickness=0, cursor="hand2"
                       ).pack(anchor="w", padx=2)

        g = tk.Frame(f, bg=PANEL)
        g.pack(fill="x", pady=6)
        for i in range(4):
            g.grid_columnconfigure(i, weight=1)

        specs = [
            ("起飞", self.on_takeoff, OK, 0, 0),
            ("自动巡航", self.on_cruise, ACCENT2, 0, 1),
            ("暂停/继续", self.on_pause, WARN, 0, 2),
            ("停止", self.on_stop, PANEL2, 0, 3),
            ("返航降落", self.on_rtl, PANEL2, 1, 0),
            ("降落", self.on_land, PANEL2, 1, 1),
            ("就地悬停", self.on_hold, PANEL2, 1, 2),
            ("急停", self.on_estop, DANGER, 1, 3),
        ]
        for txt, cmd, bg, r, c in specs:
            fg = "#FFFFFF" if bg in (ACCENT2, DANGER, OK) else TEXT
            self._btn(g, txt, cmd, bg, fg, FONT_B if txt == "急停" else FONT).grid(
                row=r, column=c, sticky="ew", padx=2, pady=2, ipady=1)

    # =======================================================================
    # 日志 / 状态
    # =======================================================================

    def _log(self, msg, tag="info"):
        stamp = time.strftime("%H:%M:%S")
        self.ui_q.put(("log", ("[%s] %s" % (stamp, msg), tag)))

    def _set_mode(self, m):
        """切换模式。

        注意：本方法会被后台线程调用，所以只改普通属性 + 往队列里塞消息，
        真正碰控件（pill）的动作留给界面线程做。Tk 不是线程安全的。
        """
        self.mode = m
        self.ui_q.put(("mode", m))

    # =======================================================================
    # 连接与基础飞行（都丢到后台线程执行，界面不卡）
    # =======================================================================

    def _submit(self, fn, *args):
        self.cmd_q.put((fn, args))

    def _runner_loop(self):
        """后台指令线程：所有会阻塞的 AirSim 调用都在这里排成一队执行。"""
        while self._runner_on:
            try:
                fn, args = self.cmd_q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                fn(*args)
            except Exception as exc:
                self._log("指令执行出错：%s" % exc, "err")
                self._set_mode("异常")

    def _telem_loop(self):
        """遥测线程：固定 20 Hz 采样，写 CSV，并把数据丢给界面线程显示。

        整个循环体包在 try 里——采样线程一旦抛异常退出，状态栏和轨迹就会
        永远停在最后一帧，而界面上看不出任何报错，排查起来非常费劲。
        """
        while self._telem_on:
            try:
                self._telem_once()
            except Exception as exc:
                self.ui_q.put(("log", ("[遥测异常] %s" % exc, "err")))
                time.sleep(0.2)

    def _telem_once(self):
        """采一拍。"""
        if not self.connected or self.client is None:
            time.sleep(0.2)
            return

        try:
            s = self.client.getMultirotorState(vehicle_name="")
            k = s.kinematics_estimated
            roll, pitch, yaw = airsim.to_eularian_angles(k.orientation)
            x, y, z = k.position.x_val, k.position.y_val, k.position.z_val
            row = {
                "x": x, "y": y, "z": z, "alt": -z,
                "vx": k.linear_velocity.x_val,
                "vy": k.linear_velocity.y_val,
                "vz": k.linear_velocity.z_val,
                "roll": round(math.degrees(roll), 2),
                "pitch": round(math.degrees(pitch), 2),
                "yaw": round(math.degrees(yaw), 2),
            }
        except Exception:
            time.sleep(0.1)          # 仿真器短暂无响应，跳过这一拍
            return

        # 到规划航线的横向偏差：手动模式靠它告警，状态栏也一直在显示
        if len(self.route_xy) >= 2:
            dev = deviation_to_path(x, y, self.route_xy, self.route_cum)
        else:
            dev = {"dist": 0.0, "progress": 0.0, "side": ""}
        row["dev"] = round(dev["dist"], 3)
        row["progress"] = dev["progress"]
        row["wp"] = self.cruise_index

        with self._lock:
            self.telem = row
            self.trail.append((x, y, z))
            if len(self.trail) > 4000:   # 只留最近的一段，画布才画得动
                del self.trail[:2000]

        recorder = self.recorder
        if recorder is not None:
            recorder.write({**row, "mode": self.mode})
        self.ui_q.put(("telem", row))

        # 偏差告警，限流 3 秒一次，免得刷屏
        if self.mode == "手动" and dev["dist"] > DEV_WARN:
            now = time.time()
            if now - self._last_dev_warn > 3.0:
                self._last_dev_warn = now
                self._log("偏离规划航线 %.1f 米（阈值 %.0f 米），偏在%s侧"
                          % (dev["dist"], DEV_WARN, dev["side"] or "中"), "warn")

        # 碰撞检测：约每 0.5 秒查一次
        if int(time.time() * 2) % 10 == 0:
            try:
                c = self.client.simGetCollisionInfo(vehicle_name="")
                if c.has_collided:
                    self.ui_q.put(("collision", c.object_name))
            except Exception:
                pass
        time.sleep(0.05)

    def on_connect(self):
        if not HAS_AIRSIM:
            messagebox.showerror("缺少依赖",
                                 "未安装 airsim 库，无法连接仿真器。\n\n"
                                 "请用 Python 3.10 执行：\npip install numpy\npip install airsim==1.8.1")
            return
        if self.connected:
            self._submit(self._job_disconnect)
            return
        ip, port = self.var_ip.get().strip(), self.var_port.get().strip()
        self.btn_conn.configure(text="连接中…", state="disabled")
        self._log("正在连接 %s:%s ..." % (ip, port))
        self._submit(self._job_connect, ip, port)

    def _job_connect(self, ip, port):
        self.client = LockedClient(airsim.MultirotorClient(ip=ip, port=int(port)),
                                   self._rpc_lock)
        self.client.confirmConnection()
        self.connected = True
        self.home = self._pos()
        self.ui_q.put(("connected", True))
        self._log("已连接仿真器（客户端 v%d / 服务端 v%d）"
                  % (self.client.getClientVersion(), self.client.getServerVersion()), "ok")
        self.recorder = FlightRecorder()
        path = self.recorder.start()
        self._log("开始记录飞行数据 -> %s" % os.path.basename(path), "ok")

    def _job_disconnect(self):
        self.stop_evt.set()
        self.manual_evt.clear()
        try:
            if self.flying:
                self.client.landAsync(timeout_sec=60).join()
                self.flying = False
            self.client.armDisarm(False)
            self.client.enableApiControl(False)
        except Exception:
            pass
        self.armed = False
        self.connected = False
        if self.recorder is not None:
            self.recorder.stop()
            self.recorder = None
        self.ui_q.put(("connected", False))
        self._log("已断开连接。")

    def on_arm(self):
        if not self._require_conn():
            return
        self._submit(self._job_arm)

    def _job_arm(self):
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        time.sleep(1.0)
        self.armed = True
        self._log("API 控制已打开，电机已解锁。", "ok")

    def _pos(self):
        p = self.client.getMultirotorState().kinematics_estimated.position
        return (p.x_val, p.y_val, p.z_val)

    def _require_conn(self):
        if not self.connected:
            messagebox.showwarning("未连接", "请先点「连接」连上 AirSim 仿真器。\n\n"
                                             "确认已经双击启动 Africa_001.exe 并且场景加载完毕。")
            return False
        return True

    def _num(self, var, default):
        try:
            return float(var.get())
        except (ValueError, TypeError):
            return default

    # =======================================================================
    # 航点编辑
    # =======================================================================

    def _add_waypoint(self, x, y, z=None):
        z = -self._num(self.var_alt, DEFAULT_ALT) if z is None else z
        self.waypoints.append(Waypoint(x, y, z, self._num(self.var_speed, DEFAULT_SPD)))
        self._refresh_wp_tree()

    @staticmethod
    def _wp_row(i, w):
        return (i + 1, "%.1f" % w.x, "%.1f" % w.y,
                "%.1f" % w.altitude, "%.1f" % w.speed, "%.1f" % w.hold)

    def _update_wp_row(self, i):
        """只刷一行。拖航点时会一帧一次地调，重建整张表会卡。"""
        self.wp_tree.item(str(i), values=self._wp_row(i, self.waypoints[i]))

    def _refresh_wp_tree(self):
        sel = self.wp_tree.selection()
        keep = sel[0] if sel else None
        self.wp_tree.delete(*self.wp_tree.get_children())
        for i, w in enumerate(self.waypoints):
            self.wp_tree.insert("", "end", iid=str(i), values=self._wp_row(i, w))
        if keep is not None and keep in self.wp_tree.get_children():
            self.wp_tree.selection_set(keep)
        self._invalidate_route()

    def _refresh_obs_tree(self):
        self.obs_tree.delete(*self.obs_tree.get_children())
        for i, o in enumerate(self.obstacles):
            self.obs_tree.insert("", "end", iid=str(i),
                                 values=(i + 1, "%.1f" % o.x, "%.1f" % o.y, "%.1f" % o.r))
        self._invalidate_route()

    def _invalidate_route(self):
        """航点或障碍一改，之前规划出来的航线就作废了。"""
        if self.route:
            self.route = []
            self.route_xy = []
            self.route_cum = []
            self.route_dense = []
            self.lbl_plan.configure(text="航线已失效（航点或障碍发生了变化），请重新规划。",
                                    fg=WARN)

    def _selected_wp(self):
        sel = self.wp_tree.selection()
        if not sel:
            return None
        try:
            return int(sel[0])
        except ValueError:
            return None

    def _on_wp_select(self, _evt=None):
        i = self._selected_wp()
        if i is None or i >= len(self.waypoints):
            return
        w = self.waypoints[i]
        self.wp_vars["x"].set("%.1f" % w.x)
        self.wp_vars["y"].set("%.1f" % w.y)
        self.wp_vars["alt"].set("%.1f" % w.altitude)
        self.wp_vars["spd"].set("%.1f" % w.speed)
        self.wp_vars["hold"].set("%.1f" % w.hold)

    def wp_add(self):
        self._add_waypoint(self._num(self.wp_vars["x"], 0.0),
                           self._num(self.wp_vars["y"], 0.0),
                           -abs(self._num(self.wp_vars["alt"], DEFAULT_ALT)))
        self._log("已按表格数值添加航点。")

    def wp_update(self):
        i = self._selected_wp()
        if i is None:
            messagebox.showinfo("未选中", "请先在上面的航点表里点一行。")
            return
        w = self.waypoints[i]
        w.x = self._num(self.wp_vars["x"], w.x)
        w.y = self._num(self.wp_vars["y"], w.y)
        w.z = -abs(self._num(self.wp_vars["alt"], w.altitude))
        w.speed = self._num(self.wp_vars["spd"], w.speed)
        w.hold = self._num(self.wp_vars["hold"], w.hold)
        self._refresh_wp_tree()
        self.wp_tree.selection_set(str(i))
        self._log("已更新航点 %d。" % (i + 1))

    def wp_delete(self):
        i = self._selected_wp()
        if i is None:
            return
        del self.waypoints[i]
        self._refresh_wp_tree()
        self._log("已删除航点 %d。" % (i + 1))

    def wp_clear(self):
        if self.waypoints and not messagebox.askyesno("清空", "确定清空全部航点？"):
            return
        self.waypoints.clear()
        self._refresh_wp_tree()

    def wp_move(self, delta):
        i = self._selected_wp()
        if i is None:
            return
        j = i + delta
        if not (0 <= j < len(self.waypoints)):
            return
        self.waypoints[i], self.waypoints[j] = self.waypoints[j], self.waypoints[i]
        self._refresh_wp_tree()
        self.wp_tree.selection_set(str(j))

    def wp_reverse(self):
        if len(self.waypoints) < 2:
            return
        self.waypoints.reverse()
        self._refresh_wp_tree()
        self._log("航点顺序已反向。")

    def load_demo(self):
        self.waypoints = demo_waypoints()
        if not self.obstacles:
            self.obstacles = demo_obstacles()
            self._refresh_obs_tree()
        self._refresh_wp_tree()
        self.fit_view()
        self._log("已载入示范航线（5 个航点 + 2 个障碍物），可直接按「规划航线」。", "hi")

    def save_route(self):
        path = filedialog.asksaveasfilename(
            title="保存航线", defaultextension=".json",
            filetypes=[("航线文件", "*.json")], initialfile="route.json")
        if not path:
            return
        data = {"waypoints": [w.to_dict() for w in self.waypoints],
                "obstacles": [o.to_dict() for o in self.obstacles]}
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(data, fh, ensure_ascii=False, indent=2)
        self._log("航线已保存 -> %s" % os.path.basename(path), "ok")

    def load_route(self):
        path = filedialog.askopenfilename(title="载入航线",
                                          filetypes=[("航线文件", "*.json")])
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            self.waypoints = [Waypoint.from_dict(d) for d in data.get("waypoints", [])]
            self.obstacles = [Obstacle.from_dict(d) for d in data.get("obstacles", [])]
        except Exception as exc:
            messagebox.showerror("载入失败", "文件读不出来：%s" % exc)
            return
        self._refresh_wp_tree()
        self._refresh_obs_tree()
        self.fit_view()
        self._log("已载入航线：%d 航点 / %d 障碍。" % (len(self.waypoints), len(self.obstacles)), "ok")

    # =======================================================================
    # 障碍物
    # =======================================================================

    def _add_obstacle(self, x, y, r):
        self.obstacles.append(Obstacle(x, y, r))
        self._refresh_obs_tree()

    def obs_add_demo(self):
        self.obstacles.extend(demo_obstacles())
        self._refresh_obs_tree()
        self._log("已添加 2 个示例障碍（正好挡住示范航线，用来演示自动绕行）。", "hi")

    def obs_delete(self):
        sel = self.obs_tree.selection()
        if not sel:
            return
        del self.obstacles[int(sel[0])]
        self._refresh_obs_tree()

    def obs_clear(self):
        self.obstacles.clear()
        self._refresh_obs_tree()

    # =======================================================================
    # 规划
    # =======================================================================

    def on_plan(self):
        self._submit(self._job_plan,
                     self._num(self.var_res, 0.5), self._num(self.var_clr, 0.8),
                     self._num(self.var_alt, DEFAULT_ALT))

    def _job_plan(self, res=0.5, clr=0.8, alt=DEFAULT_ALT):
        if len(self.waypoints) == 0:
            self._log("还没有航点，先在地图上点几个（或按「示范航线」）。", "warn")
            return

        # 起点必须取「起飞之后」的位置。飞机停在地面时 z 约等于 0，
        # 直接拿它当起点，会规划出一条「先俯冲到地面」的航线。
        if self.connected and self.flying:
            start = self._pos()
        elif self.connected:
            x, y, _ = self._pos()
            start = (x, y, -abs(alt))
        else:
            start = (0.0, 0.0, -abs(alt))

        t0 = time.time()
        route, dense, info = plan_route(start, self.waypoints, self.obstacles,
                                        resolution=max(res, 0.2), clearance=max(clr, 0.0))
        self.route = route
        self.route_xy = [(p[0], p[1]) for p in route]
        self.route_cum = cumulative_lengths(route)
        self.route_dense = dense
        self.info = info

        msg = ("规划完成：%.1f 米 / %d 个折点（障碍 %d 个，耗时 %.2f 秒）"
               % (info["length"], len(route), len(self.obstacles), time.time() - t0))
        self._log(msg, "ok")
        for w in info["warnings"]:
            self._log(w, "warn")

        def upd():
            self.lbl_plan.configure(
                text=("航线总长 %.1f 米，共 %d 个折点。\n"
                      "各段长度：%s\n"
                      "A* 原始路径 %d 点，拉直后 %d 点。"
                      % (info["length"], len(route),
                         " / ".join("%.1f" % L for L in info["legs"]),
                         len(dense), len(route))),
                fg=OK)
        self.ui_q.put(("call", upd))

    def on_upload(self):
        if not self._require_conn():
            return
        if not self.route:
            messagebox.showinfo("未规划", "请先按「规划航线」。")
            return
        self._submit(self._job_upload)

    def _job_upload(self):
        """把规划航线和航点画进仿真器，现场演示时对照实际轨迹非常直观。"""
        self.client.simFlushPersistentMarkers()
        pts = [airsim.Vector3r(x, y, z) for (x, y, z) in self.route]
        self.client.simPlotLineStrip(pts, color_rgba=[0.573, 0.153, 0.561, 1.0],
                                     thickness=6.0, duration=-1.0, is_persistent=True)
        wps = [airsim.Vector3r(w.x, w.y, w.z) for w in self.waypoints]
        self.client.simPlotPoints(wps, color_rgba=[0.69, 0.42, 0.96, 1.0],
                                  size=18.0, duration=-1.0, is_persistent=True)
        for o in self.obstacles:
            self.client.simPlotPoints([airsim.Vector3r(o.x, o.y, 0.0)],
                                      color_rgba=[0.97, 0.44, 0.44, 1.0],
                                      size=max(10.0, o.r * 40), duration=-1.0,
                                      is_persistent=True)
        self._log("已把航线、航点、障碍位置画进仿真器。", "ok")

    def on_clear_markers(self):
        if not self._require_conn():
            return
        self._submit(lambda: (self.client.simFlushPersistentMarkers(),
                              self._log("已清除仿真器里的标记。", "ok")))

    # =======================================================================
    # 飞行执行
    # =======================================================================

    def on_takeoff(self):
        if not self._require_conn():
            return
        self._submit(self._job_takeoff, self._num(self.var_alt, DEFAULT_ALT))

    def _job_takeoff(self, alt=DEFAULT_ALT):
        alt = abs(alt)
        if not self.armed:
            self._job_arm()
        self.home = self._pos()
        self._log("起飞，爬升到 %.1f 米 ..." % alt)
        self.client.takeoffAsync(timeout_sec=20).join()
        self.client.moveToZAsync(-alt, 2.0).join()
        self.flying = True
        self.manual_z = -alt
        self._log("已到达 %.1f 米，可以开始巡航。" % alt, "ok")

    def on_cruise(self):
        if not self._require_conn():
            return
        # 控件值一律在界面线程读好再传进后台，后台线程不碰 Tk
        self._submit(self._job_cruise,
                     self._num(self.var_speed, DEFAULT_SPD),
                     self.var_use_path.get(),
                     self._num(self.var_alt, DEFAULT_ALT),
                     self._num(self.var_res, 0.5),
                     self._num(self.var_clr, 0.8))

    def _job_cruise(self, speed=DEFAULT_SPD, use_path=False,
                    alt=DEFAULT_ALT, res=0.5, clr=0.8):
        if not self.flying:
            self._job_takeoff(alt)
        if not self.route:
            self._log("还没规划航线，先自动规划一次。")
            self._job_plan(res, clr, alt)
        if not self.route:
            self._log("航线规划失败，巡航中止。", "err")
            return

        self.stop_evt.clear()
        self.pause_evt.clear()
        self.manual_evt.clear()
        speed = max(speed, 0.5)
        self._set_mode("自动巡航")

        if use_path:
            self._cruise_path_api(speed)
        else:
            self._cruise_waypoint_by_waypoint(speed)

        if self.stop_evt.is_set():
            self._log("巡航已停止。", "warn")
        elif not use_path:
            self._log("巡航完成，已到达航线终点并悬停。", "ok")
            self._set_mode("已连接")

    def _nearest_route_index(self):
        """当前位置最接近航线上的第几个折点，用于暂停后从断点续飞。"""
        p = self._pos()
        best, best_d = 0, float("inf")
        for i, (x, y, z) in enumerate(self.route):
            d = math.dist(p, (x, y, z))
            if d < best_d:
                best, best_d = i, d
        return best

    def _cruise_path_api(self, speed):
        """任务书要求的做法：一次性把整条路径交给 moveOnPathAsync。"""
        total = self.info.get("length", 0.0)
        timeout = total / speed * 4 + 60
        goal = self.route[-1]

        while True:
            # 暂停恢复后，从最近的折点接着往后续飞，而不是从头再飞一遍
            i0 = self._nearest_route_index()
            pts = [airsim.Vector3r(x, y, z) for (x, y, z) in self.route[i0:]]
            if not pts:
                break
            self._log("moveOnPathAsync 下发 %d 个路径点（自第 %d 点起），"
                      "总长 %.1f 米，速度 %.1f m/s。" % (len(pts), i0 + 1, total, speed), "hi")
            self.client.moveOnPathAsync(pts, speed, timeout_sec=timeout,
                                        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                                        yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0.0),
                                        lookahead=-1, adaptive_lookahead=1)
            res = self._poll(lambda p: math.dist(p, goal) < POSITION_EPS, timeout=timeout)
            if res != "resume":
                break

        if res == "timeout":
            self._log("整段巡航超时，飞行可能被障碍或风力影响，请检查。", "warn")
            self.client.cancelLastTask()
        self.client.hoverAsync().join()
        if not self.stop_evt.is_set():
            self._log("已到达终点并悬停。", "ok")

    def _cruise_waypoint_by_waypoint(self, speed):
        """逐点巡航：每个折点单独下发，因此可以随时暂停、停止、看进度。"""
        n = len(self.route)
        for i, (x, y, z) in enumerate(self.route):
            if self.stop_evt.is_set():
                return
            self.cruise_index = i
            wp_speed = speed
            hold = 0.0
            # 如果这个折点正好是一个用户航点，就用它自己的速度和悬停时间
            for w in self.waypoints:
                if abs(w.x - x) < 1e-6 and abs(w.y - y) < 1e-6:
                    wp_speed = w.speed
                    hold = w.hold
                    break

            self._log("飞往折点 %d/%d  (%.1f, %.1f, 高度 %.1f)" % (i + 1, n, x, y, -z))
            # 暂停恢复后要重新下发「同一个」折点：飞机被 cancelLastTask 停在半路，
            # 直接进下一个折点就等于跳过一个，航线会走形。
            while True:
                self.client.moveToPositionAsync(
                    x, y, z, wp_speed, timeout_sec=120,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0.0))
                res = self._poll(lambda p, t=(x, y, z): math.dist(p, t) < POSITION_EPS,
                                 timeout=120)
                if res != "resume":
                    break
            if res == "stop":
                return
            if res == "timeout":
                self._log("折点 %d 等待超时，继续下一段。" % (i + 1), "warn")
            if hold > 0:
                self._log("到点悬停 %.1f 秒。" % hold)
                self.client.hoverAsync()
                if self._sleep_watch(hold) == "stop":
                    return
        self.cruise_index = -1

    def _poll(self, cond, timeout=60.0):
        """轮询直到条件满足；期间响应「停止」与「暂停」。

        用轮询而不是 .join() 是有意的：join 会把后台线程卡死，
        暂停按钮就再也点不动了。轮询的代价只是 10 Hz 的位置读取。
        """
        t0 = time.time()
        while True:
            if self.stop_evt.is_set():
                self.client.cancelLastTask()
                self.client.hoverAsync()
                return "stop"
            if self.pause_evt.is_set():
                self.client.cancelLastTask()
                self.client.hoverAsync()
                self._log("已暂停，飞机原地悬停。", "warn")
                while self.pause_evt.is_set() and not self.stop_evt.is_set():
                    time.sleep(0.1)
                if self.stop_evt.is_set():
                    self.client.cancelLastTask()
                    self.client.hoverAsync()
                    return "stop"
                self._log("继续巡航。", "ok")
                return "resume"
            try:
                if cond(self._pos()):
                    return "ok"
            except Exception:
                pass
            if time.time() - t0 > timeout:
                return "timeout"
            time.sleep(0.1)

    def _sleep_watch(self, seconds):
        """分片睡眠，期间依然响应停止 / 暂停。返回 "stop" 或 "ok"。

        暂停期间不计入悬停时长——否则「按暂停看一眼，一放开悬停已经结束了」。
        """
        t0 = time.time()
        while True:
            if self.stop_evt.is_set():
                return "stop"
            if self.pause_evt.is_set():
                self._log("已暂停（到点悬停中）。", "warn")
                while self.pause_evt.is_set() and not self.stop_evt.is_set():
                    time.sleep(0.1)
                if self.stop_evt.is_set():
                    return "stop"
                self._log("继续巡航。", "ok")
                t0 = time.time()
                continue
            if time.time() - t0 >= seconds:
                return "ok"
            time.sleep(0.1)

    def on_pause(self):
        if not self.connected:
            return
        if self.pause_evt.is_set():
            self.pause_evt.clear()
            self._log("已恢复。")
        else:
            self.pause_evt.set()
            self._log("暂停请求已发出。")

    def on_stop(self):
        if not self.connected:
            return
        self.stop_evt.set()
        self.manual_evt.clear()
        self._set_mode("已连接")
        self._log("停止请求已发出，飞机会就近悬停。")

    def on_estop(self):
        """急停：立刻打断当前任务并原地悬停，不清除航线、不降落。"""
        if not self.connected:
            return
        self.stop_evt.set()
        self.pause_evt.clear()
        self.manual_evt.clear()
        self._set_mode("已连接")
        self._submit(self._job_estop)

    def _job_estop(self):
        try:
            self.client.cancelLastTask()
            self.client.hoverAsync().join()
        except Exception:
            pass
        self._log("已急停，飞机原地悬停。", "err")

    def on_hold(self):
        if not self._require_conn():
            return
        self._submit(lambda: (self.client.cancelLastTask(), self.client.hoverAsync(),
                              self._log("已就地悬停。", "ok")))

    def on_rtl(self):
        if not self._require_conn():
            return
        if self.home is None:
            messagebox.showinfo("未知起点", "还没有起飞过，不知道返航点在哪。")
            return
        self._submit(self._job_rtl, self._num(self.var_alt, DEFAULT_ALT),
                     self._num(self.var_speed, DEFAULT_SPD))

    def _job_rtl(self, alt=DEFAULT_ALT, speed=DEFAULT_SPD):
        self.stop_evt.clear()
        self.pause_evt.clear()
        self.manual_evt.clear()
        self._set_mode("自动巡航")
        hx, hy, _ = self.home
        alt = abs(alt)
        self._log("返航至起飞点 (%.1f, %.1f) ..." % (hx, hy))
        self.client.moveToPositionAsync(hx, hy, -alt, max(speed, 0.5), timeout_sec=180)
        res = self._poll(lambda p: math.dist(p[:2], (hx, hy)) < 1.0, timeout=180)
        if res == "stop":
            return
        self._log("已到达起飞点上空，开始降落。", "ok")
        self._job_land()

    def on_land(self):
        if not self._require_conn():
            return
        self._submit(self._job_land)

    def _job_land(self):
        self.stop_evt.set()          # 让可能在跑的巡航循环退出去
        try:
            self.client.cancelLastTask()
        except Exception:
            pass
        self._log("降落中 ...")
        self.client.landAsync(timeout_sec=60).join()
        self.client.armDisarm(False)
        self.client.enableApiControl(False)
        self.armed = False
        self.flying = False
        self.cruise_index = -1
        self._set_mode("已连接")
        self._log("已落地并锁桨。", "ok")

    # =======================================================================
    # 手动模式
    # =======================================================================

    def on_manual_toggle(self):
        if self.manual_evt.is_set():
            self.manual_evt.clear()
            self._log("退出手动模式。")
            return
        if not self._require_conn():
            return
        self._submit(self._job_manual, self._num(self.var_man_speed, 2.0),
                     self._num(self.var_alt, DEFAULT_ALT))

    def _job_manual(self, speed=2.0, alt=DEFAULT_ALT):
        if not self.flying:
            self._job_takeoff(alt)
        if not self.route:
            self._log("提示：还没规划航线，手动模式下看不到偏差参考线。", "warn")

        self.stop_evt.clear()
        self.pause_evt.clear()
        self.manual_evt.set()
        self._set_mode("手动")
        self.manual_z = self._pos()[2]
        self._log("已进入手动模式：WASD 平移，R/F 升降，Q/E 转向，空格悬停。", "hi")

        speed = max(speed, 0.3)
        dt = 0.2
        while self.manual_evt.is_set() and not self.stop_evt.is_set():
            # 机体系是「前后左右」，但 moveByVelocityZ 要的是 NED 的速度，
            # 所以直接用 X 正北 / Y 正东 来处理，不与机头朝向耦合。
            # 按键状态每次都现查系统，不缓存——见文件上方 key_down 的说明。
            vx = (1 if key_down("w") else 0) - (1 if key_down("s") else 0)
            vy = (1 if key_down("d") else 0) - (1 if key_down("a") else 0)
            vz_rate = (1 if key_down("r") else 0) - (1 if key_down("f") else 0)
            yaw_rate = (1 if key_down("e") else 0) - (1 if key_down("q") else 0)

            norm = math.hypot(vx, vy)
            if norm > 1e-6:                      # 斜着飞不要比直着飞更快
                vx, vy = vx / norm * speed, vy / norm * speed

            if vz_rate:                          # 高度按 1.2 m/s 连续变化
                self.manual_z -= vz_rate * 1.2 * dt
                self.manual_z = min(-0.5, max(-60.0, self.manual_z))

            self.manual_yaw_rate = yaw_rate * 45.0
            try:
                self.client.moveByVelocityZAsync(
                    vx, vy, self.manual_z, dt + 0.1,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=self.manual_yaw_rate))
            except Exception as exc:
                self._log("手动指令下发失败：%s" % exc, "err")
                break
            time.sleep(dt)

        try:
            self.client.hoverAsync()
        except Exception:
            pass
        self.manual_z = self._pos()[2]
        self._set_mode("已连接")
        self._log("已退出手动模式，飞机悬停。")

    # =======================================================================
    # 鼠标
    # =======================================================================
    # （键盘不在这里：手动模式的按键走 key_down()，直接查系统状态，
    #   不用 tkinter 的 KeyPress 事件，原因见文件上方那段注释。）

    def _on_mode_change(self):
        self.edit_mode = self.var_mode.get()
        cursor = {"pan": "fleur", "wp": "crosshair",
                  "obs": "crosshair", "del": "X_cursor"}.get(self.edit_mode, "arrow")
        self.canvas.configure(cursor=cursor)

    def on_motion(self, evt):
        x, y = self.s2w(evt.x, evt.y)
        self.lbl_hint.configure(text="光标  X北 %.1f  Y东 %.1f" % (x, y))

    def on_press(self, evt):
        wx, wy = self.s2w(evt.x, evt.y)
        m = self.edit_mode

        if m == MODE_WP:
            self._add_waypoint(wx, wy)
            self._log("添加航点 (%d)：X %.1f  Y %.1f  高度 %.1f 米"
                      % (len(self.waypoints), wx, wy, self._num(self.var_alt, DEFAULT_ALT)))
            return

        if m == MODE_OBS:
            self.obs_draft = (wx, wy)         # 按下定圆心，松开时按拖动距离定半径
            return

        if m == MODE_DEL:
            self._delete_at(evt.x, evt.y)
            return

        # 选择/平移模式：按到航点上就拖航点，否则平移画布
        hit = self._hit_waypoint(evt.x, evt.y)
        if hit is not None:
            self.drag_wp = hit
            self.wp_tree.selection_set(str(hit))
        else:
            self.pan_from = (evt.x, evt.y, self.view_cx, self.view_cy)

    def on_drag(self, evt):
        if self.drag_wp is not None:
            wx, wy = self.s2w(evt.x, evt.y)
            w = self.waypoints[self.drag_wp]
            w.x, w.y = wx, wy
            self._update_wp_row(self.drag_wp)
            self._invalidate_route()
            return
        if self.pan_from is not None:
            sx, sy, cx, cy = self.pan_from
            self.view_cy = cy - (evt.x - sx) / self.scale
            self.view_cx = cx + (evt.y - sy) / self.scale

    def on_release(self, evt):
        if self.edit_mode == MODE_OBS and self.obs_draft is not None:
            wx, wy = self.s2w(evt.x, evt.y)
            ox, oy = self.obs_draft
            r = math.hypot(wx - ox, wy - oy)
            if r < 1.0:                       # 只点了一下没拖，给个默认半径
                r = 3.0
            self._add_obstacle(ox, oy, r)
            self._log("添加障碍：圆心 (%.1f, %.1f) 半径 %.1f 米" % (ox, oy, r))
            self.obs_draft = None
        self.drag_wp = None
        self.pan_from = None

    def on_right_click(self, evt):
        self._delete_at(evt.x, evt.y)

    def _delete_at(self, sx, sy):
        i = self._hit_waypoint(sx, sy)
        if i is not None:
            del self.waypoints[i]
            self._refresh_wp_tree()
            self._log("已删除航点 %d。" % (i + 1))
            return
        j = self._hit_obstacle(sx, sy)
        if j is not None:
            del self.obstacles[j]
            self._refresh_obs_tree()
            self._log("已删除障碍 %d。" % (j + 1))

    def _hit_waypoint(self, sx, sy, tol=13):
        for i, w in enumerate(self.waypoints):
            wx, wy = self.w2s(w.x, w.y)
            if (wx - sx) ** 2 + (wy - sy) ** 2 <= tol * tol:
                return i
        return None

    def _hit_obstacle(self, sx, sy):
        for i, o in enumerate(self.obstacles):
            ox, oy = self.w2s(o.x, o.y)
            if (ox - sx) ** 2 + (oy - sy) ** 2 <= (o.r * self.scale + 3) ** 2:
                return i
        return None

    def on_wheel(self, evt):
        """滚轮缩放，以光标位置为锚点，缩放时视线不乱跑。"""
        wx, wy = self.s2w(evt.x, evt.y)
        self.scale = max(1.0, min(200.0, self.scale * (1.15 if evt.delta > 0 else 1 / 1.15)))
        wx2, wy2 = self.s2w(evt.x, evt.y)
        self.view_cx += wx - wx2
        self.view_cy += wy - wy2

    def zoom(self, factor):
        self.scale = max(1.0, min(200.0, self.scale * factor))

    def fit_view(self):
        """把航线、障碍、轨迹全部框进视野。"""
        xs, ys = [], []
        for w in self.waypoints:
            xs.append(w.x)
            ys.append(w.y)
        for o in self.obstacles:
            xs += [o.x - o.r, o.x + o.r]
            ys += [o.y - o.r, o.y + o.r]
        for t in self.trail[-500:]:
            xs.append(t[0])
            ys.append(t[1])
        if not xs:
            self.view_cx = self.view_cy = 0.0
            self.scale = 12.0
            return
        w = max(self.canvas.winfo_width(), 100)
        h = max(self.canvas.winfo_height(), 100)
        span_x = max(max(xs) - min(xs), 5.0) * 1.25
        span_y = max(max(ys) - min(ys), 5.0) * 1.25
        self.view_cx = (max(xs) + min(xs)) / 2.0
        self.view_cy = (max(ys) + min(ys)) / 2.0
        self.scale = max(1.0, min(200.0, min(w / span_y, h / span_x)))

    # =======================================================================
    # 坐标变换  世界(NED) <-> 画布(像素)
    # =======================================================================

    def w2s(self, x, y):
        w = max(self.canvas.winfo_width(), 2)
        h = max(self.canvas.winfo_height(), 2)
        return w / 2 + (y - self.view_cy) * self.scale, h / 2 - (x - self.view_cx) * self.scale

    def s2w(self, sx, sy):
        w = max(self.canvas.winfo_width(), 2)
        h = max(self.canvas.winfo_height(), 2)
        return (h / 2 - sy) / self.scale + self.view_cx, (sx - w / 2) / self.scale + self.view_cy

    # =======================================================================
    # 定时刷新：收队列 + 刷状态栏 + 重画地图
    # =======================================================================

    def _tick(self):
        """界面心跳。刷新出任何异常都不能让心跳停掉——演示到一半界面卡死是最糟的。"""
        try:
            self._drain_queue()
            self._update_status()
            self._redraw()
        except Exception as exc:
            try:
                self._append_log("[刷新异常] %s" % exc, "err")
            except Exception:
                pass
        finally:
            self.root.after(80, self._tick)

    def _drain_queue(self):
        while True:
            try:
                kind, payload = self.ui_q.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                msg, tag = payload
                self._append_log(msg, tag)
            elif kind == "mode":
                self.pill.configure(text=payload, fg={
                    "自动巡航": ACCENT, "手动": CYAN, "已连接": OK,
                    "异常": DANGER, "仅规划": WARN}.get(payload, MUTED))
            elif kind == "connected":
                self.connected = bool(payload)
                if self.connected:
                    self.btn_conn.configure(text="断开", state="normal")
                    self._set_mode("已连接")
                else:
                    self.btn_conn.configure(text="连接", state="normal")
                    self._set_mode("未连接")
                    self.armed = False
                    self.flying = False
            elif kind == "call":
                try:
                    payload()
                except Exception:
                    pass
            elif kind == "collision":
                self._log("!! 检测到碰撞：%s" % (payload or "未知物体"), "err")

    def _append_log(self, msg, tag):
        self.txt_log.configure(state="normal")
        self.txt_log.insert("end", msg + "\n", tag)
        if int(self.txt_log.index("end-1c").split(".")[0]) > 400:
            self.txt_log.delete("1.0", "100.0")
        self.txt_log.see("end")
        self.txt_log.configure(state="disabled")

    def _update_status(self):
        t = self.telem
        if not t:
            self.lbl_status.configure(text="等待连接 ...  |  航点 %d  |  航线 %s"
                                           % (len(self.waypoints),
                                              "%.1f m" % self.info["length"] if self.route else "未规划"))
            return
        dev_txt = "     偏差 %.2f m" % t.get("dev", 0.0) if self.route else ""
        prog = ("      进度 %d/%d" % (self.cruise_index + 1, len(self.route))
                if 0 <= self.cruise_index < len(self.route) else "")
        self.lbl_status.configure(
            text=("X %7.2f   Y %7.2f   高度 %5.2f m   速度 %5.2f m/s   "
                  "偏航 %6.1f°%s%s      模式 %s"
                  % (t["x"], t["y"], t["alt"],
                     math.hypot(t["vx"], t["vy"]), t["yaw"], dev_txt, prog, self.mode)))

        # 手动模式的偏差大字号显示
        d = t.get("dev", 0.0)
        col = OK if d < 1.5 else (WARN if d < DEV_WARN else DANGER)
        self.lbl_dev.configure(text="%.2f  m" % d if self.route else "--  m", fg=col)
        if self.manual_evt.is_set() and self.route:
            dev = deviation_to_path(t["x"], t["y"], self.route_xy, self.route_cum)
            self.lbl_dev_side.configure(
                text="偏在航线%s侧　已完成 %.0f%%" % (dev["side"] or "中", dev["progress"] * 100))
        elif not self.route:
            self.lbl_dev_side.configure(text="尚未规划航线")
        else:
            self.lbl_dev_side.configure(text="未进入手动模式")

    # -----------------------------------------------------------------------
    # 地图绘制
    # -----------------------------------------------------------------------

    def _redraw(self):
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(), 2)
        h = max(c.winfo_height(), 2)

        self._draw_grid(w, h)

        # 实际轨迹
        if self.var_show_trail.get() and len(self.trail) > 1:
            pts = self.trail
            if len(pts) > 800:                       # 点太多就抽稀，不然画布会卡
                pts = pts[::2]
            flat = []
            for (x, y, _z) in pts:
                sx, sy = self.w2s(x, y)
                flat += [sx, sy]
            c.create_line(*flat, fill=CYAN, width=2, smooth=False)

        # A* 原始路径（细虚线，用来看规划器到底怎么绕的）
        if self.var_show_dense.get() and len(self.route_dense) > 1:
            flat = []
            for (x, y) in self.route_dense:
                sx, sy = self.w2s(x, y)
                flat += [sx, sy]
            c.create_line(*flat, fill="#5B4B75", width=1, dash=(3, 3))

        # 障碍物
        for o in self.obstacles:
            sx, sy = self.w2s(o.x, o.y)
            r = o.r * self.scale
            c.create_oval(sx - r, sy - r, sx + r, sy + r,
                          outline=DANGER, width=2, fill="#3A1E24")
            c.create_line(sx - 5, sy, sx + 5, sy, fill=DANGER)
            c.create_line(sx, sy - 5, sx, sy + 5, fill=DANGER)
        if self.obs_draft is not None:
            sx, sy = self.w2s(*self.obs_draft)
            c.create_oval(sx - 3, sy - 3, sx + 3, sy + 3, outline=WARN, width=2)

        # 规划航线
        if self.var_show_route.get() and len(self.route) > 1:
            flat = []
            for (x, y, _z) in self.route:
                sx, sy = self.w2s(x, y)
                flat += [sx, sy]
            c.create_line(*flat, fill=ACCENT, width=3)

        # 航点：起点绿、终点红、中间紫
        for i, wp in enumerate(self.waypoints):
            sx, sy = self.w2s(wp.x, wp.y)
            col = OK if i == 0 else (DANGER if i == len(self.waypoints) - 1 else ACCENT)
            if i == self.cruise_index:
                c.create_oval(sx - 12, sy - 12, sx + 12, sy + 12, outline=WARN, width=2)
            c.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill=col, outline="#FFFFFF", width=1)
            c.create_text(sx, sy - 14, text="%d" % (i + 1), fill=TEXT, font=FONT_S)

        # 起飞点
        if self.home is not None:
            sx, sy = self.w2s(self.home[0], self.home[1])
            c.create_rectangle(sx - 6, sy - 6, sx + 6, sy + 6, outline=OK, width=2)
            c.create_text(sx, sy + 16, text="HOME", fill=OK, font=("Consolas", 7))

        # 飞机（三角形，按偏航角转）
        t = self.telem
        if t:
            sx, sy = self.w2s(t["x"], t["y"])
            a = math.radians(t["yaw"])
            # 机头方向在屏幕上：北(0°)朝上，顺时针为东
            nose = (sx + math.sin(a) * 15, sy - math.cos(a) * 15)
            left = (sx + math.sin(a + 2.5) * 9, sy - math.cos(a + 2.5) * 9)
            right = (sx + math.sin(a - 2.5) * 9, sy - math.cos(a - 2.5) * 9)
            c.create_polygon(*nose, *left, *right, fill="#FFFFFF", outline=ACCENT2, width=1)

        self._draw_overlay(w, h)

    def _draw_grid(self, w, h):
        c = self.canvas
        step = 1.0
        for cand in (1, 2, 5, 10, 20, 50, 100, 200):
            step = cand
            if cand * self.scale >= 55:
                break
        x_min, y_min = self.s2w(0, h)
        x_max, y_max = self.s2w(w, 0)

        i0 = int(math.floor(x_min / step))
        i1 = int(math.ceil(x_max / step))
        for i in range(i0, i1 + 1):
            xx = i * step
            sx, _ = self.w2s(xx, y_min)
            major = abs(xx % (step * 5)) < 1e-6
            c.create_line(sx, 0, sx, h, fill=GRID2 if major else GRID)
            if major:
                c.create_text(sx + 2, h - 8, text="%.0f" % xx, fill="#4C4360",
                              font=("Consolas", 7), anchor="w")

        j0 = int(math.floor(y_min / step))
        j1 = int(math.ceil(y_max / step))
        for j in range(j0, j1 + 1):
            yy = j * step
            _, sy = self.w2s(x_min, yy)
            major = abs(yy % (step * 5)) < 1e-6
            c.create_line(0, sy, w, sy, fill=GRID2 if major else GRID)
            if major:
                c.create_text(6, sy - 8, text="%.0f" % yy, fill="#4C4360",
                              font=("Consolas", 7), anchor="w")

        # 原点十字
        ox, oy = self.w2s(0.0, 0.0)
        if 0 <= ox <= w and 0 <= oy <= h:
            c.create_line(ox - 10, oy, ox + 10, oy, fill="#4C4360")
            c.create_line(ox, oy - 10, ox, oy + 10, fill="#4C4360")

    def _draw_overlay(self, w, h):
        """右上角的指北针与左下角的比例尺。"""
        c = self.canvas
        c.create_line(w - 34, 52, w - 34, 22, fill=MUTED, width=2)
        c.create_polygon(w - 34, 16, w - 39, 26, w - 29, 26, fill=MUTED)
        c.create_text(w - 34, 64, text="N", fill=MUTED, font=FONT_B)

        if self.scale > 0:
            meters = 5.0
            px = meters * self.scale
            while px > 160:
                meters /= 2
                px = meters * self.scale
            while px < 45:
                meters *= 2
                px = meters * self.scale
            x0, y0 = 16, h - 20
            c.create_line(x0, y0, x0 + px, y0, fill=TEXT, width=2)
            c.create_line(x0, y0 - 4, x0, y0 + 4, fill=TEXT, width=2)
            c.create_line(x0 + px, y0 - 4, x0 + px, y0 + 4, fill=TEXT, width=2)
            c.create_text(x0 + px / 2, y0 - 12, text="%.0f m" % meters, fill=TEXT,
                          font=("Consolas", 8))

        c.create_text(16, 18, text="X 正北 ↑   Y 正东 →", fill="#4C4360",
                      font=FONT_S, anchor="w")

    # =======================================================================
    # 退出
    # =======================================================================

    def on_close(self):
        if self.flying and self.connected:
            if not messagebox.askyesno("飞机还在空中", "无人机仍在飞行中。\n\n"
                                                       "确定要降落并退出吗？"):
                return
            try:
                self.stop_evt.set()
                self.manual_evt.clear()
                self.client.cancelLastTask()
                self.client.landAsync(timeout_sec=60).join()
                self.client.armDisarm(False)
                self.client.enableApiControl(False)
            except Exception:
                pass

        self._runner_on = False
        self._telem_on = False
        if self.recorder is not None:
            self.recorder.stop()
            self._log("飞行数据已保存，共 %d 个采样点。" % self.recorder.count, "ok")
        self.root.destroy()


def main():
    root = tk.Tk()
    GCSApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
