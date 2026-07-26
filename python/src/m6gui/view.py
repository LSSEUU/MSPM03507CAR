"""Tk/ttk view for the M6 controller.

All Session calls are delegated to :mod:`m6gui.controller`.  The Tk thread
only drains the controller completion queue through ``after`` callbacks.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Any, Callable

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .controller import (
    ActionDraft,
    Controller,
    GuiInputError,
    arc_action,
    feature_visibility,
    heading_hold_action,
    is_emergency_key,
    model_text,
    object_mapping,
    straight_action,
    turn_action,
    wheel_action,
    wizard_drafts,
)


POLL_MS = 50
CHART_MS = 200


@dataclass(frozen=True)
class Palette:
    background: str = "#F8FAFC"
    surface: str = "#FFFFFF"
    foreground: str = "#1E293B"
    muted: str = "#64748B"
    border: str = "#CBD5E1"
    accent: str = "#C2410C"
    accent_active: str = "#9A3412"
    danger: str = "#B91C1C"
    danger_active: str = "#991B1B"
    success: str = "#047857"
    focus: str = "#2563EB"


class M6View:
    """Build and synchronize the complete desktop operator interface."""

    def __init__(self, root: tk.Tk, controller: Controller, *, fake: bool = False) -> None:
        self.root = root
        self.controller = controller
        self.fake = fake
        self.palette = Palette()
        self._closed = False
        self._poll_id: str | None = None
        self._chart_id: str | None = None
        self._last_generation = -1
        self._last_estop_key = -math.inf
        self._wizard_index = 0
        self._chart_ready = False
        self._lines: dict[str, Any] = {}

        self._configure_root()
        self._build_header()
        self._build_notebook()
        self._build_footer()
        self._bind_safety_keys()
        self.refresh_ports()
        self._sync_state(force=True)
        self._poll_id = self.root.after(POLL_MS, self._poll_loop)
        self._chart_id = self.root.after(CHART_MS, self._chart_loop)

    def _configure_root(self) -> None:
        self.root.title("M6 车辆控制与验收" + (" — Fake Session" if self.fake else ""))
        self.root.geometry("1380x900")
        self.root.minsize(1120, 720)
        self.root.configure(background=self.palette.background)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        font = ("Microsoft YaHei UI", 10)
        small = ("Microsoft YaHei UI", 9)
        heading = ("Microsoft YaHei UI", 11, "bold")
        mono = ("Consolas", 9)
        style.configure(".", font=font, background=self.palette.background, foreground=self.palette.foreground)
        style.configure("TFrame", background=self.palette.background)
        style.configure("Surface.TFrame", background=self.palette.surface)
        style.configure("TLabel", background=self.palette.background, foreground=self.palette.foreground)
        style.configure("Surface.TLabel", background=self.palette.surface)
        style.configure("Muted.TLabel", foreground=self.palette.muted)
        style.configure("Header.TLabel", font=("Microsoft YaHei UI", 16, "bold"))
        style.configure("Section.TLabel", font=heading)
        style.configure("Mono.TLabel", font=mono)
        style.configure("Status.TLabel", font=small, padding=(8, 5))
        style.configure("TButton", padding=(10, 6))
        style.configure("Primary.TButton", foreground="white", background=self.palette.accent)
        style.map("Primary.TButton", background=[("active", self.palette.accent_active), ("disabled", "#94A3B8")])
        style.configure("Danger.TButton", foreground="white", background=self.palette.danger, font=("Microsoft YaHei UI", 14, "bold"), padding=(18, 13))
        style.map("Danger.TButton", background=[("active", self.palette.danger_active), ("disabled", "#94A3B8")])
        style.configure("TLabelframe", background=self.palette.background, bordercolor=self.palette.border, padding=8)
        style.configure("TLabelframe.Label", background=self.palette.background, font=heading)
        style.configure("Treeview", rowheight=26, fieldbackground=self.palette.surface, background=self.palette.surface)
        style.configure("Treeview.Heading", font=("Microsoft YaHei UI", 9, "bold"))
        style.configure("TNotebook", background=self.palette.background, tabmargins=(0, 8, 0, 0))
        style.configure("TNotebook.Tab", padding=(14, 7))

    def _build_header(self) -> None:
        header = ttk.Frame(self.root, style="Surface.TFrame", padding=(16, 12))
        header.pack(fill="x")
        header.columnconfigure(2, weight=1)

        title = ttk.Frame(header, style="Surface.TFrame")
        title.grid(row=0, column=0, rowspan=2, sticky="nw", padx=(0, 20))
        ttk.Label(title, text="M6 Control", style="Header.TLabel", background=self.palette.surface).pack(anchor="w")
        ttk.Label(
            title,
            text="V2 实时控制、诊断与安全验收",
            style="Surface.TLabel",
            foreground=self.palette.muted,
        ).pack(anchor="w")

        connection = ttk.Frame(header, style="Surface.TFrame")
        connection.grid(row=0, column=1, rowspan=2, sticky="nw")
        ttk.Label(connection, text="串口", style="Surface.TLabel").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=1, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(connection, text="刷新", command=self.refresh_ports).grid(row=1, column=1, padx=(0, 10))
        self.legacy_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            connection,
            text="显式 V1 legacy",
            variable=self.legacy_var,
        ).grid(row=0, column=2, sticky="w")
        self.connect_button = ttk.Button(connection, text="连接（要求 V2）", style="Primary.TButton", command=self._toggle_connection)
        self.connect_button.grid(row=1, column=2, sticky="ew")

        identity_box = ttk.Frame(header, style="Surface.TFrame", padding=(18, 0))
        identity_box.grid(row=0, column=2, rowspan=2, sticky="nsew")
        self.identity_var = tk.StringVar(value="固件 —  | schema —  | 参数来源 —")
        self.capability_var = tk.StringVar(value="等待 V2 identity")
        ttk.Label(identity_box, textvariable=self.identity_var, style="Surface.TLabel", wraplength=650).pack(anchor="w")
        ttk.Label(identity_box, textvariable=self.capability_var, style="Surface.TLabel", foreground=self.palette.muted, wraplength=650).pack(anchor="w", pady=(5, 0))

        self.estop_button = ttk.Button(header, text="急停  S / SPACE", style="Danger.TButton", command=self._emergency_stop)
        self.estop_button.grid(row=0, column=3, rowspan=2, sticky="nsew", padx=(16, 0))

    def _build_notebook(self) -> None:
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=12, pady=(4, 8))
        self.monitor_tab = ttk.Frame(self.notebook, padding=8)
        self.parameter_tab = ttk.Frame(self.notebook, padding=8)
        self.action_tab = ttk.Frame(self.notebook, padding=8)
        self.wizard_tab = ttk.Frame(self.notebook, padding=8)
        self.record_tab = ttk.Frame(self.notebook, padding=8)
        self.notebook.add(self.monitor_tab, text="实时与诊断")
        self.notebook.add(self.parameter_tab, text="参数事务")
        self.notebook.add(self.action_tab, text="基础动作")
        self.notebook.add(self.wizard_tab, text="测试向导")
        self.notebook.add(self.record_tab, text="录制 · 回放 · 评分")
        self._build_monitor_tab()
        self._build_parameter_tab()
        self._build_action_tab()
        self._build_wizard_tab()
        self._build_record_tab()

    def _build_monitor_tab(self) -> None:
        pane = ttk.Panedwindow(self.monitor_tab, orient="horizontal")
        pane.pack(fill="both", expand=True)
        diag = ttk.Frame(pane, padding=(0, 0, 8, 0))
        chart = ttk.Frame(pane, padding=(8, 0, 0, 0))
        pane.add(diag, weight=1)
        pane.add(chart, weight=3)
        self.diag_vars: dict[str, tk.StringVar] = {}
        labels = (
            ("status", "Safety / 系统状态"),
            ("reset", "Reset / breadcrumb"),
            ("tasks", "任务漏期"),
            ("uart", "UART"),
            ("link", "V2 队列 / 丢包"),
            ("events", "最近事件"),
        )
        for key, label in labels:
            frame = ttk.LabelFrame(diag, text=label)
            frame.pack(fill="x", pady=(0, 8))
            variable = tk.StringVar(value="—")
            self.diag_vars[key] = variable
            ttk.Label(frame, textvariable=variable, justify="left", wraplength=320).pack(fill="x")
        ttk.Label(
            diag,
            text="曲线显示最近约 12 秒；数值空缺表示对应 telemetry 流尚未到达。",
            style="Muted.TLabel",
            wraplength=320,
        ).pack(anchor="w")
        self._build_charts(chart)

    def _build_charts(self, parent: ttk.Frame) -> None:
        try:
            from matplotlib import rcParams
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from matplotlib.figure import Figure
        except ImportError as exc:
            ttk.Label(parent, text=f"matplotlib 不可用：{exc}", foreground=self.palette.danger).pack(fill="both", expand=True)
            return
        rcParams["font.sans-serif"] = [
            "Microsoft YaHei UI",
            "Microsoft YaHei",
            "SimHei",
            "DejaVu Sans",
        ]
        rcParams["axes.unicode_minus"] = False
        figure = Figure(figsize=(9.2, 6.8), dpi=100, facecolor=self.palette.surface, constrained_layout=True)
        self.axes = {
            "wheel": figure.add_subplot(221),
            "heading": figure.add_subplot(222),
            "gray": figure.add_subplot(223),
            "centroid": figure.add_subplot(224),
        }
        colors = ("#2563EB", "#EA580C", "#059669", "#7C3AED", "#0E7490", "#BE123C", "#4D7C0F", "#A16207")
        wheel = self.axes["wheel"]
        self._lines["left"], = wheel.plot([], [], color=colors[0], linewidth=1.6, label="左轮")
        self._lines["right"], = wheel.plot([], [], color=colors[1], linewidth=1.6, linestyle="--", label="右轮")
        heading = self.axes["heading"]
        self._lines["yaw"], = heading.plot([], [], color=colors[2], linewidth=1.5, label="航向 °")
        self._lines["rate"], = heading.plot([], [], color=colors[3], linewidth=1.5, linestyle="--", label="角速度 °/s")
        gray = self.axes["gray"]
        for index in range(8):
            self._lines[f"gray{index}"], = gray.plot([], [], color=colors[index], linewidth=1.0, label=f"G{index}")
        centroid = self.axes["centroid"]
        self._lines["centroid"], = centroid.plot([], [], color=colors[4], linewidth=1.7, label="质心")
        titles = {
            "wheel": ("轮速", "mm/s"),
            "heading": ("航向 / 角速度", "° / °/s"),
            "gray": ("八路灰度", "ADC"),
            "centroid": ("灰度质心", "归一化位置"),
        }
        for key, axis in self.axes.items():
            title, unit = titles[key]
            axis.set_title(title, fontsize=10, fontweight="bold")
            axis.set_xlabel("时间 (s)", fontsize=8)
            axis.set_ylabel(unit, fontsize=8)
            axis.grid(True, color="#E2E8F0", linewidth=0.6)
            axis.tick_params(labelsize=8)
            axis.legend(loc="upper left", fontsize=7, ncol=4 if key == "gray" else 2, frameon=False)
        self.chart_canvas = FigureCanvasTkAgg(figure, master=parent)
        self.chart_canvas.get_tk_widget().pack(fill="both", expand=True)
        self._chart_ready = True

    def _build_parameter_tab(self) -> None:
        self.parameter_tab.rowconfigure(0, weight=1)
        self.parameter_tab.columnconfigure(0, weight=1)
        columns = ("key", "active", "staged", "persisted", "range", "unit")
        self.parameter_tree = ttk.Treeview(self.parameter_tab, columns=columns, show="headings", selectmode="browse")
        headings = {
            "key": "参数",
            "active": "Active",
            "staged": "Staged",
            "persisted": "Persisted",
            "range": "范围",
            "unit": "单位",
        }
        widths = {"key": 230, "active": 100, "staged": 100, "persisted": 100, "range": 170, "unit": 140}
        for name in columns:
            self.parameter_tree.heading(name, text=headings[name])
            self.parameter_tree.column(name, width=widths[name], anchor="w", stretch=name == "key")
        scroll = ttk.Scrollbar(self.parameter_tab, orient="vertical", command=self.parameter_tree.yview)
        self.parameter_tree.configure(yscrollcommand=scroll.set)
        self.parameter_tree.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")

        editor = ttk.LabelFrame(self.parameter_tab, text="编辑 staged（不会自动 apply）")
        editor.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        editor.columnconfigure(1, weight=1)
        ttk.Label(editor, text="选中参数的新值").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.parameter_value_var = tk.StringVar()
        ttk.Entry(editor, textvariable=self.parameter_value_var).grid(row=0, column=1, sticky="ew")
        self.stage_button = ttk.Button(editor, text="Stage", command=self._stage_parameter)
        self.stage_button.grid(row=0, column=2, padx=(8, 0))
        controls = ttk.Frame(editor)
        controls.grid(row=1, column=0, columnspan=3, sticky="w", pady=(10, 0))
        self.apply_button = ttk.Button(controls, text="Apply", style="Primary.TButton", command=self.controller.apply_parameters)
        self.apply_button.pack(side="left", padx=(0, 8))
        self.revert_button = ttk.Button(controls, text="Revert staged", command=self.controller.revert_parameters)
        self.revert_button.pack(side="left", padx=(0, 8))
        self.load_button = ttk.Button(controls, text="Load persisted", command=self.controller.load_parameters)
        self.load_button.pack(side="left", padx=(0, 8))
        self.defaults_button = ttk.Button(controls, text="Defaults → staged", command=self.controller.default_parameters)
        self.defaults_button.pack(side="left", padx=(0, 8))
        self.save_button = ttk.Button(controls, text="Save active", command=self.controller.save_parameters)
        self.save_button.pack(side="left", padx=(0, 8))
        ttk.Button(controls, text="刷新", command=self.controller.refresh_parameters).pack(side="left")
        self.parameter_operation_buttons = (
            (self.apply_button, "parameter_apply"),
            (self.revert_button, "parameter_stage"),
            (self.load_button, "parameter_stage"),
            (self.defaults_button, "parameter_stage"),
            (self.save_button, "parameter_persist"),
        )
        self.parameter_tree.bind("<<TreeviewSelect>>", self._parameter_selected)

    def _build_action_tab(self) -> None:
        self.action_tab.columnconfigure(0, weight=1)
        self.action_tab.columnconfigure(1, weight=1)
        self.speed_frame = ttk.LabelFrame(self.action_tab, text="轮速 / 基础轨迹（±400 mm/s，自动超时）")
        self.speed_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 8))
        self.speed_frame.columnconfigure(1, weight=1)
        self.left_speed_var = tk.StringVar(value="160")
        self.right_speed_var = tk.StringVar(value="160")
        self.speed_var = tk.StringVar(value="160")
        self.duration_var = tk.StringVar(value="1.5")
        self.ratio_var = tk.StringVar(value="0.7")
        fields = (
            ("左轮 mm/s", self.left_speed_var),
            ("右轮 mm/s", self.right_speed_var),
            ("预设速度 v", self.speed_var),
            ("持续时间 s", self.duration_var),
            ("弧线内外轮比", self.ratio_var),
        )
        for row, (label, variable) in enumerate(fields):
            ttk.Label(self.speed_frame, text=label).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Entry(self.speed_frame, textvariable=variable, width=12).grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=3)
        ttk.Button(self.speed_frame, text="Arm 独立轮", style="Primary.TButton", command=lambda: self._arm(lambda: wheel_action(self.left_speed_var.get(), self.right_speed_var.get(), self.duration_var.get()))).grid(row=5, column=0, columnspan=2, sticky="ew", pady=(10, 6))
        presets = ttk.Frame(self.speed_frame)
        presets.grid(row=6, column=0, columnspan=2, sticky="ew")
        for column in range(2):
            presets.columnconfigure(column, weight=1)
        ttk.Button(presets, text="Arm 前进", command=lambda: self._arm(lambda: straight_action(self.speed_var.get(), self.duration_var.get()))).grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=3)
        ttk.Button(presets, text="Arm 后退", command=lambda: self._arm(lambda: straight_action(self.speed_var.get(), self.duration_var.get(), reverse=True))).grid(row=0, column=1, sticky="ew", padx=(4, 0), pady=3)
        ttk.Button(presets, text="Arm 左弧", command=lambda: self._arm(lambda: arc_action(self.speed_var.get(), self.duration_var.get(), ratio=self.ratio_var.get(), left=True))).grid(row=1, column=0, sticky="ew", padx=(0, 4), pady=3)
        ttk.Button(presets, text="Arm 右弧", command=lambda: self._arm(lambda: arc_action(self.speed_var.get(), self.duration_var.get(), ratio=self.ratio_var.get(), left=False))).grid(row=1, column=1, sticky="ew", padx=(4, 0), pady=3)

        self.heading_frame = ttk.LabelFrame(self.action_tab, text="航向保持")
        self.heading_frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 8))
        self.heading_frame.columnconfigure(1, weight=1)
        self.heading_speed_var = tk.StringVar(value="160")
        self.heading_duration_var = tk.StringVar(value="2.0")
        ttk.Label(self.heading_frame, text="直行速度 mm/s").grid(row=0, column=0, sticky="w", pady=3)
        ttk.Entry(self.heading_frame, textvariable=self.heading_speed_var).grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=3)
        ttk.Label(self.heading_frame, text="持续时间 s").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Entry(self.heading_frame, textvariable=self.heading_duration_var).grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=3)
        ttk.Button(self.heading_frame, text="Arm 航向保持", style="Primary.TButton", command=lambda: self._arm(lambda: heading_hold_action(self.heading_speed_var.get(), self.heading_duration_var.get()))).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 6))

        self.turn_frame = ttk.LabelFrame(self.heading_frame, text="可配置相对角转向")
        self.turn_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        self.turn_frame.columnconfigure(1, weight=1)
        self.turn_angle_var = tk.StringVar(value="90")
        self.turn_duration_var = tk.StringVar(value="3.0")
        ttk.Label(self.turn_frame, text="角度 ±1..±360°").grid(row=0, column=0, sticky="w", pady=3)
        ttk.Entry(self.turn_frame, textvariable=self.turn_angle_var).grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=3)
        ttk.Label(self.turn_frame, text="超时 s").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Entry(self.turn_frame, textvariable=self.turn_duration_var).grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=3)
        quick = ttk.Frame(self.turn_frame)
        quick.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        for index, angle in enumerate((-90, -45, 45, 90)):
            ttk.Button(quick, text=f"{angle:+d}°", command=lambda value=angle: self.turn_angle_var.set(str(value))).pack(side="left", expand=True, fill="x", padx=(0 if index == 0 else 2, 0))
        ttk.Button(self.turn_frame, text="Arm 相对转向", style="Primary.TButton", command=lambda: self._arm(lambda: turn_action(self.turn_angle_var.get(), self.turn_duration_var.get()))).grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        self._build_lease_panel(self.action_tab, row=1, columnspan=2)

    def _build_lease_panel(self, parent: ttk.Frame, *, row: int, columnspan: int) -> None:
        lease = ttk.LabelFrame(parent, text="两阶段动作闸门")
        lease.grid(row=row, column=0, columnspan=columnspan, sticky="ew", pady=(6, 0))
        lease.columnconfigure(0, weight=1)
        self.lease_var = tk.StringVar(value="尚未 Arm")
        ttk.Label(lease, textvariable=self.lease_var, style="Mono.TLabel").grid(row=0, column=0, sticky="w")
        self.confirm_button = ttk.Button(lease, text="Confirm（仅人工点击）", style="Primary.TButton", command=self._confirm_action)
        self.confirm_button.grid(row=0, column=1, padx=(10, 6))
        ttk.Button(lease, text="Cancel 正常停止", command=self.controller.cancel_action).grid(row=0, column=2)
        ttk.Label(
            lease,
            text="Arm 只取得 10 秒 token；GUI 不会自动 Confirm。急停始终使用顶部红色按钮或 S / 空格。",
            style="Muted.TLabel",
        ).grid(row=1, column=0, columnspan=3, sticky="w", pady=(8, 0))

    def _build_wizard_tab(self) -> None:
        frame = ttk.LabelFrame(self.wizard_tab, text="架空 / 基础动作测试向导")
        frame.pack(fill="both", expand=True)
        frame.columnconfigure(1, weight=1)
        self.wizard_speed_var = tk.StringVar(value="160")
        self.wizard_ratio_var = tk.StringVar(value="0.7")
        self.wizard_duration_var = tk.StringVar(value="1.5")
        for row, (label, variable) in enumerate((
            ("速度 v (mm/s)", self.wizard_speed_var),
            ("弧线内外轮比", self.wizard_ratio_var),
            ("单步持续时间 s", self.wizard_duration_var),
        )):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=4)
            ttk.Entry(frame, textvariable=variable, width=16).grid(row=row, column=1, sticky="w", padx=(10, 0), pady=4)
        self.wizard_step_var = tk.StringVar()
        ttk.Separator(frame).grid(row=3, column=0, columnspan=3, sticky="ew", pady=12)
        ttk.Label(frame, textvariable=self.wizard_step_var, font=("Microsoft YaHei UI", 14, "bold")).grid(row=4, column=0, columnspan=3, sticky="w")
        ttk.Label(
            frame,
            text="核对架空、方向和急停后，先 Arm 当前步骤，再由操作者独立 Confirm。切换步骤不会发送动作。",
            style="Muted.TLabel",
        ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(5, 12))
        nav = ttk.Frame(frame)
        nav.grid(row=6, column=0, columnspan=3, sticky="ew")
        ttk.Button(nav, text="上一步", command=lambda: self._wizard_move(-1)).pack(side="left")
        ttk.Button(nav, text="下一步", command=lambda: self._wizard_move(1)).pack(side="left", padx=(6, 16))
        ttk.Button(nav, text="Arm 当前步骤", style="Primary.TButton", command=self._wizard_arm).pack(side="left")
        ttk.Button(nav, text="Confirm 当前 token", command=self._confirm_action).pack(side="left", padx=6)
        ttk.Button(nav, text="Cancel 正常停止", command=self.controller.cancel_action).pack(side="left")
        self._update_wizard_label()

    def _build_record_tab(self) -> None:
        self.record_tab.columnconfigure(0, weight=1)
        self.record_frame = ttk.LabelFrame(self.record_tab, text="V2 录制")
        self.record_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        self.record_frame.columnconfigure(1, weight=1)
        ttk.Label(self.record_frame, text="CSV / JSON 路径").grid(row=0, column=0, sticky="w")
        self.record_path_var = tk.StringVar(value=str(Path.cwd() / "m6-recording.csv"))
        ttk.Entry(self.record_frame, textvariable=self.record_path_var).grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Button(self.record_frame, text="选择…", command=self._choose_record_path).grid(row=0, column=2)
        self.record_button = ttk.Button(self.record_frame, text="开始录制", style="Primary.TButton", command=self._toggle_recording)
        self.record_button.grid(row=1, column=0, columnspan=3, sticky="w", pady=(10, 0))

        replay = ttk.LabelFrame(self.record_tab, text="旧记录回放与评分")
        replay.grid(row=1, column=0, sticky="nsew")
        replay.columnconfigure(1, weight=1)
        self.replay_path_var = tk.StringVar()
        ttk.Label(replay, text="记录文件").grid(row=0, column=0, sticky="w")
        ttk.Entry(replay, textvariable=self.replay_path_var).grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Button(replay, text="打开…", command=self._choose_replay_path).grid(row=0, column=2)
        controls = ttk.Frame(replay)
        controls.grid(row=1, column=0, columnspan=3, sticky="w", pady=(10, 8))
        ttk.Button(controls, text="加载回放", command=self._replay).pack(side="left", padx=(0, 8))
        ttk.Button(controls, text="评分", style="Primary.TButton", command=self._score).pack(side="left")
        self.replay_text = tk.Text(replay, height=18, wrap="word", font=("Consolas", 9), relief="solid", borderwidth=1)
        self.replay_text.grid(row=2, column=0, columnspan=3, sticky="nsew")

    def _build_footer(self) -> None:
        footer = ttk.Frame(self.root)
        footer.pack(fill="x", padx=12, pady=(0, 8))
        self.status_var = tk.StringVar(value="未连接")
        self.error_var = tk.StringVar()
        ttk.Label(footer, textvariable=self.status_var, style="Status.TLabel").pack(side="left")
        ttk.Label(footer, textvariable=self.error_var, style="Status.TLabel", foreground=self.palette.danger).pack(side="right")

    def _bind_safety_keys(self) -> None:
        for sequence in ("<KeyPress-s>", "<KeyPress-S>", "<KeyPress-space>"):
            self.root.bind_all(sequence, self._emergency_key, add="+")

    def _emergency_key(self, event: tk.Event[Any]) -> str | None:
        if not is_emergency_key(str(event.keysym), str(event.char)):
            return None
        now = time.monotonic()
        if now - self._last_estop_key >= 0.15:
            self._last_estop_key = now
            self._emergency_stop()
        return "break"

    def _emergency_stop(self) -> None:
        if self.controller.state.connected:
            self.controller.emergency_stop()

    def refresh_ports(self) -> None:
        try:
            ports = self.controller.discover_ports()
        except Exception as exc:
            self._show_error(exc)
            return
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connection(self) -> None:
        if self.controller.state.connected:
            self.controller.disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            self._show_error(GuiInputError("请选择串口"))
            return
        self.controller.connect(port, legacy=self.legacy_var.get())

    def _stage_parameter(self) -> None:
        selected = self.parameter_tree.selection()
        if not selected:
            self._show_error(GuiInputError("请先选择参数"))
            return
        key = str(self.parameter_tree.item(selected[0], "values")[0])
        value = self.parameter_value_var.get().strip()
        if not value:
            self._show_error(GuiInputError("请输入 staged 值"))
            return
        self.controller.stage_parameter(key, value)

    def _parameter_selected(self, _event: Any = None) -> None:
        selected = self.parameter_tree.selection()
        if selected:
            values = self.parameter_tree.item(selected[0], "values")
            if len(values) >= 3:
                self.parameter_value_var.set(str(values[2]))

    def _arm(self, builder: Callable[[], ActionDraft]) -> None:
        try:
            self.controller.arm_action(builder())
        except (GuiInputError, ValueError) as exc:
            self._show_error(exc)

    def _confirm_action(self) -> None:
        try:
            if not self.controller.confirm_action():
                raise GuiInputError("当前没有可 Confirm 的有效 Arm token")
        except GuiInputError as exc:
            self._show_error(exc)

    def _wizard_items(self) -> tuple[ActionDraft, ...]:
        drafts = wizard_drafts(
            self.wizard_speed_var.get(), self.wizard_duration_var.get(), self.wizard_ratio_var.get()
        )
        capabilities = self.controller.state.capabilities
        return tuple(
            draft for draft in drafts if draft.required_capabilities.issubset(capabilities)
        )

    def _wizard_move(self, delta: int) -> None:
        try:
            count = len(self._wizard_items())
        except GuiInputError as exc:
            self._show_error(exc)
            return
        if count == 0:
            self._show_error(GuiInputError("设备未发布任何向导动作能力"))
            return
        self._wizard_index = max(0, min(count - 1, self._wizard_index + delta))
        self._update_wizard_label()

    def _wizard_arm(self) -> None:
        try:
            drafts = self._wizard_items()
            if not drafts:
                raise GuiInputError("设备未发布任何向导动作能力")
            self.controller.arm_action(drafts[self._wizard_index])
        except (GuiInputError, ValueError) as exc:
            self._show_error(exc)

    def _update_wizard_label(self) -> None:
        try:
            drafts = self._wizard_items()
            if not drafts:
                self.wizard_step_var.set("连接 V2 后按设备 capability 显示支持的步骤")
                return
            draft = drafts[min(self._wizard_index, len(drafts) - 1)]
            self.wizard_step_var.set(
                f"步骤 {self._wizard_index + 1}/{len(drafts)}  {draft.label}  "
                f"L={draft.target_l_mm_s}  R={draft.target_r_mm_s}  "
                f"angle={draft.angle_mdeg / 1000:g}°  {draft.duration_ms / 1000:g}s"
            )
        except GuiInputError as exc:
            self.wizard_step_var.set(f"输入无效：{exc}")

    def _choose_record_path(self) -> None:
        path = filedialog.asksaveasfilename(
            title="保存 M6 记录",
            defaultextension=".csv",
            filetypes=(("CSV", "*.csv"), ("JSON", "*.json"), ("All files", "*.*")),
        )
        if path:
            self.record_path_var.set(path)

    def _toggle_recording(self) -> None:
        if self.controller.state.recording:
            self.controller.stop_recording()
            return
        path = self.record_path_var.get().strip()
        if not path:
            self._show_error(GuiInputError("请选择录制路径"))
            return
        self.controller.start_recording(path)

    def _choose_replay_path(self) -> None:
        path = filedialog.askopenfilename(
            title="打开 M6 记录",
            filetypes=(("M6 records", "*.csv *.json"), ("All files", "*.*")),
        )
        if path:
            self.replay_path_var.set(path)

    def _replay(self) -> None:
        path = self.replay_path_var.get().strip()
        if path:
            self.controller.replay(path)
        else:
            self._show_error(GuiInputError("请选择回放文件"))

    def _score(self) -> None:
        path = self.replay_path_var.get().strip()
        if path:
            self.controller.score(path)
        else:
            self._show_error(GuiInputError("请选择评分文件"))

    def _show_error(self, exc: Exception) -> None:
        self.controller.state.last_error = str(exc)
        self.controller.state.status_text = f"输入错误：{exc}"
        self.controller.state.generation += 1
        self._sync_state(force=True)

    def _poll_loop(self) -> None:
        if self._closed:
            return
        self.controller.tick()
        self.controller.poll()
        self._sync_state()
        self._poll_id = self.root.after(POLL_MS, self._poll_loop)

    def _chart_loop(self) -> None:
        if self._closed:
            return
        self._update_charts()
        self._chart_id = self.root.after(CHART_MS, self._chart_loop)

    def _sync_state(self, *, force: bool = False) -> None:
        state = self.controller.state
        if not force and state.generation == self._last_generation:
            self._update_lease()
            return
        self._last_generation = state.generation
        self.status_var.set(state.status_text)
        self.error_var.set(state.last_error)
        connected = state.connected
        self.connect_button.configure(
            text="断开" if connected else ("连接（显式 V1）" if self.legacy_var.get() else "连接（要求 V2）"),
            state="normal" if state.phase != "connecting" else "disabled",
        )
        self.estop_button.configure(state="normal" if connected else "disabled")
        self.port_combo.configure(state="disabled" if connected else "normal")
        identity = object_mapping(state.identity)
        firmware = identity.get("firmware_id", identity.get("firmware", "—"))
        schema = identity.get("parameter_schema", identity.get("schema", "—"))
        source = identity.get("parameter_source", identity.get("source", "—"))
        self.identity_var.set(
            f"固件 {firmware}  | schema {schema}  | 参数来源 {getattr(source, 'name', source)}  | 协议 V{state.protocol_version or '—'}"
        )
        self.capability_var.set(
            "能力：" + (", ".join(sorted(state.capabilities)) if state.capabilities else "—")
        )
        for key, variable in self.diag_vars.items():
            variable.set(model_text(state.diagnostics.get(key)))
        self._sync_capability_visibility()
        self._sync_parameters()
        self._update_lease()
        self.record_button.configure(
            text="停止录制" if state.recording else "开始录制",
            state="normal" if connected and state.protocol_version == 2 else "disabled",
        )
        if state.replay_result is not None or state.score_result is not None:
            self.replay_text.configure(state="normal")
            self.replay_text.delete("1.0", "end")
            if state.replay_result is not None:
                self.replay_text.insert("end", "回放\n" + model_text(state.replay_result, max_fields=40) + "\n\n")
            if state.score_result is not None:
                self.replay_text.insert("end", "评分\n" + model_text(state.score_result, max_fields=40))
            self.replay_text.configure(state="disabled")

    def _sync_capability_visibility(self) -> None:
        state = self.controller.state
        visible = feature_visibility(
            state.capabilities, connected=state.connected, protocol=state.protocol_version
        )
        self.notebook.tab(self.parameter_tab, state="normal" if visible["parameter_tab"] else "hidden")
        self.notebook.tab(self.action_tab, state="normal" if visible["action_tab"] else "hidden")
        self.notebook.tab(self.wizard_tab, state="normal" if visible["wizard_tab"] else "hidden")
        if visible["heading"]:
            self.heading_frame.grid()
        else:
            self.heading_frame.grid_remove()
        if visible["configurable_turn"]:
            self.turn_frame.grid()
        else:
            self.turn_frame.grid_remove()
        if visible["parameter_stage"]:
            self.stage_button.grid()
        else:
            self.stage_button.grid_remove()
        for button, feature in self.parameter_operation_buttons:
            button.pack_forget()
            if visible[feature]:
                button.pack(side="left", padx=(0, 8))
        if visible["recording"]:
            self.record_frame.grid()
        else:
            self.record_frame.grid_remove()

    def _sync_parameters(self) -> None:
        selected_key = ""
        selected = self.parameter_tree.selection()
        if selected:
            values = self.parameter_tree.item(selected[0], "values")
            if values:
                selected_key = str(values[0])
        self.parameter_tree.delete(*self.parameter_tree.get_children())
        selected_id = ""
        for key, row in sorted(self.controller.state.parameters.items()):
            bounds = ""
            if row.minimum is not None or row.maximum is not None:
                bounds = f"{row.minimum} .. {row.maximum}"
            item_id = self.parameter_tree.insert(
                "",
                "end",
                values=(key, row.active, row.staged, row.persisted, bounds, row.unit),
            )
            if key == selected_key:
                selected_id = item_id
        if selected_id:
            self.parameter_tree.selection_set(selected_id)

    def _update_lease(self) -> None:
        lease = self.controller.state.armed
        if lease is None:
            self.lease_var.set("尚未 Arm；每个动作/向导步骤都必须重新 Arm")
            self.confirm_button.configure(state="disabled")
            return
        remaining = max(0.0, lease.expires_at - self.controller.clock())
        self.lease_var.set(f"token={lease.token}  {lease.label}  剩余 {remaining:0.1f}s")
        self.confirm_button.configure(state="normal" if remaining > 0 else "disabled")

    def _update_charts(self) -> None:
        if not self._chart_ready:
            return
        points = list(self.controller.state.telemetry)
        if not points:
            return
        end = max(point.time_s for point in points if math.isfinite(point.time_s))
        points = [point for point in points if math.isfinite(point.time_s) and point.time_s >= end - 12.0]
        if not points:
            return
        x = [point.time_s - end for point in points]
        self._lines["left"].set_data(x, [point.left_mm_s for point in points])
        self._lines["right"].set_data(x, [point.right_mm_s for point in points])
        self._lines["yaw"].set_data(x, [point.yaw_deg for point in points])
        self._lines["rate"].set_data(x, [point.yaw_rate_dps for point in points])
        for index in range(8):
            self._lines[f"gray{index}"].set_data(
                x,
                [point.grayscale[index] if len(point.grayscale) > index else math.nan for point in points],
            )
        self._lines["centroid"].set_data(x, [point.centroid for point in points])
        for axis in self.axes.values():
            axis.relim()
            axis.autoscale_view()
            axis.set_xlim(-12.0, 0.0)
        self.chart_canvas.draw_idle()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._poll_id is not None:
            self.root.after_cancel(self._poll_id)
        if self._chart_id is not None:
            self.root.after_cancel(self._chart_id)
        self.controller.shutdown()
        self.root.destroy()


def run_gui(controller: Controller, *, fake: bool = False) -> None:
    root = tk.Tk()
    M6View(root, controller, fake=fake)
    root.mainloop()
