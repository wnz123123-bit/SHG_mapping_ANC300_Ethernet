from __future__ import annotations

import csv
import faulthandler
import math
import queue
import subprocess
import sys
import threading
import time
from contextlib import nullcontext
from dataclasses import asdict, dataclass, replace
from datetime import datetime
from pathlib import Path
from tkinter import BOTH, END, LEFT, RIGHT, BooleanVar, DoubleVar, IntVar, StringVar, Tk, Canvas, Label, Text, filedialog, messagebox, simpledialog
from tkinter import ttk

from anc300_stage import ANC300ScanStage, ANC300StageSettings, SimulatedANC300Stage
from app_config import default_config, load_config as load_app_config, save_config as save_app_config
from reader import read_value
from pmt_reader import (
    GATE_TIME_OPTIONS_MS,
    PMTCounter,
    PMTSettings,
    format_gate_time,
    interruptible_sleep,
    validate_pmt_settings,
)
from stage_controller import MTStage, SimulatedStage, StageError, StageSettings


APP_DIR = Path(__file__).resolve().parent
CONFIG_PATH = APP_DIR / "config.json"
CRASH_LOG_PATH = APP_DIR / "crash_log.txt"
MT_TOOL_PROCESS_NAMES = {
    "mthelper.exe",
    "mthelper_v3.exe",
    "mtsimulator.exe",
    "ioconfig.exe",
    "upgrade.exe",
    "mt_limit_tool.exe",
}
CREATE_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)
ANC_VOLTAGE_METADATA_FIELDS = (
    "x_voltage_v",
    "y_voltage_v",
    "x_origin_voltage_v",
    "y_origin_voltage_v",
    "x_um_per_v",
    "y_um_per_v",
    "x_direction",
    "y_direction",
    "calibration_source",
    "position_estimated",
)


@dataclass(frozen=True)
class RotatorMotionSnapshot:
    axis: int
    steps_per_degree: float
    direction_sign: int
    acc: int
    dec: int
    max_v: int
    start_v: int
    timeout_s: float

    def move_params(self) -> dict:
        return {
            "acc": self.acc,
            "dec": self.dec,
            "max_v": self.max_v,
            "start_v": self.start_v,
            "timeout_s": self.timeout_s,
            "steps_per_degree": self.steps_per_degree,
            "direction_sign": self.direction_sign,
        }


def measurement_source(settings: PMTSettings, simulate: bool, *, allow_reader_simulation: bool = False) -> str:
    if settings.enabled:
        return "pmt_simulation" if simulate else "pmt_hardware"
    if simulate and allow_reader_simulation:
        return "reader_simulation"
    raise ValueError("Real mapping requires the PMT hardware backend to be enabled.")


def open_result_csv(output_dir, stem, fieldnames):
    """Exclusively create a collision-safe CSV and flush its header."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    collision = 0
    while True:
        suffix = "" if collision == 0 else f"_{collision:02d}"
        path = output_dir / f"{stem}_{timestamp}{suffix}.csv"
        try:
            handle = path.open("x", newline="", encoding="utf-8-sig")
            break
        except FileExistsError:
            collision += 1
    try:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        handle.flush()
    except Exception:
        handle.close()
        raise
    return path, handle, writer


def decimal_points(start: float, end: float, step: float):
    if step <= 0:
        raise ValueError("Step must be greater than 0.")
    direction = 1.0 if end >= start else -1.0
    span = abs(end - start)
    count = int(math.floor(span / step + 1e-9)) + 1
    return [round(start + direction * i * step, 6) for i in range(count)]


def scan_points(x_start, x_end, y_start, y_end, step_um, serpentine=True):
    xs = decimal_points(x_start, x_end, step_um)
    ys = decimal_points(y_start, y_end, step_um)
    for row, y_um in enumerate(ys):
        row_xs = list(reversed(xs)) if serpentine and row % 2 else xs
        for col, x_um in enumerate(row_xs):
            yield row, col, x_um, y_um


def angle_scan_points(start_angle, stop_angle, step_deg):
    if step_deg <= 0:
        raise ValueError("Angle step must be greater than 0.")
    start_angle = float(start_angle)
    stop_angle = float(stop_angle)
    step_deg = abs(float(step_deg))
    if math.isclose(start_angle, stop_angle, abs_tol=1e-9):
        return [round(start_angle, 6)]
    direction = 1.0 if stop_angle > start_angle else -1.0
    signed_step = direction * step_deg
    values = []
    current = start_angle
    if direction > 0:
        while current < stop_angle - 1e-9:
            values.append(round(current, 6))
            current += signed_step
    else:
        while current > stop_angle + 1e-9:
            values.append(round(current, 6))
            current += signed_step
    if not values or not math.isclose(values[-1], stop_angle, abs_tol=1e-9):
        values.append(round(stop_angle, 6))
    return values


class MappingApp(Tk):
    def __init__(self):
        super().__init__()
        self.title("SHG Mapping 控制程序")
        self.geometry("1280x900")
        self.minsize(1180, 820)

        self.config_data = load_app_config(CONFIG_PATH) if CONFIG_PATH.exists() else default_config(APP_DIR)
        self.stage = None
        self.rotator_stage = None
        self.scan_thread = None
        self.rotator_thread = None
        self.point_move_thread = None
        self.origin_move_thread = None
        self.ground_thread = None
        self.angle_scan_thread = None
        self.pmt_test_thread = None
        self.stop_event = threading.Event()
        self._motion_state_lock = threading.RLock()
        self._motion_active = False
        self._motion_label = None
        self._rotator_needs_recovery = False
        self.events = queue.Queue()
        self.points = []
        self.selected_point = None
        self.angle_point_queue = []
        self.angle_points = []
        self.latest_csv = None
        self.latest_angle_csv = None
        self._last_rotator_poll_error = None
        self._last_rotator_display = None
        self._last_mt_tool_check_at = 0.0
        self._cached_mt_tool_conflicts = []
        self._last_mt_tool_warning = None
        self._active_scan_csv = None
        self._active_scan_point = None
        self._angle_motion_started = False
        self._draw_pending = False
        self._last_draw_at = 0.0
        self._draw_min_interval_ms = 250
        self._max_log_lines = 600
        self._plot_bounds = None
        self._cell_hit_bounds = []
        self._motion_sensitive_buttons = []

        self._build_variables()
        self._build_ui()
        self._normalize_initial_pmt_gate_time()
        self._poll_events()
        self._poll_rotator_angle()
        self._update_point_count()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_variables(self):
        anc300 = self.config_data["anc300"]
        scan = self.config_data["scan"]
        pmt = self.config_data["pmt"]
        rotator = self.config_data["rotator"]
        angle_scan = self.config_data.get("angle_scan", {})
        self.simulation_mode = BooleanVar(value=bool(self.config_data["simulation_mode"]))
        self.anc_host = StringVar(value=str(anc300["host"]))
        self.anc_port = IntVar(value=int(anc300["port"]))
        self.anc_password = StringVar(value=str(anc300["password"]))
        self.anc_timeout_s = DoubleVar(value=float(anc300["timeout_s"]))
        self.anc_x_axis = IntVar(value=int(anc300["x_axis"]))
        self.anc_y_axis = IntVar(value=int(anc300["y_axis"]))
        self.anc_voltage_min_v = DoubleVar(value=float(anc300["voltage_min_v"]))
        self.anc_voltage_max_v = DoubleVar(value=float(anc300["voltage_max_v"]))
        self.anc_x_um_per_v = DoubleVar(value=float(anc300["x_um_per_v"]))
        self.anc_y_um_per_v = DoubleVar(value=float(anc300["y_um_per_v"]))
        self.anc_x_direction = IntVar(value=int(anc300["x_direction"]))
        self.anc_y_direction = IntVar(value=int(anc300["y_direction"]))
        self.anc_calibration_source = StringVar(value=str(anc300["calibration_source"]))
        self.anc_max_ramp_step_v = DoubleVar(value=float(anc300["max_ramp_step_v"]))
        self.anc_pre_read_settle_s = DoubleVar(value=float(anc300["pre_read_settle_s"]))
        self.anc_hardware_profile_confirmed = BooleanVar(value=bool(anc300["hardware_profile_confirmed"]))

        self.x_start_um = DoubleVar(value=float(scan["x_start_um"]))
        self.x_end_um = DoubleVar(value=float(scan["x_end_um"]))
        self.y_start_um = DoubleVar(value=float(scan["y_start_um"]))
        self.y_end_um = DoubleVar(value=float(scan["y_end_um"]))
        self.step_um = DoubleVar(value=float(scan["step_um"]))
        self.dwell_s = DoubleVar(value=float(scan["dwell_ms"]) / 1000.0)
        self.serpentine = BooleanVar(value=bool(scan["serpentine"]))
        self.return_to_origin = BooleanVar(value=bool(scan["return_to_origin"]))
        self.output_dir = StringVar(value=scan["output_dir"])

        self.pmt_enabled = BooleanVar(value=bool(pmt["enabled"]))
        self.pmt_dll_path = StringVar(value=pmt["dll_path"])
        self.pmt_gate_time_ms = StringVar(value=format_gate_time(float(pmt["gate_time_ms"])))
        self.pmt_samples_to_average = IntVar(value=int(pmt["samples_to_average"]))
        self.pmt_sample_extra_wait_s = DoubleVar(value=float(pmt["sample_extra_wait_s"]))
        self.pmt_transfer_mode = IntVar(value=int(pmt["transfer_mode"]))
        self.pmt_trigger_mode = IntVar(value=int(pmt["trigger_mode"]))
        self.pmt_trigger_edge = IntVar(value=int(pmt["trigger_edge"]))
        self.pmt_simulate_value = DoubleVar(value=float(pmt["simulate_value"]))
        self.pmt_simulate_noise = DoubleVar(value=float(pmt["simulate_noise"]))

        self.rotator_dll_path = StringVar(value=str(rotator["dll_path"]))
        self.rotator_connection_type = StringVar(value=str(rotator["connection_type"]))
        self.rotator_com_port = StringVar(value=str(rotator["com_port"]))
        self.rotator_ip_address = StringVar(value=str(rotator["ip_address"]))
        self.rotator_ip_port = IntVar(value=int(rotator["ip_port"]))
        self.rotator_axis_display = IntVar(value=int(rotator["axis_display"]))
        self.rotator_usb_device_index = IntVar(value=int(rotator["usb_device_index"]))
        self.rotator_usb_serial = StringVar(value=str(rotator.get("usb_serial", "")))
        self.rotator_steps_per_degree = DoubleVar(value=float(rotator["steps_per_degree"]))
        self.rotator_direction_sign = IntVar(value=int(rotator.get("direction_sign", -1)))
        self.rotator_target_angle_deg = DoubleVar(value=float(rotator["target_angle_deg"]))
        self.rotator_acc = IntVar(value=int(rotator["acc"]))
        self.rotator_dec = IntVar(value=int(rotator["dec"]))
        self.rotator_max_v = IntVar(value=int(rotator["max_v"]))
        self.rotator_start_v = IntVar(value=int(rotator["start_v"]))
        self.rotator_move_timeout_s = DoubleVar(value=float(rotator["move_timeout_s"]))

        self.angle_start_deg = DoubleVar(value=float(angle_scan.get("start_angle_deg", 0.0)))
        self.angle_stop_deg = DoubleVar(value=float(angle_scan.get("stop_angle_deg", 180.0)))
        self.angle_step_deg = DoubleVar(value=float(angle_scan.get("step_deg", 10.0)))
        self.angle_settle_s = DoubleVar(value=float(angle_scan.get("settle_s", 0.2)))
        self.angle_return_to_start = BooleanVar(value=bool(angle_scan.get("return_to_start", True)))
        self.angle_selected_point_text = StringVar(value="Selected point: none")
        self.angle_progress_text = StringVar(value="Angle scan: idle")
        self.angle_latest_count_text = StringVar(value="Latest angle count: --")

        self.status_text = StringVar(value="未连接")
        self.run_indicator_text = StringVar(value="空闲")
        self.position_text = StringVar(value="X: -- um, Y: -- um")
        self.progress_text = StringVar(value="0 / 0")
        self.point_count_text = StringVar(value="预计点数: --")
        self.pmt_status_text = StringVar(value="PMT: 未启用")
        self.rotator_position_text = StringVar(value="半波片: -- deg")
        self.anc_identity_text = StringVar(value="版本/序列号: -- / --")
        self.anc_x_status_text = StringVar(value="X: axis --, mode --, voltage -- V, origin -- V")
        self.anc_y_status_text = StringVar(value="Y: axis --, mode --, voltage -- V, origin -- V")
        self.anc_calibration_text = StringVar(value="Calibration: --")

    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=BOTH, expand=True)

        style = ttk.Style(self)
        style.configure("Primary.TButton", padding=(8, 6))

        left_shell = ttk.Frame(root, width=390)
        left_shell.pack(side=LEFT, fill="y")
        left_shell.pack_propagate(False)
        right_panel = ttk.Frame(root)
        right_panel.pack(side=RIGHT, fill=BOTH, expand=True, padx=(10, 0))

        control_scroll = ttk.Frame(left_shell)
        control_scroll.pack(side="top", fill=BOTH, expand=True)
        self.control_canvas = Canvas(control_scroll, bg="#f8fafc", highlightthickness=0, borderwidth=0)
        control_scrollbar = ttk.Scrollbar(control_scroll, orient="vertical", command=self.control_canvas.yview)
        self.control_canvas.configure(yscrollcommand=control_scrollbar.set)
        control_scrollbar.pack(side=RIGHT, fill="y")
        self.control_canvas.pack(side=LEFT, fill=BOTH, expand=True)

        left_panel = ttk.Frame(self.control_canvas)
        self.control_window = self.control_canvas.create_window((0, 0), window=left_panel, anchor="nw")
        left_panel.bind("<Configure>", self._on_control_frame_configure)
        self.control_canvas.bind("<Configure>", self._on_control_canvas_configure)
        self.control_canvas.bind("<Enter>", self._activate_control_mousewheel)
        self.control_canvas.bind("<Leave>", self._deactivate_control_mousewheel)

        self._build_connection_frame(left_panel)
        self._build_motion_frame(left_panel)
        self._build_rotator_frame(left_panel)
        self._build_pmt_frame(left_panel)
        self._build_scan_frame(left_panel)
        self._build_angle_scan_frame(left_panel)
        self._bind_control_mousewheel(left_panel)
        self._build_actions_frame(left_shell)

        canvas_frame = ttk.LabelFrame(right_panel, text="Mapping 预览")
        canvas_frame.pack(fill=BOTH, expand=True)
        self.canvas = Canvas(canvas_frame, bg="#f8fafc", highlightthickness=1, highlightbackground="#cbd5e1")
        self.canvas.pack(fill=BOTH, expand=True, padx=8, pady=8)
        self.canvas.bind("<Configure>", lambda _event: self._schedule_draw(force=True))
        self.canvas.bind("<Button-1>", self.on_mapping_click)

        info_frame = ttk.Frame(right_panel)
        info_frame.pack(fill="x", pady=(8, 0))
        ttk.Label(info_frame, textvariable=self.status_text).pack(side=LEFT)
        self.run_indicator = Label(
            info_frame,
            textvariable=self.run_indicator_text,
            bg="#16a34a",
            fg="white",
            padx=10,
            pady=2,
            font=("", 9, "bold"),
        )
        self.run_indicator.pack(side=LEFT, padx=(16, 0))
        ttk.Label(info_frame, textvariable=self.position_text).pack(side=LEFT, padx=20)
        ttk.Label(info_frame, textvariable=self.progress_text).pack(side=RIGHT)

        angle_frame = ttk.LabelFrame(right_panel, text="Angle dependence")
        angle_frame.pack(fill="both", expand=False, pady=(8, 0))
        angle_results_frame = ttk.Frame(angle_frame)
        angle_results_frame.pack(side=LEFT, fill="both", expand=True, padx=(8, 0), pady=8)
        queue_frame = ttk.LabelFrame(angle_frame, text="Point order")
        queue_frame.pack(side=RIGHT, fill="y", padx=8, pady=8)
        self.angle_table = ttk.Treeview(
            angle_results_frame,
            columns=("angle", "count", "samples"),
            show="headings",
            height=6,
        )
        self.angle_table.heading("angle", text="Angle (deg)")
        self.angle_table.heading("count", text="Avg count")
        self.angle_table.heading("samples", text="Samples")
        self.angle_table.column("angle", width=100, anchor="center")
        self.angle_table.column("count", width=120, anchor="center")
        self.angle_table.column("samples", width=420, anchor="w")
        angle_scrollbar = ttk.Scrollbar(angle_results_frame, orient="vertical", command=self.angle_table.yview)
        self.angle_table.configure(yscrollcommand=angle_scrollbar.set)
        self.angle_table.pack(side=LEFT, fill="both", expand=True)
        angle_scrollbar.pack(side=RIGHT, fill="y")
        self.angle_queue_table = ttk.Treeview(
            queue_frame,
            columns=("order", "xy", "status"),
            show="headings",
            height=6,
        )
        self.angle_queue_table.heading("order", text="#")
        self.angle_queue_table.heading("xy", text="X, Y")
        self.angle_queue_table.heading("status", text="Status")
        self.angle_queue_table.column("order", width=36, anchor="center", stretch=False)
        self.angle_queue_table.column("xy", width=112, anchor="center", stretch=False)
        self.angle_queue_table.column("status", width=72, anchor="center", stretch=False)
        self.angle_queue_table.pack(fill="y", expand=False)

        bottom_frame = ttk.Frame(right_panel)
        bottom_frame.pack(fill="both", expand=False, pady=(8, 0))

        log_frame = ttk.LabelFrame(bottom_frame, text="运行日志")
        log_frame.pack(side=LEFT, fill="both", expand=True, padx=(0, 8))
        self.log_box = Text(log_frame, height=5, wrap="word")
        self.log_box.pack(fill=BOTH, expand=True, padx=6, pady=4)

    def _on_control_frame_configure(self, _event=None):
        self.control_canvas.configure(scrollregion=self.control_canvas.bbox("all"))

    def _on_control_canvas_configure(self, event):
        self.control_canvas.itemconfigure(self.control_window, width=event.width)

    def _bind_control_mousewheel(self, widget):
        widget.bind("<Enter>", self._activate_control_mousewheel)
        widget.bind("<Leave>", self._deactivate_control_mousewheel)
        for child in widget.winfo_children():
            self._bind_control_mousewheel(child)

    def _activate_control_mousewheel(self, _event=None):
        self.bind_all("<MouseWheel>", self._on_control_mousewheel)

    def _deactivate_control_mousewheel(self, _event=None):
        self.unbind_all("<MouseWheel>")

    def _on_control_mousewheel(self, event):
        if event.delta == 0:
            return
        step = int(-event.delta / 120)
        if step == 0:
            step = -1 if event.delta > 0 else 1
        self.control_canvas.yview_scroll(step, "units")

    def _grid_labeled_entries(self, frame, labels, start_row=0, keyrelease_vars=()):
        keyrelease_var_ids = {id(var) for var in keyrelease_vars}
        for offset, (text, var) in enumerate(labels):
            row = start_row + offset
            ttk.Label(frame, text=text).grid(row=row, column=0, sticky="w", padx=8, pady=3)
            entry = ttk.Entry(frame, textvariable=var)
            entry.grid(row=row, column=1, sticky="ew", padx=8, pady=3)
            if id(var) in keyrelease_var_ids:
                entry.bind("<KeyRelease>", lambda _event: self._update_point_count())
        return start_row + len(labels)

    def _grid_button(
        self,
        frame,
        text,
        command,
        row,
        column,
        columnspan=1,
        style=None,
        motion_sensitive=False,
        pady=5,
    ):
        kwargs = {"text": text, "command": command}
        if style is not None:
            kwargs["style"] = style
        button = ttk.Button(frame, **kwargs)
        button.grid(row=row, column=column, columnspan=columnspan, sticky="ew", padx=8, pady=pady)
        if motion_sensitive:
            self._motion_sensitive_buttons.append(button)
        return button

    def _block_combobox_mousewheel(self, event):
        return "break"

    def _bind_combobox_mousewheel_block(self, combo):
        combo.bind("<MouseWheel>", self._block_combobox_mousewheel)
        combo.bind("<Button-4>", self._block_combobox_mousewheel)
        combo.bind("<Button-5>", self._block_combobox_mousewheel)

    def _on_pmt_gate_time_selected(self, _event=None):
        try:
            self.pmt_gate_time_ms.set(self._normalized_pmt_gate_time_text())
        except Exception as exc:
            messagebox.showerror("PMT gate time error", str(exc))

    def _build_connection_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="控制器连接")
        frame.pack(fill="x", pady=(0, 8))
        self.simulation_checkbutton = ttk.Checkbutton(frame, text="模拟模式（不移动硬件）", variable=self.simulation_mode)
        self.simulation_checkbutton.grid(row=0, column=0, columnspan=2, sticky="w", padx=8, pady=4)
        ttk.Label(frame, text="ANC300 主机/IP").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(frame, textvariable=self.anc_host).grid(row=1, column=1, sticky="ew", padx=8, pady=4)
        ttk.Label(frame, text="TCP 端口").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(frame, textvariable=self.anc_port).grid(row=2, column=1, sticky="ew", padx=8, pady=4)
        ttk.Label(frame, text="密码").grid(row=3, column=0, sticky="w", padx=8, pady=4)
        self.anc_password_entry = ttk.Entry(frame, textvariable=self.anc_password, show="*")
        self.anc_password_entry.grid(row=3, column=1, sticky="ew", padx=8, pady=4)
        ttk.Label(frame, text="超时(s)").grid(row=4, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(frame, textvariable=self.anc_timeout_s).grid(row=4, column=1, sticky="ew", padx=8, pady=4)
        ttk.Label(frame, text="警告：密码将以明文保存在 config.json").grid(row=5, column=0, columnspan=2, sticky="w", padx=8, pady=4)
        ttk.Button(frame, text="连接 ANC300", command=self.connect_stage_only).grid(row=6, column=0, sticky="ew", padx=8, pady=(6, 2))
        ttk.Button(frame, text="连接 HWP", command=self.connect_rotator_only).grid(row=6, column=1, sticky="ew", padx=8, pady=(6, 2))
        ttk.Button(frame, text="断开全部", command=self.disconnect_stage).grid(row=7, column=0, columnspan=2, sticky="ew", padx=8, pady=(2, 6))
        frame.columnconfigure(1, weight=1)

    def _build_motion_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="ANC300 状态与校准")
        frame.pack(fill="x", pady=(0, 8))
        labels = [
            ("X axis", self.anc_x_axis),
            ("Y axis", self.anc_y_axis),
            ("Voltage min (V)", self.anc_voltage_min_v),
            ("Voltage max (V)", self.anc_voltage_max_v),
            ("X um/V", self.anc_x_um_per_v),
            ("Y um/V", self.anc_y_um_per_v),
            ("X direction", self.anc_x_direction),
            ("Y direction", self.anc_y_direction),
            ("Max ramp step (V)", self.anc_max_ramp_step_v),
            ("Pre-read settle (s)", self.anc_pre_read_settle_s),
        ]
        row = self._grid_labeled_entries(frame, labels)
        ttk.Label(frame, textvariable=self.anc_identity_text).grid(row=row, column=0, columnspan=2, sticky="w", padx=8, pady=3)
        ttk.Label(frame, textvariable=self.anc_x_status_text).grid(row=row + 1, column=0, columnspan=2, sticky="w", padx=8, pady=3)
        ttk.Label(frame, textvariable=self.anc_y_status_text).grid(row=row + 2, column=0, columnspan=2, sticky="w", padx=8, pady=3)
        ttk.Label(frame, textvariable=self.anc_calibration_text).grid(row=row + 3, column=0, columnspan=2, sticky="w", padx=8, pady=3)
        ttk.Label(frame, textvariable=self.position_text).grid(row=row + 4, column=0, columnspan=2, sticky="w", padx=8, pady=3)
        self._grid_button(
            frame,
            "Confirm ANSxyz100std/LT profile",
            self.confirm_anc300_hardware_profile,
            row + 5,
            0,
            columnspan=2,
            motion_sensitive=True,
            pady=3,
        )
        self._grid_button(
            frame,
            "Enable offset outputs",
            self.enable_anc300_outputs,
            row + 6,
            0,
            columnspan=2,
            motion_sensitive=True,
            pady=3,
        )
        self._grid_button(
            frame,
            "Ramp to 0 V and GND",
            self.ground_anc300_outputs,
            row + 7,
            0,
            columnspan=2,
            motion_sensitive=True,
            pady=3,
        )
        self._grid_button(
            frame,
            "校准X轴",
            lambda: self.calibrate_axis("X"),
            row + 8,
            0,
            motion_sensitive=True,
        )
        self._grid_button(
            frame,
            "校准Y轴",
            lambda: self.calibrate_axis("Y"),
            row + 8,
            1,
            motion_sensitive=True,
        )
        frame.columnconfigure(1, weight=1)

    def _build_rotator_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="半波片旋转")
        frame.pack(fill="x", pady=(0, 8))
        labels = [
            ("HWP MT DLL", self.rotator_dll_path),
            ("HWP 连接方式", self.rotator_connection_type),
            ("HWP COM", self.rotator_com_port),
            ("HWP IP", self.rotator_ip_address),
            ("HWP port", self.rotator_ip_port),
            ("HWP USB device", self.rotator_usb_device_index),
            ("HWP USB serial", self.rotator_usb_serial),
            ("轴号(1-4)", self.rotator_axis_display),
            ("脉冲/度", self.rotator_steps_per_degree),
            ("方向系数", self.rotator_direction_sign),
            ("目标角度(deg)", self.rotator_target_angle_deg),
            ("加速度", self.rotator_acc),
            ("减速度", self.rotator_dec),
            ("最大速度", self.rotator_max_v),
            ("HWP start speed", self.rotator_start_v),
            ("超时(s)", self.rotator_move_timeout_s),
        ]
        self._grid_labeled_entries(frame, labels)
        ttk.Label(frame, textvariable=self.rotator_position_text).grid(row=len(labels), column=0, columnspan=2, sticky="w", padx=8, pady=4)
        self.read_rotator_button = self._grid_button(frame, "读取角度", self.read_rotator_position, len(labels) + 1, 0, motion_sensitive=True)
        self.move_rotator_button = self._grid_button(frame, "转到角度", self.move_rotator, len(labels) + 1, 1, motion_sensitive=True)
        self.diagnose_axes_button = self._grid_button(frame, "轴状态诊断", self.diagnose_axes, len(labels) + 2, 0, columnspan=2, motion_sensitive=True)
        frame.columnconfigure(1, weight=1)

    def _build_pmt_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="PMT计数")
        frame.pack(fill="x", pady=(0, 8))
        ttk.Checkbutton(frame, text="启用PMT读取", variable=self.pmt_enabled).grid(row=0, column=0, columnspan=2, sticky="w", padx=8, pady=4)
        ttk.Label(frame, text="Gate Time(ms)").grid(row=1, column=0, sticky="w", padx=8, pady=3)
        self.pmt_gate_time_combo = ttk.Combobox(
            frame,
            textvariable=self.pmt_gate_time_ms,
            values=[format_gate_time(value) for value in GATE_TIME_OPTIONS_MS],
            width=12,
            state="readonly",
        )
        self.pmt_gate_time_combo.grid(row=1, column=1, sticky="ew", padx=8, pady=3)
        self.pmt_gate_time_combo.bind("<<ComboboxSelected>>", self._on_pmt_gate_time_selected)
        self._bind_combobox_mousewheel_block(self.pmt_gate_time_combo)
        ttk.Label(frame, text="平均次数").grid(row=2, column=0, sticky="w", padx=8, pady=3)
        ttk.Entry(frame, textvariable=self.pmt_samples_to_average).grid(row=2, column=1, sticky="ew", padx=8, pady=3)
        ttk.Label(frame, text="读数余量(s)").grid(row=3, column=0, sticky="w", padx=8, pady=3)
        ttk.Entry(frame, textvariable=self.pmt_sample_extra_wait_s).grid(row=3, column=1, sticky="ew", padx=8, pady=3)
        ttk.Label(frame, text="PMT DLL").grid(row=4, column=0, sticky="w", padx=8, pady=3)
        ttk.Entry(frame, textvariable=self.pmt_dll_path).grid(row=4, column=1, sticky="ew", padx=8, pady=3)
        ttk.Label(frame, text="模拟基值").grid(row=5, column=0, sticky="w", padx=8, pady=3)
        ttk.Entry(frame, textvariable=self.pmt_simulate_value).grid(row=5, column=1, sticky="ew", padx=8, pady=3)
        ttk.Label(frame, text="模拟波动(±)").grid(row=6, column=0, sticky="w", padx=8, pady=3)
        ttk.Entry(frame, textvariable=self.pmt_simulate_noise).grid(row=6, column=1, sticky="ew", padx=8, pady=3)
        ttk.Label(frame, textvariable=self.pmt_status_text).grid(row=7, column=0, columnspan=2, sticky="w", padx=8, pady=4)
        self._grid_button(
            frame,
            "测试PMT读取",
            self.test_pmt_read,
            8,
            0,
            columnspan=2,
            motion_sensitive=True,
            pady=6,
        )
        frame.columnconfigure(1, weight=1)

    def _build_scan_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="扫描区域")
        frame.pack(fill="x", pady=(0, 8))
        labels = [
            ("X起点(um)", self.x_start_um),
            ("X终点(um)", self.x_end_um),
            ("Y起点(um)", self.y_start_um),
            ("Y终点(um)", self.y_end_um),
            ("步进(um)", self.step_um),
            ("读数后等待(s)", self.dwell_s),
        ]
        self._grid_labeled_entries(
            frame,
            labels,
            keyrelease_vars=(self.x_start_um, self.x_end_um, self.y_start_um, self.y_end_um, self.step_um),
        )
        option_row = len(labels)
        ttk.Checkbutton(frame, text="蛇形扫描", variable=self.serpentine).grid(row=option_row, column=0, columnspan=2, sticky="w", padx=8, pady=3)
        ttk.Checkbutton(frame, text="完成后回到零点", variable=self.return_to_origin).grid(row=option_row + 1, column=0, columnspan=2, sticky="w", padx=8, pady=3)
        ttk.Label(frame, textvariable=self.point_count_text).grid(row=option_row + 2, column=0, columnspan=2, sticky="w", padx=8, pady=5)
        ttk.Label(frame, text="保存目录").grid(row=option_row + 3, column=0, sticky="w", padx=8, pady=3)
        ttk.Entry(frame, textvariable=self.output_dir).grid(row=option_row + 3, column=1, sticky="ew", padx=8, pady=3)
        ttk.Button(frame, text="选择目录", command=self.choose_output_dir).grid(row=option_row + 4, column=0, columnspan=2, sticky="ew", padx=8, pady=6)
        frame.columnconfigure(1, weight=1)

    def _build_angle_scan_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="单点角度测量")
        frame.pack(fill="x", pady=(0, 8))
        ttk.Label(frame, textvariable=self.angle_selected_point_text).grid(row=0, column=0, columnspan=2, sticky="w", padx=8, pady=4)
        labels = [
            ("起始角度(deg)", self.angle_start_deg),
            ("结束角度(deg)", self.angle_stop_deg),
            ("角度步进(deg)", self.angle_step_deg),
            ("稳定时间(s)", self.angle_settle_s),
        ]
        self._grid_labeled_entries(frame, labels, start_row=1)
        option_row = len(labels) + 1
        ttk.Checkbutton(frame, text="测量后半波片回到起始角度", variable=self.angle_return_to_start).grid(
            row=option_row,
            column=0,
            columnspan=2,
            sticky="w",
            padx=8,
            pady=3,
        )
        ttk.Label(frame, textvariable=self.angle_progress_text).grid(row=option_row + 1, column=0, columnspan=2, sticky="w", padx=8, pady=3)
        ttk.Label(frame, textvariable=self.angle_latest_count_text).grid(row=option_row + 2, column=0, columnspan=2, sticky="w", padx=8, pady=3)
        self.move_selected_button = self._grid_button(
            frame,
            "移动到队列首点",
            self.move_to_selected_point,
            option_row + 3,
            0,
            motion_sensitive=True,
        )
        self.angle_scan_button = self._grid_button(
            frame,
            "开始角度测量",
            self.start_angle_scan,
            option_row + 3,
            1,
            motion_sensitive=True,
        )
        self._grid_button(frame, "清空角度结果", self.clear_angle_results, option_row + 4, 0, columnspan=2)
        frame.columnconfigure(1, weight=1)

    def _build_actions_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="操作")
        frame.pack(side="bottom", fill="x", pady=(8, 0))
        self.set_origin_button = self._grid_button(frame, "当前位置设为零点", self.set_origin, 0, 0, motion_sensitive=True)
        self.move_origin_button = self._grid_button(frame, "回到零点（原始位置）", self.move_origin, 0, 1, motion_sensitive=True)
        self.start_scan_button = self._grid_button(
            frame,
            "开始扫描",
            self.start_scan,
            1,
            0,
            style="Primary.TButton",
            motion_sensitive=True,
        )
        self._grid_button(frame, "停止", self.stop_scan, 1, 1)
        self._grid_button(frame, "保存配置", self.save_config, 2, 0, columnspan=2)
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

    def anc300_stage_settings(self):
        return ANC300StageSettings(
            host=self.anc_host.get().strip(),
            port=int(self.anc_port.get()),
            password=self.anc_password.get(),
            timeout_s=float(self.anc_timeout_s.get()),
            x_axis=int(self.anc_x_axis.get()),
            y_axis=int(self.anc_y_axis.get()),
            voltage_min_v=float(self.anc_voltage_min_v.get()),
            voltage_max_v=float(self.anc_voltage_max_v.get()),
            x_um_per_v=float(self.anc_x_um_per_v.get()),
            y_um_per_v=float(self.anc_y_um_per_v.get()),
            x_direction=int(self.anc_x_direction.get()),
            y_direction=int(self.anc_y_direction.get()),
            calibration_source=self.anc_calibration_source.get().strip(),
            max_ramp_step_v=float(self.anc_max_ramp_step_v.get()),
            pre_read_settle_s=float(self.anc_pre_read_settle_s.get()),
            hardware_profile_confirmed=bool(self.anc_hardware_profile_confirmed.get()),
        )

    def rotator_stage_settings(self):
        axis = self.rotator_api_axis()
        return StageSettings(
            dll_path=self.rotator_dll_path.get(),
            connection_type=self.rotator_connection_type.get(),
            com_port=self.rotator_com_port.get(),
            ip_address=self.rotator_ip_address.get(),
            ip_port=int(self.rotator_ip_port.get()),
            usb_device_index=int(self.rotator_usb_device_index.get()),
            usb_serial=self.rotator_usb_serial.get().strip(),
            x_axis=axis,
            y_axis=axis,
            acc=int(self.rotator_acc.get()),
            dec=int(self.rotator_dec.get()),
            max_v=int(self.rotator_max_v.get()),
            start_v=int(self.rotator_start_v.get()),
            move_timeout_s=float(self.rotator_move_timeout_s.get()),
        )

    def rotator_controller(self):
        if self.rotator_stage is not None:
            return self.rotator_stage
        raise StageError("半波片控制器未连接。")

    def _controller_summary(self):
        if self.simulation_mode.get():
            parts = []
            if self.stage is not None:
                parts.append("ANC300 simulated")
            if self.rotator_stage is not None:
                parts.append("HWP simulated")
            return "控制器已连接：" + "，".join(parts) if parts else "未连接"
        parts = []
        if self.stage is not None:
            parts.append(f"ANC300 {self.anc_host.get().strip()}")
        if self.rotator_stage is not None:
            parts.append(f"半波片ADev {int(self.rotator_usb_device_index.get())}")
        return "控制器已连接：" + "，".join(parts) if parts else "未连接"

    def _update_connection_status(self):
        self.status_text.set(self._controller_summary())
        self._update_backend_controls()

    def _update_backend_controls(self):
        if not hasattr(self, "simulation_checkbutton"):
            return
        if self.stage is not None or self.rotator_stage is not None:
            self.simulation_checkbutton.state(["disabled"])
        else:
            self.simulation_checkbutton.state(["!disabled"])

    def _backend_mode_matches(self, controller, simulated_type, label):
        actual_simulation = isinstance(controller, simulated_type)
        requested_simulation = bool(self.simulation_mode.get())
        if actual_simulation == requested_simulation:
            return True
        message = (
            f"{label} backend mode no longer matches the simulation checkbox. "
            "Disconnect all controllers before changing simulation mode."
        )
        self.log(message)
        messagebox.showwarning("Backend mode mismatch", message)
        return False

    def _store_controller_serial(self, controller, serial_var, label):
        if self.simulation_mode.get() or controller is None:
            return
        serial = getattr(controller, "product_serial", "")
        if not serial and hasattr(controller, "get_product_serial"):
            try:
                serial = controller.get_product_serial()
            except Exception as exc:
                self.log(f"{label}序列号读取失败：{exc}")
                return
        serial = str(serial).strip()
        if serial and serial != serial_var.get().strip():
            serial_var.set(serial)
            self.log(f"{label}序列号已记录：{serial}")
        elif serial:
            self.log(f"{label}序列号：{serial}")

    def _validate_controller_axis(self, controller, axis, label):
        if self.simulation_mode.get() or controller is None:
            return
        if not hasattr(controller, "get_axis_count"):
            return
        axis_count = controller.get_axis_count()
        if axis_count > 0 and int(axis) >= axis_count:
            raise StageError(f"{label}轴号 {int(axis) + 1} 超过该控制器轴数 {axis_count}。请检查设备号/SN是否对应正确控制器。")

    def pmt_settings(self):
        gate_time_ms = self._normalized_pmt_gate_time_ms()
        self.pmt_gate_time_ms.set(format_gate_time(gate_time_ms))
        return PMTSettings(
            enabled=bool(self.pmt_enabled.get()),
            dll_path=self.pmt_dll_path.get(),
            gate_time_ms=gate_time_ms,
            samples_to_average=int(self.pmt_samples_to_average.get()),
            sample_extra_wait_s=float(self.pmt_sample_extra_wait_s.get()),
            transfer_mode=int(self.pmt_transfer_mode.get()),
            trigger_mode=int(self.pmt_trigger_mode.get()),
            trigger_edge=int(self.pmt_trigger_edge.get()),
            simulate_value=float(self.pmt_simulate_value.get()),
            simulate_noise=float(self.pmt_simulate_noise.get()),
        )

    def validated_pmt_snapshot(self, *, simulate: bool, require_enabled: bool = False):
        settings = self.pmt_settings()
        return validate_pmt_settings(settings, simulate=simulate, require_enabled=require_enabled)

    def _normalized_pmt_gate_time_ms(self):
        raw_value = float(self.pmt_gate_time_ms.get())
        for value in GATE_TIME_OPTIONS_MS:
            if math.isclose(raw_value, float(value), rel_tol=0.0, abs_tol=1e-9):
                return float(value)
        allowed = ", ".join(format_gate_time(value) for value in GATE_TIME_OPTIONS_MS)
        raise ValueError(f"PMT gate time must be one of: {allowed} ms")

    def _normalized_pmt_gate_time_text(self):
        return format_gate_time(self._normalized_pmt_gate_time_ms())

    def _normalize_initial_pmt_gate_time(self):
        try:
            self.pmt_gate_time_ms.set(self._normalized_pmt_gate_time_text())
        except Exception as exc:
            default_value = float(default_config()["pmt"]["gate_time_ms"])
            self.pmt_gate_time_ms.set(format_gate_time(default_value))
            self.log(f"Invalid PMT gate time in config; reset to {format_gate_time(default_value)} ms: {exc}")

    def angle_scan_config(self):
        start_angle = float(self.angle_start_deg.get())
        stop_angle = float(self.angle_stop_deg.get())
        step_angle = float(self.angle_step_deg.get())
        settle_s = float(self.angle_settle_s.get())
        pre_read_settle_s = float(self.anc_pre_read_settle_s.get())
        numeric = (start_angle, stop_angle, step_angle, settle_s, pre_read_settle_s)
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("Angle scan values must be finite.")
        if step_angle <= 0:
            raise ValueError("Angle step must be greater than 0.")
        if settle_s < 0:
            raise ValueError("Settle time cannot be negative.")
        if pre_read_settle_s < 0:
            raise ValueError("Pre-read settle time cannot be negative.")
        angles = angle_scan_points(start_angle, stop_angle, step_angle)
        if not angles:
            raise ValueError("Angle scan point count is 0.")
        simulate_pmt = bool(self.simulation_mode.get())
        pmt = replace(self.pmt_settings(), enabled=True)
        validate_pmt_settings(pmt, simulate=simulate_pmt, require_enabled=True)
        rotator = self.rotator_motion_snapshot()
        return {
            "start_angle": start_angle,
            "stop_angle": stop_angle,
            "step_angle": step_angle,
            "settle_s": settle_s,
            "return_to_start": bool(self.angle_return_to_start.get()),
            "angles": angles,
            "pmt": pmt,
            "use_simulated_pmt": simulate_pmt,
            "measurement_source": measurement_source(pmt, simulate_pmt),
            "rotator": rotator,
            "output_dir": self.output_dir.get(),
            "pre_read_settle_s": pre_read_settle_s,
        }

    def rotator_api_axis(self):
        axis_display = int(self.rotator_axis_display.get())
        if axis_display < 1 or axis_display > 4:
            raise ValueError("半波片轴号必须是 1 到 4。")
        return axis_display - 1

    def rotator_steps_per_degree_value(self):
        value = float(self.rotator_steps_per_degree.get())
        if value == 0:
            raise ValueError("半波片脉冲/度不能为 0。")
        return value

    def rotator_direction_sign_value(self):
        value = int(self.rotator_direction_sign.get())
        if value not in (-1, 1):
            raise ValueError("半波片方向系数只能是 -1 或 1。")
        return value

    def rotator_motion_snapshot(self) -> RotatorMotionSnapshot:
        snapshot = RotatorMotionSnapshot(
            axis=self.rotator_api_axis(),
            steps_per_degree=self.rotator_steps_per_degree_value(),
            direction_sign=self.rotator_direction_sign_value(),
            acc=int(self.rotator_acc.get()),
            dec=int(self.rotator_dec.get()),
            max_v=int(self.rotator_max_v.get()),
            start_v=int(self.rotator_start_v.get()),
            timeout_s=float(self.rotator_move_timeout_s.get()),
        )
        if not math.isfinite(snapshot.steps_per_degree) or not math.isfinite(snapshot.timeout_s):
            raise ValueError("HWP scale and timeout must be finite.")
        if snapshot.timeout_s <= 0 or snapshot.acc < 0 or snapshot.dec < 0 or snapshot.max_v <= 0 or snapshot.start_v < 0:
            raise ValueError("HWP motion parameters are invalid.")
        return snapshot

    def rotator_angle_from_steps(self, steps):
        return self._rotator_angle_from_steps(
            steps,
            self.rotator_steps_per_degree_value(),
            self.rotator_direction_sign_value(),
        )

    def rotator_steps_from_delta_angle(self, angle_deg):
        return int(round(self.rotator_direction_sign_value() * float(angle_deg) * self.rotator_steps_per_degree_value()))

    def operation_busy(self):
        with self._motion_state_lock:
            motion_active = self._motion_active
        return motion_active or self._worker_thread_busy()

    def _active_operation_text(self):
        with self._motion_state_lock:
            label = self._motion_label
        labels = {
            "manual rotator move": "半波片手动转动",
            "selected-point move": "移动到选中色块",
            "angle scan": "角度依赖测量",
            "stage origin move": "位移台回零",
            "mapping scan": "Mapping扫描",
        }
        if label:
            return labels.get(label, self._operation_label_text(label))
        if self._worker_thread_busy():
            return "后台操作"
        return "当前操作"

    def _worker_thread_busy(self):
        threads = (
            self.scan_thread,
            self.rotator_thread,
            self.point_move_thread,
            self.origin_move_thread,
            self.ground_thread,
            self.angle_scan_thread,
            self.pmt_test_thread,
        )
        return any(thread is not None and thread.is_alive() for thread in threads)

    def _begin_motion(self, label, warning_title="正在运动", warning_message=None):
        with self._motion_state_lock:
            if self._motion_active or self._worker_thread_busy():
                active = self._active_operation_text()
                if warning_message is None:
                    warning_message = f"请等待{active}结束。"
                messagebox.showwarning(warning_title, warning_message)
                return False
            self._motion_active = True
            self._motion_label = label
        self.events.put(("motion_state", True, label))
        return True

    def _end_motion(self, label=None):
        with self._motion_state_lock:
            if label is not None and self._motion_label not in (None, label):
                return
            self._motion_active = False
            self._motion_label = None
        self.events.put(("motion_state", False, None))

    def _abort_motion(self, label, title=None, error=None):
        self._end_motion(label)
        if title is not None and error is not None:
            messagebox.showerror(title, str(error))
        return

    def _set_motion_controls_enabled(self, enabled):
        state = "normal" if enabled else "disabled"
        for button in self._motion_sensitive_buttons:
            try:
                button.configure(state=state)
            except Exception:
                pass

    def _set_run_indicator(self, active, label=None):
        if active:
            operation = self._active_operation_text() if label is None else self._operation_label_text(label)
            self.run_indicator_text.set(f"运行中：{operation}，勿断电")
            self.run_indicator.configure(bg="#dc2626", fg="white")
        else:
            self.run_indicator_text.set("空闲")
            self.run_indicator.configure(bg="#16a34a", fg="white")

    def _operation_label_text(self, label):
        labels = {
            "manual rotator move": "半波片手动转动",
            "selected-point move": "移动到选中色块",
            "angle scan": "角度依赖测量",
            "stage origin move": "位移台回零",
            "mapping scan": "Mapping扫描",
            "pmt test": "PMT测试",
        }
        if label:
            return labels.get(label, label)
        return "当前操作"

    def _start_worker(self, attr_name, target, args=()):
        self.stop_event.clear()
        thread = threading.Thread(target=target, args=args, daemon=True)
        setattr(self, attr_name, thread)
        thread.start()
        return thread

    def _mark_rotator_recovery_needed(self, reason):
        self._rotator_needs_recovery = True
        self.events.put(("log", f"Rotator recovery required before next angle scan: {reason}"))

    def _recover_rotator_if_needed(self, force=False):
        needs_recovery = self._rotator_needs_recovery
        if not needs_recovery and not force:
            return True
        if self._worker_thread_busy():
            messagebox.showwarning("正在运动", "请先等待当前运动结束，再恢复半波片控制器。")
            return False
        if needs_recovery and not self.simulation_mode.get() and not self._ensure_no_external_mt_tool("恢复半波片控制器"):
            return False
        try:
            if needs_recovery:
                self._reinitialize_rotator_connection()
            self._recover_rotator_axis()
            self._rotator_needs_recovery = False
            if needs_recovery:
                self.log("Rotator controller recovered and motion parameters re-applied.")
            return True
        except Exception as exc:
            self.log(f"Rotator recovery failed: {exc}")
            messagebox.showerror("半波片恢复失败", f"半波片控制器仍处于异常状态：{exc}\n\n请检查驱动器后重新连接控制器。")
            return False

    def _recover_rotator_axis(self):
        controller = self.rotator_controller()
        axis = self.rotator_api_axis()
        kwargs = {
            "acc": int(self.rotator_acc.get()),
            "dec": int(self.rotator_dec.get()),
            "max_v": int(self.rotator_max_v.get()),
            "start_v": int(self.rotator_start_v.get()),
        }
        if hasattr(controller, "recover_axis"):
            controller.recover_axis(axis, **kwargs)
        else:
            controller.halt_axis(axis)
            controller.configure_motion()
        raw_steps = controller.get_axis_raw_steps(axis)
        angle = self.rotator_angle_from_steps(raw_steps)
        self._set_rotator_position(angle, raw_steps, force=True)

    def _reinitialize_rotator_connection(self):
        if self.simulation_mode.get():
            return
        if self.rotator_stage is None:
            raise StageError("半波片控制器未连接。")
        old_rotator_stage = self.rotator_stage
        self.rotator_stage = None
        try:
            old_rotator_stage.disconnect()
        except Exception as exc:
            self.log(f"断开半波片控制器时出错：{exc}")
        self.rotator_stage = MTStage(self.rotator_stage_settings())
        self.rotator_stage.connect()

    @staticmethod
    def normalized_angle(angle):
        return float(angle) % 360.0

    @staticmethod
    def _rotator_angle_from_steps(steps, steps_per_degree, direction_sign):
        angle = int(direction_sign) * float(steps) / float(steps_per_degree)
        return angle % 360.0

    @staticmethod
    def _shortest_angle_delta(current_angle, target_angle):
        delta = (float(target_angle) % 360.0) - (float(current_angle) % 360.0)
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0
        return delta

    def read_rotator_position(self):
        if self.operation_busy():
            messagebox.showwarning("正在运动", "请先等待当前运动结束，再读取半波片角度。")
            return
        if not self.simulation_mode.get() and not self._ensure_no_external_mt_tool("读取半波片角度"):
            return
        if not self._ensure_controller(auto_connect=True):
            return
        if not self._recover_rotator_if_needed():
            return
        self._read_rotator_position_now()

    def _read_rotator_position_now(self):
        try:
            axis = self.rotator_api_axis()
            steps = self.rotator_controller().get_axis_raw_steps(axis)
            angle = self.rotator_angle_from_steps(steps)
            self._set_rotator_position(angle, steps, force=True)
            software_steps = self.rotator_controller().get_axis_steps(axis)
            self.log(f"半波片当前角度：{angle:.3f} deg，轴号 {axis + 1}，P_Now {steps} 脉冲，Software {software_steps} 脉冲")
        except Exception as exc:
            self.log(f"读取半波片角度失败：{exc}")
            messagebox.showerror("读取半波片角度失败", str(exc))

    def move_rotator(self):
        if not self._begin_motion("manual rotator move", warning_title="半波片正在运动"):
            return
        if self.rotator_thread is not None and self.rotator_thread.is_alive():
            self._abort_motion("manual rotator move")
            messagebox.showwarning("半波片正在转动", "当前半波片转动还没有结束。")
            return
        if not self.simulation_mode.get() and not self._ensure_no_external_mt_tool("控制半波片旋转"):
            return self._abort_motion("manual rotator move")
        if not self._ensure_controller(auto_connect=True, allow_reserved_motion=True):
            return self._abort_motion("manual rotator move")
        if not self._recover_rotator_if_needed(force=True):
            return self._abort_motion("manual rotator move")
        try:
            axis = self.rotator_api_axis()
            target_angle = float(self.rotator_target_angle_deg.get())
            axis, target_angle, delta_angle, delta_steps, move_params, current_steps, current_angle = self._prepare_rotator_move(target_angle)
        except Exception as exc:
            return self._abort_motion("manual rotator move", "半波片参数错误", exc)
        if abs(delta_angle) < 0.01 or delta_steps == 0:
            self._set_rotator_position(current_angle, current_steps, force=True)
            self.log(f"半波片已在目标角度附近：{current_angle:.3f} deg")
            return self._abort_motion("manual rotator move")
        self._start_rotator_move(axis, target_angle, delta_angle, delta_steps, move_params)

    def _prepare_rotator_move(self, target_angle, forced_delta_angle=None, rotator_snapshot=None):
        snapshot = rotator_snapshot or self.rotator_motion_snapshot()
        axis = snapshot.axis
        steps_per_degree = snapshot.steps_per_degree
        direction_sign = snapshot.direction_sign
        current_steps = self.rotator_controller().get_axis_raw_steps(axis)
        current_angle = self._rotator_angle_from_steps(current_steps, steps_per_degree, direction_sign)
        if forced_delta_angle is None:
            delta_angle = self._shortest_angle_delta(current_angle, target_angle)
        else:
            delta_angle = float(forced_delta_angle)
        delta_steps = int(round(direction_sign * delta_angle * steps_per_degree))
        move_params = snapshot.move_params()
        move_params["timeout_s"] = self._adaptive_rotator_timeout(delta_angle, move_params)
        return axis, target_angle, delta_angle, delta_steps, move_params, current_steps, current_angle

    def _rotator_move_params(self, steps_per_degree, direction_sign):
        return {
            "acc": int(self.rotator_acc.get()),
            "dec": int(self.rotator_dec.get()),
            "max_v": int(self.rotator_max_v.get()),
            "start_v": int(self.rotator_start_v.get()),
            "timeout_s": float(self.rotator_move_timeout_s.get()),
            "steps_per_degree": steps_per_degree,
            "direction_sign": direction_sign,
        }

    @staticmethod
    def _adaptive_rotator_timeout(delta_angle, move_params):
        base_timeout = float(move_params["timeout_s"])
        max_v = max(1.0, abs(float(move_params["max_v"])))
        steps_per_degree = max(1e-9, float(move_params["steps_per_degree"]))
        deg_per_second = max_v / steps_per_degree
        if deg_per_second <= 0:
            return base_timeout
        estimated = abs(float(delta_angle)) / deg_per_second
        return max(base_timeout, estimated * 1.5 + 10.0)

    def _start_rotator_move(self, axis, target_angle, delta_angle, delta_steps, move_params):
        self._start_worker(
            "rotator_thread",
            self._move_rotator_worker,
            args=(axis, target_angle, delta_angle, delta_steps, move_params),
        )

    def _move_rotator_worker(self, axis, target_angle, delta_angle, delta_steps, move_params):
        try:
            move_result = self._move_rotator_relative(axis, delta_steps, move_params, stop_event=self.stop_event)
            controller = self.rotator_controller()
            steps = controller.get_axis_steps(axis)
            raw_steps = controller.get_axis_raw_steps(axis)
            angle = self._rotator_angle_from_steps(
                raw_steps,
                move_params["steps_per_degree"],
                move_params["direction_sign"],
            )
            self.events.put(("rotator_position", angle, raw_steps))
            raw_delta = move_result.get("end_raw", steps) - move_result.get("start_raw", steps) if isinstance(move_result, dict) else 0
            software_delta = move_result.get("end_software", steps) - move_result.get("start_software", steps) if isinstance(move_result, dict) else 0
            running_seen = move_result.get("running_seen", False) if isinstance(move_result, dict) else False
            self.events.put((
                "log",
                f"半波片命令完成：当前 {angle:.3f} deg，目标 {target_angle % 360.0:.3f} deg，"
                f"命令转动 {delta_angle:.3f} deg，轴号 {axis + 1}，"
                f"P_Now变化 {raw_delta} 脉冲，Software变化 {software_delta} 脉冲，Run={running_seen}",
            ))
            if not running_seen or abs(raw_delta) < max(10, abs(delta_steps) * 0.1):
                self.events.put((
                    "log",
                    "警告：控制器位置反馈没有明显变化，半波片可能没有实际转动。请确认半波片驱动器电源、轴4电机线、使能状态正常。",
                ))
        except Exception as exc:
            self._mark_rotator_recovery_needed(exc)
            self.events.put(("error", f"半波片转动失败：{exc}"))
        finally:
            self._end_motion("manual rotator move")

    def _move_rotator_relative(self, axis, delta_steps, move_params, stop_event=None):
        controller = self.rotator_controller()
        old_timeout = getattr(controller.settings, "move_timeout_s", None)
        if old_timeout is not None:
            controller.settings.move_timeout_s = move_params["timeout_s"]
        try:
            return controller.move_axis_steps_rel(
                axis,
                delta_steps,
                acc=move_params["acc"],
                dec=move_params["dec"],
                max_v=move_params["max_v"],
                start_v=move_params["start_v"],
                stop_event=stop_event,
                progress_callback=self._rotator_progress_callback(move_params),
            )
        finally:
            if old_timeout is not None:
                controller.settings.move_timeout_s = old_timeout

    def _rotator_progress_callback(self, move_params):
        def progress_callback(steps):
            angle = self._rotator_angle_from_steps(
                steps,
                move_params["steps_per_degree"],
                move_params["direction_sign"],
            )
            self.events.put(("rotator_position", angle, steps))

        return progress_callback

    def diagnose_axes(self):
        if self.operation_busy():
            messagebox.showwarning("正在运动", "请先等待当前运动结束，再做轴状态诊断。")
            return
        if not self._ensure_controller(auto_connect=True):
            return
        try:
            self.log("轴状态诊断开始（不移动电机）")
            for axis in range(4):
                info = self.rotator_controller().get_axis_diagnostics(axis)
                angle_text = ""
                if axis == self.rotator_api_axis():
                    angle = self.rotator_angle_from_steps(info["raw_steps"])
                    angle_text = f", angle={angle:.3f}deg"
                self.log(
                    f"Axis {axis + 1}: raw={info['raw_steps']}, software={info['software_steps']}, "
                    f"run={info['run']}, dir={info['dir']}, neg={info['neg']}, pos={info['pos']}, "
                    f"zero={info['zero']}, mode={info['mode']}, soft_neg={info['soft_neg']}, soft_pos={info['soft_pos']}"
                    f"{angle_text}"
                )
            self.log("轴状态诊断结束")
        except Exception as exc:
            self.log(f"轴状态诊断失败：{exc}")
            messagebox.showerror("轴状态诊断失败", str(exc))

    def test_pmt_read(self):
        if not self._begin_motion("pmt test"):
            return False
        self.events.put(("run_indicator", True, "pmt test"))
        try:
            simulate = bool(self.simulation_mode.get())
            settings = replace(self.pmt_settings(), enabled=True)
            validate_pmt_settings(settings, simulate=simulate, require_enabled=True)
            self._start_worker(
                "pmt_test_thread",
                self._test_pmt_read_worker,
                args=(settings, simulate),
            )
            return True
        except Exception as exc:
            self.events.put(("run_indicator", False, None))
            self._end_motion("pmt test")
            self.events.put(("error", f"PMT测试启动失败：{exc}"))
            return False

    def _test_pmt_read_worker(self, settings, simulate):
        try:
            with PMTCounter(settings, simulate=simulate) as pmt:
                average, samples = pmt.read_average(stop_event=self.stop_event)
            message = f"PMT测试成功：平均 {average:.2f}，原始值 {samples}"
            self.events.put(("pmt_status", f"PMT: {average:.2f}"))
            self.events.put(("log", message))
        except Exception as exc:
            self.events.put(("pmt_status", "PMT: 测试失败"))
            self.events.put(("error", f"PMT测试失败：{exc}"))
        finally:
            self.events.put(("run_indicator", False, None))
            self._end_motion("pmt test")

    def _motion_blocks_connection(self, allow_reserved_motion=False):
        if (not allow_reserved_motion and self.operation_busy()) or self._worker_thread_busy():
            messagebox.showwarning("正在运动", "请先停止或等待当前运动结束，再重新连接控制器。")
            return True
        return False

    def connect_stage(self, allow_reserved_motion=False):
        if self._motion_blocks_connection(allow_reserved_motion=allow_reserved_motion):
            return False
        failures = []
        if not self._connect_stage_controller(show_error=False):
            failures.append("位移台")
        if not self._connect_rotator_controller(show_error=False):
            failures.append("半波片")
        self._update_connection_status()
        if self.rotator_stage is not None:
            self._rotator_needs_recovery = False
            self._read_rotator_position_now()
        if failures:
            message = "，".join(failures) + "连接失败。已保留其它成功连接的控制器。"
            self.log(message)
            messagebox.showwarning("部分连接失败", message)
        return self.stage is not None or self.rotator_stage is not None

    def connect_stage_only(self):
        if self._motion_blocks_connection():
            return False
        if self.rotator_stage is not None and not self._backend_mode_matches(self.rotator_stage, SimulatedStage, "HWP"):
            return False
        ok = self._connect_stage_controller(show_error=True)
        self._update_connection_status()
        return ok

    def connect_rotator_only(self, allow_reserved_motion=False):
        if self._motion_blocks_connection(allow_reserved_motion=allow_reserved_motion):
            return False
        if self.stage is not None and not self._backend_mode_matches(self.stage, SimulatedANC300Stage, "ANC300"):
            return False
        if not self.simulation_mode.get() and not self._ensure_no_external_mt_tool("连接半波片控制器"):
            return False
        ok = self._connect_rotator_controller(show_error=True)
        self._update_connection_status()
        if ok:
            self._rotator_needs_recovery = False
            self._read_rotator_position_now()
        return ok

    def _connect_stage_controller(self, show_error=True):
        if self.stage is not None:
            message = "ANC300 is already connected. Disconnect it explicitly before changing connection settings."
            self.log(message)
            if show_error:
                messagebox.showwarning("ANC300 already connected", message)
            return False
        candidate = None
        try:
            settings = self.anc300_stage_settings()
            if not self.simulation_mode.get() and not settings.host:
                raise StageError("ANC300 host/IP must not be empty.")
            candidate = SimulatedANC300Stage(settings) if self.simulation_mode.get() else ANC300ScanStage(settings)
            status = candidate.connect()
            self.stage = candidate
            self.log("ANC300 连接成功（未改变输出状态）")
            self._display_anc300_status(status)
            self._update_backend_controls()
            return True
        except Exception as exc:
            self.stage = None
            try:
                if candidate is not None:
                    candidate.disconnect()
            except Exception as close_exc:
                self.log(f"清理位移台失败连接时出错：{close_exc}")
            self.log(f"ANC300 连接失败：{exc}")
            if show_error:
                messagebox.showerror("ANC300 连接失败", str(exc))
            return False

    def _rotator_usb_indices_to_try(self):
        configured = int(self.rotator_usb_device_index.get())
        return [configured]

    def _open_rotator_stage(self):
        original_index = int(self.rotator_usb_device_index.get())
        errors = []
        for index in self._rotator_usb_indices_to_try():
            settings = self.rotator_stage_settings()
            settings.usb_device_index = int(index)
            controller = MTStage(settings)
            try:
                controller.connect()
                self._validate_controller_axis(controller, self.rotator_api_axis(), "半波片")
                if int(index) != original_index:
                    self.rotator_usb_device_index.set(int(index))
                    self.log(f"半波片ADev {original_index} 打开失败，已自动改用 ADev {index} 连接。")
                return controller
            except Exception as exc:
                errors.append(f"ADev {index}: {exc}")
                try:
                    controller.disconnect()
                except Exception:
                    pass
        raise StageError("半波片控制器连接失败：" + "；".join(errors))

    def _connect_rotator_controller(self, show_error=True):
        try:
            if self.simulation_mode.get():
                old_rotator_stage = self.rotator_stage
                self.rotator_stage = None
                if old_rotator_stage is not None:
                    old_rotator_stage.disconnect()
                self.rotator_stage = SimulatedStage(self.rotator_stage_settings())
                self.rotator_stage.connect()
            else:
                old_rotator_stage = self.rotator_stage
                self.rotator_stage = None
                if old_rotator_stage is not None:
                    old_rotator_stage.disconnect()
                self.rotator_stage = self._open_rotator_stage()
            if self.rotator_stage is None:
                raise StageError("半波片控制器未连接。")
            self._validate_controller_axis(self.rotator_stage, self.rotator_api_axis(), "半波片")
            self._store_controller_serial(self.rotator_stage, self.rotator_usb_serial, "半波片")
            self._rotator_needs_recovery = False
            self.log(f"半波片控制器连接成功：ADev {int(self.rotator_usb_device_index.get())}")
            return True
        except Exception as exc:
            new_rotator_stage = self.rotator_stage
            self.rotator_stage = None
            try:
                if new_rotator_stage is not None and new_rotator_stage is not self.stage:
                    new_rotator_stage.disconnect()
            except Exception as close_exc:
                self.log(f"清理半波片失败连接时出错：{close_exc}")
            self._rotator_needs_recovery = False
            self.log(f"半波片控制器连接失败：{exc}")
            if show_error:
                messagebox.showerror("半波片连接失败", str(exc))
            return False

    def disconnect_stage(self):
        if self.operation_busy():
            messagebox.showwarning("正在运动", "请先停止或等待当前运动结束，再断开控制器。")
            return False
        return self._disconnect_stage_now()

    def _disconnect_stage_now(self):
        errors = []
        old_stage = self.stage
        old_rotator_stage = self.rotator_stage

        if old_rotator_stage is not None:
            try:
                old_rotator_stage.disconnect()
            except Exception as exc:
                errors.append(f"HWP disconnect failed: {exc}")
            else:
                if self.rotator_stage is old_rotator_stage:
                    self.rotator_stage = None

        if old_stage is not None:
            try:
                old_stage.disconnect()
            except Exception as exc:
                errors.append(f"ANC300 disconnect failed: {exc}")
            else:
                if self.stage is old_stage:
                    self.stage = None

        if self.rotator_stage is None:
            self.rotator_position_text.set("半波片: -- deg")
            self._last_rotator_display = None
            self._rotator_needs_recovery = False
        if self.stage is None:
            self.refresh_anc300_status()
        elif hasattr(self.stage, "get_status"):
            self.refresh_anc300_status()
        self._update_connection_status()

        if errors:
            message = "\n".join(errors)
            self.log(message.replace("\n", "；"))
            messagebox.showerror("Disconnect failed", message)
            return False
        if old_stage is not None or old_rotator_stage is not None:
            self.log("已断开")
        return True

    @staticmethod
    def _anc_value(value, digits=3):
        return "--" if value is None else f"{float(value):.{digits}f}"

    def refresh_anc300_status(self):
        if self.stage is None:
            self.anc_identity_text.set("版本/序列号: -- / --")
            self.anc_x_status_text.set("X: axis --, mode --, voltage -- V, origin -- V")
            self.anc_y_status_text.set("Y: axis --, mode --, voltage -- V, origin -- V")
            self.anc_calibration_text.set("Calibration: --")
            self.position_text.set("X/Y estimated position: unavailable (session origin unset)")
            return
        try:
            self._display_anc300_status(self.stage.get_status())
        except Exception as exc:
            self.log(f"ANC300 状态刷新失败：{exc}")

    def _display_anc300_status(self, status):
        identity = status.get("device_identity", {})
        version = identity.get("version") or ("simulated" if identity.get("simulated") else "--")
        serial = identity.get("controller_serial") or "--"
        axes, modes = status.get("axes", {}), status.get("modes", {})
        self.anc_identity_text.set(f"版本/序列号: {version} / {serial}")
        self.anc_x_status_text.set(
            f"X: axis {axes.get('x', '--')}, mode {modes.get('x') or '--'}, "
            f"voltage {self._anc_value(status.get('x_voltage_v'))} V, "
            f"origin {self._anc_value(status.get('x_origin_voltage_v'))} V"
        )
        self.anc_y_status_text.set(
            f"Y: axis {axes.get('y', '--')}, mode {modes.get('y') or '--'}, "
            f"voltage {self._anc_value(status.get('y_voltage_v'))} V, "
            f"origin {self._anc_value(status.get('y_origin_voltage_v'))} V"
        )
        self.anc_calibration_text.set(
            f"Calibration: X {float(status.get('x_um_per_v')):.6g} um/V dir {int(status.get('x_direction')):+d}; "
            f"Y {float(status.get('y_um_per_v')):.6g} um/V dir {int(status.get('y_direction')):+d}; "
            f"source {status.get('calibration_source') or '--'}"
        )
        position = status.get("estimated_position_um")
        if position is None:
            self.position_text.set("X/Y estimated position: unavailable (session origin unset)")
        else:
            self.position_text.set(f"X: {float(position[0]):.2f} um, Y: {float(position[1]):.2f} um (estimated)")

    def confirm_anc300_hardware_profile(self):
        label = "ANC300 profile confirmation"
        if not self._begin_motion(label):
            return False
        try:
            confirmed = messagebox.askyesno(
                "确认 ANC300 硬件配置",
                "请确认当前硬件为 ANSxyz100std/LT，工作温度约 4 K，允许的 offset 电压范围为 0-150 V。\n\n"
                "确认后才允许启用 offset outputs。",
                icon="warning",
            )
            if not confirmed:
                return False
            self.anc_hardware_profile_confirmed.set(True)
            if self.stage is not None:
                self.stage.confirm_hardware_profile(True)
            self.save_config(silent=True)
            self.refresh_anc300_status()
            self.log("已确认 ANSxyz100std/LT / 约 4 K / 0-150 V 硬件配置")
            return True
        finally:
            self._end_motion(label)

    def enable_anc300_outputs(self):
        label = "ANC300 output enable"
        if not self._begin_motion(label):
            return False
        try:
            if not self._ensure_stage():
                return False
            try:
                self.stage.enable_outputs()
                self.refresh_anc300_status()
                self.log("ANC300 offset outputs 已启用")
                return True
            except Exception as exc:
                self.refresh_anc300_status()
                self.log(f"启用 ANC300 offset outputs 失败：{exc}")
                messagebox.showerror("ANC300 输出启用失败", str(exc))
                return False
        finally:
            self._end_motion(label)

    def ground_anc300_outputs(self):
        label = "ANC300 grounding"
        if not self._begin_motion(label):
            return False
        if not self._ensure_stage():
            return self._abort_motion(label)
        try:
            self._start_worker("ground_thread", self._ground_anc300_outputs_worker)
            return True
        except Exception as exc:
            self._abort_motion(label)
            messagebox.showerror("ANC300 安全接地启动失败", str(exc))
            return False

    def _ground_anc300_outputs_worker(self):
        try:
            self.stage.ground_outputs(stop_event=self.stop_event)
            self.events.put(("log", "ANC300 已缓慢回到 0 V 并切换到 GND"))
        except Exception as exc:
            self.events.put(("error", f"ANC300 安全接地失败：{exc}"))
        finally:
            self.events.put(("anc_status", None))
            self._end_motion("ANC300 grounding")

    def set_origin(self):
        if self.operation_busy():
            messagebox.showwarning("正在运动", "请先等待当前运动结束，再设置位移台零点。")
            return
        if not self._ensure_stage():
            return
        try:
            self.stage.set_origin()
            self.refresh_anc300_status()
            self.log("当前位置已设为零点")
        except Exception as exc:
            self.log(f"设零失败：{exc}")
            messagebox.showerror("设零失败", str(exc))

    def on_mapping_click(self, event):
        if self.operation_busy():
            self.log("Current movement or scan is running; mapping point queue changes are locked.")
            return
        if not self._cell_hit_bounds:
            self.log("No mapping point is available for selection.")
            return
        for hit in reversed(self._cell_hit_bounds):
            left, top, right, bottom = hit["bounds"]
            if left <= event.x <= right and top <= event.y <= bottom:
                point = dict(hit["point"])
                key = self._point_key(point)
                existing_index = self._queue_index_for_key(key)
                if existing_index is None:
                    queued = dict(point)
                    queued["queue_status"] = "pending"
                    self.angle_point_queue.append(queued)
                    self.selected_point = queued
                    self.log(
                        f"Queued mapping point #{len(self.angle_point_queue)}: X {float(point['x_um']):.2f} um, "
                        f"Y {float(point['y_um']):.2f} um"
                    )
                else:
                    removed = self.angle_point_queue.pop(existing_index)
                    self.selected_point = self.angle_point_queue[-1] if self.angle_point_queue else None
                    self.log(
                        f"Removed queued point #{existing_index + 1}: X {float(removed['x_um']):.2f} um, "
                        f"Y {float(removed['y_um']):.2f} um"
                    )
                self._refresh_angle_queue_ui()
                self._schedule_draw(force=True)
                return
        self.log("Clicked outside measured mapping points.")

    @staticmethod
    def _point_key(point):
        return round(float(point["x_um"]), 6), round(float(point["y_um"]), 6)

    def _queue_index_for_key(self, key):
        for index, point in enumerate(self.angle_point_queue):
            if self._point_key(point) == key:
                return index
        return None

    def _refresh_angle_queue_ui(self):
        if self.angle_point_queue:
            first = self._first_pending_queue_point() or self.angle_point_queue[0]
            self.angle_selected_point_text.set(
                f"Queued {len(self.angle_point_queue)} point(s); next starts at X {float(first['x_um']):.2f} um, "
                f"Y {float(first['y_um']):.2f} um"
            )
        else:
            self.angle_selected_point_text.set("Selected point: none")
        if not hasattr(self, "angle_queue_table"):
            return
        rows = self.angle_queue_table.get_children()
        if rows:
            self.angle_queue_table.delete(*rows)
        for index, point in enumerate(self.angle_point_queue, start=1):
            status = str(point.get("queue_status", "pending"))
            self.angle_queue_table.insert(
                "",
                "end",
                values=(
                    index,
                    f"{float(point['x_um']):.2f}, {float(point['y_um']):.2f}",
                    status,
                ),
            )

    def _set_queue_point_status(self, point, status):
        key = self._point_key(point)
        index = self._queue_index_for_key(key)
        if index is not None:
            self.angle_point_queue[index]["queue_status"] = status
        self.events.put(("angle_queue", None))

    def move_to_selected_point(self):
        target_point = self._first_pending_queue_point() or self.selected_point
        if target_point is None:
            messagebox.showwarning("No point selected", "Click a measured mapping point first.")
            return
        if not self._begin_motion(
            "selected-point move",
            warning_message="角度扫描或归零还没结束，先等它完成后再移动到下一个色块。",
        ):
            return
        if not self._ensure_stage():
            return self._abort_motion("selected-point move")
        self._start_worker("point_move_thread", self._move_to_selected_point_worker, args=(dict(target_point),))

    def _first_pending_queue_point(self):
        for point in self.angle_point_queue:
            if point.get("queue_status") != "done":
                return point
        return self.angle_point_queue[0] if self.angle_point_queue else None

    def _move_to_selected_point_worker(self, point):
        try:
            x_um = float(point["x_um"])
            y_um = float(point["y_um"])
            self.events.put(("log", f"Moving to selected point: X {x_um:.2f} um, Y {y_um:.2f} um"))
            self.stage.move_to_um(x_um, y_um, stop_event=self.stop_event)
            self.events.put(("position", None))
            self.events.put(("log", f"Reached selected point: X {x_um:.2f} um, Y {y_um:.2f} um"))
        except Exception as exc:
            self.events.put(("error", f"Move to selected point failed: {exc}"))
        finally:
            self._end_motion("selected-point move")

    def clear_angle_results(self):
        self.angle_points = []
        self.angle_latest_count_text.set("Latest angle count: --")
        self.angle_progress_text.set("Angle scan: idle")
        if hasattr(self, "angle_table"):
            rows = self.angle_table.get_children()
            if rows:
                self.angle_table.delete(*rows)

    def start_angle_scan(self):
        points = self._angle_scan_points_to_run()
        if not points:
            messagebox.showwarning("No pending point", "Click one or more measured mapping points first, or reselect completed points to measure again.")
            return
        try:
            current_position = self.stage.get_status().get("estimated_position_um") if self.stage is not None else None
            self.log(f"Angle scan request received at stage position: {current_position}")
        except Exception:
            pass
        if not self._begin_motion(
            "angle scan",
            warning_message="已有运动或扫描正在执行，等当前动作结束后再开始角度依赖测量。",
        ):
            return
        if not self._ensure_stage():
            return self._abort_motion("angle scan")
        try:
            self._preflight_stage_points([(point["x_um"], point["y_um"]) for point in points])
            config = self.angle_scan_config()
        except Exception as exc:
            return self._abort_motion("angle scan", "Angle scan preflight error", exc)
        if not self.simulation_mode.get() and not self._ensure_no_external_mt_tool("run angle scan"):
            return self._abort_motion("angle scan")
        if not self._ensure_controller(auto_connect=True, allow_reserved_motion=True):
            return self._abort_motion("angle scan")
        if not self._recover_rotator_if_needed(force=True):
            return self._abort_motion("angle scan")
        try:
            self.save_config(silent=True)
        except Exception as exc:
            return self._abort_motion("angle scan", "Angle scan parameter error", exc)
        self.clear_angle_results()
        for point in points:
            self._set_queue_point_status(point, "pending")
        self._start_worker("angle_scan_thread", self._angle_scan_sequence_worker, args=(points, config))
        self.log(f"Angle scan queue started: {len(points)} point(s), {len(config['angles'])} angles each")

    def _angle_scan_points_to_run(self):
        if self.angle_point_queue:
            return [dict(point) for point in self.angle_point_queue if point.get("queue_status") != "done"]
        if self.selected_point is None:
            return []
        return [dict(self.selected_point)]

    def _angle_scan_sequence_worker(self, points, config):
        completed = 0
        current_point = None
        try:
            total_points = len(points)
            for point_index, point in enumerate(points, start=1):
                current_point = point
                if self.stop_event.is_set():
                    raise StageError("角度测量已停止")
                self._set_queue_point_status(point, "running")
                self.events.put((
                    "angle_progress",
                    f"Point {point_index}/{total_points}: X {float(point['x_um']):.2f} um, Y {float(point['y_um']):.2f} um",
                ))
                csv_path = self._run_single_angle_scan(point, config, point_index=point_index, total_points=total_points)
                completed += 1
                self._set_queue_point_status(point, "done")
                self.events.put(("angle_point_done", point, str(csv_path), point_index, total_points))
            self.events.put(("angle_done", f"{completed} point(s) completed"))
        except Exception as exc:
            self._mark_rotator_recovery_needed(exc)
            if self._angle_motion_started:
                try:
                    self.rotator_controller().halt_axis(config["rotator"].axis)
                except Exception:
                    pass
            if current_point is not None:
                self._set_queue_point_status(current_point, "failed")
            self.events.put(("error", f"Angle scan failed: {exc}"))
            self.events.put(("angle_progress", "Angle scan failed"))
        finally:
            self._end_motion("angle scan")

    def _run_single_angle_scan(self, point, config, point_index=1, total_points=1):
        output_dir = Path(config["output_dir"]) / "angle_scans"
        x_um = float(point["x_um"])
        y_um = float(point["y_um"])
        pmt_settings = config["pmt"]
        simulate_pmt = bool(config["use_simulated_pmt"])
        validate_pmt_settings(pmt_settings, simulate=simulate_pmt, require_enabled=True)
        source = config.get("measurement_source") or measurement_source(pmt_settings, simulate_pmt)
        rotator_snapshot = config["rotator"]
        if not isinstance(rotator_snapshot, RotatorMotionSnapshot):
            raise ValueError("Angle scan HWP settings snapshot is invalid.")
        fields = [
            "index",
            "queue_point_index",
            "queue_point_total",
            "x_um",
            "y_um",
            "target_angle_deg",
            "reached_angle_deg",
            "value",
            "pmt_samples",
            "timestamp",
            "measurement_source",
            *ANC_VOLTAGE_METADATA_FIELDS,
        ]
        stem = f"angle_scan_p{point_index:02d}_x{x_um:.3f}_y{y_um:.3f}"
        self._angle_motion_started = False
        csv_path, file_handle, writer = open_result_csv(output_dir, stem, fields)
        self.latest_angle_csv = csv_path
        with file_handle as f:
            self.events.put((
                "angle_progress",
                f"Point {point_index}/{total_points}: moving stage slowly to X {x_um:.2f} um, Y {y_um:.2f} um",
            ))
            self._angle_motion_started = True
            self.stage.move_to_um(x_um, y_um, stop_event=self.stop_event)
            self.events.put(("position", None))
            voltage_metadata = self._stage_voltage_metadata()
            pre_read_settle_s = float(config.get("pre_read_settle_s", 0.2))
            if pre_read_settle_s:
                interruptible_sleep(pre_read_settle_s, stop_event=self.stop_event)
            angles = config["angles"]
            total = len(angles)
            for index, target_angle in enumerate(angles, start=1):
                if self.stop_event.is_set():
                    raise StageError("角度测量已停止")
                self.events.put((
                    "angle_progress",
                    f"Point {point_index}/{total_points}, angle {index}/{total}: moving to {target_angle:.3f} deg",
                ))
                reached_angle = self._move_rotator_to_angle_blocking(
                    target_angle,
                    settle_s=config["settle_s"],
                    stop_event=self.stop_event,
                    rotator_snapshot=rotator_snapshot,
                )
                if self.stop_event.is_set():
                    raise StageError("角度测量已停止")
                self.events.put(("angle_progress", f"Point {point_index}/{total_points}, angle {index}/{total}: reading PMT"))
                with PMTCounter(pmt_settings, simulate=simulate_pmt) as pmt:
                    value, samples = pmt.read_average(stop_event=self.stop_event)
                item = {
                    "index": index,
                    "queue_point_index": point_index,
                    "queue_point_total": total_points,
                    "x_um": x_um,
                    "y_um": y_um,
                    "target_angle_deg": self.normalized_angle(target_angle),
                    "reached_angle_deg": reached_angle,
                    "value": value,
                    "pmt_samples": ";".join(f"{sample:g}" for sample in samples),
                    "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                    "measurement_source": source,
                    **voltage_metadata,
                }
                writer.writerow(item)
                f.flush()
                self.events.put(("angle_point", item, index, total))

        return_target_angle = float(config["start_angle"])
        if config["return_to_start"] and not self.stop_event.is_set():
            return_delta_angle = self._angle_scan_return_delta(return_target_angle, rotator_snapshot)
            self.events.put((
                "angle_progress",
                f"Point {point_index}/{total_points}: returning HWP to {return_target_angle:.3f} deg by reverse path ({return_delta_angle:.3f} deg)",
            ))
            self._move_rotator_to_angle_blocking(
                return_target_angle,
                settle_s=0.1,
                stop_event=self.stop_event,
                forced_delta_angle=return_delta_angle,
                rotator_snapshot=rotator_snapshot,
            )
        return csv_path

    def _angle_scan_return_delta(self, target_angle, rotator_snapshot):
        current_steps = self.rotator_controller().get_axis_raw_steps(rotator_snapshot.axis)
        current_angle = self._rotator_angle_from_steps(
            current_steps,
            rotator_snapshot.steps_per_degree,
            rotator_snapshot.direction_sign,
        )
        delta = (float(target_angle) % 360.0) - (float(current_angle) % 360.0)
        if delta > 0.0:
            delta -= 360.0
        if abs(delta) < 0.01:
            return 0.0
        return delta

    def _move_rotator_to_angle_blocking(
        self,
        target_angle,
        settle_s=0.0,
        stop_event=None,
        forced_delta_angle=None,
        rotator_snapshot=None,
    ):
        axis, target_angle, delta_angle, delta_steps, move_params, current_steps, current_angle = self._prepare_rotator_move(
            target_angle,
            forced_delta_angle=forced_delta_angle,
            rotator_snapshot=rotator_snapshot,
        )
        target_display_angle = self.normalized_angle(target_angle)
        if abs(delta_angle) < 0.01 or delta_steps == 0:
            self.events.put(("rotator_position", current_angle, current_steps))
            if settle_s > 0:
                interruptible_sleep(settle_s, stop_event=stop_event)
            return self._stable_rotator_angle(current_angle)
        self._move_rotator_relative(axis, delta_steps, move_params, stop_event=stop_event)
        controller = self.rotator_controller()
        raw_steps = controller.get_axis_raw_steps(axis)
        angle = self._rotator_angle_from_steps(
            raw_steps,
            move_params["steps_per_degree"],
            move_params["direction_sign"],
        )
        angle = self._wait_rotator_angle_reached(
            axis,
            target_display_angle,
            move_params,
            stop_event=stop_event,
        )
        self.events.put(("rotator_position", angle, raw_steps))
        if settle_s > 0:
            interruptible_sleep(settle_s, stop_event=stop_event)
        return self._stable_rotator_angle(angle)

    def _wait_rotator_angle_reached(self, axis, target_angle, move_params, stop_event=None, tolerance_deg=0.08):
        controller = self.rotator_controller()
        timeout_s = max(1.0, float(move_params["timeout_s"]))
        deadline = time.monotonic() + timeout_s
        last_angle = None
        while True:
            if stop_event is not None and stop_event.is_set():
                controller.halt_axis(axis)
                raise StageError("角度测量已停止")
            raw_steps = controller.get_axis_raw_steps(axis)
            angle = self._rotator_angle_from_steps(
                raw_steps,
                move_params["steps_per_degree"],
                move_params["direction_sign"],
            )
            last_angle = self._stable_rotator_angle(angle)
            self.events.put(("rotator_position", last_angle, raw_steps))
            if abs(self._shortest_angle_delta(last_angle, target_angle)) <= tolerance_deg:
                return last_angle
            if time.monotonic() > deadline:
                raise StageError(
                    f"Rotator did not reach target angle. Target {target_angle:.3f} deg, "
                    f"feedback {last_angle:.3f} deg."
                )
            time.sleep(0.05)

    def calibrate_axis(self, axis_name):
        label = f"ANC300 {axis_name}-axis calibration"
        if not self._begin_motion(label):
            return
        try:
            delta_voltage_v = simpledialog.askfloat(
                f"校准{axis_name}轴",
                "本次标定所用的电压变化 ΔV（V）：\n此操作只记录标定，不会移动硬件。",
                parent=self,
            )
            if delta_voltage_v is None:
                return
            measured_displacement_um = simpledialog.askfloat(
                f"校准{axis_name}轴",
                "实测的带符号位移 Δx（um）：\n正负号用于确定方向。",
                parent=self,
            )
            if measured_displacement_um is None:
                return
            try:
                if self.stage is not None:
                    result = self.stage.calibrate_axis(axis_name, delta_voltage_v, measured_displacement_um)
                else:
                    if not math.isfinite(float(delta_voltage_v)) or not math.isfinite(float(measured_displacement_um)):
                        raise StageError("Calibration values must be finite.")
                    if float(delta_voltage_v) == 0 or float(measured_displacement_um) == 0:
                        raise StageError("Calibration voltage and displacement must be non-zero.")
                    ratio = float(measured_displacement_um) / float(delta_voltage_v)
                    result = {
                        "axis": axis_name,
                        "um_per_v": abs(ratio),
                        "direction": 1 if ratio > 0 else -1,
                        "calibration_source": "custom",
                    }
                scale_var = self.anc_x_um_per_v if axis_name == "X" else self.anc_y_um_per_v
                direction_var = self.anc_x_direction if axis_name == "X" else self.anc_y_direction
                scale_var.set(float(result["um_per_v"]))
                direction_var.set(int(result["direction"]))
                self.anc_calibration_source.set("custom")
                self.save_config(silent=True)
                self.refresh_anc300_status()
                self.log(
                    f"{axis_name}轴标定完成：{float(result['um_per_v']):.6g} um/V，"
                    f"方向 {int(result['direction']):+d}（ΔV={float(delta_voltage_v):g} V，"
                    f"实测位移={float(measured_displacement_um):g} um）"
                )
            except Exception as exc:
                self.log(f"{axis_name}轴标定失败：{exc}")
                messagebox.showerror("校准失败", str(exc))
        finally:
            self._end_motion(label)

    def move_origin(self):
        if not self._begin_motion(
            "stage origin move",
            warning_title="正在运动",
            warning_message="角度扫描或其他移动还没结束，先等当前动作完成。",
        ):
            return
        if not self._ensure_stage():
            return self._abort_motion("stage origin move")
        self._start_worker("origin_move_thread", self._move_origin_worker)

    def _move_origin_worker(self):
        try:
            self.stage.move_origin(stop_event=self.stop_event)
            self.events.put(("log", "已回到 ANC300 会话零点"))
            self.events.put(("position", None))
        except Exception as exc:
            self.events.put(("error", f"回零失败：{exc}"))
        finally:
            self._end_motion("stage origin move")

    def start_scan(self):
        if not self._begin_motion(
            "mapping scan",
            warning_title="正在运动",
            warning_message="角度扫描或其他移动还没结束，先等当前动作完成。",
        ):
            return
        if not self._ensure_stage():
            return self._abort_motion("mapping scan")
        try:
            scan_cfg = self.current_scan_config()
            points = list(scan_points(**scan_cfg["point_args"]))
            if not points:
                raise ValueError("扫描点数为 0。")
            self._preflight_stage_points([(x_um, y_um) for _row, _col, x_um, y_um in points])
            self.save_config(silent=True)
        except Exception as exc:
            return self._abort_motion("mapping scan", "参数错误", exc)
        self.points = []
        self.selected_point = None
        self.angle_point_queue = []
        self._refresh_angle_queue_ui()
        self.angle_selected_point_text.set("Selected point: none")
        self._schedule_draw(force=True)
        self._start_worker("scan_thread", self._scan_worker, args=(scan_cfg, points))
        self.log(f"开始扫描，共 {len(points)} 个点")

    def _scan_worker(self, scan_cfg, points):
        total = len(points)
        motion_started = False
        try:
            pmt_settings = scan_cfg["pmt"]
            pmt_enabled = bool(pmt_settings.enabled)
            simulate_pmt = bool(scan_cfg["use_simulated_pmt"])
            validate_pmt_settings(
                pmt_settings,
                simulate=simulate_pmt,
                require_enabled=not simulate_pmt,
            )
            source = scan_cfg.get("measurement_source") or measurement_source(
                pmt_settings,
                simulate_pmt,
                allow_reader_simulation=True,
            )
            dwell_ms = float(scan_cfg["dwell_ms"])
            pre_read_settle_s = float(scan_cfg.get("pre_read_settle_s", 0.2))
            if (not math.isfinite(dwell_ms) or dwell_ms < 0
                    or not math.isfinite(pre_read_settle_s) or pre_read_settle_s < 0):
                raise ValueError("Mapping dwell and pre-read settle must be finite and nonnegative.")
            fields = [
                "index",
                "row",
                "col",
                "x_um",
                "y_um",
                "value",
                "pmt_samples",
                "timestamp",
                "measurement_source",
                *ANC_VOLTAGE_METADATA_FIELDS,
            ]
            csv_path, file_handle, writer = open_result_csv(scan_cfg["output_dir"], "mapping", fields)
            self.latest_csv = csv_path
            self._active_scan_csv = str(csv_path)
            self._active_scan_point = "starting"
            if bool(scan_cfg["use_simulated_pmt"]) and not pmt_enabled:
                self.events.put(("log", "提示：未启用PMT读取，PMT模拟基值不会参与本次扫描"))
            pmt_context = PMTCounter(pmt_settings, simulate=simulate_pmt) if pmt_enabled else nullcontext(None)
            with file_handle as f, pmt_context as pmt:
                for index, (row, col, x_um, y_um) in enumerate(points, start=1):
                    self._active_scan_point = f"{index}/{total}, x={x_um:g} um, y={y_um:g} um"
                    if self.stop_event.is_set():
                        raise StageError("扫描已停止")
                    motion_started = True
                    self.stage.move_to_um(x_um, y_um, stop_event=self.stop_event)
                    voltage_metadata = self._stage_voltage_metadata()
                    if pre_read_settle_s:
                        interruptible_sleep(pre_read_settle_s, stop_event=self.stop_event)
                    if pmt_enabled:
                        value, samples = pmt.read_average(stop_event=self.stop_event)
                    else:
                        value = float(read_value(x_um=x_um, y_um=y_um))
                        samples = [value]
                    item = {
                        "index": index,
                        "row": row,
                        "col": col,
                        "x_um": x_um,
                        "y_um": y_um,
                        "value": value,
                        "pmt_samples": ";".join(f"{sample:g}" for sample in samples),
                        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                        "measurement_source": source,
                        **voltage_metadata,
                    }
                    writer.writerow(item)
                    f.flush()
                    self.events.put(("point", item, index, total))
                    dwell_s = dwell_ms / 1000.0
                    if dwell_s and index < total:
                        interruptible_sleep(dwell_s, stop_event=self.stop_event)
                if scan_cfg["return_to_origin"] and not self.stop_event.is_set():
                    self.events.put(("log", "扫描完成，开始回到 ANC300 会话零点"))
                    self.stage.move_origin(stop_event=self.stop_event)
                    self.events.put(("log", "扫描完成，已回到 ANC300 会话零点"))
                self.events.put(("done", str(csv_path)))
        except Exception as exc:
            if motion_started:
                try:
                    self.stage.stop()
                except Exception:
                    pass
            self.events.put(("error", str(exc)))
        finally:
            self._active_scan_point = None
            self._active_scan_csv = None
            self._end_motion("mapping scan")

    def stop_scan(self):
        if self.operation_busy():
            active = self._active_operation_text()
            confirmed = messagebox.askyesno(
                "确认停止",
                f"{active}正在执行。\n\n停止可能会中断当前测量或移动流程，导致本次数据不完整。\n确定要停止吗？",
                icon="warning",
            )
            if not confirmed:
                self.log(f"已取消停止命令，{active}继续执行")
                return
        with self._motion_state_lock:
            owner = self._motion_label
        if owner is None:
            self.log("当前没有正在执行的操作")
            return
        self.stop_event.set()
        stop_rotator = owner in {"manual rotator move", "angle scan"}
        stop_stage = owner in {
            "mapping scan",
            "selected-point move",
            "stage origin move",
            "ANC300 grounding",
            "angle scan",
        }
        if stop_rotator and self.rotator_stage is not None and self.rotator_stage is not self.stage:
            try:
                self.rotator_stage.stop()
            except Exception as exc:
                self.log(f"半波片停止命令失败：{exc}")
        if stop_stage and self.stage is not None:
            try:
                self.stage.stop()
            except Exception as exc:
                self.log(f"停止命令失败：{exc}")
        self.log("已发送停止命令")

    def current_scan_config(self):
        x_start = float(self.x_start_um.get())
        x_end = float(self.x_end_um.get())
        y_start = float(self.y_start_um.get())
        y_end = float(self.y_end_um.get())
        step = float(self.step_um.get())
        dwell_s = float(self.dwell_s.get())
        pre_read_settle_s = float(self.anc_pre_read_settle_s.get())
        numeric = (x_start, x_end, y_start, y_end, step, dwell_s, pre_read_settle_s)
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("扫描范围、步进和采集时序必须是有限数。")
        if step <= 0:
            raise ValueError("步进必须大于 0。")
        if dwell_s < 0:
            raise ValueError("驻留时间不能为负数。")
        if pre_read_settle_s < 0:
            raise ValueError("Pre-read settle time cannot be negative.")
        simulate_pmt = bool(self.simulation_mode.get())
        pmt = self.validated_pmt_snapshot(
            simulate=simulate_pmt,
            require_enabled=not simulate_pmt,
        )
        return {
            "point_args": {
                "x_start": x_start,
                "x_end": x_end,
                "y_start": y_start,
                "y_end": y_end,
                "step_um": step,
                "serpentine": bool(self.serpentine.get()),
            },
            "dwell_ms": int(round(dwell_s * 1000)),
            "return_to_origin": bool(self.return_to_origin.get()),
            "output_dir": self.output_dir.get(),
            "pmt": pmt,
            "use_simulated_pmt": simulate_pmt,
            "measurement_source": measurement_source(
                pmt,
                simulate_pmt,
                allow_reader_simulation=True,
            ),
            "pre_read_settle_s": pre_read_settle_s,
        }

    def save_config(self, silent=False):
        config = {
            "simulation_mode": bool(self.simulation_mode.get()),
            "anc300": asdict(self.anc300_stage_settings()),
            "scan": {
                "x_start_um": float(self.x_start_um.get()),
                "x_end_um": float(self.x_end_um.get()),
                "y_start_um": float(self.y_start_um.get()),
                "y_end_um": float(self.y_end_um.get()),
                "step_um": float(self.step_um.get()),
                "dwell_ms": int(round(float(self.dwell_s.get()) * 1000)),
                "serpentine": bool(self.serpentine.get()),
                "return_to_origin": bool(self.return_to_origin.get()),
                "output_dir": self.output_dir.get(),
            },
            "pmt": asdict(self.pmt_settings()),
            "rotator": {
                "dll_path": self.rotator_dll_path.get(),
                "connection_type": self.rotator_connection_type.get(),
                "com_port": self.rotator_com_port.get(),
                "ip_address": self.rotator_ip_address.get(),
                "ip_port": int(self.rotator_ip_port.get()),
                "usb_device_index": int(self.rotator_usb_device_index.get()),
                "usb_serial": self.rotator_usb_serial.get().strip(),
                "axis_display": int(self.rotator_axis_display.get()),
                "steps_per_degree": float(self.rotator_steps_per_degree.get()),
                "direction_sign": int(self.rotator_direction_sign.get()),
                "target_angle_deg": float(self.rotator_target_angle_deg.get()),
                "acc": int(self.rotator_acc.get()),
                "dec": int(self.rotator_dec.get()),
                "max_v": int(self.rotator_max_v.get()),
                "start_v": int(self.rotator_start_v.get()),
                "move_timeout_s": float(self.rotator_move_timeout_s.get()),
            },
            "angle_scan": {
                "start_angle_deg": float(self.angle_start_deg.get()),
                "stop_angle_deg": float(self.angle_stop_deg.get()),
                "step_deg": float(self.angle_step_deg.get()),
                "settle_s": float(self.angle_settle_s.get()),
                "return_to_start": bool(self.angle_return_to_start.get()),
            },
        }
        save_app_config(CONFIG_PATH, config)
        self.config_data = config
        if not silent:
            self.log("配置已保存")

    def choose_output_dir(self):
        chosen = filedialog.askdirectory(initialdir=self.output_dir.get())
        if chosen:
            self.output_dir.set(chosen)

    def update_position(self):
        self.refresh_anc300_status()

    def _poll_rotator_angle(self):
        try:
            busy = (
                self.rotator_thread is not None and self.rotator_thread.is_alive()
            ) or (
                self.scan_thread is not None and self.scan_thread.is_alive()
            ) or (
                self.point_move_thread is not None and self.point_move_thread.is_alive()
            ) or (
                self.origin_move_thread is not None and self.origin_move_thread.is_alive()
            ) or (
                self.angle_scan_thread is not None and self.angle_scan_thread.is_alive()
            ) or self.operation_busy()
            if busy:
                return
            if self.rotator_stage is not None and not self.simulation_mode.get():
                conflicts = self._running_mt_tools(max_age_s=3.0)
                if conflicts:
                    names = ", ".join(conflicts)
                    message = f"检测到官方MT工具正在运行：{names}。半波片角度读取已暂停。"
                    if message != self._last_mt_tool_warning:
                        self._last_mt_tool_warning = message
                        self.log(message)
                    self.rotator_position_text.set("半波片: 暂停读取（关闭MTHelper）")
                    return
            if self.rotator_stage is not None and not busy:
                axis = self.rotator_api_axis()
                steps = self.rotator_controller().get_axis_raw_steps(axis)
                angle = self.rotator_angle_from_steps(steps)
                self._set_rotator_position(angle, steps)
                self._last_rotator_poll_error = None
        except Exception as exc:
            message = str(exc)
            if message != self._last_rotator_poll_error:
                self._last_rotator_poll_error = message
                self.log(f"半波片角度刷新失败：{message}")
        finally:
            self.after(200, self._poll_rotator_angle)

    @staticmethod
    def _stable_rotator_angle(angle):
        value = float(angle) % 360.0
        if value < 0.0005 or value >= 359.9995:
            return 0.0
        return value

    def _set_rotator_position(self, angle, steps, force=False):
        display_angle = self._stable_rotator_angle(angle)
        steps = int(steps)
        if self._last_rotator_display is not None and not force:
            old_angle, old_steps = self._last_rotator_display
            angle_delta = abs(self._shortest_angle_delta(old_angle, display_angle))
            if angle_delta < 0.002 and abs(old_steps - steps) <= 5:
                return
        self._last_rotator_display = (display_angle, steps)
        self.rotator_position_text.set(f"半波片: {display_angle:.3f} deg ({steps} 脉冲)")

    def _poll_events(self):
        draw_needed = False
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            kind = event[0]
            if kind == "point":
                item, index, total = event[1], event[2], event[3]
                self.points.append(item)
                self.progress_text.set(f"{index} / {total}")
                self.position_text.set(f"X: {item['x_um']:.2f} um, Y: {item['y_um']:.2f} um")
                draw_needed = True
            elif kind == "log":
                self.log(event[1])
            elif kind == "position":
                self.update_position()
            elif kind == "anc_status":
                self.refresh_anc300_status()
            elif kind == "pmt_status":
                self.pmt_status_text.set(event[1])
            elif kind == "rotator_position":
                angle, steps = event[1], event[2]
                self._set_rotator_position(angle, steps)
            elif kind == "motion_state":
                active = bool(event[1])
                label = event[2]
                self._set_motion_controls_enabled(not active)
                self._set_run_indicator(active, label)
                if active and label:
                    self.status_text.set(f"正在执行：{self._operation_label_text(label)}")
                else:
                    self._update_connection_status()
            elif kind == "run_indicator":
                self._set_run_indicator(bool(event[1]), event[2])
            elif kind == "angle_progress":
                self.angle_progress_text.set(event[1])
            elif kind == "angle_point":
                item, index, total = event[1], event[2], event[3]
                self.angle_points.append(item)
                point_index = int(item.get("queue_point_index", 1))
                point_total = int(item.get("queue_point_total", 1))
                self.angle_progress_text.set(f"Angle scan: point {point_index}/{point_total}, angle {index}/{total}")
                self.angle_latest_count_text.set(f"Latest angle count: {float(item['value']):.2f}")
                if hasattr(self, "angle_table"):
                    self.angle_table.insert(
                        "",
                        "end",
                        values=(
                            f"{float(item['target_angle_deg']):.3f}",
                            f"{float(item['value']):.2f}",
                            str(item["pmt_samples"]),
                        ),
                    )
                    rows = self.angle_table.get_children()
                    if rows:
                        self.angle_table.see(rows[-1])
            elif kind == "angle_point_done":
                point, csv_path, index, total = event[1], event[2], event[3], event[4]
                self.log(
                    f"Angle scan point {index}/{total} completed: X {float(point['x_um']):.2f} um, "
                    f"Y {float(point['y_um']):.2f} um, data saved: {csv_path}"
                )
                self._refresh_angle_queue_ui()
                draw_needed = True
            elif kind == "angle_done":
                self.angle_progress_text.set("Angle scan: completed")
                self.log(f"Angle scan completed: {event[1]}")
            elif kind == "angle_queue":
                self._refresh_angle_queue_ui()
                draw_needed = True
            elif kind == "done":
                self.log(f"扫描完成，数据已保存：{event[1]}")
                self.update_position()
                draw_needed = True
            elif kind == "error":
                self.log(f"错误：{event[1]}")
                messagebox.showerror("运行错误", event[1])
                draw_needed = True
        if draw_needed:
            self._schedule_draw()
        self.after(100, self._poll_events)

    def _schedule_draw(self, force=False):
        if force:
            self._draw_pending = False
            self._last_draw_at = time.monotonic()
            self._draw_points()
            return
        if self._draw_pending:
            return
        elapsed_ms = int((time.monotonic() - self._last_draw_at) * 1000)
        delay_ms = max(0, self._draw_min_interval_ms - elapsed_ms)
        self._draw_pending = True
        self.after(delay_ms, self._flush_draw)

    def _flush_draw(self):
        self._draw_pending = False
        self._last_draw_at = time.monotonic()
        self._draw_points()

    def _draw_points(self):
        self.canvas.delete("all")
        self._plot_bounds = None
        self._cell_hit_bounds = []
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        if width < 120 or height < 120:
            return

        x0 = 68
        y0 = 46
        x1 = max(x0 + 40, width - 108)
        y1 = max(y0 + 40, height - 58)
        self.canvas.create_rectangle(0, 0, width, height, fill="#f8fafc", outline="")
        self.canvas.create_rectangle(x0, y0, x1, y1, fill="#ffffff", outline="")
        self.canvas.create_text(width / 2, 20, text="SHG / PMT Mapping", fill="#1f2937", font=("", 11, "bold"))

        try:
            x_min = min(float(self.x_start_um.get()), float(self.x_end_um.get()))
            x_max = max(float(self.x_start_um.get()), float(self.x_end_um.get()))
            y_min = min(float(self.y_start_um.get()), float(self.y_end_um.get()))
            y_max = max(float(self.y_start_um.get()), float(self.y_end_um.get()))
        except Exception:
            return
        if x_max == x_min:
            x_max += 1
        if y_max == y_min:
            y_max += 1

        try:
            x_centers = decimal_points(
                float(self.x_start_um.get()),
                float(self.x_end_um.get()),
                float(self.step_um.get()),
            )
            y_centers = decimal_points(
                float(self.y_start_um.get()),
                float(self.y_end_um.get()),
                float(self.step_um.get()),
            )
        except Exception:
            x_centers = [x_min, x_max]
            y_centers = [y_min, y_max]
        if self.points:
            x_centers = list(x_centers) + [round(float(point["x_um"]), 6) for point in self.points]
            y_centers = list(y_centers) + [round(float(point["y_um"]), 6) for point in self.points]
        x_centers = sorted({round(float(value), 6) for value in x_centers})
        y_centers = sorted({round(float(value), 6) for value in y_centers})
        column_count = max(1, len(x_centers))
        row_count = max(1, len(y_centers))
        available_w = x1 - x0
        available_h = y1 - y0
        cell_size = max(1.0, min(available_w / column_count, available_h / row_count))
        grid_w = cell_size * column_count
        grid_h = cell_size * row_count
        x0 = x0 + (available_w - grid_w) / 2
        x1 = x0 + grid_w
        y0 = y0 + (available_h - grid_h) / 2
        y1 = y0 + grid_h
        x_bounds = self._cell_bounds(x_centers, x0, cell_size)
        y_bounds = self._cell_bounds(y_centers, y0, cell_size, invert=True)
        self._plot_bounds = (x0, y0, x1, y1)
        self.canvas.create_rectangle(x0, y0, x1, y1, fill="#ffffff", outline="#94a3b8")

        tick_count = 5
        for i in range(tick_count + 1):
            t = i / tick_count
            gx = x0 + t * (x1 - x0)
            gy = y1 - t * (y1 - y0)
            self.canvas.create_line(gx, y0, gx, y1, fill="#e2e8f0")
            self.canvas.create_line(x0, gy, x1, gy, fill="#e2e8f0")
            x_value = x_min + t * (x_max - x_min)
            y_value = y_min + t * (y_max - y_min)
            self.canvas.create_text(gx, y1 + 16, text=self._format_axis_tick(x_value), fill="#64748b", font=("", 8))
            self.canvas.create_text(x0 - 10, gy, text=self._format_axis_tick(y_value), fill="#64748b", font=("", 8), anchor="e")

        self.canvas.create_text((x0 + x1) / 2, height - 12, text="X (um)", fill="#475569")
        self.canvas.create_text(24, (y0 + y1) / 2, text="Y (um)", fill="#475569")

        if not self.points:
            if len(x_centers) * len(y_centers) <= 2500:
                for x_um in x_centers:
                    left, right = x_bounds[round(float(x_um), 6)]
                    for y_um in y_centers:
                        top, bottom = y_bounds[round(float(y_um), 6)]
                        self.canvas.create_rectangle(left, top, right, bottom, fill="#e2e8f0", outline="#e2e8f0")
                self.canvas.create_rectangle(x0, y0, x1, y1, outline="#94a3b8")
            self.canvas.create_text((x0 + x1) / 2, (y0 + y1) / 2, text="等待扫描数据", fill="#94a3b8")
            self._draw_colorbar(x1 + 28, y0, y1, None, None)
            return

        values = [float(point["value"]) for point in self.points]
        vmin = min(values)
        vmax = max(values)
        latest_bounds = None
        selected_bounds = None
        queued_overlays = []
        for point in self.points:
            value = float(point["value"])
            color = self._value_color(value, vmin, vmax)
            x_key = round(float(point["x_um"]), 6)
            y_key = round(float(point["y_um"]), 6)
            left, right = x_bounds[x_key]
            top, bottom = y_bounds[y_key]
            self.canvas.create_rectangle(left, top, right, bottom, fill=color, outline=color)
            self._cell_hit_bounds.append({"bounds": (left, top, right, bottom), "point": point})
            if self.selected_point is not None:
                if (
                    round(float(self.selected_point.get("x_um", 0.0)), 6) == x_key
                    and round(float(self.selected_point.get("y_um", 0.0)), 6) == y_key
                ):
                    selected_bounds = (left, top, right, bottom)
            queue_index = self._queue_index_for_key((x_key, y_key))
            if queue_index is not None:
                queued_overlays.append((queue_index + 1, self.angle_point_queue[queue_index], (left, top, right, bottom)))
            latest_bounds = (left, top, right, bottom)

        if self.points:
            last = self.points[-1]
            if latest_bounds is not None:
                left, top, right, bottom = latest_bounds
                self.canvas.create_rectangle(left, top, right, bottom, outline="#111827", width=2)
            if selected_bounds is not None:
                left, top, right, bottom = selected_bounds
                self.canvas.create_rectangle(left + 2, top + 2, right - 2, bottom - 2, outline="#f8fafc", width=2)
                self.canvas.create_rectangle(left + 4, top + 4, right - 4, bottom - 4, outline="#0f172a", width=2)
            for order, point, bounds in queued_overlays:
                left, top, right, bottom = bounds
                status = str(point.get("queue_status", "pending"))
                outline = {
                    "running": "#f59e0b",
                    "done": "#16a34a",
                    "failed": "#dc2626",
                }.get(status, "#2563eb")
                self.canvas.create_rectangle(left + 1, top + 1, right - 1, bottom - 1, outline=outline, width=2)
                cx = (left + right) / 2
                cy = (top + bottom) / 2
                radius = max(7, min(12, (right - left) * 0.28, (bottom - top) * 0.28))
                self.canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, fill="#ffffff", outline=outline, width=2)
                self.canvas.create_text(cx, cy, text=str(order), fill="#111827", font=("", 8, "bold"))
            self.canvas.create_text(
                x0,
                y0 - 18,
                text=f"最新: X {float(last['x_um']):.2f} um, Y {float(last['y_um']):.2f} um, Count {self._format_value(float(last['value']))}",
                fill="#334155",
                anchor="w",
            )
        self.canvas.create_rectangle(x0, y0, x1, y1, outline="#94a3b8")
        self._draw_colorbar(x1 + 28, y0, y1, vmin, vmax)

    @staticmethod
    def _value_color(value, vmin, vmax):
        t = MappingApp._normalized_value(value, vmin, vmax)
        return MappingApp._color_from_t(t)

    @staticmethod
    def _normalized_value(value, vmin, vmax):
        if vmax <= vmin:
            value = max(0.0, float(value))
            return max(0.0, min(1.0, math.log10(value + 1.0) / 5.0))
        return max(0.0, min(1.0, (float(value) - float(vmin)) / (float(vmax) - float(vmin))))

    @staticmethod
    def _cell_bounds(centers, start_px, cell_size, invert=False):
        bounds = {}
        count = len(centers)
        for index, center in enumerate(centers):
            grid_index = count - index - 1 if invert else index
            first = start_px + grid_index * cell_size
            second = start_px + (grid_index + 1) * cell_size
            bounds[round(float(center), 6)] = (first - 0.5, second + 0.5)
        return bounds

    @staticmethod
    def _color_from_t(t):
        t = max(0.0, min(1.0, float(t)))
        stops = [
            (0.00, (48, 18, 59)),
            (0.08, (62, 55, 151)),
            (0.16, (70, 117, 237)),
            (0.25, (57, 162, 252)),
            (0.33, (27, 207, 212)),
            (0.42, (36, 236, 166)),
            (0.50, (97, 252, 108)),
            (0.58, (164, 252, 60)),
            (0.67, (209, 232, 52)),
            (0.75, (243, 198, 58)),
            (0.83, (254, 155, 45)),
            (0.92, (239, 90, 17)),
            (1.00, (194, 42, 4)),
        ]
        for index in range(len(stops) - 1):
            left_t, left_color = stops[index]
            right_t, right_color = stops[index + 1]
            if left_t <= t <= right_t:
                local_t = (t - left_t) / (right_t - left_t)
                r = int(left_color[0] + (right_color[0] - left_color[0]) * local_t)
                g = int(left_color[1] + (right_color[1] - left_color[1]) * local_t)
                b = int(left_color[2] + (right_color[2] - left_color[2]) * local_t)
                return f"#{r:02x}{g:02x}{b:02x}"
        r, g, b = stops[-1][1]
        return f"#{r:02x}{g:02x}{b:02x}"

    def _draw_colorbar(self, x, y0, y1, vmin, vmax):
        bar_width = 16
        segments = max(16, int(y1 - y0))
        for i in range(segments):
            t = 1.0 - i / max(1, segments - 1)
            y = y0 + i * (y1 - y0) / segments
            self.canvas.create_rectangle(
                x,
                y,
                x + bar_width,
                y + (y1 - y0) / segments + 1,
                fill=self._color_from_t(t),
                outline="",
            )
        self.canvas.create_rectangle(x, y0, x + bar_width, y1, outline="#94a3b8")
        self.canvas.create_text(x + bar_width + 8, y0, text="计数", fill="#475569", anchor="w", font=("", 8, "bold"))
        if vmin is None or vmax is None:
            self.canvas.create_text(x + bar_width + 8, y0 + 18, text="高", fill="#64748b", anchor="w", font=("", 8))
            self.canvas.create_text(x + bar_width + 8, y1 - 4, text="低", fill="#64748b", anchor="w", font=("", 8))
            return
        self.canvas.create_text(x + bar_width + 8, y0 + 18, text=self._format_value(vmax), fill="#64748b", anchor="w", font=("", 8))
        self.canvas.create_text(x + bar_width + 8, y1 - 4, text=self._format_value(vmin), fill="#64748b", anchor="w", font=("", 8))

    @staticmethod
    def _format_axis_tick(value):
        value = float(value)
        if abs(value) >= 1000:
            return f"{value:.0f}"
        if abs(value) >= 10:
            return f"{value:.1f}"
        return f"{value:.2f}"

    @staticmethod
    def _format_value(value):
        value = float(value)
        if abs(value) >= 10000:
            return f"{value:.3g}"
        if abs(value) >= 100:
            return f"{value:.0f}"
        if abs(value) >= 10:
            return f"{value:.1f}"
        return f"{value:.2f}"

    def _update_point_count(self):
        try:
            points = list(scan_points(
                float(self.x_start_um.get()),
                float(self.x_end_um.get()),
                float(self.y_start_um.get()),
                float(self.y_end_um.get()),
                float(self.step_um.get()),
                bool(self.serpentine.get()),
            ))
            self.point_count_text.set(f"预计点数: {len(points)}")
        except Exception:
            self.point_count_text.set("预计点数: 参数错误")

    def _ensure_stage(self):
        if self.stage is None:
            messagebox.showwarning("未连接", "请先连接控制器。")
            return False
        return self._backend_mode_matches(self.stage, SimulatedANC300Stage, "ANC300")

    def _preflight_stage_points(self, points):
        status = self.stage.get_status()
        if not status.get("connected"):
            raise StageError("ANC300 stage is not connected.")
        if not status.get("origin_set"):
            raise StageError("Set the ANC300 session origin before starting a scan.")
        if not status.get("outputs_enabled"):
            raise StageError("ANC300 offset outputs must be explicitly enabled before starting a scan.")
        return self.stage.preflight_points(points)

    def _stage_voltage_metadata(self):
        metadata = self.stage.get_voltage_metadata()
        return {field: metadata[field] for field in ANC_VOLTAGE_METADATA_FIELDS}

    def _ensure_controller(self, auto_connect=False, allow_reserved_motion=False):
        if self.stage is not None and not self._backend_mode_matches(self.stage, SimulatedANC300Stage, "ANC300"):
            return False
        if self.rotator_stage is not None:
            return self._backend_mode_matches(self.rotator_stage, SimulatedStage, "HWP")
        if auto_connect:
            with self._motion_state_lock:
                reserved = self._motion_active and self._motion_label is not None
            if reserved and not allow_reserved_motion:
                return False
            return self.connect_rotator_only(allow_reserved_motion=allow_reserved_motion)
        messagebox.showwarning("未连接", "请先连接控制器。")
        return False

    def _ensure_no_external_mt_tool(self, action):
        conflicts = self._running_mt_tools(max_age_s=0.0)
        if not conflicts:
            self._last_mt_tool_warning = None
            return True
        names = ", ".join(conflicts)
        message = (
            f"检测到官方MT工具正在运行：{names}。\n\n"
            f"请先关闭这些程序，再{action}。\n"
            "同一个USB控制器不能同时被MTHelper和Python控制。"
        )
        self.status_text.set("控制器被官方工具占用")
        if message != self._last_mt_tool_warning:
            self._last_mt_tool_warning = message
            self.log(message.replace("\n", " "))
        messagebox.showwarning("控制器被占用", message)
        return False

    def _running_mt_tools(self, max_age_s=0.0):
        now = time.monotonic()
        if max_age_s > 0 and now - self._last_mt_tool_check_at < max_age_s:
            return list(self._cached_mt_tool_conflicts)
        self._last_mt_tool_check_at = now
        conflicts = []
        try:
            result = subprocess.run(
                ["tasklist", "/FO", "CSV", "/NH"],
                capture_output=True,
                text=True,
                timeout=2,
                creationflags=CREATE_NO_WINDOW,
            )
        except Exception:
            result = None
        if result is not None and result.returncode == 0:
            for row in csv.reader(result.stdout.splitlines()):
                if not row:
                    continue
                process_name = row[0].strip()
                if process_name.lower() in MT_TOOL_PROCESS_NAMES and process_name not in conflicts:
                    conflicts.append(process_name)
        if not conflicts:
            conflicts = self._running_mt_tools_from_powershell()
        self._cached_mt_tool_conflicts = conflicts
        return list(conflicts)

    def _running_mt_tools_from_powershell(self):
        names = ["MTHelper", "MTHelper_V3", "MTSimulator", "IOConfig", "upgrade"]
        quoted_names = ", ".join(f"'{name}'" for name in names)
        command = (
            f"$names=@({quoted_names}); "
            "Get-Process -ErrorAction SilentlyContinue | "
            "Where-Object { $names -contains $_.ProcessName } | "
            "Select-Object -ExpandProperty ProcessName"
        )
        try:
            result = subprocess.run(
                ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", command],
                capture_output=True,
                text=True,
                timeout=2,
                creationflags=CREATE_NO_WINDOW,
            )
        except Exception:
            return []
        conflicts = []
        for line in result.stdout.splitlines():
            process_name = line.strip()
            if not process_name:
                continue
            display_name = f"{process_name}.exe"
            if display_name not in conflicts:
                conflicts.append(display_name)
        return conflicts

    def log(self, message):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_box.insert(END, f"[{ts}] {message}\n")
        self._trim_log_box()
        self.log_box.see(END)

    def _trim_log_box(self):
        try:
            line_count = int(self.log_box.index("end-1c").split(".", 1)[0])
        except Exception:
            return
        if line_count <= self._max_log_lines:
            return
        delete_to_line = max(2, line_count - self._max_log_lines + 1)
        self.log_box.delete("1.0", f"{delete_to_line}.0")

    def _write_crash_log(self, message):
        try:
            with CRASH_LOG_PATH.open("a", encoding="utf-8") as f:
                f.write(f"[{datetime.now().isoformat(timespec='seconds')}] {message}\n")
                f.write(f"active_scan_csv={self._active_scan_csv}\n")
                f.write(f"active_scan_point={self._active_scan_point}\n")
        except Exception:
            pass

    def on_close(self):
        if self.operation_busy():
            active = self._active_operation_text()
            messagebox.showwarning(
                "禁止关闭",
                f"{active}还没有结束。\n\n为避免半波片、位移台或PMT测量被中断，程序暂时不能关闭。请等待所有运动、扫描或半波片归零完成后再关闭。",
            )
            self.log(f"已阻止关闭程序：{active}仍在执行")
            return
        if self._disconnect_stage_now():
            self.destroy()
        else:
            self.log("窗口保持打开：请处理断开失败后重试。")


if __name__ == "__main__":
    with CRASH_LOG_PATH.open("a", encoding="utf-8") as crash_log:
        crash_log.write(f"\n[{datetime.now().isoformat(timespec='seconds')}] SHG Mapping starting\n")
        faulthandler.enable(crash_log, all_threads=True)
        sys.stderr = crash_log
        app = MappingApp()
        try:
            app.mainloop()
        except Exception as exc:
            app._write_crash_log(f"Unhandled exception: {exc}")
            raise
