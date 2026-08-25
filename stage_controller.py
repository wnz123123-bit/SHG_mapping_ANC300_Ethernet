from __future__ import annotations

import ctypes
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path


R_OK = 0


@dataclass
class StageSettings:
    dll_path: str
    connection_type: str = "USB"
    com_port: str = "COM1"
    ip_address: str = "127.0.0.1"
    ip_port: int = 8888
    usb_device_index: int = 0
    usb_serial: str = ""
    x_axis: int = 0
    y_axis: int = 1
    x_steps_per_um: float = 1.6
    y_steps_per_um: float = 1.6
    acc: int = 1000
    dec: int = 1000
    max_v: int = 2000
    start_v: int = 0
    move_timeout_s: float = 30.0


class StageError(RuntimeError):
    pass


class _MTDllSession:
    def __init__(self, path: Path):
        self.path = path
        self.dll = None
        self.users = 0
        self.initialized = False
        self.lock = threading.RLock()


class BaseStage:
    def connect(self):
        raise NotImplementedError

    def disconnect(self):
        raise NotImplementedError

    def configure_motion(self):
        raise NotImplementedError

    def set_origin(self):
        raise NotImplementedError

    def move_to_um(self, x_um: float, y_um: float, stop_event=None):
        raise NotImplementedError

    def move_origin(self, stop_event=None):
        self.move_to_um(0.0, 0.0, stop_event=stop_event)

    def stop(self):
        raise NotImplementedError

    def halt_axis(self, axis: int):
        raise NotImplementedError

    def recover_axis(self, axis: int, acc=None, dec=None, max_v=None, start_v=None):
        raise NotImplementedError

    def get_position_um(self):
        raise NotImplementedError

    def get_axis_steps(self, axis: int) -> int:
        raise NotImplementedError

    def get_axis_raw_steps(self, axis: int) -> int:
        raise NotImplementedError

    def get_axis_diagnostics(self, axis: int) -> dict:
        raise NotImplementedError

    def get_axis_count(self) -> int:
        raise NotImplementedError

    def set_axis_origin(self, axis: int):
        raise NotImplementedError

    def set_axis_position(self, axis: int, steps: int):
        raise NotImplementedError

    def move_axis_steps_abs(self, axis: int, target_steps: int, acc=None, dec=None, max_v=None, start_v=None, stop_event=None):
        raise NotImplementedError

    def move_axis_steps_rel(self, axis: int, delta_steps: int, acc=None, dec=None, max_v=None, start_v=None, stop_event=None, progress_callback=None):
        raise NotImplementedError


class SimulatedStage(BaseStage):
    def __init__(self, settings: StageSettings):
        self.settings = settings
        self.connected = False
        self.x_um = 0.0
        self.y_um = 0.0
        self.axis_steps = {}

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def configure_motion(self):
        self._require_connected()

    def set_origin(self):
        self._require_connected()
        self.x_um = 0.0
        self.y_um = 0.0

    def move_to_um(self, x_um: float, y_um: float, stop_event=None):
        self._require_connected()
        if stop_event is not None and stop_event.is_set():
            raise StageError("Scan stopped by user.")
        distance = abs(x_um - self.x_um) + abs(y_um - self.y_um)
        time.sleep(min(0.15, 0.001 + distance / 100000.0))
        if stop_event is not None and stop_event.is_set():
            raise StageError("Scan stopped by user.")
        self.x_um = float(x_um)
        self.y_um = float(y_um)

    def stop(self):
        return None

    def halt_axis(self, axis: int):
        self._require_connected()

    def recover_axis(self, axis: int, acc=None, dec=None, max_v=None, start_v=None):
        self._require_connected()

    def get_position_um(self):
        self._require_connected()
        return self.x_um, self.y_um

    def get_axis_steps(self, axis: int) -> int:
        self._require_connected()
        return int(self.axis_steps.get(int(axis), 0))

    def get_axis_raw_steps(self, axis: int) -> int:
        return self.get_axis_steps(axis)

    def get_axis_diagnostics(self, axis: int) -> dict:
        steps = self.get_axis_steps(axis)
        return {
            "axis": int(axis),
            "software_steps": steps,
            "raw_steps": steps,
            "run": 0,
            "dir": 0,
            "neg": 0,
            "pos": 0,
            "zero": 0,
            "mode": 0,
            "soft_neg": 0,
            "soft_pos": 0,
        }

    def get_axis_count(self) -> int:
        self._require_connected()
        configured = [self.settings.x_axis, self.settings.y_axis, *self.axis_steps.keys()]
        return max(4, max((int(axis) for axis in configured), default=0) + 1)

    def set_axis_origin(self, axis: int):
        self._require_connected()
        self.set_axis_position(axis, 0)

    def set_axis_position(self, axis: int, steps: int):
        self._require_connected()
        self.axis_steps[int(axis)] = int(steps)

    def move_axis_steps_abs(self, axis: int, target_steps: int, acc=None, dec=None, max_v=None, start_v=None, stop_event=None):
        self._require_connected()
        if stop_event is not None and stop_event.is_set():
            raise StageError("Move stopped by user.")
        old = self.get_axis_steps(axis)
        time.sleep(min(0.15, 0.001 + abs(int(target_steps) - old) / 100000.0))
        if stop_event is not None and stop_event.is_set():
            raise StageError("Move stopped by user.")
        self.axis_steps[int(axis)] = int(target_steps)

    def move_axis_steps_rel(self, axis: int, delta_steps: int, acc=None, dec=None, max_v=None, start_v=None, stop_event=None, progress_callback=None):
        self._require_connected()
        axis = int(axis)
        start = self.get_axis_steps(axis)
        target = start + int(delta_steps)
        if stop_event is not None and stop_event.is_set():
            raise StageError("Move stopped by user.")
        duration = min(0.25, 0.001 + abs(int(delta_steps)) / 200000.0)
        slices = max(1, int(duration / 0.02))
        for index in range(1, slices + 1):
            if stop_event is not None and stop_event.is_set():
                raise StageError("Move stopped by user.")
            self.axis_steps[axis] = int(round(start + int(delta_steps) * index / slices))
            if progress_callback is not None:
                progress_callback(self.axis_steps[axis])
            time.sleep(duration / slices)
        self.axis_steps[axis] = target
        if progress_callback is not None:
            progress_callback(target)

    def _require_connected(self):
        if not self.connected:
            raise StageError("Stage is not connected.")


class MTStage(BaseStage):
    _sessions = {}
    _sessions_lock = threading.RLock()

    def __init__(self, settings: StageSettings):
        self.settings = settings
        self.dll = None
        self.connected = False
        self._initialized = False
        self._multi_usb = False
        self._session = None
        self._transport_open = False
        self.product_serial = ""

    def connect(self):
        if self.connected:
            return
        self._load_dll()
        self._init_dll()

        connection_type = self.settings.connection_type.upper()
        if connection_type == "USB":
            if int(self.settings.usb_device_index) >= 0 and hasattr(self.dll, "MT_M_Open_USB"):
                self._multi_usb = True
                self._call("MT_M_Open_USB", ctypes.c_int32(int(self.settings.usb_device_index)))
                self._transport_open = True
            else:
                self._call("MT_Close_UART", allow_missing=True, ignore_error=True)
                self._call("MT_Close_Net", allow_missing=True, ignore_error=True)
                self._call("MT_Open_USB")
                self._transport_open = True
        elif connection_type == "UART":
            self._call("MT_Close_USB", allow_missing=True, ignore_error=True)
            self._call("MT_Close_Net", allow_missing=True, ignore_error=True)
            port = self.settings.com_port.encode("gbk")
            self._call("MT_Open_UART", ctypes.c_char_p(port))
            self._transport_open = True
        elif connection_type == "NET":
            self._call("MT_Close_USB", allow_missing=True, ignore_error=True)
            self._call("MT_Close_UART", allow_missing=True, ignore_error=True)
            ip_parts = self._parse_ip(self.settings.ip_address)
            self._call(
                "MT_Open_Net",
                ctypes.c_uint8(ip_parts[0]),
                ctypes.c_uint8(ip_parts[1]),
                ctypes.c_uint8(ip_parts[2]),
                ctypes.c_uint8(ip_parts[3]),
                ctypes.c_uint16(self.settings.ip_port),
            )
            self._transport_open = True
        else:
            raise StageError(f"Unsupported connection type: {self.settings.connection_type}")

        self._device_call("MT_Check")
        self.connected = True
        self.product_serial = self.get_product_serial()
        self.configure_motion()

    def disconnect(self):
        if self.dll is None:
            return
        try:
            if self.connected or self._transport_open:
                if self._multi_usb:
                    self._call("MT_M_Close_USB", ctypes.c_int32(int(self.settings.usb_device_index)), allow_missing=True, ignore_error=True)
                else:
                    self._call("MT_Close_UART", allow_missing=True, ignore_error=True)
                    self._call("MT_Close_Net", allow_missing=True, ignore_error=True)
                    self._call("MT_Close_USB", allow_missing=True, ignore_error=True)
        finally:
            self.connected = False
            self._transport_open = False
            self._release_dll()

    def configure_motion(self):
        self._require_connected()
        for axis in dict.fromkeys((int(self.settings.x_axis), int(self.settings.y_axis))):
            self._device_call("MT_Set_Axis_Mode_Position", ctypes.c_uint16(axis))
            self._device_call("MT_Set_Axis_Position_Acc", ctypes.c_uint16(axis), ctypes.c_int32(self.settings.acc))
            self._device_call("MT_Set_Axis_Position_Dec", ctypes.c_uint16(axis), ctypes.c_int32(self.settings.dec))
            if self.settings.start_v > 0:
                self._device_call(
                    "MT_Set_Axis_Position_V_Start",
                    ctypes.c_uint16(axis),
                    ctypes.c_int32(self.settings.start_v),
                    allow_missing=True,
                )
            self._device_call("MT_Set_Axis_Position_V_Max", ctypes.c_uint16(axis), ctypes.c_int32(self.settings.max_v))

    def set_origin(self):
        self._require_connected()
        for axis in dict.fromkeys((int(self.settings.x_axis), int(self.settings.y_axis))):
            self._device_call("MT_Set_Axis_Software_P", ctypes.c_uint16(axis), ctypes.c_int32(0))

    def move_to_um(self, x_um: float, y_um: float, stop_event=None):
        self._require_connected()
        axes = (self.settings.x_axis, self.settings.y_axis)
        start_raw_positions = {int(axis): self._get_axis_raw_position(int(axis)) for axis in axes}
        x_steps = self._um_to_steps(x_um, self.settings.x_steps_per_um)
        y_steps = self._um_to_steps(y_um, self.settings.y_steps_per_um)
        self._device_call(
            "MT_Set_Axis_Position_P_Target_Abs",
            ctypes.c_uint16(self.settings.x_axis),
            ctypes.c_int32(x_steps),
        )
        self._device_call(
            "MT_Set_Axis_Position_P_Target_Abs",
            ctypes.c_uint16(self.settings.y_axis),
            ctypes.c_int32(y_steps),
        )
        self._wait_for_axes(axes, stop_event=stop_event, start_raw_positions=start_raw_positions)

    def stop(self):
        if self.dll is not None:
            for axis in dict.fromkeys((int(self.settings.x_axis), int(self.settings.y_axis))):
                self.halt_axis(axis)

    def halt_axis(self, axis: int):
        if self.dll is not None:
            self._device_call("MT_Set_Axis_Halt", ctypes.c_uint16(int(axis)), allow_missing=True, ignore_error=True)

    def recover_axis(self, axis: int, acc=None, dec=None, max_v=None, start_v=None):
        self._require_connected()
        axis = int(axis)
        self.halt_axis(axis)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                if not self._axis_is_running(axis):
                    break
            except Exception:
                break
            time.sleep(0.05)
        self._configure_axis_position(axis, acc=acc, dec=dec, max_v=max_v, start_v=start_v)

    def get_position_um(self):
        self._require_connected()
        x_steps = self._get_axis_software_position(self.settings.x_axis)
        y_steps = self._get_axis_software_position(self.settings.y_axis)
        return (
            x_steps / self.settings.x_steps_per_um,
            y_steps / self.settings.y_steps_per_um,
        )

    def get_axis_steps(self, axis: int) -> int:
        self._require_connected()
        return self._get_axis_software_position(int(axis))

    def get_axis_raw_steps(self, axis: int) -> int:
        self._require_connected()
        return self._get_axis_raw_position(int(axis))

    def get_axis_diagnostics(self, axis: int) -> dict:
        self._require_connected()
        axis = int(axis)
        data = {
            "axis": axis,
            "software_steps": self._get_axis_software_position(axis),
            "raw_steps": self._get_axis_raw_position(axis),
        }
        data.update(self._get_axis_status(axis))
        return data

    def get_axis_count(self) -> int:
        self._require_connected()
        if not self._has_device_function("MT_Get_Axis_Num"):
            return 0
        value = ctypes.c_int32(0)
        self._device_call("MT_Get_Axis_Num", ctypes.byref(value))
        return int(value.value)

    def get_product_serial(self) -> str:
        self._require_connected()
        for name in ("MT_Get_Product_SN", "MT_Get_Product_SN3"):
            if not self._has_device_function(name):
                continue
            value = ctypes.create_string_buffer(128)
            try:
                self._device_call(name, value)
            except Exception:
                continue
            serial = value.value.decode("ascii", errors="ignore").strip()
            if serial:
                return serial
        return ""

    def set_axis_origin(self, axis: int):
        self._require_connected()
        self.set_axis_position(axis, 0)

    def set_axis_position(self, axis: int, steps: int):
        self._require_connected()
        axis = int(axis)
        steps = int(steps)
        self._device_call("MT_Set_Axis_P_Now", ctypes.c_uint16(axis), ctypes.c_int32(steps), allow_missing=True)
        self._device_call("MT_Set_Axis_Software_P", ctypes.c_uint16(axis), ctypes.c_int32(steps))

    def move_axis_steps_abs(self, axis: int, target_steps: int, acc=None, dec=None, max_v=None, start_v=None, stop_event=None):
        self._require_connected()
        axis = int(axis)
        start_raw_positions = {axis: self._get_axis_raw_position(axis)}
        self._configure_axis_position(axis, acc=acc, dec=dec, max_v=max_v, start_v=start_v)
        self._device_call(
            "MT_Set_Axis_Position_P_Target_Abs",
            ctypes.c_uint16(axis),
            ctypes.c_int32(int(target_steps)),
        )
        self._wait_for_axes((axis,), stop_event=stop_event, start_raw_positions=start_raw_positions)

    def move_axis_steps_rel(self, axis: int, delta_steps: int, acc=None, dec=None, max_v=None, start_v=None, stop_event=None, progress_callback=None):
        self._require_connected()
        axis = int(axis)
        start_raw = self._get_axis_raw_position(axis)
        start_software = self._get_axis_software_position(axis)
        self._configure_axis_position(axis, acc=acc, dec=dec, max_v=max_v, start_v=start_v)
        self._device_call(
            "MT_Set_Axis_Position_P_Target_Rel",
            ctypes.c_uint16(axis),
            ctypes.c_int32(int(delta_steps)),
        )
        running_seen = self._wait_for_axes(
            (axis,),
            stop_event=stop_event,
            progress_callback=progress_callback,
            start_raw_positions={axis: start_raw},
        )
        end_raw = self._get_axis_raw_position(axis)
        end_software = self._get_axis_software_position(axis)
        return {
            "start_raw": start_raw,
            "end_raw": end_raw,
            "start_software": start_software,
            "end_software": end_software,
            "running_seen": running_seen,
        }

    def _configure_axis_position(self, axis: int, acc=None, dec=None, max_v=None, start_v=None):
        self._device_call("MT_Set_Axis_Mode_Position", ctypes.c_uint16(axis))
        if acc is not None:
            self._device_call("MT_Set_Axis_Position_Acc", ctypes.c_uint16(axis), ctypes.c_int32(int(acc)))
        if dec is not None:
            self._device_call("MT_Set_Axis_Position_Dec", ctypes.c_uint16(axis), ctypes.c_int32(int(dec)))
        if start_v is not None and int(start_v) > 0:
            self._device_call(
                "MT_Set_Axis_Position_V_Start",
                ctypes.c_uint16(axis),
                ctypes.c_int32(int(start_v)),
                allow_missing=True,
            )
        if max_v is not None:
            self._device_call("MT_Set_Axis_Position_V_Max", ctypes.c_uint16(axis), ctypes.c_int32(int(max_v)))

    def _load_dll(self):
        if self.dll is not None:
            return
        dll_path = Path(self.settings.dll_path).resolve()
        if not dll_path.exists():
            raise StageError(f"MT_API.dll not found: {dll_path}")
        key = str(dll_path).lower()
        with self._sessions_lock:
            session = self._sessions.get(key)
            if session is None:
                session = _MTDllSession(dll_path)
                self._sessions[key] = session
        with session.lock:
            if session.dll is None:
                if hasattr(os, "add_dll_directory"):
                    os.add_dll_directory(str(dll_path.parent))
                session.dll = ctypes.WinDLL(str(dll_path))
            session.users += 1
            self._session = session
            self.dll = session.dll
        self._configure_signatures()

    def _init_dll(self):
        if self._session is None:
            raise StageError("MT_API.dll session is not loaded.")
        with self._session.lock:
            if not self._session.initialized:
                self._call("MT_Init")
                self._session.initialized = True
        self._initialized = True

    def _release_dll(self):
        session = self._session
        if session is None:
            self.dll = None
            return
        with session.lock:
            if session.users > 0:
                session.users -= 1
            if session.users == 0:
                if session.initialized:
                    self._call("MT_DeInit", allow_missing=True, ignore_error=True)
                    session.initialized = False
                session.dll = None
        self._session = None
        self.dll = None
        self._initialized = False

    def _configure_signatures(self):
        signatures = {
            "MT_Init": ([], ctypes.c_int32),
            "MT_DeInit": ([], ctypes.c_int32),
            "MT_Open_USB": ([], ctypes.c_int32),
            "MT_Close_USB": ([], ctypes.c_int32),
            "MT_M_Open_USB": ([ctypes.c_int32], ctypes.c_int32),
            "MT_M_Open_USB_SN": ([ctypes.c_int32, ctypes.c_char_p], ctypes.c_int32),
            "MT_M_Close_USB": ([ctypes.c_int32], ctypes.c_int32),
            "MT_Open_UART": ([ctypes.c_char_p], ctypes.c_int32),
            "MT_Close_UART": ([], ctypes.c_int32),
            "MT_Open_Net": ([ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16], ctypes.c_int32),
            "MT_Close_Net": ([], ctypes.c_int32),
            "MT_Check": ([], ctypes.c_int32),
            "MT_M_Check": ([ctypes.c_int32], ctypes.c_int32),
            "MT_Get_Product_SN": ([ctypes.c_char_p], ctypes.c_int32),
            "MT_M_Get_Product_SN": ([ctypes.c_int32, ctypes.c_char_p], ctypes.c_int32),
            "MT_Get_Product_SN3": ([ctypes.c_char_p], ctypes.c_int32),
            "MT_M_Get_Product_SN3": ([ctypes.c_int32, ctypes.c_char_p], ctypes.c_int32),
            "MT_Get_Axis_Num": ([ctypes.POINTER(ctypes.c_int32)], ctypes.c_int32),
            "MT_M_Get_Axis_Num": ([ctypes.c_int32, ctypes.POINTER(ctypes.c_int32)], ctypes.c_int32),
            "MT_Set_Axis_Mode_Position": ([ctypes.c_uint16], ctypes.c_int32),
            "MT_M_Set_Axis_Mode_Position": ([ctypes.c_int32, ctypes.c_uint16], ctypes.c_int32),
            "MT_Set_Axis_Position_Acc": ([ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_M_Set_Axis_Position_Acc": ([ctypes.c_int32, ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_Set_Axis_Position_Dec": ([ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_M_Set_Axis_Position_Dec": ([ctypes.c_int32, ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_Set_Axis_Position_V_Start": ([ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_M_Set_Axis_Position_V_Start": ([ctypes.c_int32, ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_Set_Axis_Position_V_Max": ([ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_M_Set_Axis_Position_V_Max": ([ctypes.c_int32, ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_Set_Axis_Position_P_Target_Abs": ([ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_M_Set_Axis_Position_P_Target_Abs": ([ctypes.c_int32, ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_Set_Axis_Position_P_Target_Rel": ([ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_M_Set_Axis_Position_P_Target_Rel": ([ctypes.c_int32, ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_Set_Axis_Position_Stop": ([ctypes.c_uint16], ctypes.c_int32),
            "MT_M_Set_Axis_Position_Stop": ([ctypes.c_int32, ctypes.c_uint16], ctypes.c_int32),
            "MT_Set_Axis_Halt": ([ctypes.c_uint16], ctypes.c_int32),
            "MT_M_Set_Axis_Halt": ([ctypes.c_int32, ctypes.c_uint16], ctypes.c_int32),
            "MT_Set_Axis_Halt_All": ([], ctypes.c_int32),
            "MT_M_Set_Axis_Halt_All": ([ctypes.c_int32], ctypes.c_int32),
            "MT_Set_Axis_Software_P": ([ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_M_Set_Axis_Software_P": ([ctypes.c_int32, ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_Set_Axis_P_Now": ([ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_M_Set_Axis_P_Now": ([ctypes.c_int32, ctypes.c_uint16, ctypes.c_int32], ctypes.c_int32),
            "MT_Get_Axis_P_Now": ([ctypes.c_uint16, ctypes.POINTER(ctypes.c_int32)], ctypes.c_int32),
            "MT_M_Get_Axis_P_Now": ([ctypes.c_int32, ctypes.c_uint16, ctypes.POINTER(ctypes.c_int32)], ctypes.c_int32),
            "MT_Get_Axis_Software_P_Now": ([ctypes.c_uint16, ctypes.POINTER(ctypes.c_int32)], ctypes.c_int32),
            "MT_M_Get_Axis_Software_P_Now": ([ctypes.c_int32, ctypes.c_uint16, ctypes.POINTER(ctypes.c_int32)], ctypes.c_int32),
            "MT_Get_Axis_Status2": ([
                ctypes.c_uint16,
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
            ], ctypes.c_int32),
            "MT_M_Get_Axis_Status2": ([
                ctypes.c_int32,
                ctypes.c_uint16,
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
            ], ctypes.c_int32),
            "MT_Get_Axis_Status3": ([
                ctypes.c_uint16,
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
            ], ctypes.c_int32),
            "MT_M_Get_Axis_Status3": ([
                ctypes.c_int32,
                ctypes.c_uint16,
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
                ctypes.POINTER(ctypes.c_int32),
            ], ctypes.c_int32),
            "MT_Get_Axis_Status_Run": ([ctypes.c_uint16, ctypes.POINTER(ctypes.c_int32)], ctypes.c_int32),
            "MT_M_Get_Axis_Status_Run": ([ctypes.c_int32, ctypes.c_uint16, ctypes.POINTER(ctypes.c_int32)], ctypes.c_int32),
        }
        for name, (argtypes, restype) in signatures.items():
            if hasattr(self.dll, name):
                func = getattr(self.dll, name)
                func.argtypes = argtypes
                func.restype = restype

    def _call(self, name, *args, allow_missing=False, ignore_error=False):
        session_lock = self._session.lock if self._session is not None else self._sessions_lock
        with session_lock:
            if self.dll is None:
                raise StageError("MT_API.dll is not loaded.")
            if not hasattr(self.dll, name):
                if allow_missing:
                    return None
                raise StageError(f"MT_API function not found: {name}")
            result = getattr(self.dll, name)(*args)
            if not ignore_error and result != R_OK:
                raise StageError(f"{name} failed, return code: {result}")
            return result

    def _device_call(self, name, *args, allow_missing=False, ignore_error=False):
        if self._multi_usb:
            multi_name = self._multi_function_name(name)
            if self.dll is not None and hasattr(self.dll, multi_name):
                return self._call(
                    multi_name,
                    ctypes.c_int32(int(self.settings.usb_device_index)),
                    *args,
                    allow_missing=allow_missing,
                    ignore_error=ignore_error,
                )
            if allow_missing:
                return None
            raise StageError(
                f"MT_API function not found for USB device scoped call: {multi_name}. "
                "Refusing to fall back to a non-device-scoped call because multiple USB controllers may share axis numbers."
            )
        return self._call(name, *args, allow_missing=allow_missing, ignore_error=ignore_error)

    @staticmethod
    def _multi_function_name(name: str) -> str:
        if name.startswith("MT_M_"):
            return name
        if name.startswith("MT_"):
            return "MT_M_" + name[3:]
        return name

    def _get_axis_software_position(self, axis: int) -> int:
        value = ctypes.c_int32(0)
        self._device_call("MT_Get_Axis_Software_P_Now", ctypes.c_uint16(axis), ctypes.byref(value))
        return int(value.value)

    def _get_axis_raw_position(self, axis: int) -> int:
        value = ctypes.c_int32(0)
        self._device_call("MT_Get_Axis_P_Now", ctypes.c_uint16(axis), ctypes.byref(value))
        return int(value.value)

    def _axis_is_running(self, axis: int) -> bool:
        value = ctypes.c_int32(0)
        self._device_call("MT_Get_Axis_Status_Run", ctypes.c_uint16(axis), ctypes.byref(value))
        return value.value != 0

    def _get_axis_status(self, axis: int) -> dict:
        names = ("run", "dir", "neg", "pos", "zero", "mode", "soft_neg", "soft_pos")
        values = {name: ctypes.c_int32(0) for name in names}
        if self._has_device_function("MT_Get_Axis_Status3"):
            self._device_call(
                "MT_Get_Axis_Status3",
                ctypes.c_uint16(axis),
                ctypes.byref(values["run"]),
                ctypes.byref(values["dir"]),
                ctypes.byref(values["neg"]),
                ctypes.byref(values["pos"]),
                ctypes.byref(values["zero"]),
                ctypes.byref(values["mode"]),
                ctypes.byref(values["soft_neg"]),
                ctypes.byref(values["soft_pos"]),
            )
            return {name: int(value.value) for name, value in values.items()}
        if self._has_device_function("MT_Get_Axis_Status2"):
            self._device_call(
                "MT_Get_Axis_Status2",
                ctypes.c_uint16(axis),
                ctypes.byref(values["run"]),
                ctypes.byref(values["dir"]),
                ctypes.byref(values["neg"]),
                ctypes.byref(values["pos"]),
                ctypes.byref(values["zero"]),
                ctypes.byref(values["mode"]),
            )
        else:
            self._device_call("MT_Get_Axis_Status_Run", ctypes.c_uint16(axis), ctypes.byref(values["run"]))
        return {name: int(value.value) for name, value in values.items()}

    def _has_device_function(self, name: str) -> bool:
        if self.dll is None:
            return False
        if self._multi_usb:
            return hasattr(self.dll, self._multi_function_name(name))
        return hasattr(self.dll, name)

    def _wait_for_axes(self, axes, stop_event=None, progress_callback=None, start_raw_positions=None):
        deadline = time.monotonic() + self.settings.move_timeout_s
        start_time = time.monotonic()
        startup_grace_s = 0.15
        running_seen = False
        position_changed = False
        axes = tuple(int(axis) for axis in axes)
        while True:
            if stop_event is not None and stop_event.is_set():
                self.stop()
                raise StageError("Scan stopped by user.")
            if progress_callback is not None:
                for axis in axes:
                    progress_callback(self._get_axis_raw_position(axis))
            if start_raw_positions is not None:
                for axis in axes:
                    if self._get_axis_raw_position(axis) != int(start_raw_positions.get(axis, 0)):
                        position_changed = True
                        break
            running = [self._axis_is_running(axis) for axis in axes]
            if any(running):
                running_seen = True
            if not any(running):
                if running_seen or position_changed or time.monotonic() - start_time >= startup_grace_s:
                    if progress_callback is not None:
                        for axis in axes:
                            progress_callback(self._get_axis_raw_position(axis))
                    return running_seen
            if time.monotonic() > deadline:
                self.stop()
                raise StageError("Move timeout. Axes were halted.")
            time.sleep(0.02)

    def _require_connected(self):
        if not self.connected:
            raise StageError("Stage is not connected.")

    @staticmethod
    def _um_to_steps(value_um: float, steps_per_um: float) -> int:
        return int(round(float(value_um) * float(steps_per_um)))

    @staticmethod
    def _parse_ip(value: str):
        parts = value.split(".")
        if len(parts) != 4:
            raise StageError(f"Invalid IP address: {value}")
        numbers = [int(part) for part in parts]
        if any(number < 0 or number > 255 for number in numbers):
            raise StageError(f"Invalid IP address: {value}")
        return numbers
