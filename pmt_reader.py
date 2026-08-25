from __future__ import annotations

import ctypes
import random
import time
from dataclasses import dataclass
import math
from pathlib import Path


C8855_SOFTWARE_TRIGGER = 0
C8855_ERROR_TRANSFER = 255
SET_FALL_EDGE = 0
DEFAULT_PMT_DLL_PATH = str(Path(__file__).resolve().parent / "vendor_dlls" / "C8855-01api.dll")
VALID_TRANSFER_MODES = frozenset({0, 1, 2})
VALID_TRIGGER_MODES = frozenset({0, 1})
VALID_TRIGGER_EDGES = frozenset({0, 1})

GATE_TIME_MAP = {
    0.05: 0x02,
    0.1: 0x03,
    0.2: 0x04,
    0.5: 0x05,
    1.0: 0x06,
    2.0: 0x07,
    5.0: 0x08,
    10.0: 0x09,
    20.0: 0x0A,
    50.0: 0x0B,
    100.0: 0x0C,
    200.0: 0x0D,
    500.0: 0x0E,
    1000.0: 0x0F,
    2000.0: 0x10,
    5000.0: 0x11,
    10000.0: 0x12,
}
GATE_TIME_OPTIONS_MS = tuple(GATE_TIME_MAP.keys())


class PMTError(RuntimeError):
    pass


@dataclass(frozen=True)
class PMTSettings:
    enabled: bool = False
    dll_path: str = DEFAULT_PMT_DLL_PATH
    gate_time_ms: float = 200.0
    samples_to_average: int = 5
    sample_extra_wait_s: float = 0.05
    transfer_mode: int = 2
    trigger_mode: int = C8855_SOFTWARE_TRIGGER
    trigger_edge: int = SET_FALL_EDGE
    simulate_value: float = 1.0
    simulate_noise: float = 0.0


def gate_time_code(gate_time_ms: float) -> int:
    try:
        return GATE_TIME_MAP[float(gate_time_ms)]
    except (TypeError, ValueError, KeyError):
        allowed = ", ".join(format_gate_time(value) for value in GATE_TIME_OPTIONS_MS)
        raise PMTError(f"PMT gate time must be one of: {allowed} ms")


def format_gate_time(value: float) -> str:
    value = float(value)
    if value.is_integer():
        return str(int(value))
    return f"{value:g}"


def interruptible_sleep(seconds: float, stop_event=None):
    deadline = time.monotonic() + max(0.0, float(seconds))
    while True:
        if stop_event is not None and stop_event.is_set():
            raise PMTError("Scan stopped by user.")
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(0.05, remaining))


def validate_pmt_settings(settings: PMTSettings, *, simulate: bool, require_enabled: bool = False) -> PMTSettings:
    """Validate a plain PMT snapshot before a worker, file, or hardware action."""
    if not isinstance(settings, PMTSettings):
        raise PMTError("PMT settings snapshot is invalid.")
    if require_enabled and not settings.enabled:
        raise PMTError("Real mapping requires the PMT hardware backend to be enabled.")
    gate_time_code(settings.gate_time_ms)
    discrete_values = (
        settings.samples_to_average,
        settings.transfer_mode,
        settings.trigger_mode,
        settings.trigger_edge,
    )
    if any(isinstance(value, bool) or not isinstance(value, int) for value in discrete_values):
        raise PMTError("PMT sample count and acquisition modes must be integers.")
    if settings.samples_to_average < 1:
        raise PMTError("PMT average sample count must be at least 1.")
    finite_values = (
        settings.gate_time_ms,
        settings.sample_extra_wait_s,
        settings.simulate_value,
        settings.simulate_noise,
    )
    try:
        finite = all(not isinstance(value, bool) and math.isfinite(float(value)) for value in finite_values)
    except (TypeError, ValueError):
        finite = False
    if not finite:
        raise PMTError("PMT acquisition values must be finite.")
    if float(settings.sample_extra_wait_s) < 0:
        raise PMTError("PMT extra sample wait cannot be negative.")
    if float(settings.simulate_noise) < 0:
        raise PMTError("PMT simulation noise cannot be negative.")
    if settings.transfer_mode not in VALID_TRANSFER_MODES:
        raise PMTError("PMT transfer mode is invalid.")
    if settings.trigger_mode not in VALID_TRIGGER_MODES:
        raise PMTError("PMT trigger mode is invalid.")
    if settings.trigger_edge not in VALID_TRIGGER_EDGES:
        raise PMTError("PMT trigger edge is invalid.")
    if settings.enabled and not simulate and not Path(settings.dll_path).is_file():
        raise PMTError(f"C8855-01 API DLL not found: {Path(settings.dll_path)}")
    return settings


class PMTCounter:
    def __init__(self, settings: PMTSettings, simulate: bool = False):
        self.settings = settings
        self.simulate = simulate
        self.dll = None
        self.handle = None
        self._opened = False
        self._counting = False

    def __enter__(self):
        if not self.simulate:
            try:
                self.open()
                self.reset()
                self.setup()
            except Exception:
                try:
                    self.close()
                except Exception:
                    pass
                raise
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def open(self):
        dll_path = Path(self.settings.dll_path)
        if not dll_path.exists():
            raise PMTError(f"C8855-01 API DLL not found: {dll_path}")

        self.dll = ctypes.CDLL(str(dll_path))
        self._configure_signatures()
        self.handle = self.dll.C8855Open()
        if not self.handle:
            raise PMTError("C8855Open failed. Check PMT USB connection and close the vendor software.")
        self._opened = True

    def close(self):
        if self.simulate:
            return
        if self.dll is None or not self._opened:
            return
        try:
            try:
                self.stop_counting()
            except Exception:
                pass
            if not self.dll.C8855Close(self.handle):
                raise PMTError("C8855Close failed.")
        finally:
            self._opened = False
            self._counting = False
            self.handle = None

    def reset(self):
        if not self.dll.C8855Reset(self.handle):
            raise PMTError("C8855Reset failed.")

    def setup(self):
        if not self.dll.C8855Setup(
            self.handle,
            ctypes.c_ubyte(gate_time_code(self.settings.gate_time_ms)),
            ctypes.c_ubyte(int(self.settings.transfer_mode)),
            ctypes.c_long(1),
            ctypes.c_ubyte(int(self.settings.trigger_edge)),
        ):
            raise PMTError("C8855Setup failed.")

    def start_counting(self):
        if self._counting:
            return
        if not self.dll.C8855CountStart(self.handle, ctypes.c_ubyte(int(self.settings.trigger_mode))):
            raise PMTError("C8855CountStart failed.")
        self._counting = True

    def stop_counting(self):
        if not self._counting:
            return
        if not self.dll.C8855CountStop(self.handle):
            raise PMTError("C8855CountStop failed.")
        self._counting = False

    def read_average(self, stop_event=None):
        count = int(self.settings.samples_to_average)
        if count < 1:
            raise PMTError("PMT average sample count must be at least 1.")
        if not self.simulate:
            self.reset()
            self.setup()
            self.start_counting()
        try:
            samples = [self.read_one(stop_event=stop_event) for _ in range(count)]
        finally:
            if not self.simulate:
                self.stop_counting()
        return sum(samples) / len(samples), samples

    def read_one(self, stop_event=None):
        wait_s = float(self.settings.gate_time_ms) / 1000.0 + float(self.settings.sample_extra_wait_s)
        if self.simulate:
            interruptible_sleep(wait_s, stop_event=stop_event)
            noise = random.uniform(-float(self.settings.simulate_noise), float(self.settings.simulate_noise))
            return float(self.settings.simulate_value) + noise

        interruptible_sleep(wait_s, stop_event=stop_event)
        buffer_type = ctypes.c_ulong * 1
        data_buffer = buffer_type()
        result_returned = ctypes.c_ubyte()
        if not self.dll.C8855ReadData(self.handle, data_buffer, ctypes.byref(result_returned)):
            raise PMTError("C8855ReadData failed.")
        if result_returned.value == C8855_ERROR_TRANSFER:
            raise PMTError("C8855ReadData transfer error.")
        return float(data_buffer[0])

    def _configure_signatures(self):
        handle_type = ctypes.c_void_p
        self.dll.C8855Open.argtypes = []
        self.dll.C8855Open.restype = handle_type
        self.dll.C8855Close.argtypes = [handle_type]
        self.dll.C8855Close.restype = ctypes.c_bool
        self.dll.C8855Reset.argtypes = [handle_type]
        self.dll.C8855Reset.restype = ctypes.c_bool
        self.dll.C8855Setup.argtypes = [handle_type, ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_long, ctypes.c_ubyte]
        self.dll.C8855Setup.restype = ctypes.c_bool
        self.dll.C8855CountStart.argtypes = [handle_type, ctypes.c_ubyte]
        self.dll.C8855CountStart.restype = ctypes.c_bool
        self.dll.C8855CountStop.argtypes = [handle_type]
        self.dll.C8855CountStop.restype = ctypes.c_bool
        self.dll.C8855ReadData.argtypes = [
            handle_type,
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_ubyte),
        ]
        self.dll.C8855ReadData.restype = ctypes.c_bool
