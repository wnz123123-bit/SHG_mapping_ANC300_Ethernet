"""Safety-conscious open-loop X/Y scan stage built on an ANC300 controller."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from anc300_controller import (
    ANC300_MAX_OFFSET_V,
    ANC300_MIN_OFFSET_V,
    ANC300CommandError,
    ANC300ConnectionError,
    ANC300Controller,
    ANC300Error,
    ANC300ProtocolError,
)
from stage_controller import BaseStage, StageError


ANC300_MAX_RAMP_STEP_V = 1.0
ANC300_OUTPUT_VERIFY_TOLERANCE_V = 0.05


@dataclass
class ANC300StageSettings:
    host: str
    port: int = 7230
    password: str = "123456"
    timeout_s: float = 3.0
    x_axis: int = 1
    y_axis: int = 2
    voltage_min_v: float = 0.0
    voltage_max_v: float = 150.0
    x_um_per_v: float = 0.2
    y_um_per_v: float = 0.2
    x_direction: int = 1
    y_direction: int = 1
    calibration_source: str = "nominal_4K"
    max_ramp_step_v: float = 1.0
    pre_read_settle_s: float = 0.2
    hardware_profile_confirmed: bool = False


class ANC300ScanStage(BaseStage):
    """Open-loop two-axis stage that never changes outputs during connect."""

    _OUTPUT_VERIFY_TOLERANCE_V = ANC300_OUTPUT_VERIFY_TOLERANCE_V

    def __init__(self, settings: ANC300StageSettings, controller=None):
        self.settings = settings
        self.controller = controller or ANC300Controller(
            settings.host, settings.port, settings.password, settings.timeout_s
        )
        self.connected = False
        self.outputs_enabled = False
        self.origin_set = False
        self._origin_voltages = (None, None)
        self._last_confirmed_voltages = (None, None)
        self._modes = {settings.x_axis: None, settings.y_axis: None}
        self._device_identity = {}

    @property
    def axes(self):
        return self.settings.x_axis, self.settings.y_axis

    def connect(self):
        self._validate_settings()
        if self.connected:
            return self.get_status()
        try:
            snapshot = self.controller.connect(axes=self.axes)
            axes = snapshot.get("axes", {})
            values = []
            modes = {}
            for axis in self.axes:
                item = axes.get(axis)
                if not isinstance(item, dict):
                    raise StageError("ANC300 snapshot is missing an axis")
                mode, offset, filter_value = item.get("mode"), item.get("offset"), item.get("filter")
                if not isinstance(mode, str) or not mode.strip() or not isinstance(filter_value, str) or not filter_value.strip():
                    raise StageError("ANC300 snapshot lacks readable mode or filter")
                if not self._finite(offset):
                    raise StageError("ANC300 snapshot lacks a finite offset")
                values.append(float(offset))
                modes[axis] = mode.strip().lower()
            self.connected = True
            self.outputs_enabled = False
            self.origin_set = False
            self._origin_voltages = (None, None)
            self._last_confirmed_voltages = tuple(values)
            self._modes = modes
            self._device_identity = {
                "version": snapshot.get("version"),
                "controller_serial": snapshot.get("controller_serial"),
                "axis_serials": {axis: axes[axis].get("serial") for axis in self.axes},
                "filters": {axis: axes[axis].get("filter") for axis in self.axes},
            }
            return self.get_status()
        except (ANC300Error, KeyError, TypeError, ValueError, StageError) as exc:
            try:
                self.controller.disconnect()
            except Exception:
                pass
            self._clear_connection_state()
            raise StageError("Could not establish a valid ANC300 stage session: %s" % exc) from exc

    def disconnect(self):
        try:
            self.controller.disconnect()
        finally:
            self._clear_connection_state()

    def configure_motion(self):
        self._require_connected()

    def set_origin(self):
        self._require_connected()
        if not self.outputs_enabled:
            raise StageError("Outputs must be successfully enabled before setting the session origin.")
        values = self._read_offsets_in_bounds()
        self._origin_voltages = values
        self._last_confirmed_voltages = values
        self.origin_set = True

    def confirm_hardware_profile(self, confirmed=True):
        self.settings.hardware_profile_confirmed = bool(confirmed)
        return self.settings.hardware_profile_confirmed

    def enable_outputs(self):
        self._require_connected()
        if not self.settings.hardware_profile_confirmed:
            raise StageError("Hardware profile must be explicitly confirmed before enabling outputs.")
        self.outputs_enabled = False
        transitioned_from_ground = False
        try:
            for axis in self.axes:
                mode = self.controller.get_mode(axis).strip().lower()
                if mode == "gnd":
                    transitioned_from_ground = True
                    self.controller.set_offset(axis, 0.0)
                    self.controller.set_ac_input(axis, False)
                    self.controller.set_dc_input(axis, False)
                    self._require_external_inputs_off(axis)
                    self.controller.set_mode(axis, "off")
                elif mode not in {"off", "offs"}:
                    raise StageError("Axis %d is in unsupported mode %r" % (axis, mode))
                else:
                    self._require_external_inputs_off(axis)
                # Read after the required mode transition, or to verify the retained mode.
                verified_mode = self.controller.get_mode(axis).strip().lower()
                verified_voltage = self._read_voltage(axis)
                measured_output = self._read_output(axis)
                if verified_mode not in {"off", "offs"}:
                    raise StageError("Axis %d did not enter offset mode" % axis)
                if abs(measured_output - verified_voltage) > self._OUTPUT_VERIFY_TOLERANCE_V:
                    raise StageError("Axis %d measured output does not match its saved offset." % axis)
                if mode == "gnd" and abs(verified_voltage) > self._OUTPUT_VERIFY_TOLERANCE_V:
                    raise StageError("Axis %d offset did not verify at 0 V after enabling." % axis)
                if mode == "gnd" and abs(measured_output) > self._OUTPUT_VERIFY_TOLERANCE_V:
                    raise StageError("Axis %d measured output did not verify at 0 V after enabling." % axis)
                self._modes[axis] = verified_mode
                self._set_last_voltage(axis, verified_voltage)
            self.outputs_enabled = True
            if transitioned_from_ground:
                self.origin_set = False
                self._origin_voltages = (None, None)
        except (ANC300Error, AttributeError, StageError, ValueError) as exc:
            self.outputs_enabled = False
            raise StageError("Could not safely enable ANC300 outputs: %s" % exc) from exc

    def ground_outputs(self, stop_event=None):
        self._require_connected()
        self.outputs_enabled = False
        try:
            for axis in self.axes:
                mode = self.controller.get_mode(axis).strip().lower()
                if mode == "gnd":
                    self._modes[axis] = mode
                    continue
                if mode not in {"off", "offs"}:
                    raise StageError("Axis %d is in unsupported mode %r" % (axis, mode))
                self._require_external_inputs_off(axis)
                self._ramp_axis(axis, 0.0, stop_event)
                readback = self._read_voltage(axis)
                measured_output = self._read_output(axis)
                self._set_last_voltage(axis, readback)
                if abs(readback) > self._OUTPUT_VERIFY_TOLERANCE_V:
                    raise StageError("Axis %d offset did not reach 0 V before grounding." % axis)
                if abs(measured_output) > self._OUTPUT_VERIFY_TOLERANCE_V:
                    raise StageError("Axis %d measured output did not reach 0 V before grounding." % axis)
            for axis in self.axes:
                if stop_event is not None and stop_event.is_set():
                    raise StageError("Grounding stopped by user.")
                self.controller.set_mode(axis, "gnd")
            for axis in self.axes:
                mode = self.controller.get_mode(axis).strip().lower()
                if mode != "gnd":
                    raise StageError("Axis %d did not enter ground mode" % axis)
                self._modes[axis] = mode
        except (ANC300Error, AttributeError, StageError, ValueError) as exc:
            self.outputs_enabled = False
            raise StageError("Could not ground ANC300 outputs: %s" % exc) from exc

    def target_voltages(self, x_um, y_um):
        self._require_origin()
        x, y = self._numeric_pair(x_um, y_um, "position")
        ox, oy = self._origin_voltages
        return (
            ox + self.settings.x_direction * x / self.settings.x_um_per_v,
            oy + self.settings.y_direction * y / self.settings.y_um_per_v,
        )

    def preflight_points(self, points: Iterable[tuple[float, float]]):
        targets = []
        try:
            iterator = iter(points)
        except TypeError as exc:
            raise StageError("Points must be an iterable of X/Y pairs.") from exc
        for point in iterator:
            try:
                x_um, y_um = point
            except (TypeError, ValueError) as exc:
                raise StageError("Each point must contain exactly X and Y positions.") from exc
            target = self.target_voltages(x_um, y_um)
            if not all(self._in_bounds(value) for value in target):
                raise StageError("Requested voltage is outside configured ANC300 bounds.")
            targets.append(target)
        return targets

    def move_to_um(self, x_um, y_um, stop_event=None):
        self._require_ready_for_motion()
        x_target, y_target = self.preflight_points([(x_um, y_um)])[0]
        self._ramp_axis(self.settings.x_axis, x_target, stop_event)
        self._ramp_axis(self.settings.y_axis, y_target, stop_event)

    def move_origin(self, stop_event=None):
        self._require_ready_for_motion()
        self._ramp_axis(self.settings.x_axis, self._origin_voltages[0], stop_event)
        self._ramp_axis(self.settings.y_axis, self._origin_voltages[1], stop_event)

    def stop(self):
        self._require_connected()
        errors = []
        for axis in self.axes:
            try:
                self.controller.stop(axis)
            except (ANC300Error, ValueError, AttributeError) as exc:
                errors.append("axis %d: %s" % (axis, exc))
        self.outputs_enabled = False
        if errors:
            raise StageError("ANC300 stop failed for " + "; ".join(errors))

    def get_position_um(self):
        self._require_origin()
        x, y = self._last_confirmed_voltages
        ox, oy = self._origin_voltages
        return (
            self.settings.x_direction * (x - ox) * self.settings.x_um_per_v,
            self.settings.y_direction * (y - oy) * self.settings.y_um_per_v,
        )

    def get_voltage_metadata(self):
        return {
            "x_voltage_v": self._last_confirmed_voltages[0],
            "y_voltage_v": self._last_confirmed_voltages[1],
            "x_origin_voltage_v": self._origin_voltages[0],
            "y_origin_voltage_v": self._origin_voltages[1],
            "x_um_per_v": self.settings.x_um_per_v,
            "y_um_per_v": self.settings.y_um_per_v,
            "x_direction": self.settings.x_direction,
            "y_direction": self.settings.y_direction,
            "calibration_source": self.settings.calibration_source,
            "position_estimated": True,
        }

    def get_status(self):
        metadata = self.get_voltage_metadata()
        position = None
        if self.connected and self.origin_set:
            position = self.get_position_um()
        return {
            "connected": self.connected,
            "outputs_enabled": self.outputs_enabled,
            "origin_set": self.origin_set,
            "hardware_profile_confirmed": self.settings.hardware_profile_confirmed,
            "axes": {"x": self.settings.x_axis, "y": self.settings.y_axis},
            "modes": {"x": self._modes.get(self.settings.x_axis), "y": self._modes.get(self.settings.y_axis)},
            "device_identity": dict(self._device_identity),
            "estimated_position_um": position,
            **metadata,
        }

    def calibrate_axis(self, axis_name, delta_voltage_v, measured_displacement_um):
        if not isinstance(axis_name, str) or axis_name.upper() not in {"X", "Y"}:
            raise StageError("axis_name must be 'X' or 'Y'.")
        if not self._finite(delta_voltage_v) or not self._finite(measured_displacement_um):
            raise StageError("Calibration values must be finite.")
        delta_voltage_v, measured_displacement_um = float(delta_voltage_v), float(measured_displacement_um)
        if delta_voltage_v == 0 or measured_displacement_um == 0:
            raise StageError("Calibration voltage and displacement must be non-zero.")
        ratio = measured_displacement_um / delta_voltage_v
        scale, direction = abs(ratio), 1 if ratio > 0 else -1
        if axis_name.upper() == "X":
            self.settings.x_um_per_v, self.settings.x_direction = scale, direction
        else:
            self.settings.y_um_per_v, self.settings.y_direction = scale, direction
        self.settings.calibration_source = "custom"
        return {"axis": axis_name.upper(), "um_per_v": scale, "direction": direction,
                "calibration_source": self.settings.calibration_source}

    def _ramp_axis(self, axis, target, stop_event):
        if not self._in_bounds(target):
            raise StageError("Requested voltage is outside configured ANC300 bounds.")
        start = self._last_voltage(axis)
        if start is None:
            raise StageError("No confirmed voltage is available for axis %d." % axis)
        direction = 1.0 if target >= start else -1.0
        value = start
        while not math.isclose(value, target, rel_tol=0.0, abs_tol=1e-12):
            if stop_event is not None and stop_event.is_set():
                raise StageError("Move stopped by user.")
            value = target if abs(target - value) <= self.settings.max_ramp_step_v else value + direction * self.settings.max_ramp_step_v
            try:
                self.controller.set_offset(axis, value)
            except (ANC300Error, ValueError, AttributeError) as exc:
                raise StageError("ANC300 offset command failed on axis %d: %s" % (axis, exc)) from exc
            self._set_last_voltage(axis, value)

    def _read_offsets_in_bounds(self):
        values = tuple(self._read_voltage(axis) for axis in self.axes)
        return values

    def _read_voltage(self, axis):
        try:
            value = self.controller.get_offset(axis)
        except (ANC300Error, ValueError, AttributeError) as exc:
            raise StageError("Could not read ANC300 axis %d offset: %s" % (axis, exc)) from exc
        if not self._in_bounds(value):
            raise StageError("ANC300 axis %d offset is outside configured bounds." % axis)
        return float(value)

    def _read_output(self, axis):
        try:
            value = self.controller.get_output(axis)
        except (ANC300Error, ValueError, AttributeError) as exc:
            raise StageError("Could not read ANC300 axis %d measured output: %s" % (axis, exc)) from exc
        if not self._finite(value) or not ANC300_MIN_OFFSET_V <= float(value) <= ANC300_MAX_OFFSET_V:
            raise StageError("ANC300 axis %d measured output is outside physical bounds." % axis)
        return float(value)

    def _require_external_inputs_off(self, axis):
        try:
            ac_enabled = self.controller.get_ac_input(axis)
            dc_enabled = self.controller.get_dc_input(axis)
        except (ANC300Error, ValueError, AttributeError) as exc:
            raise StageError("Could not verify ANC300 axis %d external inputs: %s" % (axis, exc)) from exc
        if ac_enabled or dc_enabled:
            raise StageError("Axis %d AC-IN and DC-IN must both be off." % axis)

    def _set_last_voltage(self, axis, value):
        values = list(self._last_confirmed_voltages)
        values[self.axes.index(axis)] = float(value)
        self._last_confirmed_voltages = tuple(values)

    def _last_voltage(self, axis):
        return self._last_confirmed_voltages[self.axes.index(axis)]

    def _require_connected(self):
        if not self.connected:
            raise StageError("Stage is not connected.")

    def _require_origin(self):
        self._require_connected()
        if not self.origin_set:
            raise StageError("Set the session origin before using positions.")

    def _require_ready_for_motion(self):
        self._require_origin()
        if not self.settings.hardware_profile_confirmed:
            raise StageError("Hardware profile must be explicitly confirmed before movement.")
        if not self.outputs_enabled:
            raise StageError("Outputs must be successfully enabled before movement.")

    def _clear_connection_state(self):
        self.connected = False
        self.outputs_enabled = False
        self.origin_set = False
        self._origin_voltages = (None, None)
        self._last_confirmed_voltages = (None, None)
        self._modes = {self.settings.x_axis: None, self.settings.y_axis: None}
        self._device_identity = {}

    def _validate_settings(self, require_host=True):
        if require_host and (not isinstance(self.settings.host, str) or not self.settings.host.strip()):
            raise ValueError("host must be a non-empty string")
        if (isinstance(self.settings.x_axis, bool) or isinstance(self.settings.y_axis, bool)
                or not isinstance(self.settings.x_axis, int) or not isinstance(self.settings.y_axis, int)
                or self.settings.x_axis == self.settings.y_axis
                or self.settings.x_axis == 3 or self.settings.y_axis == 3
                or not 1 <= self.settings.x_axis <= 7 or not 1 <= self.settings.y_axis <= 7):
            raise ValueError("X/Y axes must be distinct integers from 1 through 7")
        numeric = (self.settings.timeout_s, self.settings.voltage_min_v, self.settings.voltage_max_v,
                   self.settings.x_um_per_v, self.settings.y_um_per_v, self.settings.max_ramp_step_v,
                   self.settings.pre_read_settle_s)
        if not all(self._finite(value) for value in numeric):
            raise ValueError("ANC300 settings must be finite")
        if (self.settings.timeout_s <= 0 or self.settings.x_um_per_v <= 0 or self.settings.y_um_per_v <= 0
                or self.settings.max_ramp_step_v <= 0
                or self.settings.max_ramp_step_v > ANC300_MAX_RAMP_STEP_V
                or self.settings.pre_read_settle_s < 0):
            raise ValueError("ANC300 timeout, calibration, and ramp step must be positive")
        if self.settings.x_direction not in (-1, 1) or self.settings.y_direction not in (-1, 1):
            raise ValueError("ANC300 directions must be exactly +1 or -1")
        if not self.settings.voltage_min_v < self.settings.voltage_max_v:
            raise ValueError("ANC300 voltage_min_v must be less than voltage_max_v")
        if (self.settings.voltage_min_v < ANC300_MIN_OFFSET_V
                or self.settings.voltage_max_v > ANC300_MAX_OFFSET_V):
            raise ValueError("ANC300 voltage bounds must stay within 0 through 150 V")

    @staticmethod
    def _finite(value):
        try:
            return not isinstance(value, bool) and math.isfinite(float(value))
        except (TypeError, ValueError):
            return False

    def _in_bounds(self, value):
        return self._finite(value) and self.settings.voltage_min_v <= float(value) <= self.settings.voltage_max_v

    def _numeric_pair(self, x, y, name):
        if not self._finite(x) or not self._finite(y):
            raise StageError("%s values must be finite." % name)
        return float(x), float(y)


class SimulatedANC300Stage(BaseStage):
    """Network-free stage model retaining the same open-loop coordinates."""

    def __init__(self, settings: ANC300StageSettings):
        self.settings = settings
        self.connected = False
        self.outputs_enabled = False
        self.origin_set = False
        self._origin_voltages = (None, None)
        self._last_confirmed_voltages = (None, None)
        self._modes = {settings.x_axis: "gnd", settings.y_axis: "gnd"}
        self._device_identity = {"simulated": True}

    def connect(self):
        ANC300ScanStage._validate_settings(self, require_host=False)
        self.connected, self.outputs_enabled, self.origin_set = True, False, False
        self._origin_voltages = (0.0, 0.0)
        self._last_confirmed_voltages = (0.0, 0.0)
        return self.get_status()

    def disconnect(self):
        self.connected, self.outputs_enabled, self.origin_set = False, False, False
        self._origin_voltages = (None, None)
        self._last_confirmed_voltages = (None, None)

    def configure_motion(self):
        self._require_connected()

    def set_origin(self):
        self._require_connected()
        if not self.outputs_enabled:
            raise StageError("Outputs must be successfully enabled before setting the session origin.")
        self._origin_voltages = self._last_confirmed_voltages
        self.origin_set = True

    def confirm_hardware_profile(self, confirmed=True):
        self.settings.hardware_profile_confirmed = bool(confirmed)
        return self.settings.hardware_profile_confirmed

    def enable_outputs(self):
        self._require_connected()
        transitioned_from_ground = any(mode == "gnd" for mode in self._modes.values())
        self.outputs_enabled = True
        self._modes = {axis: "off" for axis in self._modes}
        if transitioned_from_ground:
            self.origin_set = False
            self._origin_voltages = (None, None)

    def ground_outputs(self, stop_event=None):
        self._require_connected()
        if stop_event is not None and stop_event.is_set():
            raise StageError("Grounding stopped by user.")
        self._last_confirmed_voltages = (0.0, 0.0)
        self._modes = {axis: "gnd" for axis in self._modes}
        self.outputs_enabled = False

    def target_voltages(self, x_um, y_um):
        return ANC300ScanStage.target_voltages(self, x_um, y_um)

    def preflight_points(self, points):
        return ANC300ScanStage.preflight_points(self, points)

    def move_to_um(self, x_um, y_um, stop_event=None):
        self._require_ready_for_motion()
        x_target, y_target = self.preflight_points([(x_um, y_um)])[0]
        if stop_event is not None and stop_event.is_set():
            raise StageError("Move stopped by user.")
        self._last_confirmed_voltages = (x_target, y_target)

    def move_origin(self, stop_event=None):
        self._require_ready_for_motion()
        if stop_event is not None and stop_event.is_set():
            raise StageError("Move stopped by user.")
        self._last_confirmed_voltages = self._origin_voltages

    def stop(self):
        self._require_connected()
        self.outputs_enabled = False

    def get_position_um(self):
        return ANC300ScanStage.get_position_um(self)

    def get_voltage_metadata(self):
        return ANC300ScanStage.get_voltage_metadata(self)

    def get_status(self):
        status = ANC300ScanStage.get_status(self)
        status["device_identity"] = {"simulated": True}
        return status

    _require_connected = ANC300ScanStage._require_connected
    _require_origin = ANC300ScanStage._require_origin

    def _require_ready_for_motion(self):
        self._require_origin()
        if not self.outputs_enabled:
            raise StageError("Outputs must be successfully enabled before movement.")
    _finite = staticmethod(ANC300ScanStage._finite)
    _in_bounds = ANC300ScanStage._in_bounds
    _numeric_pair = ANC300ScanStage._numeric_pair
    calibrate_axis = ANC300ScanStage.calibrate_axis
