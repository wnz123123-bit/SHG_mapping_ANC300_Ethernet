"""Behavioral tests for the safe two-axis ANC300 scan stage."""

from __future__ import annotations

import math
import unittest

from anc300_controller import ANC300CommandError, ANC300ConnectionError, ANC300ProtocolError
from anc300_stage import ANC300ScanStage, ANC300StageSettings, SimulatedANC300Stage
from stage_controller import StageError


class FakeANC300Controller:
    """In-memory complete public ANC300Controller contract for stage tests."""

    def __init__(self, modes=None, offsets=None):
        self.connected = False
        self.modes = {1: "gnd", 2: "gnd"}
        self.offsets = {1: 10.0, 2: 20.0}
        self.filters = {1: "16 Hz", 2: "16 Hz"}
        self.readback_offsets = {}
        self.readback_outputs = {}
        self.ac_inputs = {1: False, 2: False}
        self.dc_inputs = {1: False, 2: False}
        self.history = []
        self.fail_connect = False
        self.fail_on_set = None
        self.fail_on_mode = None
        self.fail_on_stop = None
        if modes:
            self.modes.update(modes)
        if offsets:
            self.offsets.update(offsets)

    def connect(self, axes=(1, 2)):
        self.history.append(("connect", tuple(axes)))
        if self.fail_connect:
            raise ANC300ConnectionError("simulated connection failure")
        self.connected = True
        return self.get_device_snapshot(axes) | {"connected": True}

    def disconnect(self):
        self.history.append(("disconnect",))
        self.connected = False

    def query(self, command):
        self.history.append(("query", command))
        return []

    def get_version(self):
        return "ANC300 version test"

    def get_controller_serial(self):
        return "CTRL-TEST"

    def get_axis_serial(self, axis):
        return "AX-%s" % axis

    def get_mode(self, axis):
        self.history.append(("get_mode", axis))
        return self.modes[axis]

    def get_offset(self, axis):
        self.history.append(("get_offset", axis))
        return self.readback_offsets.get(axis, self.offsets[axis])

    def get_output(self, axis):
        self.history.append(("get_output", axis))
        default = 0.0 if self.modes[axis] == "gnd" else self.offsets[axis]
        return self.readback_outputs.get(axis, default)

    def get_ac_input(self, axis):
        self.history.append(("get_ac_input", axis))
        return self.ac_inputs[axis]

    def get_dc_input(self, axis):
        self.history.append(("get_dc_input", axis))
        return self.dc_inputs[axis]

    def get_filter(self, axis):
        return self.filters[axis]

    def get_axis_snapshot(self, axis):
        return {"serial": self.get_axis_serial(axis), "mode": self.get_mode(axis),
                "offset": self.get_offset(axis), "filter": self.get_filter(axis)}

    def get_device_snapshot(self, axes=(1, 2)):
        return {"version": self.get_version(), "controller_serial": self.get_controller_serial(),
                "axes": {axis: self.get_axis_snapshot(axis) for axis in axes}}

    def set_mode(self, axis, mode):
        self.history.append(("set_mode", axis, mode))
        if self.fail_on_mode == axis:
            raise ANC300CommandError("simulated mode failure")
        self.modes[axis] = mode
        return []

    def set_offset(self, axis, voltage):
        self.history.append(("set_offset", axis, float(voltage)))
        if self.fail_on_set == axis:
            raise ANC300CommandError("simulated offset failure")
        self.offsets[axis] = float(voltage)
        return []

    def set_ac_input(self, axis, enabled):
        self.history.append(("set_ac_input", axis, bool(enabled)))
        self.ac_inputs[axis] = bool(enabled)
        return []

    def set_dc_input(self, axis, enabled):
        self.history.append(("set_dc_input", axis, bool(enabled)))
        self.dc_inputs[axis] = bool(enabled)
        return []

    def stop(self, axis):
        self.history.append(("stop", axis))
        if self.fail_on_stop == axis:
            raise ANC300CommandError("simulated stop failure")
        return []


class SetAfterFirstCommand:
    def __init__(self):
        self.calls = 0

    def is_set(self):
        self.calls += 1
        return self.calls > 1


class ANC300ScanStageTests(unittest.TestCase):
    def stage(self, controller=None, **setting_overrides):
        settings = ANC300StageSettings(host="127.0.0.1", **setting_overrides)
        return ANC300ScanStage(settings, controller or FakeANC300Controller())

    def ready_stage(self, controller=None, **setting_overrides):
        stage = self.stage(controller, **setting_overrides)
        stage.connect()
        stage.confirm_hardware_profile()
        stage.enable_outputs()
        stage.set_origin()
        return stage

    def test_invalid_settings_are_rejected_before_a_connection(self):
        """Missing finite/safe settings validation must fail this test."""
        invalid = [
            {"x_axis": 1, "y_axis": 1}, {"x_axis": 3}, {"y_axis": 8},
            {"timeout_s": 0}, {"max_ramp_step_v": 0}, {"x_um_per_v": 0},
            {"max_ramp_step_v": 1.000001}, {"voltage_min_v": -0.000001},
            {"voltage_max_v": 150.000001},
            {"y_um_per_v": math.inf}, {"x_direction": 0}, {"y_direction": 2},
            {"voltage_min_v": 4, "voltage_max_v": 4}, {"voltage_min_v": math.nan},
        ]
        for values in invalid:
            with self.subTest(values=values):
                fake = FakeANC300Controller()
                with self.assertRaises((ValueError, StageError)):
                    self.stage(fake, **values).connect()
                self.assertFalse(fake.connected)
                self.assertNotIn(("connect", (1, 2)), fake.history)

    def test_axis_three_is_rejected_before_connect_even_when_fake_exposes_it(self):
        """Allowing protected axis 3 into a session must fail before any connection."""
        fake = FakeANC300Controller()
        fake.modes[3], fake.offsets[3], fake.filters[3] = "off", 1.0, "16 Hz"
        with self.assertRaises((ValueError, StageError)):
            self.stage(fake, x_axis=3, y_axis=2).connect()
        self.assertEqual(fake.history, [])

    def test_connect_rejects_blank_filter_and_disconnects(self):
        """Treating a blank axis filter as a readable snapshot must fail."""
        fake = FakeANC300Controller()
        fake.filters[1] = "  "
        with self.assertRaises(StageError):
            self.stage(fake).connect()
        self.assertEqual(fake.history[-1], ("disconnect",))
        self.assertFalse(fake.connected)

    def test_connect_is_read_only_and_failure_closes_controller(self):
        """A connect path that changes output or leaks failed transport must fail."""
        fake = FakeANC300Controller()
        stage = self.stage(fake)
        stage.connect()
        self.assertTrue(stage.connected)
        self.assertFalse(stage.outputs_enabled)
        self.assertFalse(stage.origin_set)
        self.assertFalse(any(item[0] in {"set_mode", "set_offset", "stop"} for item in fake.history))
        fake = FakeANC300Controller()
        fake.fail_connect = True
        with self.assertRaises(StageError):
            self.stage(fake).connect()
        self.assertIn(("disconnect",), fake.history)

    def test_reconnect_clears_prior_session_origin(self):
        """Retaining a stale origin across a reconnect must fail."""
        stage = self.stage()
        stage.connect(); stage.confirm_hardware_profile(); stage.enable_outputs(); stage.set_origin()
        stage.disconnect(); stage.connect()
        with self.assertRaises(StageError):
            stage.get_position_um()

    def test_origin_conversion_and_metadata_follow_direction(self):
        """Ignoring measured origin or direction sign must fail."""
        fake = FakeANC300Controller(modes={1: "off", 2: "off"}, offsets={1: 10, 2: 20})
        stage = self.ready_stage(fake, x_direction=-1, y_direction=1, x_um_per_v=0.5, y_um_per_v=0.25)
        self.assertEqual(stage.target_voltages(1.0, -0.5), (8.0, 18.0))
        stage.move_to_um(1.0, -0.5)
        self.assertEqual(stage.get_position_um(), (1.0, -0.5))
        metadata = stage.get_voltage_metadata()
        self.assertEqual(metadata["x_origin_voltage_v"], 10.0)
        self.assertEqual(metadata["y_voltage_v"], 18.0)
        self.assertTrue(metadata["position_estimated"])

    def test_calibration_updates_nominal_to_custom_without_motion(self):
        """A calibration that writes hardware or loses direction must fail."""
        stage = self.stage()
        result = stage.calibrate_axis("X", 2, -1)
        self.assertEqual(result["um_per_v"], 0.5)
        self.assertEqual(result["direction"], -1)
        self.assertEqual(result["calibration_source"], "custom")
        self.assertEqual(stage.settings.x_um_per_v, 0.5)
        with self.assertRaises(StageError):
            stage.calibrate_axis("z", 1, 1)
        with self.assertRaises(StageError):
            stage.calibrate_axis("Y", 0, 1)

    def test_preflight_accepts_inclusive_bounds_and_checks_whole_path_before_writes(self):
        """Writing early or rejecting a legal end point must fail."""
        fake = FakeANC300Controller(modes={1: "off", 2: "off"}, offsets={1: 10, 2: 10})
        stage = self.ready_stage(fake, voltage_min_v=0, voltage_max_v=10, x_um_per_v=1, y_um_per_v=1)
        self.assertEqual(stage.preflight_points([(0, -10)]), [(10.0, 0.0)])
        before = list(fake.history)
        with self.assertRaises(StageError):
            stage.preflight_points([(0, 0), (1, 0)])
        self.assertEqual(fake.history, before)

    def test_ramp_uses_exact_endpoint_and_no_increment_larger_than_limit(self):
        """Skipping endpoint or exceeding ramp limit must fail."""
        fake = FakeANC300Controller(modes={1: "off", 2: "off"}, offsets={1: 1, 2: 2})
        stage = self.ready_stage(fake, max_ramp_step_v=1, x_um_per_v=1, y_um_per_v=1, voltage_max_v=10)
        fake.history.clear()
        stage.move_to_um(2.5, 0)
        x_commands = [item[2] for item in fake.history if item[:2] == ("set_offset", 1)]
        self.assertEqual(x_commands, [2.0, 3.0, 3.5])
        self.assertTrue(all(abs(b - a) <= 1 for a, b in zip([1.0] + x_commands, x_commands)))
        self.assertEqual(stage.get_voltage_metadata()["x_voltage_v"], 3.5)

    def test_two_axis_row_flyback_completes_all_x_ramps_before_first_y_ramp(self):
        """Interleaving Y while X flies back must fail the controller command history contract."""
        fake = FakeANC300Controller(modes={1: "off", 2: "off"}, offsets={1: 3, 2: 4})
        stage = self.ready_stage(
            fake,
            max_ramp_step_v=1,
            x_um_per_v=1,
            y_um_per_v=1,
            voltage_max_v=20,
        )
        fake.history.clear()

        stage.move_to_um(3, 0)
        stage.move_to_um(0, 2)

        offset_history = [item for item in fake.history if item[0] == "set_offset"]
        self.assertEqual(
            offset_history,
            [
                ("set_offset", 1, 4.0),
                ("set_offset", 1, 5.0),
                ("set_offset", 1, 6.0),
                ("set_offset", 1, 5.0),
                ("set_offset", 1, 4.0),
                ("set_offset", 1, 3.0),
                ("set_offset", 2, 5.0),
                ("set_offset", 2, 6.0),
            ],
        )
        first_y = next(index for index, item in enumerate(offset_history) if item[1] == 2)
        self.assertTrue(all(item[1] == 1 for item in offset_history[:first_y]))

    def test_stop_event_and_command_failure_preserve_last_confirmed_voltage(self):
        """Advancing state after an unissued/failed command must fail."""
        fake = FakeANC300Controller(offsets={1: 0, 2: 0})
        stage = self.ready_stage(fake, max_ramp_step_v=1, x_um_per_v=1, y_um_per_v=1)
        fake.history.clear()
        with self.assertRaises(StageError):
            stage.move_to_um(2, 0, stop_event=SetAfterFirstCommand())
        self.assertEqual([item[2] for item in fake.history if item[:2] == ("set_offset", 1)], [1.0])
        self.assertEqual(stage.get_voltage_metadata()["x_voltage_v"], 1.0)
        fake.fail_on_set = 1
        with self.assertRaises(StageError):
            stage.move_to_um(3, 0)
        self.assertEqual(stage.get_voltage_metadata()["x_voltage_v"], 1.0)

    def test_enable_outputs_requires_profile_and_handles_gnd_off_and_offs(self):
        """Bypassing profile gate or changing off-mode offset must fail."""
        fake = FakeANC300Controller(modes={1: "gnd", 2: "offs"}, offsets={1: 7, 2: 8})
        stage = self.stage(fake)
        stage.connect()
        with self.assertRaises(StageError):
            stage.set_origin()
        with self.assertRaises(StageError):
            stage.enable_outputs()
        stage.confirm_hardware_profile()
        stage.enable_outputs()
        stage.set_origin()
        self.assertEqual(fake.offsets[1], 0.0)
        self.assertEqual(fake.offsets[2], 8.0)
        self.assertEqual(fake.modes, {1: "off", 2: "offs"})
        self.assertTrue(stage.outputs_enabled)

    def test_enable_from_ground_disables_and_verifies_inputs_before_offset_mode(self):
        """Changing mode before the grounded input-off checks must fail this sequence."""
        fake = FakeANC300Controller(offsets={1: 7, 2: 8})
        fake.ac_inputs[1] = True
        fake.dc_inputs[1] = True
        stage = self.stage(fake)
        stage.connect(); stage.confirm_hardware_profile(); fake.history.clear()
        stage.enable_outputs()
        for axis in (1, 2):
            history = fake.history
            self.assertLess(history.index(("set_offset", axis, 0.0)), history.index(("set_ac_input", axis, False)))
            self.assertLess(history.index(("set_ac_input", axis, False)), history.index(("get_ac_input", axis)))
            self.assertLess(history.index(("set_dc_input", axis, False)), history.index(("get_dc_input", axis)))
            self.assertLess(history.index(("get_dc_input", axis)), history.index(("set_mode", axis, "off")))
            self.assertIn(("get_output", axis), history)
        self.assertTrue(stage.outputs_enabled)

    def test_enable_from_ground_requires_absolute_offset_and_output_zero_checks(self):
        """A small geto-geta delta must not hide measured output above 0.05 V."""
        fake = FakeANC300Controller(offsets={1: 7, 2: 8})
        fake.readback_offsets[1] = 0.049
        fake.readback_outputs[1] = 0.098
        stage = self.stage(fake)
        stage.connect()
        stage.confirm_hardware_profile()

        with self.assertRaises(StageError):
            stage.enable_outputs()

        self.assertFalse(stage.outputs_enabled)
        self.assertIn(("get_offset", 1), fake.history)
        self.assertIn(("get_output", 1), fake.history)

    def test_enable_live_offset_refuses_external_inputs_or_output_mismatch_without_changing_inputs(self):
        """Silently disabling a live external input or trusting only geta must fail."""
        for input_name in ("ac_inputs", "dc_inputs"):
            with self.subTest(input_name=input_name):
                fake = FakeANC300Controller(modes={1: "off", 2: "off"}, offsets={1: 10, 2: 20})
                getattr(fake, input_name)[1] = True
                stage = self.stage(fake); stage.connect(); stage.confirm_hardware_profile(); fake.history.clear()
                with self.assertRaises(StageError):
                    stage.enable_outputs()
                self.assertFalse(stage.outputs_enabled)
                self.assertFalse(any(item[0] in {"set_ac_input", "set_dc_input"} for item in fake.history))
        fake = FakeANC300Controller(modes={1: "off", 2: "off"}, offsets={1: 10, 2: 20})
        fake.readback_outputs[1] = 10.051
        stage = self.stage(fake); stage.connect(); stage.confirm_hardware_profile()
        with self.assertRaises(StageError):
            stage.enable_outputs()
        self.assertFalse(stage.outputs_enabled)

    def test_ground_ramps_then_verifies_and_failure_is_not_claimed_safe(self):
        """Grounding without zero ramp or claiming success after failure must fail."""
        fake = FakeANC300Controller(offsets={1: 2, 2: 1})
        stage = self.ready_stage(fake, max_ramp_step_v=1)
        fake.history.clear()
        stage.ground_outputs()
        self.assertEqual(fake.modes, {1: "gnd", 2: "gnd"})
        self.assertEqual(fake.offsets, {1: 0.0, 2: 0.0})
        self.assertFalse(stage.outputs_enabled)
        fake = FakeANC300Controller(offsets={1: 1, 2: 1})
        stage = self.ready_stage(fake)
        fake.fail_on_mode = 2
        with self.assertRaises(StageError):
            stage.ground_outputs()
        self.assertFalse(stage.outputs_enabled)

    def test_ground_refuses_nonzero_readback_before_any_ground_mode_command(self):
        """Grounding after a nonzero post-ramp readback must fail."""
        fake = FakeANC300Controller(modes={1: "off", 2: "off"}, offsets={1: 1, 2: 1})
        stage = self.ready_stage(fake)
        fake.readback_offsets[1] = 0.051
        fake.history.clear()
        with self.assertRaises(StageError):
            stage.ground_outputs()
        self.assertFalse(any(item == ("set_mode", 1, "gnd") or item == ("set_mode", 2, "gnd")
                             for item in fake.history))

    def test_ground_refuses_live_inputs_or_nonzero_measured_output_before_ground_command(self):
        """Grounding with an external input or nonzero geto must fail before setm gnd."""
        fake = FakeANC300Controller(modes={1: "off", 2: "off"}, offsets={1: 1, 2: 1})
        stage = self.ready_stage(fake)
        fake.ac_inputs[1] = True
        fake.history.clear()
        with self.assertRaises(StageError):
            stage.ground_outputs()
        self.assertFalse(any(item[0] in {"set_offset", "set_mode"} for item in fake.history))

        fake = FakeANC300Controller(modes={1: "off", 2: "off"}, offsets={1: 1, 2: 1})
        stage = self.ready_stage(fake)
        fake.readback_outputs[1] = 0.051
        fake.history.clear()
        with self.assertRaises(StageError):
            stage.ground_outputs()
        self.assertFalse(any(item[:3] == ("set_mode", axis, "gnd") for axis in (1, 2) for item in fake.history))

    def test_disconnect_never_grounds_and_stop_uses_only_configured_axes(self):
        """Disconnect writes or stop touches axis 3 must fail."""
        fake = FakeANC300Controller()
        stage = self.ready_stage(fake)
        fake.history.clear(); stage.disconnect()
        self.assertEqual(fake.history, [("disconnect",)])
        stage.connect(); stage.stop()
        self.assertEqual([item for item in fake.history if item[0] == "stop"], [("stop", 1), ("stop", 2)])
        self.assertFalse(stage.outputs_enabled)
        self.assertFalse(any(3 in item[1:] for item in fake.history if item[0] in {"set_mode", "set_offset", "stop"}))

    def test_simulated_stage_requires_connect_enable_origin_order_but_not_profile_confirmation(self):
        """Simulation movement or origin capture before explicit enable must fail."""
        stage = SimulatedANC300Stage(ANC300StageSettings(host="sim"))
        with self.assertRaises(StageError):
            stage.move_to_um(1, 1)
        stage.connect()
        with self.assertRaises(StageError):
            stage.set_origin()
        stage.enable_outputs(); stage.set_origin(); stage.move_to_um(2, 3)
        self.assertEqual(stage.get_position_um(), (2.0, 3.0))
        self.assertTrue(stage.get_voltage_metadata()["position_estimated"])
        stage.ground_outputs(); stage.enable_outputs()
        with self.assertRaises(StageError):
            stage.move_to_um(0, 0)
        stage.set_origin(); stage.move_to_um(1, 1)

    def test_supported_enable_then_origin_sequence_rebases_grounded_saved_offsets(self):
        """Capturing a stale grounded offset as origin must fail this zero-position check."""
        fake = FakeANC300Controller(offsets={1: 10, 2: 20})
        stage = self.stage(fake)
        stage.connect(); stage.confirm_hardware_profile(); stage.enable_outputs(); stage.set_origin()
        fake.history.clear()
        stage.move_to_um(0, 0)
        self.assertFalse(any(item[0] == "set_offset" for item in fake.history))
        self.assertEqual(stage.get_position_um(), (0.0, 0.0))

    def test_simulated_stage_accepts_empty_host_and_calibrates_with_signed_direction(self):
        """Simulation must not require a network host and must retain calibration parity."""
        stage = SimulatedANC300Stage(ANC300StageSettings(host=""))
        stage.connect()
        result = stage.calibrate_axis("Y", -4, 2)
        self.assertEqual(result, {"axis": "Y", "um_per_v": 0.5, "direction": -1,
                                  "calibration_source": "custom"})
        metadata = stage.get_voltage_metadata()
        self.assertEqual(metadata["y_um_per_v"], 0.5)
        self.assertEqual(metadata["y_direction"], -1)
        self.assertEqual(metadata["calibration_source"], "custom")


if __name__ == "__main__":
    unittest.main(verbosity=2)
