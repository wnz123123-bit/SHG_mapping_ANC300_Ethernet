"""Integration coverage for the ANC300 mapping UI and independent MT rotator."""

from __future__ import annotations

import json
import tempfile
import threading
import unittest
from pathlib import Path
from tkinter import ttk
from unittest import mock

import main
from anc300_stage import ANC300ScanStage, SimulatedANC300Stage
from app_config import default_config, save_config
from stage_controller import SimulatedStage


class RecordingANCStage:
    """Complete in-memory ANC stage boundary used instead of real hardware."""

    def __init__(self, settings):
        self.settings = settings
        self.calls = []
        self.connected = False
        self.outputs_enabled = True
        self.origin_set = False
        self.position = None

    def connect(self):
        self.calls.append(("connect",))
        self.connected = True
        return self.get_status()

    def disconnect(self):
        self.calls.append(("disconnect",))
        self.connected = False

    def confirm_hardware_profile(self, confirmed=True):
        self.calls.append(("confirm_hardware_profile", bool(confirmed)))
        self.settings.hardware_profile_confirmed = bool(confirmed)

    def enable_outputs(self):
        self.calls.append(("enable_outputs",))
        self.outputs_enabled = True

    def ground_outputs(self, stop_event=None):
        self.calls.append(("ground_outputs", stop_event))
        self.outputs_enabled = False

    def set_origin(self):
        self.calls.append(("set_origin",))
        self.origin_set = True
        self.position = (0.0, 0.0)

    def calibrate_axis(self, axis_name, delta_voltage_v, measured_displacement_um):
        ratio = float(measured_displacement_um) / float(delta_voltage_v)
        scale = abs(ratio)
        direction = 1 if ratio > 0 else -1
        if axis_name == "X":
            self.settings.x_um_per_v = scale
            self.settings.x_direction = direction
        else:
            self.settings.y_um_per_v = scale
            self.settings.y_direction = direction
        self.settings.calibration_source = "custom"
        self.calls.append(("calibrate_axis", axis_name, delta_voltage_v, measured_displacement_um))
        return {"axis": axis_name, "um_per_v": scale, "direction": direction, "calibration_source": "custom"}

    def move_to_um(self, x_um, y_um, stop_event=None):
        self.calls.append(("move_to_um", float(x_um), float(y_um), stop_event))

    def move_origin(self, stop_event=None):
        self.calls.append(("move_origin", stop_event))

    def stop(self):
        self.calls.append(("stop",))

    def get_status(self):
        return {
            "connected": self.connected,
            "outputs_enabled": self.outputs_enabled,
            "origin_set": self.origin_set,
            "hardware_profile_confirmed": self.settings.hardware_profile_confirmed,
            "axes": {"x": self.settings.x_axis, "y": self.settings.y_axis},
            "modes": {"x": "off", "y": "gnd"},
            "device_identity": {
                "version": "ANC300 test version",
                "controller_serial": "CTRL-42",
            },
            "x_voltage_v": 12.5,
            "y_voltage_v": 9.25,
            "x_origin_voltage_v": None,
            "y_origin_voltage_v": None,
            "x_um_per_v": self.settings.x_um_per_v,
            "y_um_per_v": self.settings.y_um_per_v,
            "x_direction": self.settings.x_direction,
            "y_direction": self.settings.y_direction,
            "calibration_source": self.settings.calibration_source,
            "estimated_position_um": self.position,
            "position_estimated": True,
        }


class RecordingDisconnectable:
    def __init__(self, fail=False):
        self.fail = fail
        self.calls = []

    def disconnect(self):
        self.calls.append(("disconnect",))
        if self.fail:
            raise RuntimeError("simulated disconnect failure")


class BlockingGroundANCStage(RecordingANCStage):
    """ANC stage whose grounding call stays live until the test releases it."""

    def __init__(self, settings):
        super().__init__(settings)
        self.ground_started = threading.Event()
        self.release_ground = threading.Event()

    def ground_outputs(self, stop_event=None):
        self.calls.append(("ground_outputs", stop_event))
        self.ground_started.set()
        if not self.release_ground.wait(2.0):
            raise RuntimeError("timed out waiting to release grounding")
        self.outputs_enabled = False


class BlockingPMTCounter:
    """PMT context whose read remains live until the test releases it."""

    def __init__(self):
        self.read_started = threading.Event()
        self.release_read = threading.Event()

    def __call__(self, settings, simulate=False):
        return self

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        return False

    def read_average(self, stop_event=None):
        self.read_started.set()
        if not self.release_read.wait(2.0):
            raise RuntimeError("timed out waiting to release PMT read")
        return 1.0, [1.0]


class MainANC300IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.config_path = Path(self.temp_dir.name) / "config.json"
        self.config = default_config(Path(self.temp_dir.name))
        save_config(self.config_path, self.config)
        self.config_patch = mock.patch.object(main, "CONFIG_PATH", self.config_path)
        self.config_patch.start()
        self.apps = []

    def tearDown(self):
        for app in reversed(self.apps):
            try:
                app.destroy()
            except Exception:
                pass
        self.config_patch.stop()
        self.temp_dir.cleanup()

    def make_app(self):
        app = main.MappingApp()
        app.withdraw()
        app.update_idletasks()
        self.apps.append(app)
        return app

    @staticmethod
    def widget_texts(widget):
        texts = []
        for child in widget.winfo_children():
            try:
                text = child.cget("text")
            except Exception:
                text = ""
            if text:
                texts.append(str(text))
            texts.extend(MainANC300IntegrationTests.widget_texts(child))
        return texts

    def test_canonical_startup_is_disconnected_and_constructs_no_controller(self):
        """Catches startup that requires legacy stage config or auto-connects hardware."""
        with mock.patch.object(main, "ANC300ScanStage") as anc_cls, mock.patch.object(main, "MTStage") as mt_cls:
            app = self.make_app()
        self.assertIsNone(app.stage)
        self.assertIsNone(app.rotator_stage)
        anc_cls.assert_not_called()
        mt_cls.assert_not_called()

    def test_required_anc_controls_are_visible_and_legacy_mapping_controls_are_absent(self):
        """Catches retention of the old MT X/Y/Z mapping surface."""
        app = self.make_app()
        texts = self.widget_texts(app)
        joined = "\n".join(texts)
        for required in (
            "ANC300 主机/IP",
            "TCP 端口",
            "密码",
            "连接 ANC300",
            "连接 HWP",
            "ANC300 状态与校准",
            "Enable offset outputs",
            "Ramp to 0 V and GND",
            "明文",
        ):
            self.assertIn(required, joined)
        for removed in ("位移台设备号", "X脉冲/um", "Y脉冲/um", "XYZ pulses", "回零速度比例"):
            self.assertNotIn(removed, joined)
        self.assertEqual(app.anc_password_entry.cget("show"), "*")
        motion_sensitive_texts = {str(button.cget("text")) for button in app._motion_sensitive_buttons}
        self.assertTrue(
            {
                "Confirm ANSxyz100std/LT profile",
                "Enable offset outputs",
                "Ramp to 0 V and GND",
                "校准X轴",
                "校准Y轴",
                "测试PMT读取",
            }.issubset(motion_sensitive_texts)
        )

    def test_anc_and_rotator_settings_are_built_from_independent_variables(self):
        """Catches ANC values leaking into MT rotator transport or vice versa."""
        app = self.make_app()
        app.anc_host.set("192.0.2.50")
        app.anc_port.set(8123)
        app.anc_password.set("plain-secret")
        app.anc_timeout_s.set(4.5)
        app.anc_x_axis.set(2)
        app.anc_y_axis.set(1)
        app.anc_x_um_per_v.set(0.31)
        app.anc_y_um_per_v.set(0.42)
        app.anc_x_direction.set(-1)
        app.anc_y_direction.set(1)
        app.rotator_dll_path.set("C:/rotator/MT_API.dll")
        app.rotator_connection_type.set("NET")
        app.rotator_com_port.set("COM9")
        app.rotator_ip_address.set("198.51.100.8")
        app.rotator_ip_port.set(9001)
        app.rotator_usb_device_index.set(7)

        anc = app.anc300_stage_settings()
        rotator = app.rotator_stage_settings()

        self.assertEqual((anc.host, anc.port, anc.password, anc.timeout_s), ("192.0.2.50", 8123, "plain-secret", 4.5))
        self.assertEqual((anc.x_axis, anc.y_axis, anc.x_um_per_v, anc.y_um_per_v), (2, 1, 0.31, 0.42))
        self.assertEqual((anc.x_direction, anc.y_direction), (-1, 1))
        self.assertEqual(rotator.dll_path, "C:/rotator/MT_API.dll")
        self.assertEqual((rotator.connection_type, rotator.com_port), ("NET", "COM9"))
        self.assertEqual((rotator.ip_address, rotator.ip_port, rotator.usb_device_index), ("198.51.100.8", 9001, 7))

    def test_simulation_connects_separate_mapping_and_rotator_objects(self):
        """Catches the old shared simulated controller path."""
        app = self.make_app()
        app.simulation_mode.set(True)
        self.assertTrue(app.connect_stage_only())
        self.assertTrue(app.connect_rotator_only())
        self.assertIsInstance(app.stage, SimulatedANC300Stage)
        self.assertIsInstance(app.rotator_stage, SimulatedStage)
        self.assertIsNot(app.stage, app.rotator_stage)
        self.assertIs(app.rotator_controller(), app.rotator_stage)

    def test_real_mapping_connect_is_read_only_and_blank_host_fails_locally(self):
        """Catches output-changing connect flows and attempts to open a blank host."""
        app = self.make_app()
        app.simulation_mode.set(False)
        app.anc_host.set("192.0.2.9")
        created = []

        def build(settings):
            stage = RecordingANCStage(settings)
            created.append(stage)
            return stage

        with mock.patch.object(main, "ANC300ScanStage", side_effect=build):
            self.assertTrue(app.connect_stage_only())
        self.assertEqual(created[0].calls, [("connect",)])

        app._disconnect_stage_now()
        app.anc_host.set("   ")
        with mock.patch.object(main, "ANC300ScanStage") as anc_cls, mock.patch.object(main.messagebox, "showerror") as error:
            self.assertFalse(app.connect_stage_only())
        anc_cls.assert_not_called()
        self.assertIn("host", error.call_args.args[1].lower())

    def test_disconnect_only_disconnects_and_does_not_change_outputs(self):
        """Catches implicit grounding, enabling, stopping, or homing on disconnect."""
        app = self.make_app()
        stage = RecordingANCStage(app.anc300_stage_settings())
        stage.connected = True
        stage.outputs_enabled = True
        app.stage = stage
        app._disconnect_stage_now()
        self.assertEqual(stage.calls, [("disconnect",)])
        self.assertTrue(stage.outputs_enabled)

    def test_profile_enable_and_ground_actions_call_only_the_matching_safety_methods(self):
        """Catches safety buttons routed to the wrong controller action."""
        app = self.make_app()
        stage = RecordingANCStage(app.anc300_stage_settings())
        stage.connected = True
        app.stage = stage
        confirmation_text = []

        def approve(title, message, **_kwargs):
            confirmation_text.append(f"{title}\n{message}")
            return True

        with mock.patch.object(main.messagebox, "askyesno", side_effect=approve):
            app.confirm_anc300_hardware_profile()
        app.enable_anc300_outputs()
        with mock.patch.object(app, "_start_worker", side_effect=lambda _name, target, args=(): target(*args)):
            app.ground_anc300_outputs()

        self.assertIn("ANSxyz100std/LT", confirmation_text[0])
        self.assertIn("4 K", confirmation_text[0])
        self.assertIn("0-150 V", confirmation_text[0])
        self.assertEqual(
            [call[0] for call in stage.calls],
            ["confirm_hardware_profile", "enable_outputs", "ground_outputs"],
        )
        self.assertTrue(app.anc_hardware_profile_confirmed.get())
        self.assertFalse(app.operation_busy())

    def test_synchronous_safety_and_calibration_actions_hold_and_release_reservation(self):
        """Calling hardware without a reservation, or leaking it after failure, must fail."""
        app = self.make_app()
        stage = RecordingANCStage(app.anc300_stage_settings())
        stage.connected = True
        app.stage = stage
        observed = []

        def approve(*_args, **_kwargs):
            observed.append(("confirm_dialog", app._motion_active))
            return True

        with mock.patch.object(main.messagebox, "askyesno", side_effect=approve):
            self.assertTrue(app.confirm_anc300_hardware_profile())
        self.assertFalse(app.operation_busy())

        def fail_enable():
            observed.append(("enable", app._motion_active))
            raise RuntimeError("simulated enable failure")

        stage.enable_outputs = fail_enable
        with mock.patch.object(main.messagebox, "showerror"):
            self.assertFalse(app.enable_anc300_outputs())
        self.assertFalse(app.operation_busy())

        def calibrate(axis_name, delta_voltage_v, measured_displacement_um):
            observed.append(("calibrate", app._motion_active))
            return {
                "axis": axis_name,
                "um_per_v": abs(float(measured_displacement_um) / float(delta_voltage_v)),
                "direction": 1,
                "calibration_source": "custom",
            }

        stage.calibrate_axis = calibrate
        with mock.patch.object(main.simpledialog, "askfloat", side_effect=[2.0, 1.0]):
            app.calibrate_axis("X")
        self.assertFalse(app.operation_busy())
        self.assertEqual(observed, [("confirm_dialog", True), ("enable", True), ("calibrate", True)])

    def test_live_grounding_worker_refuses_scan_enable_profile_and_calibration(self):
        """A live grounding thread without exclusive ownership must not overlap hardware work."""
        app = self.make_app()
        stage = BlockingGroundANCStage(app.anc300_stage_settings())
        stage.connected = True
        stage.origin_set = True
        app.stage = stage
        scan_config = {
            "point_args": {
                "x_start": 0.0,
                "x_end": 0.0,
                "y_start": 0.0,
                "y_end": 0.0,
                "step_um": 1.0,
                "serpentine": False,
            }
        }

        self.assertTrue(app.ground_anc300_outputs())
        self.assertTrue(stage.ground_started.wait(1.0))
        try:
            with (
                mock.patch.object(app, "current_scan_config", return_value=scan_config),
                mock.patch.object(app, "_preflight_stage_points"),
                mock.patch.object(app, "save_config"),
                mock.patch.object(app, "_start_worker") as scan_worker,
                mock.patch.object(main.simpledialog, "askfloat", side_effect=[2.0, 1.0]) as askfloat,
                mock.patch.object(main.messagebox, "askyesno", return_value=True) as askyesno,
                mock.patch.object(main.messagebox, "showwarning"),
            ):
                app.start_scan()
                app.enable_anc300_outputs()
                app.confirm_anc300_hardware_profile()
                app.calibrate_axis("X")
            scan_started = scan_worker.called
            calibration_prompted = askfloat.called
            profile_prompted = askyesno.called
        finally:
            stage.release_ground.set()
            app.ground_thread.join(1.0)

        self.assertFalse(scan_started)
        self.assertFalse(calibration_prompted)
        self.assertFalse(profile_prompted)
        self.assertEqual([call[0] for call in stage.calls], ["ground_outputs"])
        self.assertFalse(app.operation_busy())

    def test_live_pmt_test_prevents_mapping_scan(self):
        """A standalone PMT session must exclude a mapping scan that would own PMT access."""
        app = self.make_app()
        blocker = BlockingPMTCounter()
        scan_config = {
            "point_args": {
                "x_start": 0.0,
                "x_end": 0.0,
                "y_start": 0.0,
                "y_end": 0.0,
                "step_um": 1.0,
                "serpentine": False,
            }
        }

        with (
            mock.patch.object(main, "PMTCounter", blocker),
            mock.patch.object(app, "pmt_settings", return_value=main.PMTSettings(enabled=True)),
            mock.patch.object(app.simulation_mode, "get", return_value=True),
        ):
            app.test_pmt_read()
            self.assertTrue(blocker.read_started.wait(1.0))
            try:
                with (
                    mock.patch.object(app, "current_scan_config", return_value=scan_config),
                    mock.patch.object(app, "_preflight_stage_points"),
                    mock.patch.object(app, "save_config"),
                    mock.patch.object(app, "_ensure_stage", return_value=True),
                    mock.patch.object(app, "_start_worker") as scan_worker,
                    mock.patch.object(main.messagebox, "showwarning"),
                ):
                    app.start_scan()
                scan_started = scan_worker.called
            finally:
                blocker.release_read.set()
                app.pmt_test_thread.join(1.0)

        self.assertFalse(scan_started)
        self.assertFalse(app.operation_busy())

    def test_status_without_origin_shows_voltages_and_unavailable_position_without_logging_errors(self):
        """Catches repeated get_position failures before a session origin is set."""
        app = self.make_app()
        stage = RecordingANCStage(app.anc300_stage_settings())
        stage.connected = True
        app.stage = stage
        before = app.log_box.get("1.0", "end")
        app.refresh_anc300_status()
        after = app.log_box.get("1.0", "end")
        self.assertIn("12.500", app.anc_x_status_text.get())
        self.assertIn("9.250", app.anc_y_status_text.get())
        self.assertIn("unavailable", app.position_text.get().lower())
        self.assertEqual(after, before)

    def test_calibration_updates_signed_scale_without_moving_hardware(self):
        """Catches the legacy pulse calibration formula or a calibration-triggered move."""
        app = self.make_app()
        stage = RecordingANCStage(app.anc300_stage_settings())
        stage.connected = True
        app.stage = stage
        with mock.patch.object(main.simpledialog, "askfloat", side_effect=[20.0, -6.0]):
            app.calibrate_axis("X")
        self.assertAlmostEqual(app.anc_x_um_per_v.get(), 0.3)
        self.assertEqual(app.anc_x_direction.get(), -1)
        self.assertEqual(app.anc_calibration_source.get(), "custom")
        self.assertEqual([call[0] for call in stage.calls], ["calibrate_axis"])

    def test_save_persists_plaintext_password_and_no_legacy_stage(self):
        """Catches custom JSON persistence that retains the removed stage section."""
        app = self.make_app()
        app.anc_password.set("secret-明文")
        app.save_config(silent=True)
        saved = json.loads(self.config_path.read_text(encoding="utf-8"))
        self.assertEqual(saved["anc300"]["password"], "secret-明文")
        self.assertNotIn("stage", saved)
        self.assertIn("dll_path", saved["rotator"])
        self.assertIn("connection_type", saved["rotator"])

    def test_mapping_motion_paths_use_only_two_axis_stage_interface(self):
        """Catches selected-point or origin motion escaping the high-level ANC interface."""
        app = self.make_app()
        stage = RecordingANCStage(app.anc300_stage_settings())
        stage.connected = True
        app.stage = stage
        app._move_to_selected_point_worker({"x_um": 1.25, "y_um": -2.5})
        app._move_origin_worker()
        self.assertEqual(
            stage.calls,
            [
                ("move_to_um", 1.25, -2.5, app.stop_event),
                ("move_origin", app.stop_event),
            ],
        )

    def test_simulation_toggle_locks_while_connected_and_backend_mismatch_is_refused(self):
        """Catches a UI-mode flip making a real backend operate as simulation or vice versa."""
        app = self.make_app()
        app.simulation_mode.set(False)
        app.anc_host.set("192.0.2.20")
        created = []

        def build(settings):
            stage = RecordingANCStage(settings)
            created.append(stage)
            return stage

        with mock.patch.object(main, "ANC300ScanStage", side_effect=build):
            self.assertTrue(app.connect_stage_only())
        self.assertIn("disabled", app.simulation_checkbutton.state())

        app.simulation_mode.set(True)
        with mock.patch.object(main.messagebox, "showwarning") as warning:
            self.assertFalse(app._ensure_stage())
        warning.assert_called_once()
        self.assertEqual(created[0].calls, [("connect",)])

        with mock.patch.object(app, "connect_rotator_only") as connect_rotator, mock.patch.object(main.messagebox, "showwarning"):
            self.assertFalse(app._ensure_controller(auto_connect=True))
        connect_rotator.assert_not_called()

        real_rotator = RecordingDisconnectable()
        app.rotator_stage = real_rotator
        with mock.patch.object(main.messagebox, "showwarning") as warning:
            self.assertFalse(app._ensure_controller())
        warning.assert_called_once()

        app.simulation_mode.set(False)
        self.assertTrue(app._disconnect_stage_now())
        self.assertNotIn("disabled", app.simulation_checkbutton.state())

    def test_connected_mapping_stage_rejects_reconnect_without_touching_energized_object(self):
        """Catches reconnect replacing or disconnecting an already energized ANC stage."""
        app = self.make_app()
        app.simulation_mode.set(False)
        stage = RecordingANCStage(app.anc300_stage_settings())
        stage.connected = True
        stage.outputs_enabled = True
        app.stage = stage
        app.anc_host.set("   ")
        with (
            mock.patch.object(main, "ANC300ScanStage") as anc_cls,
            mock.patch.object(main.messagebox, "showwarning") as warning,
            mock.patch.object(main.messagebox, "showerror"),
        ):
            self.assertFalse(app.connect_stage_only())
        self.assertIs(app.stage, stage)
        self.assertTrue(stage.connected)
        self.assertTrue(stage.outputs_enabled)
        self.assertEqual(stage.calls, [])
        anc_cls.assert_not_called()
        self.assertIn("disconnect", warning.call_args.args[1].lower())

    def test_disconnect_attempts_both_and_retains_only_hwp_that_failed(self):
        """Catches an HWP failure preventing ANC disconnect or losing the failed HWP reference."""
        app = self.make_app()
        anc = RecordingDisconnectable(fail=False)
        hwp = RecordingDisconnectable(fail=True)
        app.stage = anc
        app.rotator_stage = hwp
        with mock.patch.object(main.messagebox, "showerror") as error:
            self.assertFalse(app._disconnect_stage_now())
        self.assertEqual(anc.calls, [("disconnect",)])
        self.assertEqual(hwp.calls, [("disconnect",)])
        self.assertIsNone(app.stage)
        self.assertIs(app.rotator_stage, hwp)
        error.assert_called_once()
        self.assertIn("disabled", app.simulation_checkbutton.state())

    def test_disconnect_attempts_both_and_retains_only_anc_that_failed(self):
        """Catches an ANC failure preventing HWP disconnect or losing the failed ANC reference."""
        app = self.make_app()
        anc = RecordingDisconnectable(fail=True)
        hwp = RecordingDisconnectable(fail=False)
        app.stage = anc
        app.rotator_stage = hwp
        with mock.patch.object(main.messagebox, "showerror") as error:
            self.assertFalse(app._disconnect_stage_now())
        self.assertEqual(anc.calls, [("disconnect",)])
        self.assertEqual(hwp.calls, [("disconnect",)])
        self.assertIs(app.stage, anc)
        self.assertIsNone(app.rotator_stage)
        error.assert_called_once()
        self.assertIn("disabled", app.simulation_checkbutton.state())

    def test_on_close_keeps_window_open_when_disconnect_fails(self):
        """Catches destruction of the only UI path available to retry a failed disconnect."""
        app = self.make_app()
        app.stage = RecordingDisconnectable(fail=True)
        with mock.patch.object(app, "destroy") as destroy, mock.patch.object(main.messagebox, "showerror"):
            app.on_close()
        destroy.assert_not_called()
        self.assertIsNotNone(app.stage)

    def test_hwp_start_speed_is_visible(self):
        """Catches omission of the rotator start-speed setting from the HWP panel."""
        app = self.make_app()
        self.assertIn("HWP start speed", self.widget_texts(app))


if __name__ == "__main__":
    unittest.main()
