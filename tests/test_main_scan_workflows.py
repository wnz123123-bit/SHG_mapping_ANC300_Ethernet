import csv
import math
import queue
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import main
from app_config import default_config, save_config
from pmt_reader import PMTError, PMTSettings
from stage_controller import StageError


VOLTAGE_FIELDS = [
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
]


class FakeANCStage:
    """Strict fake for the complete ANC boundary used by scan workflows."""

    def __init__(self, trace, *, connected=True, origin_set=True, outputs_enabled=True):
        self.trace = trace
        self.connected = connected
        self.origin_set = origin_set
        self.outputs_enabled = outputs_enabled
        self.position = (0.0, 0.0)
        self.preflighted = []
        self.moves = []
        self.invalid_points = set()
        self.fail_moves = set()
        self.voltage_writes = 0

    def get_status(self):
        self.trace.append(("anc_status",))
        return {
            "connected": self.connected,
            "origin_set": self.origin_set,
            "outputs_enabled": self.outputs_enabled,
            "estimated_position_um": self.position if self.origin_set else None,
        }

    def preflight_points(self, points):
        points = [(float(x), float(y)) for x, y in points]
        self.preflighted.append(points)
        self.trace.append(("anc_preflight", tuple(points)))
        for point in points:
            if point in self.invalid_points:
                raise StageError("Requested voltage is outside configured ANC300 bounds.")
        return [(10.0 + x, 20.0 - y) for x, y in points]

    def move_to_um(self, x_um, y_um, stop_event=None):
        point = (float(x_um), float(y_um))
        self.trace.append(("anc_move", *point))
        if point in self.fail_moves:
            raise StageError("simulated ANC move failure")
        if stop_event is not None and stop_event.is_set():
            raise StageError("Move stopped by user.")
        self.position = point
        self.moves.append(point)
        self.voltage_writes += 1

    def get_voltage_metadata(self):
        x_um, y_um = self.position
        self.trace.append(("anc_metadata", x_um, y_um))
        return {
            "x_voltage_v": 10.0 + x_um,
            "y_voltage_v": 20.0 - y_um,
            "x_origin_voltage_v": 10.0,
            "y_origin_voltage_v": 20.0,
            "x_um_per_v": 0.5,
            "y_um_per_v": 0.25,
            "x_direction": 1,
            "y_direction": -1,
            "calibration_source": "fake-calibration",
            "position_estimated": True,
        }

    def move_origin(self, stop_event=None):
        self.trace.append(("anc_origin",))
        self.position = (0.0, 0.0)

    def stop(self):
        self.trace.append(("anc_stop",))

    def ground_outputs(self, stop_event=None):
        raise AssertionError("scan workflow must not ground ANC outputs")

    def disconnect(self):
        raise AssertionError("scan workflow must not disconnect ANC")

    def move_axis_steps_rel(self, *args, **kwargs):
        raise AssertionError("mapping must never use MT axis-level movement")

    def get_axis_raw_steps(self, *args, **kwargs):
        raise AssertionError("mapping must never read an MT or Z axis")


class FakePMTCounter:
    def __init__(self, trace, outcomes, on_read=None):
        self.trace = trace
        self.outcomes = list(outcomes)
        self.on_read = on_read

    def __call__(self, settings, simulate=False):
        self.trace.append(("pmt_construct", bool(simulate)))
        return self

    def __enter__(self):
        self.trace.append(("pmt_open",))
        return self

    def __exit__(self, exc_type, exc, tb):
        self.trace.append(("pmt_close",))
        return False

    def read_average(self, stop_event=None):
        self.trace.append(("pmt_read",))
        if self.on_read is not None:
            self.on_read()
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome


class FakeRotator:
    """Minimal MT HWP boundary; any axis choice is recorded for safety checks."""

    def __init__(self, trace):
        self.trace = trace
        self.raw_steps = 0
        self.settings = SimpleNamespace(move_timeout_s=2.0)
        self.axes_used = []

    def get_axis_raw_steps(self, axis):
        self.axes_used.append(int(axis))
        self.trace.append(("hwp_raw", int(axis), self.raw_steps))
        return self.raw_steps

    def move_axis_steps_rel(self, axis, delta_steps, **kwargs):
        axis = int(axis)
        self.axes_used.append(axis)
        self.trace.append(("hwp_move", axis, int(delta_steps)))
        start = self.raw_steps
        self.raw_steps += int(delta_steps)
        callback = kwargs.get("progress_callback")
        if callback is not None:
            callback(self.raw_steps)
        return {
            "start_raw": start,
            "end_raw": self.raw_steps,
            "start_software": start,
            "end_software": self.raw_steps,
            "running_seen": True,
        }

    def halt_axis(self, axis):
        self.axes_used.append(int(axis))
        self.trace.append(("hwp_halt", int(axis)))

    def stop(self):
        self.trace.append(("hwp_stop",))


class MainThreadOnlyVar:
    """Tiny Tk-variable stand-in that rejects worker-thread access."""

    def __init__(self, value):
        self.value = value
        self.owner = threading.get_ident()

    def get(self):
        if threading.get_ident() != self.owner:
            raise AssertionError("Tk variable read from worker thread")
        return self.value

    def set(self, value):
        if threading.get_ident() != self.owner:
            raise AssertionError("Tk variable written from worker thread")
        self.value = value


class StrictUnusedRotator:
    """Attached HWP boundary that fails if a mapping workflow touches MT methods."""

    def get_axis_raw_steps(self, *args, **kwargs):
        raise AssertionError("mapping must never read the attached MT rotator")

    def get_axis_steps(self, *args, **kwargs):
        raise AssertionError("mapping must never read the attached MT rotator")

    def move_axis_steps_rel(self, *args, **kwargs):
        raise AssertionError("mapping must never move the attached MT rotator")


class MainScanWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        bundled_pmt = self.root / "vendor_dlls" / "C8855-01api.dll"
        bundled_pmt.parent.mkdir(parents=True)
        bundled_pmt.touch()
        self.config_path = self.root / "config.json"
        save_config(self.config_path, default_config(self.root))
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
        app.simulation_mode.set(False)
        app.output_dir.set(str(self.root / "output"))
        self.apps.append(app)
        return app

    @staticmethod
    def run_worker_inline(_attr_name, target, args=()):
        target(*args)
        return None

    @staticmethod
    def read_csv(path):
        with Path(path).open("r", newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            return list(reader.fieldnames), list(reader)

    def test_scan_points_have_exact_raster_and_serpentine_trajectories(self):
        raster = list(main.scan_points(0, 1, 0, 1, 1, serpentine=False))
        serpentine = list(main.scan_points(0, 1, 0, 1, 1, serpentine=True))
        self.assertEqual(
            raster,
            [(0, 0, 0.0, 0.0), (0, 1, 1.0, 0.0), (1, 0, 0.0, 1.0), (1, 1, 1.0, 1.0)],
        )
        self.assertEqual(
            serpentine,
            [(0, 0, 0.0, 0.0), (0, 1, 1.0, 0.0), (1, 0, 1.0, 1.0), (1, 1, 0.0, 1.0)],
        )

    def test_angle_config_rejects_negative_anc_pre_read_settle(self):
        app = self.make_app()
        app.anc_pre_read_settle_s.set(-0.01)
        with self.assertRaises(ValueError):
            app.angle_scan_config()

    def test_mapping_rejects_later_out_of_range_point_before_worker_file_or_move(self):
        trace = []
        app = self.make_app()
        stage = FakeANCStage(trace)
        stage.invalid_points.add((2.0, 0.0))
        app.stage = stage
        app.x_start_um.set(0)
        app.x_end_um.set(2)
        app.y_start_um.set(0)
        app.y_end_um.set(0)
        app.step_um.set(1)
        workers = []
        with (
            mock.patch.object(app, "_start_worker", side_effect=lambda *args, **kwargs: workers.append((args, kwargs))),
            mock.patch.object(app, "save_config") as save,
            mock.patch.object(main.messagebox, "showerror"),
        ):
            app.start_scan()
        self.assertEqual(stage.preflighted, [[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]])
        self.assertEqual(workers, [])
        self.assertEqual(stage.moves, [])
        self.assertEqual(stage.voltage_writes, 0)
        self.assertFalse((self.root / "output").exists())
        save.assert_not_called()

    def test_mapping_and_angle_require_connected_origin_and_explicit_outputs(self):
        for workflow in ("mapping", "angle"):
            for state in (
                {"connected": False, "origin_set": True, "outputs_enabled": True},
                {"connected": True, "origin_set": False, "outputs_enabled": True},
                {"connected": True, "origin_set": True, "outputs_enabled": False},
            ):
                with self.subTest(workflow=workflow, state=state):
                    trace = []
                    app = self.make_app()
                    stage = FakeANCStage(trace, **state)
                    app.stage = stage
                    app.rotator_stage = FakeRotator(trace)
                    app.selected_point = {"x_um": 0.0, "y_um": 0.0}
                    workers = []
                    with (
                        mock.patch.object(app, "_start_worker", side_effect=lambda *args, **kwargs: workers.append((args, kwargs))),
                        mock.patch.object(app, "save_config"),
                        mock.patch.object(app, "_ensure_no_external_mt_tool", return_value=True),
                        mock.patch.object(app, "_recover_rotator_if_needed", return_value=True),
                        mock.patch.object(main.messagebox, "showwarning"),
                        mock.patch.object(main.messagebox, "showerror"),
                    ):
                        app.start_scan() if workflow == "mapping" else app.start_angle_scan()
                    self.assertEqual(workers, [])
                    self.assertEqual(stage.preflighted, [])
                    self.assertEqual(stage.moves, [])

    def test_mapping_orders_settle_read_dwell_writes_metadata_and_returns_normally(self):
        trace = []
        app = self.make_app()
        stage = FakeANCStage(trace)
        app.stage = stage
        app.rotator_stage = StrictUnusedRotator()
        points = list(main.scan_points(0, 1, 0, 1, 1, serpentine=False))
        config = {
            "dwell_ms": 300,
            "return_to_origin": True,
            "output_dir": str(self.root / "mapping-normal"),
            "pmt": PMTSettings(enabled=True),
            "use_simulated_pmt": False,
        }
        pmt = FakePMTCounter(trace, [(11.0, [10.0, 12.0]), (21.0, [20.0, 22.0]),
                                     (31.0, [30.0, 32.0]), (41.0, [40.0, 42.0])])

        def record_sleep(seconds, stop_event=None):
            trace.append(("sleep", float(seconds)))

        with mock.patch.object(main, "PMTCounter", pmt), mock.patch.object(main, "interruptible_sleep", side_effect=record_sleep):
            app._scan_worker(config, points)

        self.assertEqual(stage.moves, [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)])
        self.assertEqual([item for item in trace if item[0] == "sleep"],
                         [("sleep", 0.2), ("sleep", 0.3), ("sleep", 0.2), ("sleep", 0.3),
                          ("sleep", 0.2), ("sleep", 0.3), ("sleep", 0.2)])
        for point_index in range(4):
            move_at = next(i for i, item in enumerate(trace) if item[:1] == ("anc_move",) and item[1:] == stage.moves[point_index])
            metadata_at = next(i for i in range(move_at + 1, len(trace)) if trace[i][0] == "anc_metadata")
            settle_at = next(i for i in range(move_at + 1, len(trace)) if trace[i] == ("sleep", 0.2))
            read_at = next(i for i in range(move_at + 1, len(trace)) if trace[i][0] == "pmt_read")
            self.assertLess(move_at, metadata_at)
            self.assertLess(move_at, settle_at)
            self.assertLess(settle_at, read_at)
        self.assertEqual([item for item in trace if item[0] == "anc_origin"], [("anc_origin",)])

        fields, rows = self.read_csv(app.latest_csv)
        self.assertEqual(fields, ["index", "row", "col", "x_um", "y_um", "value", "pmt_samples", "timestamp", "measurement_source", *VOLTAGE_FIELDS])
        self.assertEqual(len(rows), 4)
        self.assertEqual(rows[2]["x_voltage_v"], "10.0")
        self.assertEqual(rows[2]["y_voltage_v"], "19.0")
        self.assertEqual(rows[2]["x_origin_voltage_v"], "10.0")
        self.assertEqual(rows[2]["y_origin_voltage_v"], "20.0")
        self.assertEqual(rows[2]["x_um_per_v"], "0.5")
        self.assertEqual(rows[2]["y_um_per_v"], "0.25")
        self.assertEqual(rows[2]["x_direction"], "1")
        self.assertEqual(rows[2]["y_direction"], "-1")
        self.assertEqual(rows[2]["calibration_source"], "fake-calibration")
        self.assertEqual(rows[2]["position_estimated"], "True")
        self.assertEqual(rows[2]["measurement_source"], "pmt_hardware")

    def test_mapping_stop_anc_error_and_pmt_error_keep_partial_csv_without_return(self):
        for failure_kind in ("stop", "anc_error", "pmt_error"):
            with self.subTest(failure_kind=failure_kind):
                trace = []
                app = self.make_app()
                stage = FakeANCStage(trace)
                app.stage = stage
                output_dir = self.root / f"mapping-{failure_kind}"
                config = {
                    "dwell_ms": 0,
                    "return_to_origin": True,
                    "output_dir": str(output_dir),
                    "pmt": PMTSettings(enabled=True),
                    "use_simulated_pmt": False,
                    "pre_read_settle_s": 0.0,
                }
                points = list(main.scan_points(0, 2, 0, 0, 1, serpentine=False))
                if failure_kind == "anc_error":
                    stage.fail_moves.add((1.0, 0.0))
                    on_read = None
                else:
                    on_read = app.stop_event.set
                outcomes = [(5.0, [5.0]), RuntimeError("simulated PMT failure")] if failure_kind == "pmt_error" else [
                    (5.0, [5.0]), (6.0, [6.0])
                ]
                pmt = FakePMTCounter(trace, outcomes, on_read=on_read)
                with mock.patch.object(main, "PMTCounter", pmt), mock.patch.object(main, "interruptible_sleep"):
                    app._scan_worker(config, points)
                self.assertNotIn(("anc_origin",), trace)
                self.assertIn(("anc_stop",), trace)
                self.assertTrue(app.latest_csv.exists())
                _, rows = self.read_csv(app.latest_csv)
                self.assertEqual(len(rows), 1)

    def test_angle_preflights_entire_queue_before_worker_or_output(self):
        trace = []
        app = self.make_app()
        stage = FakeANCStage(trace)
        stage.invalid_points.add((3.0, 4.0))
        app.stage = stage
        app.rotator_stage = FakeRotator(trace)
        app.angle_point_queue = [
            {"x_um": 1.0, "y_um": 2.0, "queue_status": "pending"},
            {"x_um": 3.0, "y_um": 4.0, "queue_status": "pending"},
        ]
        workers = []
        with (
            mock.patch.object(app, "_start_worker", side_effect=lambda *args, **kwargs: workers.append((args, kwargs))),
            mock.patch.object(app, "save_config") as save,
            mock.patch.object(app, "_ensure_no_external_mt_tool", return_value=True),
            mock.patch.object(app, "_recover_rotator_if_needed", return_value=True) as recover,
            mock.patch.object(main.messagebox, "showerror"),
        ):
            app.start_angle_scan()
        self.assertEqual(stage.preflighted, [[(1.0, 2.0), (3.0, 4.0)]])
        self.assertEqual(workers, [])
        self.assertEqual(stage.moves, [])
        self.assertFalse((self.root / "output" / "angle_scans").exists())
        save.assert_not_called()
        recover.assert_not_called()

    def test_multi_point_angle_scan_moves_anc_once_per_point_and_returns_hwp(self):
        trace = []
        app = self.make_app()
        stage = FakeANCStage(trace)
        rotator = FakeRotator(trace)
        app.stage = stage
        app.rotator_stage = rotator
        app.anc_pre_read_settle_s.set(0.17)
        app.angle_start_deg.set(0)
        app.angle_stop_deg.set(10)
        app.angle_step_deg.set(10)
        app.angle_settle_s.set(0.05)
        app.angle_return_to_start.set(True)
        app.rotator_axis_display.set(1)
        app.rotator_steps_per_degree.set(10)
        app.rotator_direction_sign.set(1)
        app.angle_point_queue = [
            {"x_um": 1.0, "y_um": 2.0, "queue_status": "pending"},
            {"x_um": 3.0, "y_um": 4.0, "queue_status": "pending"},
        ]
        pmt = FakePMTCounter(trace, [(10.0, [9.0, 11.0]), (20.0, [19.0, 21.0]),
                                     (30.0, [29.0, 31.0]), (40.0, [39.0, 41.0])])

        def record_sleep(seconds, stop_event=None):
            trace.append(("sleep", float(seconds)))

        with (
            mock.patch.object(app, "_start_worker", side_effect=self.run_worker_inline),
            mock.patch.object(app, "save_config"),
            mock.patch.object(app, "_ensure_no_external_mt_tool", return_value=True),
            mock.patch.object(app, "_recover_rotator_if_needed", return_value=True),
            mock.patch.object(main, "PMTCounter", pmt),
            mock.patch.object(main, "interruptible_sleep", side_effect=record_sleep),
        ):
            app.start_angle_scan()

        self.assertEqual(stage.preflighted, [[(1.0, 2.0), (3.0, 4.0)]])
        self.assertEqual(stage.moves, [(1.0, 2.0), (3.0, 4.0)])
        self.assertNotIn(("anc_origin",), trace)
        self.assertEqual([item for item in trace if item == ("sleep", 0.17)], [("sleep", 0.17), ("sleep", 0.17)])
        self.assertEqual(len([item for item in trace if item == ("sleep", 0.05)]), 4)
        self.assertEqual(len([item for item in trace if item == ("sleep", 0.1)]), 2)
        for point in stage.moves:
            move_at = next(i for i, item in enumerate(trace) if item == ("anc_move", *point))
            metadata_at = next(i for i in range(move_at + 1, len(trace)) if trace[i][0] == "anc_metadata")
            settle_at = next(i for i in range(move_at + 1, len(trace)) if trace[i] == ("sleep", 0.17))
            hwp_at = next(i for i in range(move_at + 1, len(trace)) if trace[i][0] == "hwp_raw")
            self.assertLess(move_at, metadata_at)
            self.assertLess(metadata_at, settle_at)
            self.assertLess(settle_at, hwp_at)
        self.assertEqual([item for item in trace if item[0] == "hwp_move"],
                         [("hwp_move", 0, 100), ("hwp_move", 0, -100),
                          ("hwp_move", 0, 100), ("hwp_move", 0, -100)])
        self.assertTrue(rotator.axes_used)
        self.assertNotIn(3, rotator.axes_used)

        csv_paths = sorted((self.root / "output" / "angle_scans").glob("*.csv"))
        self.assertEqual(len(csv_paths), 2)
        expected_fields = [
            "index", "queue_point_index", "queue_point_total", "x_um", "y_um",
            "target_angle_deg", "reached_angle_deg", "value", "pmt_samples", "timestamp", "measurement_source", *VOLTAGE_FIELDS,
        ]
        fields1, rows1 = self.read_csv(csv_paths[0])
        fields2, rows2 = self.read_csv(csv_paths[1])
        self.assertEqual(fields1, expected_fields)
        self.assertEqual(fields2, expected_fields)
        self.assertEqual([row["x_voltage_v"] for row in rows1], ["11.0", "11.0"])
        self.assertEqual([row["y_voltage_v"] for row in rows1], ["18.0", "18.0"])
        self.assertEqual([row["x_voltage_v"] for row in rows2], ["13.0", "13.0"])
        self.assertEqual([row["y_voltage_v"] for row in rows2], ["16.0", "16.0"])
        self.assertTrue(all(row["calibration_source"] == "fake-calibration" for row in rows1 + rows2))
        self.assertTrue(all(row["measurement_source"] == "pmt_hardware" for row in rows1 + rows2))

    def test_acquisition_settings_are_validated_before_worker_or_result_creation(self):
        """Late rejection or silent coercion of acquisition/timing settings must fail."""
        app = self.make_app()
        invalid_cases = (
            (app.dwell_s, -0.001, app.current_scan_config),
            (app.dwell_s, math.inf, app.current_scan_config),
            (app.anc_pre_read_settle_s, math.nan, app.current_scan_config),
            (app.pmt_samples_to_average, 0, app.current_scan_config),
            (app.pmt_sample_extra_wait_s, -0.001, app.current_scan_config),
            (app.pmt_gate_time_ms, "0.3", app.current_scan_config),
            (app.pmt_transfer_mode, -1, app.current_scan_config),
            (app.pmt_trigger_mode, 99, app.current_scan_config),
            (app.pmt_trigger_edge, 99, app.current_scan_config),
            (app.pmt_simulate_noise, -0.1, app.current_scan_config),
            (app.angle_settle_s, math.nan, app.angle_scan_config),
        )
        for variable, value, build_config in invalid_cases:
            with self.subTest(variable=str(variable), value=value):
                original = variable.get()
                try:
                    variable.set(value)
                    with self.assertRaises((ValueError, StageError, PMTError, OverflowError)):
                        build_config()
                finally:
                    variable.set(original)

        app.pmt_dll_path.set(str(self.root / "missing.dll"))
        with self.assertRaises((ValueError, StageError, PMTError)):
            app.current_scan_config()
        self.assertFalse((self.root / "output").exists())

    def test_real_mapping_refuses_disabled_pmt_before_worker_file_or_motion(self):
        """Recording reader.py's constant as a real measurement must fail closed."""
        trace = []
        app = self.make_app()
        app.stage = FakeANCStage(trace)
        app.pmt_enabled.set(False)
        workers = []
        with (
            mock.patch.object(app, "_start_worker", side_effect=lambda *args, **kwargs: workers.append((args, kwargs))),
            mock.patch.object(app, "save_config"),
            mock.patch.object(main.messagebox, "showerror"),
        ):
            app.start_scan()
        self.assertEqual(workers, [])
        self.assertEqual(app.stage.moves, [])
        self.assertFalse((self.root / "output").exists())

        app._scan_worker(
            {
                "dwell_ms": 0,
                "return_to_origin": False,
                "output_dir": str(self.root / "direct-real-disabled"),
                "pmt": PMTSettings(enabled=False),
                "use_simulated_pmt": False,
                "pre_read_settle_s": 0.0,
            },
            [(0, 0, 0.0, 0.0)],
        )
        self.assertEqual(app.stage.moves, [])
        self.assertFalse(list((self.root / "direct-real-disabled").glob("*.csv")))

    def test_simulation_reader_fallback_is_explicitly_labeled(self):
        """An unlabeled simulated reader value must fail the CSV data-integrity contract."""
        trace = []
        app = self.make_app()
        app.stage = FakeANCStage(trace)
        config = {
            "dwell_ms": 0,
            "return_to_origin": False,
            "output_dir": str(self.root / "reader-simulation"),
            "pmt": PMTSettings(enabled=False),
            "use_simulated_pmt": True,
            "pre_read_settle_s": 0.0,
        }
        with mock.patch.object(main, "read_value", return_value=9.0):
            app._scan_worker(config, [(0, 0, 0.0, 0.0)])
        _, rows = self.read_csv(app.latest_csv)
        self.assertEqual(rows[0]["value"], "9.0")
        self.assertEqual(rows[0]["measurement_source"], "reader_simulation")

    def test_stop_routes_commands_only_to_the_active_operation_owner(self):
        """Broadcasting Stop to unrelated devices or touching hardware while idle must fail."""
        expectations = {
            "mapping scan": [("anc_stop",)],
            "selected-point move": [("anc_stop",)],
            "stage origin move": [("anc_stop",)],
            "manual rotator move": [("hwp_stop",)],
            "angle scan": [("hwp_stop",), ("anc_stop",)],
            "pmt test": [],
            None: [],
        }
        for owner, expected in expectations.items():
            with self.subTest(owner=owner):
                trace = []
                app = self.make_app()
                app.stage = FakeANCStage(trace)
                app.rotator_stage = FakeRotator(trace)
                app._motion_active = owner is not None
                app._motion_label = owner
                with mock.patch.object(main.messagebox, "askyesno", return_value=True):
                    app.stop_scan()
                self.assertEqual([item for item in trace if item[0] in {"anc_stop", "hwp_stop"}], expected)

    def test_real_pmt_test_rejects_missing_dll_before_worker_when_checkbox_is_disabled(self):
        """Real PMT test must validate the DLL before worker/device access regardless of checkbox."""
        trace = []
        app = self.make_app()
        app.stage = FakeANCStage(trace)
        app.rotator_stage = FakeRotator(trace)
        app.pmt_enabled.set(False)
        app.pmt_dll_path.set(str(self.root / "missing-pmt.dll"))
        workers = []

        with mock.patch.object(
            app,
            "_start_worker",
            side_effect=lambda *args, **kwargs: workers.append((args, kwargs)),
        ):
            started = app.test_pmt_read()

        self.assertFalse(started)
        self.assertEqual(workers, [])
        self.assertEqual(trace, [])
        self.assertIsNone(app.pmt_test_thread)
        self.assertFalse(app._motion_active)

    def test_pmt_test_snapshots_tk_settings_and_stop_event_interrupts_without_device_broadcast(self):
        """Worker Tk reads or a PMT test that ignores Stop must fail."""
        trace = []
        app = self.make_app()
        app.stage = FakeANCStage(trace)
        app.rotator_stage = FakeRotator(trace)
        app.simulation_mode = MainThreadOnlyVar(True)
        for name in (
            "pmt_enabled", "pmt_dll_path", "pmt_gate_time_ms", "pmt_samples_to_average",
            "pmt_sample_extra_wait_s", "pmt_transfer_mode", "pmt_trigger_mode",
            "pmt_trigger_edge", "pmt_simulate_value", "pmt_simulate_noise",
        ):
            original = getattr(app, name).get()
            setattr(app, name, MainThreadOnlyVar(original))

        started = threading.Event()
        stopped = threading.Event()

        class StopAwarePMT:
            def __init__(self, settings, simulate=False):
                pass

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def read_average(self, stop_event=None):
                self_case.assertIsNotNone(stop_event)
                started.set()
                if not stop_event.wait(1.0):
                    raise AssertionError("PMT stop_event was never set")
                stopped.set()
                raise StageError("stopped")

        self_case = self
        with mock.patch.object(main, "PMTCounter", StopAwarePMT):
            self.assertTrue(app.test_pmt_read())
            self.assertTrue(started.wait(0.5))
            with mock.patch.object(main.messagebox, "askyesno", return_value=True):
                app.stop_scan()
            app.pmt_test_thread.join(1.0)
        self.assertTrue(stopped.is_set())
        self.assertEqual([item for item in trace if item[0] in {"anc_stop", "hwp_stop"}], [])

    def test_angle_worker_uses_one_hwp_snapshot_after_gui_values_change(self):
        """A running angle scan adopting later HWP GUI edits must fail."""
        trace = []
        app = self.make_app()
        app.stage = FakeANCStage(trace)
        rotator = FakeRotator(trace)
        app.rotator_stage = rotator
        app.selected_point = {"x_um": 1.0, "y_um": 2.0}
        app.angle_start_deg.set(0)
        app.angle_stop_deg.set(10)
        app.angle_step_deg.set(10)
        app.angle_return_to_start.set(False)
        app.rotator_axis_display.set(1)
        app.rotator_steps_per_degree.set(10)
        app.rotator_direction_sign.set(1)
        captured = []
        with (
            mock.patch.object(app, "_start_worker", side_effect=lambda attr, target, args=(): captured.append((target, args))),
            mock.patch.object(app, "save_config"),
            mock.patch.object(app, "_ensure_no_external_mt_tool", return_value=True),
            mock.patch.object(app, "_ensure_controller", return_value=True),
            mock.patch.object(app, "_recover_rotator_if_needed", return_value=True),
        ):
            app.start_angle_scan()
        self.assertEqual(len(captured), 1)
        app.rotator_axis_display.set(2)
        app.rotator_steps_per_degree.set(100)
        app.rotator_direction_sign.set(-1)
        pmt = FakePMTCounter(trace, [(1.0, [1.0]), (2.0, [2.0])])
        with mock.patch.object(main, "PMTCounter", pmt), mock.patch.object(main, "interruptible_sleep"):
            captured[0][0](*captured[0][1])
        self.assertTrue(rotator.axes_used)
        self.assertEqual(set(rotator.axes_used), {0})
        self.assertIn(("hwp_move", 0, 100), trace)

    def test_angle_csv_header_exists_before_motion_and_open_failure_moves_nothing(self):
        """Moving before an exclusive angle CSV header is flushed must fail."""
        trace = []
        app = self.make_app()
        stage = FakeANCStage(trace)
        app.stage = stage
        app.rotator_stage = FakeRotator(trace)
        config = app.angle_scan_config()
        config["angles"] = [0.0]
        point = {"x_um": 1.0, "y_um": 2.0}
        original_move = stage.move_to_um

        def assert_header_then_move(*args, **kwargs):
            self.assertTrue(app.latest_angle_csv.exists())
            with app.latest_angle_csv.open("r", encoding="utf-8-sig") as handle:
                self.assertIn("measurement_source", handle.readline())
            return original_move(*args, **kwargs)

        stage.move_to_um = assert_header_then_move
        pmt = FakePMTCounter(trace, [(1.0, [1.0])])
        with mock.patch.object(main, "PMTCounter", pmt), mock.patch.object(main, "interruptible_sleep"):
            app._run_single_angle_scan(point, config)
        self.assertEqual(stage.moves, [(1.0, 2.0)])

        trace.clear(); stage.moves.clear()
        with mock.patch.object(Path, "open", side_effect=PermissionError("denied")):
            with self.assertRaises(PermissionError):
                app._run_single_angle_scan(point, config)
        self.assertEqual(stage.moves, [])
        self.assertFalse(any(item[0] == "hwp_move" for item in trace))

    def test_second_angle_file_failure_issues_no_motion_or_halt_for_failed_point(self):
        """A prior point's motion state must not trigger cleanup commands after the next open fails."""
        trace = []
        app = self.make_app()
        app.stage = FakeANCStage(trace)
        app.rotator_stage = FakeRotator(trace)
        config = app.angle_scan_config()
        config.update({
            "angles": [10.0],
            "settle_s": 0.0,
            "pre_read_settle_s": 0.0,
            "return_to_start": False,
        })
        points = [
            {"x_um": 1.0, "y_um": 2.0},
            {"x_um": 3.0, "y_um": 4.0},
        ]
        pmt = FakePMTCounter(trace, [(1.0, [1.0])])
        real_open_result_csv = main.open_result_csv
        exclusive_calls = 0
        failed_point_trace_start = None

        def open_first_then_fail(*args, **kwargs):
            nonlocal exclusive_calls, failed_point_trace_start
            exclusive_calls += 1
            if exclusive_calls == 2:
                failed_point_trace_start = len(trace)
                raise PermissionError("second angle file denied")
            return real_open_result_csv(*args, **kwargs)

        with (
            mock.patch.object(main, "open_result_csv", side_effect=open_first_then_fail),
            mock.patch.object(main, "PMTCounter", pmt),
            mock.patch.object(main, "interruptible_sleep"),
        ):
            app._angle_scan_sequence_worker(points, config)

        self.assertEqual(app.stage.moves, [(1.0, 2.0)])
        self.assertTrue(any(item[0] == "hwp_move" for item in trace))
        self.assertIsNotNone(failed_point_trace_start)
        failed_point_trace = trace[failed_point_trace_start:]
        self.assertFalse(any(item[0] in {"anc_move", "hwp_move", "hwp_halt"} for item in failed_point_trace))

    def test_result_filenames_use_exclusive_collision_suffixes(self):
        """Two runs with the same timestamp truncating one result must fail."""
        trace = []
        app = self.make_app()
        app.stage = FakeANCStage(trace)
        app.rotator_stage = FakeRotator(trace)

        class FrozenDateTime:
            @classmethod
            def now(cls):
                from datetime import datetime
                return datetime(2026, 8, 25, 12, 0, 0, 123456)

        mapping_config = {
            "dwell_ms": 0,
            "return_to_origin": False,
            "output_dir": str(self.root / "collision-mapping"),
            "pmt": PMTSettings(enabled=False),
            "use_simulated_pmt": True,
            "pre_read_settle_s": 0.0,
        }
        angle_config = app.angle_scan_config()
        angle_config.update({"output_dir": str(self.root / "collision-angle"), "angles": [0.0], "return_to_start": False})
        pmt = FakePMTCounter(trace, [(1.0, [1.0]), (2.0, [2.0])])
        with (
            mock.patch.object(main, "datetime", FrozenDateTime),
            mock.patch.object(main, "PMTCounter", pmt),
            mock.patch.object(main, "interruptible_sleep"),
        ):
            app._scan_worker(mapping_config, [(0, 0, 0.0, 0.0)])
            app._scan_worker(mapping_config, [(0, 0, 0.0, 0.0)])
            app._run_single_angle_scan({"x_um": 0.0, "y_um": 0.0}, angle_config)
            app._run_single_angle_scan({"x_um": 0.0, "y_um": 0.0}, angle_config)
        self.assertEqual(len(list((self.root / "collision-mapping").glob("*.csv"))), 2)
        self.assertEqual(len(list((self.root / "collision-angle" / "angle_scans").glob("*.csv"))), 2)

    def test_angle_failure_keeps_partial_csv_and_does_not_return_or_disable_anc(self):
        trace = []
        app = self.make_app()
        stage = FakeANCStage(trace)
        rotator = FakeRotator(trace)
        app.stage = stage
        app.rotator_stage = rotator
        app.anc_pre_read_settle_s.set(0)
        app.angle_start_deg.set(0)
        app.angle_stop_deg.set(10)
        app.angle_step_deg.set(10)
        app.angle_settle_s.set(0)
        app.angle_return_to_start.set(True)
        app.selected_point = {"x_um": 2.0, "y_um": 3.0}
        pmt = FakePMTCounter(trace, [(7.0, [7.0]), RuntimeError("simulated PMT failure")])
        with (
            mock.patch.object(app, "_start_worker", side_effect=self.run_worker_inline),
            mock.patch.object(app, "save_config"),
            mock.patch.object(app, "_ensure_no_external_mt_tool", return_value=True),
            mock.patch.object(app, "_recover_rotator_if_needed", return_value=True),
            mock.patch.object(main, "PMTCounter", pmt),
            mock.patch.object(main, "interruptible_sleep"),
        ):
            app.start_angle_scan()
        self.assertNotIn(("anc_origin",), trace)
        self.assertNotIn(("anc_stop",), trace)
        self.assertTrue(stage.outputs_enabled)
        self.assertTrue(app.latest_angle_csv.exists())
        _, rows = self.read_csv(app.latest_angle_csv)
        self.assertEqual(len(rows), 1)


if __name__ == "__main__":
    unittest.main()
