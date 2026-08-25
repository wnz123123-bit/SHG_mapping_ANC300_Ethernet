import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from app_config import (
    default_config,
    ensure_legacy_backup,
    load_config,
    merge_dict,
    migrate_config,
    save_config,
)


class AppConfigTests(unittest.TestCase):
    def write_json(self, directory, name, value, *, indent=2):
        path = Path(directory) / name
        path.write_text(json.dumps(value, ensure_ascii=False, indent=indent), encoding="utf-8")
        return path

    def test_defaults_define_anc300_and_independent_rotator(self):
        """Catches a default that retains the removed MT X/Y mapping."""
        config = default_config(Path("C:/mapping"))

        self.assertFalse(config["simulation_mode"])
        self.assertEqual(config["anc300"]["port"], 7230)
        self.assertEqual(config["anc300"]["x_axis"], 1)
        self.assertEqual(config["anc300"]["y_axis"], 2)
        self.assertEqual(config["scan"]["x_end_um"], 24.0)
        self.assertFalse(config["scan"]["serpentine"])
        self.assertNotIn("return_speed_factor", config["scan"])
        self.assertEqual(config["rotator"]["connection_type"], "USB")
        self.assertEqual(config["rotator"]["usb_device_index"], 0)
        self.assertNotIn("x_axis", config["rotator"])
        self.assertNotIn("stage", config)
        self.assertEqual(
            Path(config["pmt"]["dll_path"]),
            Path("C:/mapping/vendor_dlls/C8855-01api.dll"),
        )

    def test_merge_dict_deeply_overrides_nested_values(self):
        """Catches replacement of a whole nested section during merge."""
        merged = merge_dict(
            {"anc300": {"host": "", "port": 7230}, "scan": {"step_um": 1.6}},
            {"anc300": {"host": "192.0.2.5"}},
        )

        self.assertEqual(merged, {"anc300": {"host": "192.0.2.5", "port": 7230}, "scan": {"step_um": 1.6}})

    def test_migration_uses_legacy_transport_only_when_rotator_lacks_it(self):
        """Catches stage transport overwriting a rotator-specific connection."""
        migrated, changed = migrate_config(
            {
                "stage": {
                    "dll_path": "legacy.dll",
                    "connection_type": "NET",
                    "com_port": "COM8",
                    "ip_address": "192.0.2.44",
                    "ip_port": 5000,
                    "x_axis": 7,
                    "y_axis": 8,
                    "x_steps_per_um": 3.0,
                },
                "rotator": {
                    "connection_type": "USB",
                    "usb_device_index": 4,
                    "usb_serial": "HWP-42",
                    "axis_display": 4,
                },
            }
        )

        self.assertTrue(changed)
        self.assertNotIn("stage", migrated)
        self.assertEqual(migrated["rotator"]["dll_path"], "legacy.dll")
        self.assertEqual(migrated["rotator"]["connection_type"], "USB")
        self.assertEqual(migrated["rotator"]["com_port"], "COM8")
        self.assertEqual(migrated["rotator"]["ip_address"], "192.0.2.44")
        self.assertEqual(migrated["rotator"]["ip_port"], 5000)
        self.assertEqual(migrated["rotator"]["usb_device_index"], 4)
        self.assertEqual(migrated["rotator"]["usb_serial"], "HWP-42")
        self.assertNotIn("x_axis", migrated["rotator"])
        self.assertNotIn("x_steps_per_um", migrated["rotator"])

    def test_legacy_load_creates_exact_backup_once_and_removes_stage(self):
        """Catches lossy backup creation or repeated backup overwrites."""
        legacy_text = '{\n  "stage": {"dll_path": "relative/mt.dll", "ip_address": "10.0.0.7"},\n  "scan": {"output_dir": "data"},\n  "password_note": "保留"\n}\n'
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "config.json"
            path.write_bytes(legacy_text.encode("utf-8"))

            loaded = load_config(path)
            backup = Path(temp) / "config.pre_anc300_migration.json"
            self.assertEqual(backup.read_bytes(), legacy_text.encode("utf-8"))
            self.assertNotIn("stage", loaded)
            self.assertEqual(loaded["password_note"], "保留")
            self.assertEqual(loaded["rotator"]["dll_path"], str(Path(temp) / "relative" / "mt.dll"))
            self.assertEqual(loaded["scan"]["output_dir"], str(Path(temp) / "data"))
            self.assertNotIn("stage", json.loads(path.read_text(encoding="utf-8")))

            backup.write_bytes(b"do not replace")
            ensure_legacy_backup(path)
            self.assertEqual(backup.read_bytes(), b"do not replace")

    def test_null_legacy_stage_creates_exact_backup_and_rewrites_canonical_config(self):
        """Treating stage null as canonical must fail migration and backup guarantees."""
        legacy_text = '{\n  "stage": null,\n  "scan": {"step_um": 2.5}\n}\n'
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "config.json"
            path.write_bytes(legacy_text.encode("utf-8"))

            loaded = load_config(path)
            backup = Path(temp) / "config.pre_anc300_migration.json"
            saved = json.loads(path.read_text(encoding="utf-8"))

            self.assertEqual(backup.read_bytes(), legacy_text.encode("utf-8"))
            self.assertNotIn("stage", loaded)
            self.assertNotIn("stage", saved)
            self.assertEqual(saved["scan"]["step_um"], 2.5)
            self.assertEqual(set(saved), {"simulation_mode", "anc300", "scan", "pmt", "rotator", "angle_scan"})

    def test_canonical_load_does_not_create_backup_and_resolves_relative_paths(self):
        """Catches backup creation for canonical files and un-resolved paths."""
        with tempfile.TemporaryDirectory() as temp:
            path = self.write_json(
                temp,
                "config.json",
                {
                    "rotator": {"dll_path": "SDK/MT_API.dll"},
                    "pmt": {"dll_path": "vendor_dlls/C8855-01api.dll"},
                    "scan": {"output_dir": "results"},
                },
            )

            loaded = load_config(path)

            self.assertFalse((Path(temp) / "config.pre_anc300_migration.json").exists())
            self.assertEqual(loaded["rotator"]["dll_path"], str(Path(temp) / "SDK" / "MT_API.dll"))
            self.assertEqual(loaded["pmt"]["dll_path"], str(Path(temp) / "vendor_dlls" / "C8855-01api.dll"))
            self.assertEqual(loaded["scan"]["output_dir"], str(Path(temp) / "results"))

    def test_atomic_save_round_trips_utf8_password(self):
        """Catches non-atomic/incorrect encoding saves that lose a password."""
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "config.json"
            config = default_config(Path(temp))
            config["anc300"]["password"] = "密码-123"

            save_config(path, config)

            self.assertFalse(list(Path(temp).glob(".config.json.*.tmp")))
            self.assertIn("密码-123", path.read_text(encoding="utf-8"))
            self.assertEqual(load_config(path)["anc300"]["password"], "密码-123")

    def test_load_drops_legacy_stage_mapping_from_saved_canonical_file(self):
        """Catches accidental persistence of old XY/Z mapping controls."""
        with tempfile.TemporaryDirectory() as temp:
            path = self.write_json(
                temp,
                "config.json",
                {
                    "stage": {"x_axis": 0, "y_axis": 1, "z_axis": 2, "manual_step_pulses": 1},
                    "rotator": {"usb_device_index": 2, "usb_serial": "keep"},
                },
            )

            load_config(path)
            saved = json.loads(path.read_text(encoding="utf-8"))

            self.assertNotIn("stage", saved)
            self.assertEqual(saved["rotator"]["usb_device_index"], 2)
            self.assertEqual(saved["rotator"]["usb_serial"], "keep")
            self.assertNotIn("x_axis", saved["rotator"])

    def test_migration_removes_return_speed_factor_from_saved_scan(self):
        """Catches retention of the obsolete scan return-speed setting."""
        with tempfile.TemporaryDirectory() as temp:
            path = self.write_json(
                temp,
                "config.json",
                {"stage": {}, "scan": {"return_speed_factor": 0.1, "step_um": 2.0}},
            )

            load_config(path)
            saved = json.loads(path.read_text(encoding="utf-8"))

            self.assertNotIn("return_speed_factor", saved["scan"])
            self.assertEqual(saved["scan"]["step_um"], 2.0)

    def test_backup_write_failure_removes_partial_backup_and_keeps_source(self):
        """Catches a failed backup leaving a file that blocks all later backups."""
        class PartiallyFailingBackup:
            def __init__(self, handle):
                self.handle = handle

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc_value, traceback):
                self.handle.close()

            def write(self, data):
                self.handle.write(data[:3])
                raise OSError("injected backup write failure")

        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "config.json"
            source_bytes = b'{\n  "stage": {}\n}\n'
            path.write_bytes(source_bytes)
            backup = Path(temp) / "config.pre_anc300_migration.json"
            original_open = Path.open

            def patched_open(file_path, *args, **kwargs):
                mode = args[0] if args else kwargs.get("mode", "r")
                handle = original_open(file_path, *args, **kwargs)
                if file_path == backup and mode == "xb":
                    return PartiallyFailingBackup(handle)
                return handle

            with mock.patch("app_config.Path.open", new=patched_open):
                with self.assertRaisesRegex(OSError, "injected backup write failure"):
                    ensure_legacy_backup(path)

            self.assertFalse(backup.exists())
            self.assertEqual(path.read_bytes(), source_bytes)


if __name__ == "__main__":
    unittest.main()
