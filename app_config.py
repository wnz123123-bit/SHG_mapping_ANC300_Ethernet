"""Configuration loading, migration, and safe persistence for SHG mapping."""

from __future__ import annotations

import copy
import json
import os
import tempfile
from pathlib import Path
from typing import Any


_KNOWN_TOP_LEVEL = ("simulation_mode", "anc300", "scan", "pmt", "rotator", "angle_scan")
_MT_TRANSPORT_KEYS = ("dll_path", "connection_type", "com_port", "ip_address", "ip_port")


def default_config(app_dir: Path | None = None) -> dict:
    """Return a complete, independent default configuration dictionary."""
    base_dir = Path(app_dir) if app_dir is not None else Path(__file__).resolve().parent
    return {
        "simulation_mode": False,
        "anc300": {
            "host": "",
            "port": 7230,
            "password": "123456",
            "timeout_s": 3.0,
            "x_axis": 1,
            "y_axis": 2,
            "voltage_min_v": 0.0,
            "voltage_max_v": 150.0,
            "x_um_per_v": 0.2,
            "y_um_per_v": 0.2,
            "x_direction": 1,
            "y_direction": 1,
            "calibration_source": "nominal_4K",
            "max_ramp_step_v": 1.0,
            "pre_read_settle_s": 0.2,
            "hardware_profile_confirmed": False,
        },
        "scan": {
            "x_start_um": 0.0,
            "x_end_um": 24.0,
            "y_start_um": 0.0,
            "y_end_um": 24.0,
            "step_um": 1.6,
            "dwell_ms": 1000,
            "serpentine": False,
            "return_to_origin": True,
            "output_dir": str(base_dir / "results"),
        },
        "pmt": {
            "enabled": True,
            "dll_path": str(base_dir / "vendor_dlls" / "C8855-01api.dll"),
            "gate_time_ms": 200.0,
            "samples_to_average": 1,
            "sample_extra_wait_s": 0.05,
            "transfer_mode": 2,
            "trigger_mode": 0,
            "trigger_edge": 0,
            "simulate_value": 0.0,
            "simulate_noise": 0.0,
        },
        "rotator": {
            "dll_path": str(base_dir / "SDK" / "二次开发" / "4.6" / "WIN64" / "MT_API.dll"),
            "connection_type": "USB",
            "com_port": "COM1",
            "ip_address": "127.0.0.1",
            "ip_port": 8888,
            "usb_device_index": 0,
            "usb_serial": "",
            "axis_display": 4,
            "steps_per_degree": 3200.0,
            "direction_sign": -1,
            "target_angle_deg": 0.0,
            "acc": 15000,
            "dec": 15000,
            "max_v": 15000,
            "start_v": 0,
            "move_timeout_s": 30.0,
        },
        "angle_scan": {
            "start_angle_deg": 0.0,
            "stop_angle_deg": 180.0,
            "step_deg": 3.0,
            "settle_s": 0.2,
            "return_to_start": True,
        },
    }


def merge_dict(target: dict, source: dict) -> dict:
    """Deeply merge source into target and return the updated target."""
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            merge_dict(target[key], value)
        else:
            target[key] = copy.deepcopy(value)
    return target


def migrate_config(loaded: dict) -> tuple[dict, bool]:
    """Remove a legacy MT stage section, retaining only its transport values."""
    migrated = copy.deepcopy(loaded)
    if "stage" not in migrated:
        return migrated, False
    legacy_stage = migrated.pop("stage")

    rotator = migrated.get("rotator")
    if not isinstance(rotator, dict):
        rotator = {}
        migrated["rotator"] = rotator
    if isinstance(legacy_stage, dict):
        for key in _MT_TRANSPORT_KEYS:
            if key not in rotator and key in legacy_stage:
                rotator[key] = copy.deepcopy(legacy_stage[key])
    return migrated, True


def ensure_legacy_backup(path: Path) -> Path | None:
    """Create the one-time byte-exact sibling legacy backup, if absent."""
    path = Path(path)
    backup = path.with_name("config.pre_anc300_migration.json")
    if backup.exists():
        return None
    source_bytes = path.read_bytes()
    created_backup = False
    try:
        with backup.open("xb") as handle:
            created_backup = True
            handle.write(source_bytes)
            handle.flush()
            os.fsync(handle.fileno())
    except FileExistsError:
        return None
    except Exception:
        if created_backup:
            try:
                backup.unlink(missing_ok=True)
            except OSError:
                pass
        raise
    return backup


def save_config(path: Path, config: dict) -> None:
    """Atomically save configuration as readable UTF-8 JSON."""
    path = Path(path)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temporary_name = handle.name
            json.dump(config, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, path)
        temporary_name = None
    finally:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)


def _resolve_config_paths(config: dict, base_dir: Path) -> None:
    for section, key in (("rotator", "dll_path"), ("pmt", "dll_path"), ("scan", "output_dir")):
        value = config.get(section, {}).get(key)
        if isinstance(value, str) and value:
            resolved = Path(value)
            if not resolved.is_absolute():
                config[section][key] = str(base_dir / resolved)


def _canonical_for_save(config: dict) -> dict:
    schema = default_config()
    canonical: dict = {}
    for key in _KNOWN_TOP_LEVEL:
        if isinstance(schema[key], dict):
            source = config.get(key, {})
            canonical[key] = {
                nested_key: copy.deepcopy(source[nested_key])
                for nested_key in schema[key]
                if isinstance(source, dict) and nested_key in source
            }
        elif key in config:
            canonical[key] = copy.deepcopy(config[key])
    return canonical


def load_config(path: Path) -> dict:
    """Load, migrate, complete, and path-resolve an application configuration."""
    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        loaded: Any = json.load(handle)
    if not isinstance(loaded, dict):
        raise ValueError("configuration root must be a JSON object")

    migrated, did_migrate = migrate_config(loaded)
    config = default_config(path.parent.resolve())
    merge_dict(config, migrated)
    _resolve_config_paths(config, path.parent.resolve())

    if did_migrate:
        ensure_legacy_backup(path)
        save_config(path, _canonical_for_save(config))
    return config
