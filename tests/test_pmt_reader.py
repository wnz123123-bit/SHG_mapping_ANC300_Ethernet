"""Direct fake-DLL tests for the C8855 PMT lifecycle and cleanup paths."""

from __future__ import annotations

import ctypes
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from pmt_reader import PMTCounter, PMTError, PMTSettings, validate_pmt_settings


class FakeFunction:
    def __init__(self, name, fake):
        self.name = name
        self.fake = fake
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        self.fake.history.append(self.name)
        if self.name == "C8855ReadData" and self.fake.results.get(self.name, True):
            args[1][0] = self.fake.count_value
            ctypes.cast(args[2], ctypes.POINTER(ctypes.c_ubyte))[0] = self.fake.result_state
        if self.name == "C8855Open":
            return self.fake.results.get(self.name, 1234)
        return self.fake.results.get(self.name, True)


class FakePMTDll:
    def __init__(self, **results):
        self.history = []
        self.results = results
        self.count_value = 321
        self.result_state = 1
        for name in (
            "C8855Open",
            "C8855Close",
            "C8855Reset",
            "C8855Setup",
            "C8855CountStart",
            "C8855CountStop",
            "C8855ReadData",
        ):
            setattr(self, name, FakeFunction(name, self))


class PMTCounterDllTests(unittest.TestCase):
    def settings(self, dll_path):
        return PMTSettings(
            enabled=True,
            dll_path=str(dll_path),
            gate_time_ms=0.05,
            samples_to_average=2,
            sample_extra_wait_s=0.0,
        )

    def test_validation_rejects_coercible_noninteger_counts_and_modes(self):
        """Fractional/Boolean discrete values must not be silently coerced with int()."""
        settings = PMTSettings(enabled=True, gate_time_ms=0.05)
        invalid = (
            replace(settings, samples_to_average=1.5),
            replace(settings, transfer_mode=1.5),
            replace(settings, trigger_mode=True),
            replace(settings, trigger_edge=0.5),
        )
        for candidate in invalid:
            with self.subTest(candidate=candidate):
                with self.assertRaises(PMTError):
                    validate_pmt_settings(candidate, simulate=True, require_enabled=True)

    def test_fake_dll_open_setup_read_stop_and_close_success(self):
        """Breaking the real PMT wrapper lifecycle or data extraction must fail."""
        fake = FakePMTDll()
        with tempfile.TemporaryDirectory() as tmp:
            dll_path = Path(tmp) / "C8855-01api.dll"
            dll_path.touch()
            with patch("pmt_reader.ctypes.CDLL", return_value=fake):
                with PMTCounter(self.settings(dll_path)) as counter:
                    average, samples = counter.read_average()
        self.assertEqual(average, 321.0)
        self.assertEqual(samples, [321.0, 321.0])
        self.assertEqual(fake.history[0], "C8855Open")
        self.assertIn("C8855Setup", fake.history)
        self.assertEqual(fake.history.count("C8855ReadData"), 2)
        self.assertIn("C8855CountStop", fake.history)
        self.assertEqual(fake.history[-1], "C8855Close")

    def test_setup_failure_closes_the_open_handle(self):
        """Leaking an opened PMT handle when context entry setup fails must fail."""
        fake = FakePMTDll(C8855Setup=False)
        with tempfile.TemporaryDirectory() as tmp:
            dll_path = Path(tmp) / "C8855-01api.dll"
            dll_path.touch()
            counter = PMTCounter(self.settings(dll_path))
            with patch("pmt_reader.ctypes.CDLL", return_value=fake):
                with self.assertRaises(PMTError):
                    counter.__enter__()
        self.assertIn("C8855Close", fake.history)
        self.assertFalse(counter._opened)
        self.assertIsNone(counter.handle)

    def test_read_and_stop_failures_still_close_the_device(self):
        """Read/stop failures masking cleanup or leaving the handle open must fail."""
        for failure in ("C8855ReadData", "C8855CountStop"):
            with self.subTest(failure=failure):
                fake = FakePMTDll(**{failure: False})
                with tempfile.TemporaryDirectory() as tmp:
                    dll_path = Path(tmp) / "C8855-01api.dll"
                    dll_path.touch()
                    with patch("pmt_reader.ctypes.CDLL", return_value=fake):
                        with self.assertRaises(PMTError):
                            with PMTCounter(self.settings(dll_path)) as counter:
                                counter.read_average()
                self.assertIn("C8855CountStop", fake.history)
                self.assertEqual(fake.history[-1], "C8855Close")

    def test_open_and_setup_failures_are_reported_without_real_dll_loading(self):
        """Accepting a null handle or failed setup result must fail explicitly."""
        with tempfile.TemporaryDirectory() as tmp:
            dll_path = Path(tmp) / "C8855-01api.dll"
            dll_path.touch()
            for results in ({"C8855Open": 0}, {"C8855Setup": False}):
                with self.subTest(results=results):
                    fake = FakePMTDll(**results)
                    counter = PMTCounter(self.settings(dll_path))
                    with patch("pmt_reader.ctypes.CDLL", return_value=fake):
                        with self.assertRaises(PMTError):
                            counter.__enter__()


if __name__ == "__main__":
    unittest.main(verbosity=2)
