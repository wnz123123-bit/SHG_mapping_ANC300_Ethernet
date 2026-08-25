"""Direct fake-DLL tests for the MT HWP wrapper safety boundary."""

from __future__ import annotations

import threading
import unittest
from pathlib import Path

from stage_controller import MTStage, StageSettings, _MTDllSession


class FakeFunction:
    def __init__(self, name, history, action=None):
        self.name = name
        self.history = history
        self.action = action

    def __call__(self, *args):
        values = tuple(getattr(arg, "value", arg) for arg in args)
        self.history.append((self.name, *values))
        if self.action is not None:
            self.action()
        return 0


class FakeMTDll:
    def __init__(self, names):
        self.history = []
        for name in names:
            setattr(self, name, FakeFunction(name, self.history))


class MTStageDllSafetyTests(unittest.TestCase):
    def stage_with_dll(self, fake, axis=3):
        stage = MTStage(StageSettings(dll_path="fake.dll", x_axis=axis, y_axis=axis))
        stage.dll = fake
        stage.connected = True
        stage._transport_open = True
        return stage

    def test_disconnect_closes_transport_without_any_halt_or_motion_command(self):
        """Ordinary disconnect issuing Halt All or another motion command must fail."""
        fake = FakeMTDll(
            [
                "MT_Close_UART",
                "MT_Close_Net",
                "MT_Close_USB",
                "MT_Set_Axis_Halt",
                "MT_Set_Axis_Halt_All",
            ]
        )
        stage = self.stage_with_dll(fake)
        stage.disconnect()
        names = [item[0] for item in fake.history]
        self.assertEqual(names, ["MT_Close_UART", "MT_Close_Net", "MT_Close_USB"])
        self.assertFalse(stage.connected)

    def test_stop_halts_only_the_configured_hwp_axis(self):
        """A controller-wide halt or a halt of any non-HWP axis must fail."""
        fake = FakeMTDll(["MT_Set_Axis_Halt", "MT_Set_Axis_Halt_All"])
        stage = self.stage_with_dll(fake, axis=3)
        stage.stop()
        self.assertEqual(fake.history, [("MT_Set_Axis_Halt", 3)])

    def test_each_vendor_call_uses_the_shared_session_lock(self):
        """Concurrent entry into one process-global MT DLL must fail this test."""
        first_entered = threading.Event()
        second_entered = threading.Event()
        release_first = threading.Event()
        fake = FakeMTDll([])
        fake.MT_First = FakeFunction(
            "MT_First",
            fake.history,
            action=lambda: (first_entered.set(), release_first.wait(1.0)),
        )
        fake.MT_Second = FakeFunction("MT_Second", fake.history, action=second_entered.set)
        session = _MTDllSession(Path("fake.dll"))
        session.dll = fake
        stages = [self.stage_with_dll(fake), self.stage_with_dll(fake)]
        for stage in stages:
            stage._session = session

        first = threading.Thread(target=stages[0]._call, args=("MT_First",))
        second = threading.Thread(target=stages[1]._call, args=("MT_Second",))
        first.start()
        self.assertTrue(first_entered.wait(0.5))
        second.start()
        self.assertFalse(second_entered.wait(0.1))
        release_first.set()
        first.join(1.0)
        second.join(1.0)
        self.assertTrue(second_entered.is_set())
        self.assertEqual([item[0] for item in fake.history], ["MT_First", "MT_Second"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
