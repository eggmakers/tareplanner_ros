#!/usr/bin/env python3
import pathlib
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from uav_offboard_manager import UavOffboardManager  # noqa: E402


def make_manager(reassert=False):
    manager = UavOffboardManager.__new__(UavOffboardManager)
    manager.state = SimpleNamespace(mode="")
    manager.offboard_mode = "OFFBOARD"
    manager.offboard_achieved = False
    manager.suspended = False
    manager.reassert_offboard_after_mode_exit = reassert
    return manager


class OffboardModeGuardTest(unittest.TestCase):
    def test_initial_non_offboard_mode_does_not_suspend_startup(self):
        manager = make_manager()
        manager.state_callback(SimpleNamespace(mode="POSCTL"))
        self.assertFalse(manager.offboard_achieved)
        self.assertFalse(manager.suspended)

    def test_mode_takeover_after_offboard_suspends_reassertion(self):
        manager = make_manager()
        manager.state_callback(SimpleNamespace(mode="OFFBOARD"))
        with patch("uav_offboard_manager.rospy.logwarn"):
            manager.state_callback(SimpleNamespace(mode="AUTO.LAND"))
        self.assertTrue(manager.offboard_achieved)
        self.assertTrue(manager.suspended)

    def test_explicit_reassert_override_keeps_legacy_behavior(self):
        manager = make_manager(reassert=True)
        manager.state_callback(SimpleNamespace(mode="OFFBOARD"))
        manager.state_callback(SimpleNamespace(mode="POSCTL"))
        self.assertFalse(manager.suspended)


if __name__ == "__main__":
    unittest.main()
