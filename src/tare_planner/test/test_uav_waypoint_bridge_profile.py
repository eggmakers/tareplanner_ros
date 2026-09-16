#!/usr/bin/env python3
import math
import pathlib
import sys
import threading
import unittest
from types import SimpleNamespace


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from uav_waypoint_bridge import UavWaypointBridge  # noqa: E402


def make_profile(target):
    profile = UavWaypointBridge.__new__(UavWaypointBridge)
    profile.target_position = list(target)
    profile.command_position = [0.0, 0.0, 0.0]
    profile.command_velocity = [0.0, 0.0, 0.0]
    profile.command_acceleration = [0.0, 0.0, 0.0]
    profile.max_xy_speed = 0.35
    profile.max_z_speed = 0.35
    profile.max_xy_accel = 0.20
    profile.max_z_accel = 0.25
    profile.max_xy_jerk = 0.6
    profile.max_z_jerk = 0.5
    profile.max_yaw_rate = 0.35
    profile.current_yaw = 0.0
    profile.command_yaw = 0.0
    profile.target_yaw = 0.0
    profile.yaw_from_motion = True
    profile.heading_alignment_enabled = True
    profile.heading_alignment_tolerance = 0.26
    profile.motion_yaw_min_distance = 0.25
    profile.xy_arrival_radius = 0.08
    profile.z_arrival_radius = 0.04
    profile.lock = threading.Lock()
    profile.current_position = [0.0, 0.0, 0.0]
    profile.manual_override_latched = False
    profile.rc_override_channel = 5
    profile.rc_override_pwm = 1900
    profile.rc_override_debounce_seconds = 0.1
    profile.rc_override_high_since = None
    return profile


class WaypointProfileTest(unittest.TestCase):
    def step_profile(self, profile, steps=1200, dt=0.05):
        maximum_speed = 0.0
        maximum_acceleration = 0.0
        for _ in range(steps):
            profile.update_horizontal_profile(dt)
            profile.update_vertical_profile(dt)
            maximum_speed = max(
                maximum_speed,
                math.hypot(profile.command_velocity[0], profile.command_velocity[1]),
            )
            maximum_acceleration = max(
                maximum_acceleration,
                math.hypot(
                    profile.command_acceleration[0],
                    profile.command_acceleration[1],
                ),
            )
        self.assertLessEqual(maximum_speed, profile.max_xy_speed + 1e-6)
        self.assertLessEqual(maximum_acceleration, profile.max_xy_accel + 1e-6)

    def test_long_step_converges_within_limits(self):
        profile = make_profile((10.0, 0.0, 1.5))
        self.step_profile(profile)
        self.assertAlmostEqual(profile.command_position[0], 10.0, places=2)
        self.assertAlmostEqual(profile.command_position[1], 0.0, places=2)
        self.assertAlmostEqual(profile.command_position[2], 1.5, places=2)

    def test_retarget_reverses_without_exceeding_limits(self):
        profile = make_profile((8.0, 2.0, 1.5))
        self.step_profile(profile, steps=160)
        profile.target_position = [-3.0, -1.0, 1.5]
        self.step_profile(profile)
        self.assertAlmostEqual(profile.command_position[0], -3.0, places=2)
        self.assertAlmostEqual(profile.command_position[1], -1.0, places=2)
        self.assertAlmostEqual(profile.command_position[2], 1.5, places=2)

    def test_command_geofence_clamps_horizontal_target(self):
        profile = make_profile((0.0, 0.0, 1.5))
        profile.geofence_enabled = True
        profile.geofence_min_x = -3.25
        profile.geofence_max_x = 3.25
        profile.geofence_min_y = -3.0
        profile.geofence_max_y = 31.0
        target, changed = profile.clamp_to_geofence([5.2, -5.3, 1.5])
        self.assertTrue(changed)
        self.assertEqual(target, [3.25, -3.0, 1.5])

    def test_command_geofence_leaves_safe_target_unchanged(self):
        profile = make_profile((0.0, 0.0, 1.5))
        profile.geofence_enabled = False
        target, changed = profile.clamp_to_geofence([5.2, -5.3, 1.5])
        self.assertFalse(changed)
        self.assertEqual(target, [5.2, -5.3, 1.5])

    def test_takeoff_xy_hold_does_not_follow_exploration_target(self):
        profile = make_profile((5.0, 4.0, 1.5))
        for _ in range(100):
            profile.update_horizontal_profile(0.05, target_xy=(0.0, 0.0))
            profile.update_vertical_profile(0.05)
        self.assertAlmostEqual(profile.command_position[0], 0.0, places=6)
        self.assertAlmostEqual(profile.command_position[1], 0.0, places=6)
        self.assertGreater(profile.command_position[2], 0.5)

    def test_relative_fixed_height_resolves_from_first_odometry_height(self):
        profile = make_profile((0.0, 0.0, 1.0))
        profile.fixed_z = 1.0
        profile.fixed_z_relative_to_start = True
        profile.initial_z = -0.063
        self.assertAlmostEqual(profile.resolved_fixed_z(), 0.937, places=6)

    def test_absolute_fixed_height_is_not_offset(self):
        profile = make_profile((0.0, 0.0, 1.0))
        profile.fixed_z = 1.0
        profile.fixed_z_relative_to_start = False
        profile.initial_z = -0.063
        self.assertAlmostEqual(profile.resolved_fixed_z(), 1.0, places=6)

    def test_turns_nose_before_horizontal_translation(self):
        profile = make_profile((0.0, 5.0, 1.5))
        profile.update_smoothed_exploration_profile(0.05)
        self.assertAlmostEqual(profile.command_position[0], 0.0, places=6)
        self.assertAlmostEqual(profile.command_position[1], 0.0, places=6)
        self.assertGreater(profile.command_yaw, 0.0)
        self.assertAlmostEqual(profile.target_yaw, math.pi * 0.5, places=6)

        profile.current_yaw = profile.target_yaw
        profile.command_yaw = profile.target_yaw
        profile.update_smoothed_exploration_profile(0.05)
        self.assertGreater(profile.command_position[1], 0.0)

    def test_motion_yaw_uses_command_to_target_direction(self):
        profile = make_profile((-4.0, 4.0, 1.5))
        self.assertTrue(profile.update_motion_yaw_target())
        self.assertAlmostEqual(profile.target_yaw, math.radians(135.0), places=6)

    def test_exploration_start_waits_for_stabilized_armed_vehicle(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        profile = make_profile((0.0, 0.0, 1.5))
        profile.start_exploration_after_takeoff = True
        profile.exploration_start_sent = False
        profile.vehicle_armed = True
        profile.takeoff_stabilized = False
        profile.exploration_start_pub = Publisher()

        profile.publish_exploration_start_if_ready()
        self.assertEqual(profile.exploration_start_pub.messages, [])

        profile.takeoff_stabilized = True
        profile.publish_exploration_start_if_ready()
        profile.publish_exploration_start_if_ready()
        self.assertEqual(len(profile.exploration_start_pub.messages), 1)
        self.assertTrue(profile.exploration_start_pub.messages[0].data)

    def test_rc_override_uses_one_based_channel_and_debounce(self):
        profile = make_profile((5.0, 0.0, 1.5))
        channels = [1500, 1500, 1500, 1500, 1950]
        self.assertFalse(profile.rc_override_triggered(channels, 1.00))
        self.assertFalse(profile.rc_override_triggered(channels, 1.05))
        self.assertTrue(profile.rc_override_triggered(channels, 1.11))

    def test_rc_override_debounce_resets_when_switch_returns_low(self):
        profile = make_profile((5.0, 0.0, 1.5))
        high = [1500, 1500, 1500, 1500, 1950]
        low = [1500, 1500, 1500, 1500, 1500]
        self.assertFalse(profile.rc_override_triggered(high, 1.00))
        self.assertFalse(profile.rc_override_triggered(low, 1.05))
        self.assertFalse(profile.rc_override_triggered(high, 1.10))
        self.assertFalse(profile.rc_override_triggered(high, 1.15))
        self.assertTrue(profile.rc_override_triggered(high, 1.21))

    def test_manual_override_blocks_exploration_start(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        profile = make_profile((0.0, 0.0, 1.5))
        profile.start_exploration_after_takeoff = True
        profile.exploration_start_sent = False
        profile.vehicle_armed = True
        profile.takeoff_stabilized = True
        profile.manual_override_latched = True
        profile.exploration_start_pub = Publisher()

        profile.publish_exploration_start_if_ready()
        self.assertEqual(profile.exploration_start_pub.messages, [])

    def test_manual_override_rejects_new_waypoints(self):
        profile = make_profile((1.0, 2.0, 1.5))
        profile.override_z = False
        profile.geofence_enabled = False
        profile.manual_override_latched = True
        waypoint = SimpleNamespace(
            point=SimpleNamespace(x=8.0, y=9.0, z=2.0)
        )

        profile.waypoint_callback(waypoint)
        self.assertEqual(profile.target_position, [1.0, 2.0, 1.5])

    def test_manual_override_latches_current_position(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        profile = make_profile((5.0, 3.0, 1.5))
        profile.current_position = [1.2, -0.4, 1.5]
        profile.current_yaw = 0.7
        profile.manual_override_pub = Publisher()
        profile.rc_override_mode = "POSCTL"

        self.assertTrue(profile.activate_manual_override())
        self.assertFalse(profile.activate_manual_override())
        self.assertEqual(profile.target_position, profile.current_position)
        self.assertEqual(profile.command_velocity, [0.0, 0.0, 0.0])
        self.assertEqual(len(profile.manual_override_pub.messages), 1)
        self.assertTrue(profile.manual_override_pub.messages[0].data)


if __name__ == "__main__":
    unittest.main()
