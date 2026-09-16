#!/usr/bin/env python3
import pathlib
import sys
import unittest


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import airsim_tunnel_scene as tunnel  # noqa: E402


class AirSimTunnelSceneTest(unittest.TestCase):
    def test_default_course_uses_many_small_obstacles(self):
        specs = tunnel.build_specs(30.0, 7.0, 4.5)
        obstacles = [item for item in specs if item["role"] == "obstacle"]
        self.assertEqual(len(specs), 29)
        self.assertEqual(len(obstacles), 24)
        self.assertLessEqual(
            max(max(item["size"]) for item in obstacles),
            0.9,
        )
        flight_height = 1.5
        world_to_map_z_offset = 0.7
        same_height = [
            item
            for item in obstacles
            if abs(
                -item["center_ned"][2]
                + world_to_map_z_offset
                - flight_height
            )
            < 1e-6
        ]
        intersects_flight_plane = [
            item
            for item in obstacles
            if abs(
                -item["center_ned"][2]
                + world_to_map_z_offset
                - flight_height
            )
            <= item["size"][2] * 0.5
        ]
        self.assertEqual(len(same_height), 6)
        self.assertGreaterEqual(len(intersects_flight_plane), 18)
        safety = tunnel.build_safety_model(
            specs,
            width=7.0,
            safety_margin=0.75,
            fixed_flight_height=1.5,
            world_to_map_z_offset=0.7,
        )
        self.assertAlmostEqual(safety["planner_surface_clearance_m"], 0.75)
        self.assertGreaterEqual(
            safety["minimum_actual_safe_center_corridor_m"],
            safety["minimum_required_safe_center_corridor_m"],
        )
        self.assertGreaterEqual(safety["obstacle_edge_min_m"], 0.5)
        self.assertGreaterEqual(
            safety["minimum_actual_longitudinal_turning_gap_m"],
            safety["minimum_required_longitudinal_turning_gap_m"],
        )

    def test_default_safety_geofence_keeps_wall_clearance(self):
        self.assertEqual(
            tunnel.safety_geofence(30.0, 7.0, 0.75),
            {
                "min_x": -2.75,
                "max_x": 2.75,
                "min_y": -3.0,
                "max_y": 25.0,
            },
        )


if __name__ == "__main__":
    unittest.main()
