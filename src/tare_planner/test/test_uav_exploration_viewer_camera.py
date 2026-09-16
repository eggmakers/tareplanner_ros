#!/usr/bin/env python3
import pathlib
import sys
import unittest


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from uav_exploration_viewer import AirSimViewCamera  # noqa: E402


class AirSimViewCameraTest(unittest.TestCase):
    def test_clamps_browser_values_to_observation_only_limits(self):
        current = {
            "orbit_degrees": 0.0,
            "distance": 3.0,
            "height": 2.0,
            "pitch_degrees": -11.5,
            "fov_degrees": 110.0,
        }
        result = AirSimViewCamera.clamp_values(
            {
                "orbit_degrees": 900.0,
                "distance": -1.0,
                "height": 20.0,
                "pitch_degrees": -90.0,
                "fov_degrees": 160.0,
            },
            current,
        )
        self.assertEqual(result["orbit_degrees"], 180.0)
        self.assertEqual(result["distance"], 1.0)
        self.assertEqual(result["height"], 8.0)
        self.assertEqual(result["pitch_degrees"], -45.0)
        self.assertEqual(result["fov_degrees"], 130.0)

    def test_rejects_non_finite_camera_values(self):
        with self.assertRaises(ValueError):
            AirSimViewCamera.clamp_values(
                {"distance": float("nan")}, {"distance": 3.0}
            )


if __name__ == "__main__":
    unittest.main()
