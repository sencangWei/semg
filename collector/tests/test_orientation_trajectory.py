import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from customer_collector.imu_client import IMUClient
from customer_collector.orientation_trajectory import build_orientation_trajectory


def axis_quaternion(axis: str, degrees: float) -> np.ndarray:
    half = np.radians(degrees) / 2.0
    q = np.array([np.cos(half), 0.0, 0.0, 0.0], dtype=np.float64)
    q["xyz".index(axis) + 1] = np.sin(half)
    return q


def trajectory_data(quaternions: np.ndarray, game: bool = False) -> dict:
    n = len(quaternions)
    data = {
        "quat_wxyz": np.asarray(quaternions, dtype=np.float64),
        "timestamps": np.arange(n, dtype=np.float64) * 0.01,
        "gyro_xyz": np.zeros((n, 3), dtype=np.float64),
    }
    if game:
        data["quat_game_wxyz"] = np.asarray(quaternions, dtype=np.float64)
        data["quat_game_accuracy"] = np.full(n, 3, dtype=np.uint8)
    return data


class OrientationTrajectoryTests(unittest.TestCase):
    def test_x_rotation_becomes_positive_roll(self):
        q = np.vstack([
            np.tile([1.0, 0.0, 0.0, 0.0], (10, 1)),
            axis_quaternion("x", 90.0),
        ])
        result, _ = build_orientation_trajectory(
            trajectory_data(q), source="rotation", baseline_s=0.09)

        self.assertAlmostEqual(result["roll_x_deg"][-1], 90.0, places=6)
        self.assertAlmostEqual(result["pitch_y_deg"][-1], 0.0, places=6)
        self.assertAlmostEqual(result["yaw_z_deg"][-1], 0.0, places=6)

    def test_quaternion_sign_flips_do_not_create_motion(self):
        q = np.tile([1.0, 0.0, 0.0, 0.0], (20, 1))
        q[1::2] *= -1.0
        result, metadata = build_orientation_trajectory(
            trajectory_data(q), source="rotation", baseline_s=0.05)

        np.testing.assert_allclose(result["rotation_angle_deg"], 0.0, atol=1e-9)
        self.assertAlmostEqual(metadata["step_angle_max_deg"], 0.0, places=9)

    def test_right_handed_axis_map_rotates_sensor_x_to_output_y(self):
        q = np.vstack([
            np.tile([1.0, 0.0, 0.0, 0.0], (10, 1)),
            axis_quaternion("x", 30.0),
        ])
        result, _ = build_orientation_trajectory(
            trajectory_data(q), source="rotation", baseline_s=0.09,
            axis_map="-y,+x,+z")

        self.assertAlmostEqual(result["roll_x_deg"][-1], 0.0, places=6)
        self.assertAlmostEqual(result["pitch_y_deg"][-1], 30.0, places=6)
        self.assertAlmostEqual(result["yaw_z_deg"][-1], 0.0, places=6)

    def test_auto_prefers_valid_game_rotation_vector_even_when_identity(self):
        q = np.tile([1.0, 0.0, 0.0, 0.0], (20, 1))
        _, metadata = build_orientation_trajectory(
            trajectory_data(q, game=True), source="auto", baseline_s=0.05)

        self.assertEqual(metadata["source_quaternion"], "quat_game_wxyz")


class IMUClientTests(unittest.TestCase):
    def test_json_parser_keeps_both_rotation_vectors_and_accuracy(self):
        client = IMUClient("127.0.0.1", 8766, "test")
        frame = client._normalize_json(json.loads(
            '{"t":123,"q":[1,0,0,0],"q6":[0.5,0.5,0.5,0.5],'
            '"qa":2,"q6a":3,"a":[1,2,3],"g":[4,5,6]}'))

        self.assertEqual(frame["sensor_time_us"], 123)
        self.assertEqual(frame["quat"], [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(frame["quat_game"], [0.5, 0.5, 0.5, 0.5])
        self.assertEqual(frame["quat_accuracy"], 2)
        self.assertEqual(frame["quat_game_accuracy"], 3)

    def test_save_unwraps_esp32_micros_rollover(self):
        with tempfile.TemporaryDirectory() as tmp:
            client = IMUClient(
                "127.0.0.1", 8766, "rollover", position="left",
                output_dir=Path(tmp))
            client.start_wall = 1000.0
            client.stop_wall = 1000.01
            client.sync_base_ts = 1000.0
            client.timestamps = [1000.0, 1000.001]
            client.sensor_time_us = [4_294_967_000, 9_704]
            client.quat_wxyz = [[1.0, 0.0, 0.0, 0.0]] * 2
            client.quat_game_wxyz = [[1.0, 0.0, 0.0, 0.0]] * 2
            client.quat_accuracy = [3, 3]
            client.quat_game_accuracy = [3, 3]
            client.accel_xyz = [[0.0, 0.0, 0.0]] * 2
            client.gyro_xyz = [[0.0, 0.0, 0.0]] * 2
            client.mag_xyz = [[0.0, 0.0, 0.0]] * 2

            out_dir = client.save()
            with np.load(out_dir / "imu_left_rollover.npz") as saved:
                np.testing.assert_allclose(saved["sensor_elapsed_s"], [0.0, 0.01])
                np.testing.assert_allclose(
                    np.diff(saved["timestamps_sensor_clock"]), [0.01])
            orientation_path = out_dir / "imu_left_rollover_orientation.npz"
            self.assertTrue(orientation_path.exists())
            self.assertTrue((out_dir / "imu_left_rollover_orientation.csv").exists())
            orientation_meta = json.loads(
                (out_dir / "imu_left_rollover_orientation_meta.json").read_text())
            self.assertEqual(orientation_meta["source_quaternion"], "quat_game_wxyz")
            self.assertEqual(orientation_meta["time_source"], "timestamps_sensor_clock")
            self.assertEqual(orientation_meta["axis_map"], "+x,+y,+z")


if __name__ == "__main__":
    unittest.main()
