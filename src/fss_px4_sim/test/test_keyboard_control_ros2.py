"""Behavioral contract tests for the ROS 2 terminal keyboard controller."""

import importlib.util
from pathlib import Path
import unittest

import rclpy
from mavros_msgs.msg import PositionTarget


SCRIPT = Path(__file__).parents[1] / "scripts" / "keyboard_control_ros2.py"
SPEC = importlib.util.spec_from_file_location("keyboard_control_ros2", SCRIPT)
keyboard_control_ros2 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(keyboard_control_ros2)


class KeyboardControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        args, _ = keyboard_control_ros2.parse_args([])
        self.node = keyboard_control_ros2.KeyboardControl(args)

    def tearDown(self):
        self.node.destroy_node()

    def test_keys_match_ros1_velocity_mapping(self):
        cases = {
            "w": (1.0, 0.0, 0.0, 0.0),
            "s": (-1.0, 0.0, 0.0, 0.0),
            "a": (0.0, 1.0, 0.0, 0.0),
            "d": (0.0, -1.0, 0.0, 0.0),
            "q": (0.0, 0.0, 1.0, 0.0),
            "e": (0.0, 0.0, -1.0, 0.0),
            "z": (0.0, 0.0, 0.0, 1.57),
            "c": (0.0, 0.0, 0.0, -1.57),
        }
        for key, expected in cases.items():
            self.node.stop()
            self.node.set_key(key)
            self.assertEqual((self.node.vx, self.node.vy, self.node.vz, self.node.yaw_rate), expected)

    def test_setpoint_selects_xy_velocity_z_position_and_yaw_rate(self):
        self.node.vx, self.node.vy = 2.0, -3.0
        self.node.altitude = 4.0
        self.node.yaw_rate = 0.5

        msg = self.node._build_setpoint()

        expected_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                         PositionTarget.IGNORE_AFX |
                         PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                         PositionTarget.IGNORE_YAW)
        self.assertEqual(msg.coordinate_frame, PositionTarget.FRAME_LOCAL_NED)
        self.assertEqual(msg.type_mask, expected_mask)
        self.assertEqual(msg.type_mask, 0b010111000011)
        self.assertEqual((msg.velocity.x, msg.velocity.y, msg.position.z, msg.yaw_rate),
                         (2.0, -3.0, 4.0, 0.5))

    def test_stop_zeros_all_commands(self):
        self.node.vx = self.node.vy = self.node.vz = self.node.yaw_rate = 1.0
        self.node.stop()
        self.assertEqual((self.node.vx, self.node.vy, self.node.vz, self.node.yaw_rate),
                         (0.0, 0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
