#!/usr/bin/env python3
"""Minimal ROS 2 terminal keyboard controller for the FastSwarmSim MAVROS Lite API."""

import argparse
import select
import sys
import termios
import tty
from threading import Lock

import rclpy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


DEFAULT_SETPOINT_TOPIC = "/mavros/setpoint_raw/local"
DEFAULT_STATE_TOPIC = "/mavros/state"
DEFAULT_ARMING_SERVICE = "/mavros/cmd/arming"
DEFAULT_MODE_SERVICE = "/mavros/set_mode"


def mavros_interfaces(namespace):
    """Return MAVROS interface names, using absolute single-vehicle defaults."""
    if namespace == "/mavros":
        return (
            DEFAULT_SETPOINT_TOPIC,
            DEFAULT_STATE_TOPIC,
            DEFAULT_ARMING_SERVICE,
            DEFAULT_MODE_SERVICE,
        )

    namespace = "/" + namespace.strip("/")
    return (
        f"{namespace}/setpoint_raw/local",
        f"{namespace}/state",
        f"{namespace}/cmd/arming",
        f"{namespace}/set_mode",
    )


class KeyboardControl(Node):
    def __init__(self, args):
        super().__init__("keyboard_control_ros2")
        self.speed = args.speed
        self.yaw_speed = args.yaw_speed
        self.altitude = args.initial_altitude
        self.vx = self.vy = self.vz = self.yaw_rate = 0.0
        self.armed = False
        self.mode = "UNKNOWN"
        self.lock = Lock()
        (self.setpoint_topic, self.state_topic,
         self.arming_service, self.mode_service) = mavros_interfaces(args.mavros_namespace)
        self.setpoint_pub = self.create_publisher(PositionTarget, self.setpoint_topic, 10)
        self.state_sub = self.create_subscription(State, self.state_topic, self._state_cb, 10)
        self.arm_client = self.create_client(CommandBool, self.arming_service)
        self.mode_client = self.create_client(SetMode, self.mode_service)
        self.timer = self.create_timer(0.05, self._publish)

    def _state_cb(self, msg):
        self.armed, self.mode = msg.armed, msg.mode

    def _publish(self):
        with self.lock:
            self.altitude += self.vz * 0.05
            msg = self._build_setpoint()
        self.setpoint_pub.publish(msg)

    def _build_setpoint(self):
        """Build a MAVROS local setpoint in the ROS ENU convention.

        MAVROS Lite converts this ROS-side representation to MAVLink LOCAL_NED.
        The mask is the exact ROS 1 controller mask. It selects x/y velocity,
        z position, yaw rate, and an explicit zero z velocity.
        """
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.velocity.x, msg.velocity.y = self.vx, self.vy
        msg.position.z = self.altitude
        msg.yaw_rate = self.yaw_rate
        msg.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                         PositionTarget.IGNORE_AFX |
                         PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                         PositionTarget.IGNORE_YAW)
        return msg

    def arm(self, value=True):
        if not self.arm_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warning(f"{self.arming_service} is unavailable")
            return
        request = CommandBool.Request(); request.value = value
        self.arm_client.call_async(request)

    def offboard(self):
        if not self.armed:
            self.get_logger().warning("refusing OFFBOARD request while the vehicle is disarmed; arm first with t")
            return
        if not self.mode_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warning(f"{self.mode_service} is unavailable")
            return
        request = SetMode.Request(); request.custom_mode = "OFFBOARD"
        self.mode_client.call_async(request)

    def set_key(self, key):
        with self.lock:
            if key == "w": self.vx, self.vy = self.speed, 0.0
            elif key == "s": self.vx, self.vy = -self.speed, 0.0
            elif key == "a": self.vx, self.vy = 0.0, self.speed
            elif key == "d": self.vx, self.vy = 0.0, -self.speed
            elif key == "q": self.vz, self.yaw_rate = self.speed, 0.0
            elif key == "e": self.vz, self.yaw_rate = -self.speed, 0.0
            elif key == "z": self.vz, self.yaw_rate = 0.0, self.yaw_speed
            elif key == "c": self.vz, self.yaw_rate = 0.0, -self.yaw_speed
            elif key in (" ", "r"): self.vx = self.vy = self.vz = self.yaw_rate = 0.0

    def stop(self):
        with self.lock:
            self.vx = self.vy = self.vz = self.yaw_rate = 0.0

    def safe_shutdown(self):
        """Send a neutral setpoint and request disarm before destroying the node."""
        self.stop()
        if not rclpy.ok():
            return
        self._publish()
        if not self.arm_client.wait_for_service(timeout_sec=0.2):
            return
        request = CommandBool.Request()
        request.value = False
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.done() and future.exception() is None:
            response = future.result()
            if not response.success:
                self.get_logger().warning("disarm request was rejected")
        else:
            self.get_logger().warning("disarm request did not complete before shutdown")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="WASD/QE/ZC keyboard control for fss_px4_sim MAVROS Lite")
    parser.add_argument("--speed", type=float, default=1.0, help="translation speed in m/s (default: 1.0)")
    parser.add_argument("--yaw-speed", type=float, default=1.57, help="yaw speed in rad/s (default: 1.57)")
    parser.add_argument("--initial-altitude", type=float, default=1.0, help="initial z setpoint in metres")
    parser.add_argument(
        "--mavros-namespace",
        default="/mavros",
        help="MAVROS namespace for a selected vehicle (default: /mavros)",
    )
    return parser.parse_known_args(argv)


def main():
    args, ros_args = parse_args()
    # Keep the context alive through Ctrl-C so finally can publish and disarm safely.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    old = None
    try:
        node = KeyboardControl(args)
        old = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
        if old:
            tty.setcbreak(sys.stdin.fileno())
            print("fss_px4_sim keyboard control | w/a/s/d move, q/e altitude, z/c yaw | +/- speed | t arm, o OFFBOARD | space stop, x disarm, Ctrl-C exit", flush=True)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                if not ready: continue
                key = sys.stdin.read(1).lower()
                if key == "\x03": break
                if key == "t": node.arm(True)
                elif key == "x": node.stop(); node.arm(False)
                elif key == "o": node.offboard()
                elif key == "+": node.speed = min(10.0, node.speed + 0.1)
                elif key == "-": node.speed = max(0.0, node.speed - 0.1)
                else: node.set_key(key)
                print("\rmode={:<10} armed={} speed={:.1f} vx={:+.1f} vy={:+.1f} vz={:+.1f} yaw={:+.2f}   ".format(node.mode, node.armed, node.speed, node.vx, node.vy, node.vz, node.yaw_rate), end="", flush=True)
        else:
            node.get_logger().warning("stdin is not a tty; publishing neutral setpoints only")
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.safe_shutdown()
        if old: termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        if node:
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
