#!/usr/bin/env python3
"""One-shot acceptance check for the deterministic Gazebo table scene."""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from table_plane_detector.msg import TablePlane


class Evaluator(Node):
    def __init__(self):
        super().__init__("table_plane_evaluator")
        self.expected_perpendicular = float(
            self.declare_parameter("expected_perpendicular_m", 1.0).value
        )
        self.expected_optical_axis = float(
            self.declare_parameter("expected_optical_axis_m", 1.15470053838).value
        )
        self.perpendicular_tolerance = float(
            self.declare_parameter("perpendicular_tolerance_m", 0.003).value
        )
        self.optical_tolerance = float(
            self.declare_parameter("optical_axis_tolerance_m", 0.005).value
        )
        self.timeout_s = float(self.declare_parameter("timeout_s", 20.0).value)
        self.started = time.monotonic()
        self.done = False
        self.exit_code = 2
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TablePlane, "/table_plane_detector/plane", self.callback, qos)

    def callback(self, message: TablePlane):
        if not message.valid:
            return
        perpendicular_error = abs(
            message.perpendicular_distance_m - self.expected_perpendicular
        )
        optical_error = abs(
            message.optical_axis_distance_m - self.expected_optical_axis
        )
        passed = (
            math.isfinite(message.optical_axis_distance_m)
            and perpendicular_error <= self.perpendicular_tolerance
            and optical_error <= self.optical_tolerance
        )
        level = self.get_logger().info if passed else self.get_logger().error
        level(
            "table-plane acceptance %s: perpendicular=%.6f (error %.6f), "
            "optical=%.6f (error %.6f), inliers=%d, ratio=%.3f, rms=%.6f"
            % (
                "PASSED" if passed else "FAILED",
                message.perpendicular_distance_m,
                perpendicular_error,
                message.optical_axis_distance_m,
                optical_error,
                message.inlier_count,
                message.inlier_ratio,
                message.rms_error_m,
            )
        )
        self.exit_code = 0 if passed else 1
        self.done = True

    def expired(self):
        return time.monotonic() - self.started >= self.timeout_s


def main():
    rclpy.init()
    node = Evaluator()
    while rclpy.ok() and not node.done and not node.expired():
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.done:
        node.get_logger().error("table-plane acceptance timed out without a valid plane")
        node.exit_code = 2
    exit_code = node.exit_code
    node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
