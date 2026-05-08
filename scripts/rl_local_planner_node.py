#!/usr/bin/env python3
"""ONNX-backed RL local planner skeleton."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from std_msgs.msg import String

from height_map_ros2.msg import MaskedHeightScan


class RlLocalPlannerNode(Node):
    def __init__(self):
        super().__init__("rl_local_planner_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("model_path", "")
        self.declare_parameter("current_pose_topic", "/localization/current_pose")
        self.declare_parameter("target_pose_topic", "/planning/target_pose")
        self.declare_parameter("height_scan_topic", "/elevation_mapping_node/local_terrain_map")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate_hz", 20.0)

        self._current_pose = None
        self._target_pose = None
        self._height_scan = None
        self._enabled = bool(self.get_parameter("enabled").value)
        self._model_path = str(self.get_parameter("model_path").value)
        self._session = None

        self.create_subscription(Pose2D, self.get_parameter("current_pose_topic").value, self._on_current_pose, 10)
        self.create_subscription(Pose2D, self.get_parameter("target_pose_topic").value, self._on_target_pose, 10)
        self.create_subscription(
            MaskedHeightScan,
            self.get_parameter("height_scan_topic").value,
            self._on_height_scan,
            10,
        )
        self._cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self._heartbeat_pub = self.create_publisher(String, "/autonomy/heartbeat/rl_local_planner_node", 10)

        period = 1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(period, self._tick)
        self.get_logger().info("RL local planner skeleton started")

    def _on_current_pose(self, msg):
        self._current_pose = msg

    def _on_target_pose(self, msg):
        self._target_pose = msg

    def _on_height_scan(self, msg):
        self._height_scan = msg

    def _tick(self):
        heartbeat = String()
        heartbeat.data = "waiting_for_inputs"
        if self._enabled and self._current_pose and self._target_pose and self._height_scan:
            heartbeat.data = "ready"
            # TODO: Load ONNX Runtime session and publish model action.
            self._cmd_pub.publish(Twist())
        self._heartbeat_pub.publish(heartbeat)

def main():
    rclpy.init()
    node = RlLocalPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
