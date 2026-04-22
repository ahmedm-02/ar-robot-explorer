#!/usr/bin/env python3
"""Compare AprilTag detections from the RealSense and iPhone cameras.

Subscribes to /apriltag_detections (RealSense) and /ar_markers (iPhone, id >= 1000),
looks for matching tag IDs, and prints both poses side by side when a match is found.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 scripts/calibration_check.py
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

MARKER_ID_OFFSET = 1000


class CalibrationCheck(Node):
    def __init__(self):
        super().__init__("calibration_check")
        self.realsense_tags = {}
        self.iphone_tags = {}

        self.create_subscription(
            Marker, "/apriltag_detections", self._on_realsense_marker, 10
        )
        self.create_subscription(
            Marker, "/ar_markers", self._on_iphone_marker, 10
        )

        self.get_logger().info(
            "Calibration check running — waiting for tags from both cameras..."
        )

    def _on_realsense_marker(self, msg: Marker):
        if msg.action != Marker.ADD:
            return
        tag_id = msg.id
        pos = msg.pose.position
        self.realsense_tags[tag_id] = (pos.x, pos.y, pos.z)
        self._check_match(tag_id)

    def _on_iphone_marker(self, msg: Marker):
        if msg.action != Marker.ADD:
            return
        if msg.id < MARKER_ID_OFFSET:
            return
        tag_id = msg.id - MARKER_ID_OFFSET
        pos = msg.pose.position
        self.iphone_tags[tag_id] = (pos.x, pos.y, pos.z)
        self._check_match(tag_id)

    def _check_match(self, tag_id: int):
        if tag_id not in self.realsense_tags or tag_id not in self.iphone_tags:
            return
        rs = self.realsense_tags[tag_id]
        ip = self.iphone_tags[tag_id]
        self.get_logger().info(
            f"\nTag {tag_id} seen by BOTH cameras:\n"
            f"  RealSense: pos=({rs[0]:+.3f}, {rs[1]:+.3f}, {rs[2]:+.3f})\n"
            f"  iPhone:    pos=({ip[0]:+.3f}, {ip[1]:+.3f}, {ip[2]:+.3f})\n"
            f"  Ready for calibration handshake"
        )


def main():
    rclpy.init()
    node = CalibrationCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
