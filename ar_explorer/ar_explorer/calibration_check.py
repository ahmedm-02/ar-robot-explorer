#!/usr/bin/env python3
"""Compare AprilTag detections from the RealSense and iPhone cameras.

Watches /detections (RealSense) and /iphone/detections (iPhone) — both are
apriltag_msgs/AprilTagDetectionArray — looks up the corresponding TF for each
detected tag, and prints both poses side by side when the same tag ID is seen
by both cameras. Useful as a pre-flight check before running calibration.

Usage:
    source install/setup.bash
    ros2 run ar_explorer calibration_check
"""

from __future__ import annotations

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


REALSENSE_FRAME = "camera_color_optical_frame"
IPHONE_FRAME = "iphone_camera"


class CalibrationCheck(Node):
    def __init__(self) -> None:
        super().__init__("calibration_check")
        self.realsense_tags: dict[int, tuple[float, float, float]] = {}
        self.iphone_tags: dict[int, tuple[float, float, float]] = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timeout = RclpyDuration(seconds=0.1)

        self.create_subscription(
            AprilTagDetectionArray, "/detections",
            lambda m: self._on_detections(m, "tag_", self.realsense_tags), 10,
        )
        self.create_subscription(
            AprilTagDetectionArray, "/iphone/detections",
            lambda m: self._on_detections(m, "iphone_tag_", self.iphone_tags), 10,
        )

        self.get_logger().info(
            "Calibration check running — waiting for tags from both cameras..."
        )

    def _on_detections(self, msg: AprilTagDetectionArray, prefix: str, store: dict) -> None:
        parent = msg.header.frame_id
        for d in msg.detections:
            tag_id = int(d.id)
            child = f"{prefix}{tag_id}"
            try:
                tf = self.tf_buffer.lookup_transform(parent, child, Time(), self.tf_timeout)
            except TransformException:
                continue
            t = tf.transform.translation
            store[tag_id] = (float(t.x), float(t.y), float(t.z))
            self._check_match(tag_id)

    def _check_match(self, tag_id: int) -> None:
        if tag_id not in self.realsense_tags or tag_id not in self.iphone_tags:
            return
        rs = self.realsense_tags[tag_id]
        ip = self.iphone_tags[tag_id]
        self.get_logger().info(
            f"\nTag {tag_id} seen by BOTH cameras:\n"
            f"  RealSense ({REALSENSE_FRAME}): pos=({rs[0]:+.3f}, {rs[1]:+.3f}, {rs[2]:+.3f})\n"
            f"  iPhone    ({IPHONE_FRAME}):     pos=({ip[0]:+.3f}, {ip[1]:+.3f}, {ip[2]:+.3f})\n"
            f"  Ready for calibration handshake"
        )


def main() -> None:
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
