#!/usr/bin/env python3
"""
test_markers.py — Publish a sequence of test markers to verify the iPhone AR app.

Publishes to /ar_markers (visualization_msgs/Marker) via rosbridge.
The iPhone app should render each marker as it arrives.

Usage:
  python3 scripts/test_markers.py
  python3 scripts/test_markers.py fender_stratocaster   # also place a USDZ model

Coordinate convention (iPhone camera-relative):
  +x = right,  +y = up,  -z = forward
"""

import sys
import time

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class TestMarkerPublisher(Node):

    def __init__(self):
        super().__init__("test_marker_publisher")
        self.pub = self.create_publisher(Marker, "/ar_markers", 10)
        self.next_id = 0

    def make_marker(
        self,
        marker_type: int,
        x: float, y: float, z: float,
        color: ColorRGBA,
        label: str = "",
        scale: float = 1.0,
        action: int = Marker.ADD,
        model_name: str = "",
    ) -> Marker:
        msg = Marker()
        msg.header.frame_id = "camera"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "ar_explorer"
        msg.id = self.next_id
        msg.type = marker_type
        msg.action = action

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0

        msg.scale.x = float(scale)
        msg.scale.y = float(scale)
        msg.scale.z = float(scale)

        msg.color = color
        msg.text = label

        if model_name:
            msg.mesh_resource = model_name

        self.next_id += 1
        return msg

    def publish_and_log(self, msg: Marker, description: str):
        self.pub.publish(msg)
        pos = msg.pose.position
        self.get_logger().info(
            f"[{msg.id}] {description} at ({pos.x}, {pos.y}, {pos.z})"
        )


def main():
    model_name = sys.argv[1] if len(sys.argv) > 1 else None

    rclpy.init()
    node = TestMarkerPublisher()

    # Give rosbridge a moment to register the publisher
    time.sleep(1.0)

    print("\n=== AR Explorer Test Sequence ===\n")

    # 1. Red sphere: "victim" — 3m ahead
    msg = node.make_marker(
        Marker.SPHERE, 0.0, 0.0, -3.0,
        ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
        label="victim",
    )
    node.publish_and_log(msg, "Red sphere 'victim'")
    time.sleep(2.0)

    # 2. Blue cube: "hazard" — 2m ahead, 1m left
    msg = node.make_marker(
        Marker.CUBE, -1.0, 0.0, -2.0,
        ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0),
        label="hazard",
    )
    node.publish_and_log(msg, "Blue cube 'hazard'")
    time.sleep(2.0)

    # 3. Green cylinder: "clear zone" — 4m ahead, 1m right
    msg = node.make_marker(
        Marker.CYLINDER, 1.0, 0.0, -4.0,
        ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
        label="clear zone",
    )
    node.publish_and_log(msg, "Green cylinder 'clear zone'")
    time.sleep(2.0)

    # 4. Optional USDZ model — 5m ahead
    if model_name:
        msg = node.make_marker(
            Marker.MESH_RESOURCE, 0.0, 0.0, -5.0,
            ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
            label=model_name,
            scale=0.5,
            model_name=model_name,
        )
        node.publish_and_log(msg, f"USDZ model '{model_name}'")
        time.sleep(2.0)

    placed = node.next_id
    print(f"\n  Placed {placed} markers. Waiting 5 seconds before clearing...\n")
    time.sleep(5.0)

    # 5. Clear all
    clear_msg = Marker()
    clear_msg.header.frame_id = "camera"
    clear_msg.header.stamp = node.get_clock().now().to_msg()
    clear_msg.ns = "ar_explorer"
    clear_msg.action = Marker.DELETEALL
    node.pub.publish(clear_msg)
    node.get_logger().info("DELETEALL — cleared all markers")

    print("\n=== Test complete ===\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
