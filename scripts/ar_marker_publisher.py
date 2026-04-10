#!/usr/bin/env python3
"""
ar_marker_publisher.py — Interactive ROS 2 marker publisher for AR Explorer.

Publishes visualization_msgs/Marker to /ar_markers for the iPhone AR app.
The iPhone subscribes via rosbridge_websocket and renders markers in AR.

Coordinate convention (iPhone camera-relative):
  +x = right of camera
  +y = above camera
  -z = forward (in front of camera)
  Example: (0.0, 0.0, -3.0) = 3 meters directly ahead

Usage:
  ros2 run --prefix 'python3' . scripts/ar_marker_publisher.py
  # or simply:
  python3 scripts/ar_marker_publisher.py

  # With legacy PointStamped output on /ar_marker_position:
  python3 scripts/ar_marker_publisher.py --legacy

Interactive commands:
  add sphere 0.0 0.0 -3.0 red "victim found"
  add cube 1.0 0.5 -2.0 blue "hazard"
  add cylinder 0.0 0.0 -4.0 0.2,0.8,0.1 "safe zone"
  add model 0.0 0.0 -3.0 fender_stratocaster "guitar" 0.5
  delete 3
  clear
  list
  help
  quit
"""

import sys
import shlex
import argparse

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header, ColorRGBA


# ── Named colors ──────────────────────────────────────────────────────────────

COLORS = {
    "red":    ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
    "green":  ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
    "blue":   ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0),
    "yellow": ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
    "orange": ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
    "white":  ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
    "cyan":   ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
    "purple": ColorRGBA(r=0.6, g=0.2, b=1.0, a=1.0),
    "pink":   ColorRGBA(r=1.0, g=0.4, b=0.7, a=1.0),
}

# ── Marker type mapping ──────────────────────────────────────────────────────

MARKER_TYPES = {
    "sphere":   Marker.SPHERE,
    "cube":     Marker.CUBE,
    "cylinder": Marker.CYLINDER,
    "arrow":    Marker.ARROW,
    "text":     Marker.TEXT_VIEW_FACING,
    "model":    Marker.MESH_RESOURCE,
}


def parse_color(color_str: str) -> ColorRGBA:
    """Parse a color name or r,g,b string into ColorRGBA."""
    if color_str.lower() in COLORS:
        return COLORS[color_str.lower()]
    parts = color_str.split(",")
    if len(parts) == 3:
        r, g, b = float(parts[0]), float(parts[1]), float(parts[2])
        return ColorRGBA(r=r, g=g, b=b, a=1.0)
    print(f"  Unknown color '{color_str}', using green.")
    return COLORS["green"]


class ARMarkerPublisher(Node):
    """ROS 2 node that publishes visualization_msgs/Marker to /ar_markers."""

    def __init__(self, legacy: bool = False):
        super().__init__("ar_marker_publisher")
        self.marker_pub = self.create_publisher(Marker, "/ar_markers", 10)
        self.next_id = 0
        self.placed_ids: list[int] = []

        self.legacy = legacy
        if legacy:
            self.legacy_pub = self.create_publisher(
                PointStamped, "/ar_marker_position", 10
            )
            self.get_logger().info(
                "Legacy mode: also publishing PointStamped to /ar_marker_position"
            )

        self.get_logger().info("Publishing Marker messages to /ar_markers")

    def publish_add(
        self,
        marker_type: str,
        x: float, y: float, z: float,
        color: ColorRGBA,
        label: str = "",
        scale: float = 1.0,
        model_name: str = "",
    ) -> int:
        """Publish an ADD marker and return its ID."""
        msg = Marker()
        msg.header = Header()
        msg.header.frame_id = "camera"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.ns = "ar_explorer"
        msg.id = self.next_id
        msg.action = Marker.ADD

        if marker_type == "model":
            msg.type = Marker.MESH_RESOURCE
            msg.mesh_resource = model_name
        else:
            msg.type = MARKER_TYPES.get(marker_type, Marker.SPHERE)

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0

        msg.scale.x = float(scale)
        msg.scale.y = float(scale)
        msg.scale.z = float(scale)

        msg.color = color
        msg.text = label

        self.marker_pub.publish(msg)

        marker_id = self.next_id
        self.placed_ids.append(marker_id)
        self.next_id += 1

        # Also publish legacy PointStamped if requested
        if self.legacy:
            ps = PointStamped()
            ps.header.frame_id = label if label else "map"
            ps.header.stamp = msg.header.stamp
            ps.point.x = x
            ps.point.y = y
            ps.point.z = z
            self.legacy_pub.publish(ps)

        return marker_id

    def publish_delete(self, marker_id: int):
        """Publish a DELETE marker for the given ID."""
        msg = Marker()
        msg.header.frame_id = "camera"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "ar_explorer"
        msg.id = marker_id
        msg.action = Marker.DELETE
        self.marker_pub.publish(msg)
        if marker_id in self.placed_ids:
            self.placed_ids.remove(marker_id)

    def publish_clear(self):
        """Publish a DELETEALL marker to clear everything."""
        msg = Marker()
        msg.header.frame_id = "camera"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "ar_explorer"
        msg.action = Marker.DELETEALL
        self.marker_pub.publish(msg)
        self.placed_ids.clear()


def print_help():
    print(
        """
Commands:
  add <type> <x> <y> <z> <color> ["label"] [scale]
      type:  sphere, cube, cylinder, arrow, text
      color: red, green, blue, yellow, orange, white, cyan, purple, pink
             or r,g,b values (e.g. 0.5,0.8,0.2)
      label: optional quoted string
      scale: optional float (default 1.0)

  add model <x> <y> <z> <model_name> ["label"] [scale]
      model_name: USDZ file name without extension (e.g. fender_stratocaster)

  delete <id>        — remove marker by ID
  clear              — remove all markers
  list               — show placed marker IDs
  help               — show this message
  quit / exit / q    — exit

Coordinate convention (iPhone camera-relative):
  +x = right,  +y = up,  -z = forward
  Example: 0.0 0.0 -3.0 = 3 meters directly ahead
"""
    )


def run_interactive(node: ARMarkerPublisher):
    """Main interactive loop."""
    print_help()
    while True:
        try:
            line = input("\nAR> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not line:
            continue

        try:
            tokens = shlex.split(line)
        except ValueError as e:
            print(f"  Parse error: {e}")
            continue

        cmd = tokens[0].lower()

        if cmd in ("quit", "exit", "q"):
            break

        elif cmd == "help":
            print_help()

        elif cmd == "list":
            if node.placed_ids:
                print(f"  Placed marker IDs: {node.placed_ids}")
            else:
                print("  No markers placed.")

        elif cmd == "clear":
            node.publish_clear()
            print("  Cleared all markers.")

        elif cmd == "delete":
            if len(tokens) < 2:
                print("  Usage: delete <id>")
                continue
            try:
                mid = int(tokens[1])
            except ValueError:
                print(f"  Invalid ID: {tokens[1]}")
                continue
            node.publish_delete(mid)
            print(f"  Deleted marker {mid}.")

        elif cmd == "add":
            if len(tokens) < 6:
                print("  Usage: add <type> <x> <y> <z> <color|model_name> [label] [scale]")
                continue

            marker_type = tokens[1].lower()

            try:
                x, y, z = float(tokens[2]), float(tokens[3]), float(tokens[4])
            except ValueError:
                print("  Invalid coordinates. Use floats: x y z")
                continue

            if marker_type == "model":
                # add model <x> <y> <z> <model_name> [label] [scale]
                model_name = tokens[5]
                label = tokens[6] if len(tokens) > 6 else model_name
                try:
                    scale = float(tokens[7]) if len(tokens) > 7 else 1.0
                except ValueError:
                    scale = 1.0
                color = COLORS["white"]
                mid = node.publish_add(
                    "model", x, y, z, color, label, scale, model_name
                )
                print(
                    f"  [{mid}] model '{model_name}' at ({x}, {y}, {z}) "
                    f"label=\"{label}\" scale={scale}"
                )
            else:
                if marker_type not in MARKER_TYPES:
                    print(
                        f"  Unknown type '{marker_type}'. "
                        f"Use: {', '.join(MARKER_TYPES.keys())}"
                    )
                    continue
                color = parse_color(tokens[5])
                label = tokens[6] if len(tokens) > 6 else ""
                try:
                    scale = float(tokens[7]) if len(tokens) > 7 else 1.0
                except ValueError:
                    scale = 1.0
                mid = node.publish_add(marker_type, x, y, z, color, label, scale)
                print(
                    f"  [{mid}] {marker_type} at ({x}, {y}, {z}) "
                    f"color={tokens[5]} label=\"{label}\" scale={scale}"
                )

        else:
            print(f"  Unknown command: '{cmd}'. Type 'help' for usage.")


def main():
    parser = argparse.ArgumentParser(description="AR Explorer marker publisher")
    parser.add_argument(
        "--legacy", action="store_true",
        help="Also publish PointStamped to /ar_marker_position"
    )
    args = parser.parse_args()

    rclpy.init()
    node = ARMarkerPublisher(legacy=args.legacy)

    try:
        run_interactive(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
