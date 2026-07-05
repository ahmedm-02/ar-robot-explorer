#!/usr/bin/env python3
"""Publish the arkit_world → odom calibration from file — the SOLE publisher.

Loads the JSON written by odom_world_calibrator and broadcasts arkit_world →
odom on /tf_static, re-reading the file so a fresh calibration is picked up
automatically (no relaunch). This is intentionally the only thing in the system
that publishes that edge, so there can never be two conflicting /tf_static
transforms for it — the bug that made the origin/dog spheres land in the wrong
place.

Failure mode is safe: if the file doesn't exist yet (never calibrated), it
publishes nothing and waits — no calibration rather than a wrong one.

Usage (normally auto-started by the launch, base group):
    ros2 run ar_explorer odom_world_tf
"""

from __future__ import annotations

import json
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

try:
    from tf2_ros import StaticTransformBroadcaster
except ImportError:
    print("ERROR: tf2_ros not available. Source /opt/ros/jazzy/setup.bash first.",
          file=sys.stderr)
    sys.exit(1)


DEFAULT_INPUT = os.path.expanduser("~/.ros/ar_explorer_odom_world.json")


class OdomWorldTf(Node):
    def __init__(self) -> None:
        super().__init__("odom_world_tf")

        self.declare_parameter("input", DEFAULT_INPUT)
        self.declare_parameter("reload_period", 2.0)

        self.path = self.get_parameter("input").value
        period = float(self.get_parameter("reload_period").value)

        self.broadcaster = StaticTransformBroadcaster(self)
        self._last_mtime = None
        self._warned_missing = False

        self.create_timer(period, self._reload)
        self.get_logger().info(
            f"odom_world_tf — publishing arkit_world → odom from {self.path} "
            f"(reload every {period:.1f}s)"
        )

    def _reload(self) -> None:
        if not os.path.exists(self.path):
            if not self._warned_missing:
                self._warned_missing = True
                self.get_logger().warn(
                    f"No calibration file at {self.path} yet — publishing nothing "
                    "until odom_world_calibrator writes one."
                )
            return

        mtime = os.path.getmtime(self.path)
        if mtime == self._last_mtime:
            return   # unchanged since last publish

        try:
            with open(self.path) as f:
                data = json.load(f)
            parent = data["parent_frame"]
            child = data["child_frame"]
            tx, ty, tz = data["translation"]
            qx, qy, qz, qw = data["quaternion"]
        except (ValueError, KeyError, OSError) as exc:
            self.get_logger().warn(f"Bad/again-writing calibration file: {exc}")
            return

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(tx)
        msg.transform.translation.y = float(ty)
        msg.transform.translation.z = float(tz)
        msg.transform.rotation.x = float(qx)
        msg.transform.rotation.y = float(qy)
        msg.transform.rotation.z = float(qz)
        msg.transform.rotation.w = float(qw)
        self.broadcaster.sendTransform(msg)

        self._last_mtime = mtime
        self._warned_missing = False
        self.get_logger().info(
            f"Published {parent} → {child}  "
            f"t=({float(tx):+.3f}, {float(ty):+.3f}, {float(tz):+.3f})"
            + (f"  [calibrated {data.get('timestamp', '?')}]"
               if "timestamp" in data else "")
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomWorldTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
