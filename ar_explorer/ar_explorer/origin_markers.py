#!/usr/bin/env python3
"""Publish two origin spheres to /ar_markers for visual verification on the iPhone.

  🔴 RED   sphere  = the ARKit world origin (arkit_world frame origin)
  🔵 BLUE  sphere  = the dog's odom origin  (where the dog's odometry booted)

Seeing both in the AR view lets you eyeball the calibration: the spatial
relationship between the two origins should match reality, and the blue sphere
should sit wherever the dog started relative to the phone's session origin.

Same path as the other markers (calibrated_forwarder / tag_to_marker): a ROS
node publishing visualization_msgs/Marker to /ar_markers, which the iPhone
renders via rosbridge. No app change.

⚠️ CONVENTION — the trap this node exists to handle correctly:
  The iPhone renders `frame_id: "arkit_world"` markers in RAW ARKit convention
  (+X right, +Y up, +Z toward viewer) — placeWorldMarker anchors at those
  coords directly. But our TF `arkit_world` frame is ROS REP-103 (+X fwd,
  +Y left, +Z up), because iphone_pose_bridge applies a basis change. Same
  name, rotated axes.
    - RED is at the origin (0,0,0) — the same physical point in both
      conventions (the basis change is a pure rotation), so no conversion.
    - BLUE is a non-origin point: we look up TF arkit_world→odom (ROS coords)
      and convert ROS→ARKit with R_BASIS.T (the inverse of iphone_pose_bridge's
      _R_BASIS) before publishing, or it lands in the wrong place.

Usage (domain 42; needs the merged TF graph + the iPhone connected to rosbridge):
    ros2 run ar_explorer origin_markers
    ros2 run ar_explorer origin_markers --ros-args -p odom_frame:=odom
"""

from __future__ import annotations

import sys

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.node import Node
from visualization_msgs.msg import Marker

try:
    import tf2_ros
    from tf2_ros import TransformException
except ImportError:
    print("ERROR: tf2_ros not available. Source /opt/ros/jazzy/setup.bash first.",
          file=sys.stderr)
    sys.exit(1)


WORLD_FRAME = "arkit_world"

# ARKit→ROS basis change, copied verbatim from iphone_pose_bridge._R_BASIS so
# the two stay in lockstep. ROS = R_BASIS @ ARKit, so ARKit = R_BASIS.T @ ROS.
# (Each row is the ROS axis expressed in ARKit components; det = +1.)
_R_BASIS = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def ros_to_arkit(p_ros: np.ndarray) -> np.ndarray:
    """Express a point given in the ROS arkit_world frame in RAW ARKit coords,
    which is what the iPhone's placeWorldMarker expects."""
    return _R_BASIS.T @ p_ros


class OriginMarkers(Node):
    def __init__(self) -> None:
        super().__init__("origin_markers")

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("diameter", 0.10)   # sphere size, meters

        self.odom_frame = self.get_parameter("odom_frame").value
        rate = float(self.get_parameter("rate_hz").value)
        self.diameter = float(self.get_parameter("diameter").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(Marker, "/ar_markers", 10)

        self._warned_no_odom = False
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"origin_markers — RED at {WORLD_FRAME} origin, "
            f"BLUE at {self.odom_frame} origin → /ar_markers"
        )

    def _make_sphere(self, marker_id: int, pos, rgb, label: str) -> Marker:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = WORLD_FRAME     # iPhone anchors this in world space
        m.ns = "origins"
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = float(pos[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = self.diameter
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 1.0
        m.text = label
        # Short lifetime + republish (below) so they persist and the blue one
        # refreshes if you recalibrate; also self-clears if this node stops.
        m.lifetime = DurationMsg(sec=2, nanosec=0)
        return m

    def _tick(self) -> None:
        # RED — ARKit world origin. (0,0,0) is correct in both conventions.
        self.marker_pub.publish(
            self._make_sphere(8001, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), "arkit_world origin")
        )

        # BLUE — dog odom origin. Look up arkit_world→odom (ROS coords); its
        # translation IS the odom origin in the ROS world frame. Convert to
        # ARKit before publishing.
        try:
            tf = self.tf_buffer.lookup_transform(
                WORLD_FRAME, self.odom_frame, rclpy.time.Time()
            )
        except TransformException:
            if not self._warned_no_odom:
                self._warned_no_odom = True
                self.get_logger().warn(
                    f"No {WORLD_FRAME} → {self.odom_frame} transform yet — blue "
                    "sphere deferred. Has odom_world_calibrator run?"
                )
            return

        t = tf.transform.translation
        p_ros = np.array([t.x, t.y, t.z])
        p_arkit = ros_to_arkit(p_ros)
        self.marker_pub.publish(
            self._make_sphere(8002, p_arkit, (0.0, 0.0, 1.0),
                              f"{self.odom_frame} origin")
        )

        if self._warned_no_odom:
            self._warned_no_odom = False
            self.get_logger().info(
                f"{WORLD_FRAME} → {self.odom_frame} acquired — publishing blue "
                f"sphere at ARKit ({p_arkit[0]:+.3f}, {p_arkit[1]:+.3f}, "
                f"{p_arkit[2]:+.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OriginMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
