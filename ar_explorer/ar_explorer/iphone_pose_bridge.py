#!/usr/bin/env python3
"""Bridge: /iphone/pose (TransformStamped, raw ARKit convention)
       → TF: arkit_world → iphone_camera (ROS REP-103 convention).

ARKit world frame: +X right, +Y up, +Z toward the viewer (out of the screen,
i.e. behind the camera at session start).
ROS REP-103 frame: +X forward, +Y left, +Z up.

The axis remap (ROS expressed in ARKit) is:

    ROS X ← -ARKit Z
    ROS Y ← -ARKit X
    ROS Z ← +ARKit Y

For translations we apply the basis-change matrix R_basis directly. For
orientation quaternions we conjugate by R_basis — the rotation matrix is
a mapping between frames, so a basis change is R_basis @ R_arkit @ R_basis.T,
not just an axis swap of the quaternion components.

Timestamps: incoming ARKit stamps are mach uptime, not Unix epoch, so TF
lookups against system-time consumers would silently fail. We rebroadcast
with self.get_clock().now() instead.
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster


# Fixed ARKit→ROS basis-change matrix. det = +1 (verified: proper rotation,
# not a reflection). Each row is the ROS axis expressed in ARKit components.
_R_BASIS = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


class IPhonePoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("iphone_pose_bridge")

        self.declare_parameter("input_topic", "/iphone/pose")
        self.declare_parameter("parent_frame", "arkit_world")
        self.declare_parameter("child_frame", "iphone_camera")

        in_topic = self.get_parameter("input_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(TransformStamped, in_topic, self._on_pose, 10)

        self._count = 0
        self.get_logger().info(
            f"iphone_pose_bridge — {in_topic} → TF "
            f"{self.parent_frame} → {self.child_frame}"
        )

    def _on_pose(self, msg: TransformStamped) -> None:
        t_arkit = np.array([
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
        ])
        t_ros = _R_BASIS @ t_arkit

        q_arkit = [
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        ]
        rot_arkit = R.from_quat(q_arkit).as_matrix()
        rot_ros = _R_BASIS @ rot_arkit @ _R_BASIS.T
        q_ros = R.from_matrix(rot_ros).as_quat()  # scipy returns (x, y, z, w)

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.parent_frame
        out.child_frame_id = self.child_frame
        out.transform.translation.x = float(t_ros[0])
        out.transform.translation.y = float(t_ros[1])
        out.transform.translation.z = float(t_ros[2])
        out.transform.rotation.x = float(q_ros[0])
        out.transform.rotation.y = float(q_ros[1])
        out.transform.rotation.z = float(q_ros[2])
        out.transform.rotation.w = float(q_ros[3])

        self.broadcaster.sendTransform(out)

        self._count += 1
        if self._count == 1 or self._count % 300 == 0:
            self.get_logger().info(
                f"Published TF {self.parent_frame} → {self.child_frame} "
                f"#{self._count}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = IPhonePoseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
