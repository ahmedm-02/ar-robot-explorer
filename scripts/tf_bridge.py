#!/usr/bin/env python3
"""Bridge the RealSense and iPhone TF trees via a shared AprilTag.

Both cameras detect the same physical tag. This node looks up both transforms
and broadcasts the relative pose between the two cameras:

    camera_color_optical_frame → iphone_camera

This lets RViz display both camera frames and their tag detections in a single
unified TF tree.

Math:
    T_rs_from_iphone = T_rs_from_tag × inverse(T_ip_from_tag)

Usage:
    python3 scripts/tf_bridge.py [--tag-id 0]
"""

import argparse
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

try:
    import tf2_ros
    from tf2_ros import TransformException
except ImportError:
    print("ERROR: tf2_ros not available. Source /opt/ros/jazzy/setup.bash first.",
          file=sys.stderr)
    sys.exit(1)


REALSENSE_FRAME = "camera_color_optical_frame"
IPHONE_FRAME = "iphone_camera"


def transform_to_matrix(transform):
    t = transform.translation
    q = transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])

    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def matrix_to_quaternion(M):
    """Extract quaternion (x, y, z, w) from a 4x4 homogeneous matrix."""
    R = M[:3, :3]
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class TFBridge(Node):
    def __init__(self, tag_id: int):
        super().__init__("tf_bridge")
        self.rs_tag = f"tag_{tag_id}"
        self.ip_tag = f"iphone_tag_{tag_id}"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._published = False
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"TF bridge waiting — will link {REALSENSE_FRAME} → {IPHONE_FRAME} "
            f"via shared tag {tag_id}"
        )

    def _tick(self):
        try:
            tf_rs = self.tf_buffer.lookup_transform(
                REALSENSE_FRAME, self.rs_tag, rclpy.time.Time()
            )
            tf_ip = self.tf_buffer.lookup_transform(
                IPHONE_FRAME, self.ip_tag, rclpy.time.Time()
            )
        except TransformException:
            return

        T_rs_from_tag = transform_to_matrix(tf_rs.transform)
        T_ip_from_tag = transform_to_matrix(tf_ip.transform)

        T_rs_from_iphone = T_rs_from_tag @ np.linalg.inv(T_ip_from_tag)

        t = T_rs_from_iphone[:3, 3]
        qx, qy, qz, qw = matrix_to_quaternion(T_rs_from_iphone)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = REALSENSE_FRAME
        msg.child_frame_id = IPHONE_FRAME
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        msg.transform.rotation.x = float(qx)
        msg.transform.rotation.y = float(qy)
        msg.transform.rotation.z = float(qz)
        msg.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(msg)

        if not self._published:
            self._published = True
            self.get_logger().info(
                f"TF bridge active: {REALSENSE_FRAME} → {IPHONE_FRAME} "
                f"(via {self.rs_tag}/{self.ip_tag})"
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tag-id", type=int, default=0,
                        help="Shared AprilTag ID (default: 0).")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TFBridge(tag_id=args.tag_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
