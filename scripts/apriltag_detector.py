#!/usr/bin/env python3
"""AprilTag detection on the RealSense RGB stream.

Subscribes to /camera/camera/color/image_raw + /camera/camera/color/camera_info,
runs pupil-apriltags on each frame (throttled to ~10 fps), logs poses, broadcasts
TF (camera_optical_frame -> tag_<id>), and publishes visualization_msgs/Marker on
/apriltag_detections for RViz. Does NOT publish to /ar_markers (iPhone topic).
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

try:
    from pupil_apriltags import Detector
except ImportError:
    print("ERROR: pupil-apriltags not installed. Run: pip install pupil-apriltags",
          file=sys.stderr)
    sys.exit(1)

try:
    from cv_bridge import CvBridge
except ImportError:
    print("ERROR: cv_bridge not available. Source /opt/ros/jazzy/setup.bash first.",
          file=sys.stderr)
    sys.exit(1)

import cv2


TARGET_FPS = 10.0
MIN_FRAME_INTERVAL = 1.0 / TARGET_FPS


DEFAULT_OPTICAL_FRAME = "camera_color_optical_frame"


def rotation_matrix_to_quaternion(R):
    """Return (x, y, z, w). Standard Shepperd/Shoemake branch-select method."""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def rotation_matrix_to_euler_deg(R):
    """ZYX intrinsic Euler angles (roll, pitch, yaw) in degrees."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0.0
    return math.degrees(rx), math.degrees(ry), math.degrees(rz)


class AprilTagDetectorNode(Node):
    def __init__(self, tag_size: float):
        super().__init__("apriltag_detector")
        self.tag_size = tag_size
        self.bridge = CvBridge()
        self.detector = Detector(families="tag36h11")
        self.camera_params = None  # (fx, fy, cx, cy)
        self.last_process_time = 0.0
        self.warned_no_info = False
        self.got_any_image = False

        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._on_camera_info,
            10,
        )
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._on_image,
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, "/apriltag_detections", 10)
        self._summarized_ids = set()

        self.create_timer(5.0, self._startup_watchdog)
        self._start_time = time.monotonic()

        self.get_logger().info(
            f"apriltag_detector up — tag_size={tag_size} m, family=tag36h11, "
            f"target {TARGET_FPS:.0f} fps. TF parent defaults to '{DEFAULT_OPTICAL_FRAME}'."
        )

    def _startup_watchdog(self):
        if self.got_any_image:
            return
        elapsed = time.monotonic() - self._start_time
        if elapsed > 10.0:
            self.get_logger().error(
                "No frames on /camera/camera/color/image_raw after 10 s. "
                "Is the RealSense node running? Try: ros2 launch realsense2_camera rs_launch.py"
            )

    def _on_camera_info(self, msg: CameraInfo):
        k = msg.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        if fx <= 0.0 or fy <= 0.0:
            return
        new_params = (fx, fy, cx, cy)
        if self.camera_params != new_params:
            self.camera_params = new_params
            self.get_logger().info(
                f"CameraInfo: fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}"
            )

    def _on_image(self, msg: Image):
        self.got_any_image = True

        now = time.monotonic()
        if now - self.last_process_time < MIN_FRAME_INTERVAL:
            return
        self.last_process_time = now

        if self.camera_params is None:
            if not self.warned_no_info:
                self.get_logger().warn(
                    "Waiting for CameraInfo on /camera/camera/color/camera_info…"
                )
                self.warned_no_info = True
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )

        parent_frame = msg.header.frame_id or DEFAULT_OPTICAL_FRAME
        stamp = msg.header.stamp

        for d in detections:
            t = d.pose_t.reshape(3)
            R = np.asarray(d.pose_R)
            rx, ry, rz = rotation_matrix_to_euler_deg(R)
            qx, qy, qz, qw = rotation_matrix_to_quaternion(R)
            child_frame = f"tag_{d.tag_id}"

            self.get_logger().info(
                f"AprilTag id={d.tag_id}  "
                f"pos=({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})  "
                f"rot={rx:+.1f},{ry:+.1f},{rz:+.1f} deg  "
                f"margin={d.decision_margin:.1f}"
            )

            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = parent_frame
            tf.child_frame_id = child_frame
            tf.transform.translation.x = float(t[0])
            tf.transform.translation.y = float(t[1])
            tf.transform.translation.z = float(t[2])
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = parent_frame
            marker.ns = "apriltag"
            marker.id = int(d.tag_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(t[0])
            marker.pose.position.y = float(t[1])
            marker.pose.position.z = float(t[2])
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = self.tag_size
            marker.scale.y = self.tag_size
            marker.scale.z = 0.005
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.text = child_frame
            marker.lifetime = DurationMsg(sec=2, nanosec=0)
            self.marker_pub.publish(marker)

            if d.tag_id not in self._summarized_ids:
                self._summarized_ids.add(d.tag_id)
                self.get_logger().info(
                    f"Tag {d.tag_id}: TF published as {parent_frame} → {child_frame}, "
                    f"Marker on /apriltag_detections"
                )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--tag-size",
        type=float,
        default=0.17,
        help="AprilTag edge length in meters (black border to black border). Default 0.17.",
    )
    # rclpy may inject its own args via ros2 run; allow unknown to pass through.
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = AprilTagDetectorNode(tag_size=args.tag_size)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
