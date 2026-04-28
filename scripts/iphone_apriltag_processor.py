#!/usr/bin/env python3
"""AprilTag detection on the iPhone's MJPEG camera stream.

Connects to the iPhone's MJPEG stream (port 8082), runs pupil-apriltags on each
frame, publishes visualization_msgs/Marker on /ar_markers (so the iPhone renders
green tag overlays), and broadcasts TF (iphone_camera → iphone_tag_<id>).

Usage:
    python3 scripts/iphone_apriltag_processor.py --url http://<iphone_ip>:8082/stream
"""

import argparse
import math
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

try:
    from pupil_apriltags import Detector
except ImportError:
    print("ERROR: pupil-apriltags not installed. Run: pip install pupil-apriltags",
          file=sys.stderr)
    sys.exit(1)

import cv2


# Approximate iPhone 16 wide camera intrinsics at 640x480.
# These are rough estimates — replace with calibrated values for accuracy.
DEFAULT_FX = 500.0
DEFAULT_FY = 500.0
DEFAULT_CX = 320.0
DEFAULT_CY = 240.0

IPHONE_CAMERA_FRAME = "iphone_camera"
MARKER_ID_OFFSET = 1000
RETRY_INTERVAL = 3.0


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


class IPhoneAprilTagProcessor(Node):
    def __init__(self, stream_url: str, tag_size: float):
        super().__init__("iphone_apriltag_processor")
        self.stream_url = stream_url
        self.tag_size = tag_size
        self.detector = Detector(families="tag36h11")
        self.camera_params = (DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, "/ar_markers", 10)
        self._summarized_ids = set()

        self._shutdown = False

        self.get_logger().info(
            f"iPhone AprilTag processor starting — stream={stream_url}, "
            f"tag_size={tag_size} m, intrinsics=(fx={DEFAULT_FX}, fy={DEFAULT_FY}, "
            f"cx={DEFAULT_CX}, cy={DEFAULT_CY}) [approximate]"
        )

        self._capture_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._capture_thread.start()

    def stop(self):
        self._shutdown = True

    def _run_loop(self):
        while not self._shutdown and rclpy.ok():
            cap = cv2.VideoCapture(self.stream_url)
            if not cap.isOpened():
                self.get_logger().warn(
                    f"Cannot open iPhone stream at {self.stream_url} — "
                    f"retrying in {RETRY_INTERVAL:.0f}s..."
                )
                for _ in range(int(RETRY_INTERVAL * 10)):
                    if self._shutdown:
                        return
                    time.sleep(0.1)
                continue

            self.get_logger().info(f"Connected to iPhone stream: {self.stream_url}")

            while not self._shutdown and rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn(
                        f"iPhone stream disconnected — retrying in {RETRY_INTERVAL:.0f}s..."
                    )
                    cap.release()
                    for _ in range(int(RETRY_INTERVAL * 10)):
                        if self._shutdown:
                            return
                        time.sleep(0.1)
                    break

                self._process_frame(frame)

    def _process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )

        now = self.get_clock().now().to_msg()

        for d in detections:
            t = d.pose_t.reshape(3)
            R = np.asarray(d.pose_R)
            qx, qy, qz, qw = rotation_matrix_to_quaternion(R)
            child_frame = f"iphone_tag_{d.tag_id}"

            # OpenCV convention: +z forward. iPhone AR convention: -z forward.
            # Negate z for the marker published to /ar_markers.
            ar_x = float(t[0])
            ar_y = float(t[1])
            ar_z = -float(t[2])

            self.get_logger().info(
                f"iPhone AprilTag id={d.tag_id} "
                f"pos=({ar_x:+.3f}, {ar_y:+.3f}, {ar_z:+.3f}) "
                f"at {now.sec}.{now.nanosec:09d}"
            )

            # TF: publish in OpenCV convention (z-forward) for consistency with tf2
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = IPHONE_CAMERA_FRAME
            tf.child_frame_id = child_frame
            tf.transform.translation.x = float(t[0])
            tf.transform.translation.y = float(t[1])
            tf.transform.translation.z = float(t[2])
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

            # Marker on /ar_markers: iPhone camera-relative convention (z negated)
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = "camera"
            marker.ns = "iphone_apriltag"
            marker.id = int(d.tag_id) + MARKER_ID_OFFSET
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = ar_x
            marker.pose.position.y = ar_y
            marker.pose.position.z = ar_z
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.tag_size
            marker.scale.y = self.tag_size
            marker.scale.z = 0.005
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            marker.text = f"Tag #{d.tag_id}"
            marker.lifetime = DurationMsg(sec=2, nanosec=0)
            self.marker_pub.publish(marker)

            if d.tag_id not in self._summarized_ids:
                self._summarized_ids.add(d.tag_id)
                self.get_logger().info(
                    f"Tag {d.tag_id}: Marker on /ar_markers (id={d.tag_id + MARKER_ID_OFFSET}), "
                    f"TF as {IPHONE_CAMERA_FRAME} → {child_frame}"
                )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--url",
        required=True,
        help="iPhone MJPEG stream URL, e.g. http://<iphone_ip>:8082/stream",
    )
    parser.add_argument(
        "--tag-size",
        type=float,
        default=0.17,
        help="AprilTag edge length in meters (default: 0.17).",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = IPhoneAprilTagProcessor(stream_url=args.url, tag_size=args.tag_size)

    def signal_handler(*_):
        node.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
