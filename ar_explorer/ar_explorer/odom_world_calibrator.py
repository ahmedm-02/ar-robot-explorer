#!/usr/bin/env python3
"""Compute and publish the fixed arkit_world → odom calibration bridge.

This is the one edge that ties the dog's odometry tree to the iPhone's ARKit
world so RealSense detections land in the first responder's AR view AND track
the dog as it walks. It is computed ONCE from a two-camera AprilTag handshake
and published as a latched /tf_static transform (both frames are world-fixed
after boot, so the edge never changes until you recalibrate).

The tree, once this node has published:

    arkit_world ──(this node, static)── odom ──(odom_relay_receiver, live)──
        base_link ──(static mount)── camera_link ──(RS driver)──
        camera_color_optical_frame ──(apriltag)── tag_<id>
    arkit_world ──(iphone_pose_bridge, live)── iphone_camera ──(apriltag)──
        iphone_tag_<id>

How it works: both apriltag_ros instances see the SAME physical tag, so
tag_<id> (RealSense tree) and iphone_tag_<id> (iPhone tree) are the same
physical frame. We look up each in its own root and compose:

    T_arkit_from_odom = T_arkit_from_tag @ inverse(T_odom_from_tag)

Going through TF is convention-clean — no manual optical/ARKit basis fudge,
because apriltag_ros defines the tag frame identically for both detectors.

PRECONDITIONS (all must be live in ONE ROS graph — on ROS_DOMAIN_ID=42 the NUC
and the base share it):
  - odom → base_link           (odom_relay_receiver, on the NUC)
  - base_link → camera_link     (static mount, launch file)
  - camera_link → camera_color_optical_frame → tag_<id>  (RealSense + apriltag)
  - arkit_world → iphone_camera → iphone_tag_<id>         (iPhone bridge + apriltag)

Usage (run deliberately, like run_calibration — NOT auto-started by the launch):
    ros2 run ar_explorer odom_world_calibrator --tag-id 17
    ros2 run ar_explorer odom_world_calibrator --tag-id 17 --duration 2.0 --keep-running
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

try:
    import tf2_ros
    from tf2_ros import StaticTransformBroadcaster, TransformException
except ImportError:
    print("ERROR: tf2_ros not available. Source /opt/ros/jazzy/setup.bash first.",
          file=sys.stderr)
    sys.exit(1)


ODOM_FRAME = "odom"
ARKIT_FRAME = "arkit_world"


def transform_to_matrix(transform) -> np.ndarray:
    """geometry_msgs/Transform → 4x4 homogeneous matrix."""
    t = transform.translation
    q = transform.rotation
    M = np.eye(4)
    M[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def average_matrices(samples: list[np.ndarray]) -> np.ndarray:
    """Average homogeneous transforms: mean translation + hemisphere-corrected
    quaternion mean (handles the q/-q double cover)."""
    translations = np.array([T[:3, 3] for T in samples])
    avg_t = np.mean(translations, axis=0)

    quats = np.array([R.from_matrix(T[:3, :3]).as_quat() for T in samples])
    ref = quats[0]
    for i in range(1, len(quats)):
        if np.dot(quats[i], ref) < 0:
            quats[i] = -quats[i]
    avg_q = quats.mean(axis=0)
    avg_q /= np.linalg.norm(avg_q)

    M = np.eye(4)
    M[:3, :3] = R.from_quat(avg_q).as_matrix()
    M[:3, 3] = avg_t
    return M


class OdomWorldCalibrator(Node):
    def __init__(self, tag_id: int, duration: float, keep_running: bool) -> None:
        super().__init__("odom_world_calibrator")
        self.tag_id = tag_id
        self.duration = duration
        self.keep_running = keep_running

        self.odom_tag_frame = f"tag_{tag_id}"
        self.arkit_tag_frame = f"iphone_tag_{tag_id}"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self._collecting = False
        self._collection_start = None
        self._odom_samples: list[np.ndarray] = []
        self._arkit_samples: list[np.ndarray] = []
        self._done = False

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"odom_world_calibrator — waiting for tag {tag_id} in BOTH trees "
            f"({ODOM_FRAME}→{self.odom_tag_frame}, "
            f"{ARKIT_FRAME}→{self.arkit_tag_frame})"
        )

    def _lookup(self, parent: str, child: str):
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return transform_to_matrix(tf.transform)
        except TransformException:
            return None

    def _tick(self) -> None:
        if self._done:
            return

        T_odom_from_tag = self._lookup(ODOM_FRAME, self.odom_tag_frame)
        T_arkit_from_tag = self._lookup(ARKIT_FRAME, self.arkit_tag_frame)

        if T_odom_from_tag is None or T_arkit_from_tag is None:
            if self._collecting:
                self.get_logger().warn(
                    "Lost the tag in one tree — restarting sample collection."
                )
                self._collecting = False
                self._odom_samples.clear()
                self._arkit_samples.clear()
            return

        if not self._collecting:
            self._collecting = True
            self._collection_start = time.monotonic()
            self._odom_samples.clear()
            self._arkit_samples.clear()
            self.get_logger().info(
                f"Both trees see tag {self.tag_id} — collecting "
                f"{self.duration:.1f}s of samples..."
            )

        self._odom_samples.append(T_odom_from_tag)
        self._arkit_samples.append(T_arkit_from_tag)

        if time.monotonic() - self._collection_start < self.duration:
            return

        self._finish()

    def _finish(self) -> None:
        n = len(self._odom_samples)
        T_odom_from_tag = average_matrices(self._odom_samples)
        T_arkit_from_tag = average_matrices(self._arkit_samples)

        # Same physical tag in both trees → compose to the odom↔world bridge.
        T_arkit_from_odom = T_arkit_from_tag @ np.linalg.inv(T_odom_from_tag)

        self._publish(T_arkit_from_odom)
        t = T_arkit_from_odom[:3, 3]
        self.get_logger().info(
            f"===== CALIBRATION COMPLETE ({n} samples) =====\n"
            f"{ARKIT_FRAME} → {ODOM_FRAME}  "
            f"t=({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})\n"
            "Published latched on /tf_static. The tree now resolves "
            f"{ARKIT_FRAME} → {ODOM_FRAME} → base_link → camera_link.\n"
            "=============================================="
        )

        self._collecting = False
        if not self.keep_running:
            self._done = True

    def _publish(self, M: np.ndarray) -> None:
        q = R.from_matrix(M[:3, :3]).as_quat()  # (x, y, z, w)
        t = M[:3, 3]
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ARKIT_FRAME
        msg.child_frame_id = ODOM_FRAME
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        msg.transform.rotation.x = float(q[0])
        msg.transform.rotation.y = float(q[1])
        msg.transform.rotation.z = float(q[2])
        msg.transform.rotation.w = float(q[3])
        self.static_broadcaster.sendTransform(msg)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--tag-id", type=int, default=17,
                        help="AprilTag ID seen by both cameras (default: 17).")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Seconds of samples to average (default: 2.0).")
    parser.add_argument("--keep-running", action="store_true",
                        help="Re-calibrate whenever the tag reappears, instead "
                             "of computing once and latching.")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = OdomWorldCalibrator(
        tag_id=args.tag_id,
        duration=args.duration,
        keep_running=args.keep_running,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
