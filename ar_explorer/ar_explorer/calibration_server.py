#!/usr/bin/env python3
"""Compute the calibration transform from RealSense to iPhone camera frame.

Subscribes to TF from both apriltag_ros instances (RealSense + iPhone),
waits until both see the same AprilTag, averages poses over 2 seconds,
and computes:

    T_iphone_from_realsense = T_iphone_from_tag × inverse(T_realsense_from_tag)

Both TF trees use OpenCV convention (+x right, +y down, +z forward) since
both detectors publish poses from rectified images in optical-frame coordinates. The resulting calibration
transform is therefore also in OpenCV convention. When forwarding detections
to /ar_markers for the iPhone, the forwarder must negate z (and handle y)
to match iPhone AR convention (+x right, +y up, -z forward).

Saves the result to scripts/calibration.json for offline reuse.

Usage:
    python3 scripts/calibration_server.py [--tag-id 0] [--tag-size 0.17] [--duration 2.0]
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone

import numpy as np
import rclpy
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
    """Convert a geometry_msgs/Transform to a 4x4 numpy homogeneous matrix."""
    t = transform.translation
    q = transform.rotation
    tx, ty, tz = t.x, t.y, t.z
    qx, qy, qz, qw = q.x, q.y, q.z, q.w

    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])

    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [tx, ty, tz]
    return M


class CalibrationServer(Node):
    def __init__(self, tag_id: int, tag_size: float, duration: float,
                 output_path: str):
        super().__init__("calibration_server")
        self.tag_id = tag_id
        self.tag_size = tag_size
        self.duration = duration
        self.output_path = output_path

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.calibration_matrix = None
        self._collecting = False
        self._collection_start = None
        self._rs_samples = []
        self._ip_samples = []

        self.realsense_tag_frame = f"tag_{tag_id}"
        self.iphone_tag_frame = f"iphone_tag_{tag_id}"

        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"Calibration server started — waiting for tag {tag_id} "
            f"in both TF trees ({REALSENSE_FRAME} → {self.realsense_tag_frame}, "
            f"{IPHONE_FRAME} → {self.iphone_tag_frame})"
        )

    def _lookup_tf(self, parent: str, child: str):
        """Try to look up a TF transform. Returns 4x4 matrix or None."""
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return transform_to_matrix(tf.transform)
        except TransformException:
            return None

    def _tick(self):
        if self.calibration_matrix is not None and not self._collecting:
            return

        T_rs = self._lookup_tf(REALSENSE_FRAME, self.realsense_tag_frame)
        T_ip = self._lookup_tf(IPHONE_FRAME, self.iphone_tag_frame)

        if T_rs is None or T_ip is None:
            if self._collecting:
                self.get_logger().warn(
                    "Lost sight of tag during calibration — restarting collection."
                )
                self._collecting = False
                self._rs_samples.clear()
                self._ip_samples.clear()
            return

        if not self._collecting:
            self._collecting = True
            self._collection_start = time.monotonic()
            self._rs_samples.clear()
            self._ip_samples.clear()
            self.get_logger().info(
                f"Both cameras see tag {self.tag_id} — "
                f"collecting samples for {self.duration:.1f}s..."
            )

        self._rs_samples.append(T_rs)
        self._ip_samples.append(T_ip)

        elapsed = time.monotonic() - self._collection_start
        if elapsed < self.duration:
            return

        n_rs = len(self._rs_samples)
        n_ip = len(self._ip_samples)
        self.get_logger().info(
            f"Collected {n_rs} RealSense and {n_ip} iPhone samples."
        )

        T_rs_avg = self._average_transforms(self._rs_samples)
        T_ip_avg = self._average_transforms(self._ip_samples)

        T_iphone_from_realsense = T_ip_avg @ np.linalg.inv(T_rs_avg)
        # T_iphone_from_realsense = np.linalg.inv(T_ip_avg) @ T_rs_avg
        self.calibration_matrix = T_iphone_from_realsense
        self._collecting = False

        self._print_result()
        self._save_to_file()

    def _average_transforms(self, samples):
        """Average homogeneous transforms by averaging translations and using
        the rotation from the median sample (simple but robust to outliers)."""
        translations = np.array([T[:3, 3] for T in samples])
        avg_t = np.mean(translations, axis=0)

        mid = len(samples) // 2
        avg_R = samples[mid][:3, :3]

        M = np.eye(4)
        M[:3, :3] = avg_R
        M[:3, 3] = avg_t
        return M

    def _print_result(self):
        M = self.calibration_matrix
        self.get_logger().info(
            "\n"
            "===== CALIBRATION COMPLETE =====\n"
            "Transform (RealSense → iPhone) [OpenCV convention]:\n"
            f"{np.array2string(M, precision=4, suppress_small=True)}\n"
            "================================"
        )

    def _save_to_file(self):
        data = {
            "transform": self.calibration_matrix.tolist(),
            "tag_id": self.tag_id,
            "tag_size": self.tag_size,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "realsense_frame": REALSENSE_FRAME,
            "iphone_frame": IPHONE_FRAME,
            "convention": "OpenCV (+x right, +y down, +z forward)",
        }
        with open(self.output_path, "w") as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f"Calibration saved to {self.output_path}")

    def recalibrate(self):
        """Reset state to trigger a new calibration pass."""
        self.get_logger().info("Recalibrating — collecting new samples...")
        self.calibration_matrix = None
        self._collecting = False
        self._rs_samples.clear()
        self._ip_samples.clear()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tag-id", type=int, default=17,
                        help="AprilTag ID to calibrate on (default: 17, matches 36h11.yaml).")
    parser.add_argument("--tag-size", type=float, default=0.120,
                        help="AprilTag edge length in meters (default: 0.120, matches 36h11.yaml).")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Seconds to collect samples for averaging (default: 2.0).")
    parser.add_argument("--output", type=str, default=None,
                        help="Output JSON path (default: ~/.ros/ar_explorer_calibration.json).")
    args, ros_args = parser.parse_known_args()

    output = args.output or os.path.expanduser(
        "~/.ros/ar_explorer_calibration.json"
    )
    os.makedirs(os.path.dirname(output), exist_ok=True)

    rclpy.init(args=ros_args)
    node = CalibrationServer(
        tag_id=args.tag_id,
        tag_size=args.tag_size,
        duration=args.duration,
        output_path=output,
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
