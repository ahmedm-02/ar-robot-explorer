#!/usr/bin/env python3
"""Forward RealSense AprilTag detections to the iPhone using calibration.

Reads the calibration transform (RealSense → iPhone, OpenCV convention) from
calibration.json or receives it at runtime, then:

1. Looks up TF: camera_color_optical_frame → tag_<id>  (RealSense detection)
2. Applies T_iphone_from_realsense to get the tag position in iPhone camera frame
3. Converts OpenCV convention (+y down, +z forward) → iPhone AR convention (+y up, -z forward)
4. Publishes a yellow marker to /ar_markers so it appears in the iPhone's AR view

The yellow marker should overlap with the green marker from `tag_to_marker.py`
(driven by the iPhone-side apriltag_ros instance) when calibration is correct.

Usage:
    python3 scripts/calibrated_forwarder.py --load scripts/calibration.json
    python3 scripts/calibrated_forwarder.py --matrix '<json 4x4 list>'
"""

import argparse
import json
import os
import sys

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.node import Node

try:
    import tf2_ros
    from tf2_ros import TransformException
except ImportError:
    print("ERROR: tf2_ros not available. Source /opt/ros/jazzy/setup.bash first.",
          file=sys.stderr)
    sys.exit(1)

from visualization_msgs.msg import Marker


REALSENSE_FRAME = "camera_color_optical_frame"
MARKER_ID_OFFSET = 2000
REALSENSE_CAMERA_MARKER_ID = 9999


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


class CalibratedForwarder(Node):
    def __init__(self, calibration_matrix: np.ndarray, tag_ids: list,
                 tag_size: float):
        super().__init__("calibrated_forwarder")
        self.T_iphone_from_realsense = calibration_matrix
        self.tag_ids = tag_ids
        self.tag_size = tag_size

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(Marker, "/ar_markers", 10)
        self._summarized_ids = set()
        self._published_cam = False

        self.create_timer(0.1, self._tick)

        tag_str = ", ".join(str(i) for i in tag_ids)
        self.get_logger().info(
            f"Calibrated forwarder running — watching tags [{tag_str}], "
            f"publishing yellow markers to /ar_markers"
        )

    def _tick(self):
        now = self.get_clock().now().to_msg()

        # Publish the RealSense camera position in iPhone coordinates.
        # The RealSense origin (0,0,0) transformed by the calibration matrix
        # gives its position in the iPhone camera frame.
        rs_origin = self.T_iphone_from_realsense @ np.array([0, 0, 0, 1])
        cam_x = float(rs_origin[0])
        cam_y = float(-rs_origin[1])
        cam_z = float(-rs_origin[2])

        cam_marker = Marker()
        cam_marker.header.stamp = now
        cam_marker.header.frame_id = "camera"
        cam_marker.ns = "calibrated_rs"
        cam_marker.id = REALSENSE_CAMERA_MARKER_ID
        cam_marker.type = Marker.SPHERE
        cam_marker.action = Marker.ADD
        cam_marker.pose.position.x = cam_x
        cam_marker.pose.position.y = cam_y
        cam_marker.pose.position.z = cam_z
        cam_marker.pose.orientation.w = 1.0
        cam_marker.scale.x = 0.08
        cam_marker.scale.y = 0.08
        cam_marker.scale.z = 0.08
        cam_marker.color.r = 1.0
        cam_marker.color.g = 1.0
        cam_marker.color.b = 0.0
        cam_marker.color.a = 0.8
        cam_marker.text = "RealSense"
        cam_marker.lifetime = DurationMsg(sec=2, nanosec=0)
        self.marker_pub.publish(cam_marker)

        if not self._published_cam:
            self._published_cam = True
            self.get_logger().info(
                f"RealSense camera marker: iPhone coords "
                f"({cam_x:+.3f}, {cam_y:+.3f}, {cam_z:+.3f})"
            )

        for tag_id in self.tag_ids:
            child_frame = f"tag_{tag_id}"
            try:
                tf = self.tf_buffer.lookup_transform(
                    REALSENSE_FRAME, child_frame, rclpy.time.Time()
                )
            except TransformException:
                continue

            T_rs_from_tag = transform_to_matrix(tf.transform)

            # Transform to iPhone camera frame (still OpenCV convention)
            T_ip_from_tag = self.T_iphone_from_realsense @ T_rs_from_tag

            # Extract position in OpenCV convention
            x_cv, y_cv, z_cv = T_ip_from_tag[:3, 3]

            # Convert to iPhone AR convention: +x right, +y up, -z forward
            # OpenCV: +x right, +y down, +z forward
            ar_x = float(x_cv)
            ar_y = float(-y_cv)
            ar_z = float(-z_cv)

            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = "camera"
            marker.ns = "calibrated_rs"
            marker.id = int(tag_id) + MARKER_ID_OFFSET
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
            marker.scale.z = 0.01
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.text = f"RS→iPhone Tag #{tag_id}"
            marker.lifetime = DurationMsg(sec=2, nanosec=0)
            self.marker_pub.publish(marker)

            if tag_id not in self._summarized_ids:
                self._summarized_ids.add(tag_id)
                self.get_logger().info(
                    f"Forwarding tag {tag_id}: RS({T_rs_from_tag[0,3]:+.3f}, "
                    f"{T_rs_from_tag[1,3]:+.3f}, {T_rs_from_tag[2,3]:+.3f}) "
                    f"→ iPhone({ar_x:+.3f}, {ar_y:+.3f}, {ar_z:+.3f})"
                )


def load_calibration(path: str) -> tuple:
    """Load calibration.json, return (matrix, tag_id, tag_size)."""
    with open(path) as f:
        data = json.load(f)
    matrix = np.array(data["transform"])
    tag_id = data.get("tag_id", 0)
    tag_size = data.get("tag_size", 0.17)
    return matrix, tag_id, tag_size


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--load", type=str, default=None,
                        help="Path to calibration.json file.")
    parser.add_argument("--matrix", type=str, default=None,
                        help="Inline JSON 4x4 matrix (alternative to --load).")
    parser.add_argument("--tag-ids", type=str, default=None,
                        help="Comma-separated tag IDs to watch (default: from calibration file).")
    parser.add_argument("--tag-size", type=float, default=None,
                        help="AprilTag edge length in meters (default: from calibration file).")
    args, ros_args = parser.parse_known_args()

    if args.load:
        matrix, file_tag_id, file_tag_size = load_calibration(args.load)
    elif args.matrix:
        matrix = np.array(json.loads(args.matrix))
        file_tag_id = 0
        file_tag_size = 0.17
    else:
        default_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "calibration.json"
        )
        if os.path.exists(default_path):
            matrix, file_tag_id, file_tag_size = load_calibration(default_path)
            print(f"Loaded calibration from {default_path}")
        else:
            print("ERROR: No calibration provided. Use --load or run calibration first.",
                  file=sys.stderr)
            sys.exit(1)

    tag_ids = ([int(x) for x in args.tag_ids.split(",")]
               if args.tag_ids else [file_tag_id])
    tag_size = args.tag_size if args.tag_size is not None else file_tag_size

    rclpy.init(args=ros_args)
    node = CalibratedForwarder(
        calibration_matrix=matrix,
        tag_ids=tag_ids,
        tag_size=tag_size,
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
