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
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

try:
    import tf2_ros
    from tf2_ros import TransformException
except ImportError:
    print("ERROR: tf2_ros not available. Source /opt/ros/jazzy/setup.bash first.",
          file=sys.stderr)
    sys.exit(1)

from visualization_msgs.msg import Marker


REALSENSE_FRAME = "camera_color_optical_frame"
WORLD_FRAME = "arkit_world"
MARKER_ID_OFFSET = 2000
REALSENSE_CAMERA_MARKER_ID = 9999

# iPhone optical (OpenCV: +x right, +y down, +z forward) → iPhone ARKit camera
# (+x right, +y up, +z toward viewer). Same origin, a 180° rotation about x.
# The calibration transform is in optical convention, but the iPhone pose from
# /iphone/pose is in raw ARKit convention, so we bridge the two with this when
# composing the world anchor.
T_ARKIT_FROM_OPTICAL = np.diag([1.0, -1.0, -1.0, 1.0])


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
    def __init__(self, calibration_matrix, tag_ids: list,
                 tag_size: float):
        super().__init__("calibrated_forwarder")
        # May be None: when no JSON is found we start without a calibration and
        # wait for the live transform from calibration_server. The live path
        # always takes precedence once a message arrives.
        self.T_iphone_from_realsense = calibration_matrix
        self._live_calibration_received = False
        self.tag_ids = tag_ids
        self.tag_size = tag_size

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(Marker, "/ar_markers", 10)
        self._summarized_ids = set()
        self._published_cam = False

        # Live calibration input (primary path). TRANSIENT_LOCAL (matching the
        # server) so we receive the latest transform even if we subscribe after
        # it was published.
        calib_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            TransformStamped, "/calibration/transform",
            self._on_calibration, calib_qos
        )

        # iPhone ARKit pose (raw ARKit convention), published by the iPhone via
        # rosbridge. We cache the latest sample and freeze it at calibration
        # time as T_world_from_iphone(t0) — the anchor that maps RealSense
        # detections into the fixed ARKit world frame.
        self._latest_iphone_pose = None
        self.T_world_from_iphone_t0 = None
        self._warned_no_pose = False
        self.create_subscription(
            TransformStamped, "/iphone/pose", self._on_iphone_pose, 10
        )

        if self.T_iphone_from_realsense is not None:
            jt = self.T_iphone_from_realsense[:3, 3]
            self.get_logger().info(
                f"Loaded calibration from JSON: "
                f"tx={float(jt[0]):+.3f}, ty={float(jt[1]):+.3f}, tz={float(jt[2]):+.3f}"
            )
        else:
            self.get_logger().info(
                "No calibration JSON found — waiting for live calibration..."
            )

        self.create_timer(0.1, self._tick)

        tag_str = ", ".join(str(i) for i in tag_ids)
        self.get_logger().info(
            f"Calibrated forwarder running — watching tags [{tag_str}], "
            f"publishing yellow markers to /ar_markers"
        )

    def _on_iphone_pose(self, msg: TransformStamped):
        """Cache the latest iPhone ARKit pose (raw ARKit convention).

        Kept live until calibration, at which point _on_calibration freezes the
        most recent sample as the world anchor.
        """
        self._latest_iphone_pose = transform_to_matrix(msg.transform)

    def _on_calibration(self, msg: TransformStamped):
        """Live calibration update from calibration_server (primary path).

        Overrides whatever was loaded from JSON. Reuses transform_to_matrix so
        the reconstructed 4x4 matches the server's exactly. Also freezes the
        iPhone's current world pose: combined with the calibration, this anchors
        the RealSense in the fixed ARKit world frame so its markers no longer
        drift when the iPhone moves.
        """
        self.T_iphone_from_realsense = transform_to_matrix(msg.transform)
        t = self.T_iphone_from_realsense[:3, 3]
        if not self._live_calibration_received:
            self._live_calibration_received = True
            self.get_logger().info(
                f"Live calibration received: "
                f"tx={float(t[0]):+.3f}, ty={float(t[1]):+.3f}, tz={float(t[2]):+.3f}"
            )
        else:
            self.get_logger().info(
                f"Calibration updated: "
                f"tx={float(t[0]):+.3f}, ty={float(t[1]):+.3f}, tz={float(t[2]):+.3f}"
            )

        # Freeze T_world_from_iphone at calibration time. Assumes the iPhone is
        # held roughly still during the calibration handshake (it is — both
        # cameras must hold the tag in view), so the most recent pose ≈ the
        # pose at the calibration instant.
        if self._latest_iphone_pose is not None:
            self.T_world_from_iphone_t0 = self._latest_iphone_pose
            self._warned_no_pose = False
            wt = self.T_world_from_iphone_t0[:3, 3]
            self.get_logger().info(
                f"Froze iPhone world pose at calibration: "
                f"x={float(wt[0]):+.3f}, y={float(wt[1]):+.3f}, z={float(wt[2]):+.3f}"
            )
        else:
            self.get_logger().warn(
                "Calibration received but no /iphone/pose yet — world anchoring "
                "deferred until the iPhone starts publishing its pose."
            )

    def _tick(self):
        # Nothing to forward until a calibration is available (JSON or live).
        if self.T_iphone_from_realsense is None:
            return

        # World anchoring needs the iPhone's frozen calibration-time pose.
        if self.T_world_from_iphone_t0 is None:
            if not self._warned_no_pose:
                self._warned_no_pose = True
                self.get_logger().warn(
                    "Have calibration but no iPhone world pose yet — markers "
                    "deferred. Is the iPhone connected and publishing /iphone/pose?"
                )
            return

        now = self.get_clock().now().to_msg()

        # RealSense optical → ARKit world. Frozen at calibration time, so the
        # RealSense's world pose is fixed and its markers stay put as the iPhone
        # moves. (Future: drive T_world_from_realsense from the dog's odometry
        # so it tracks the robot as it walks.)
        T_world_from_rs = (
            self.T_world_from_iphone_t0
            @ T_ARKIT_FROM_OPTICAL
            @ self.T_iphone_from_realsense
        )

        # RealSense camera origin in the ARKit world frame.
        rs_world = T_world_from_rs @ np.array([0, 0, 0, 1])
        cam_x, cam_y, cam_z = float(rs_world[0]), float(rs_world[1]), float(rs_world[2])

        cam_marker = Marker()
        cam_marker.header.stamp = now
        cam_marker.header.frame_id = WORLD_FRAME
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
                f"RealSense camera marker: world coords "
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

            # Tag pose in the ARKit world frame: the live RealSense detection
            # composed with the frozen world anchor. As the tag moves, this
            # updates; as the iPhone moves, it does not.
            T_world_from_tag = T_world_from_rs @ T_rs_from_tag
            wx, wy, wz = T_world_from_tag[:3, 3]

            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = WORLD_FRAME
            marker.ns = "calibrated_rs"
            marker.id = int(tag_id) + MARKER_ID_OFFSET
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(wx)
            marker.pose.position.y = float(wy)
            marker.pose.position.z = float(wz)
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
                    f"→ world({float(wx):+.3f}, {float(wy):+.3f}, {float(wz):+.3f})"
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
        default_path = os.path.expanduser("~/.ros/ar_explorer_calibration.json")
        if os.path.exists(default_path):
            matrix, file_tag_id, file_tag_size = load_calibration(default_path)
            print(f"Loaded calibration from {default_path}")
        else:
            # Not fatal anymore: start without a calibration and wait for the
            # live transform from calibration_server on /calibration/transform.
            matrix = None
            file_tag_id = 0
            file_tag_size = 0.17
            print("No calibration JSON found — waiting for live calibration "
                  "on /calibration/transform")

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
