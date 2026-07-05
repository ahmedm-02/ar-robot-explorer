#!/usr/bin/env python3
"""odom_tf_node — STANDALONE: UDP pose datagrams -> TF (odom -> base_link).

Decoupled from ar_pipeline by design: launched on its own with `python3` (or
`ros2 run` once packaged), NEVER added to ar_pipeline.launch.py. It subscribes
to NO ROS topics and publishes ONLY /tf, so it cannot disturb the existing graph.

Data path:
    dog (Foxy) --UDP/JSON--> :9870 --> [this node] --> /tf  odom -> base_link

⚠️ DEPLOYMENT CAVEAT (structure is unaffected; this is a RUN-TIME concern):
   This publishes /tf over DDS on EVERY interface. On the NUC that includes
   eno1 (the dog's cable) -> it will CRASH the dog's Foxy stack
   ("deserialization of data failed"). Do NOT run this on the NUC until its DDS
   is bound to the wifi interface only (Fast DDS interface allowlist), or run it
   off the dog's subnet. See relay_session_summary.md.
"""

import json
import math
import socket
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# How long recvfrom blocks before re-checking rclpy.ok(), so Ctrl-C is prompt.
_RECV_TIMEOUT_SEC = 0.5


class OdomTfNode(Node):
    def __init__(self):
        super().__init__("odom_tf_node")

        # --- parameters (override with: --ros-args -p port:=9870 etc.) -------
        self.declare_parameter("port", 9870)
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("normalize_quaternion", True)

        self.port = int(self.get_parameter("port").value)
        bind_address = self.get_parameter("bind_address").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.normalize = bool(self.get_parameter("normalize_quaternion").value)

        # --- OUTPUT: one TF edge ---------------------------------------------
        self.broadcaster = TransformBroadcaster(self)

        # --- INPUT: a UDP socket (NOT a ROS subscription) --------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_address, self.port))
        self.sock.settimeout(_RECV_TIMEOUT_SEC)

        # --- recv loop on a background thread; spin() keeps the node alive ---
        self._count = 0
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"odom_tf_node — UDP {bind_address}:{self.port} -> TF "
            f"{self.parent_frame} -> {self.child_frame}"
        )

    # ------------------------------------------------------------------ I/O --
    def _recv_loop(self):
        while rclpy.ok():
            try:
                data, _addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue          # loop back, re-check rclpy.ok()
            except OSError:
                break             # socket closed during shutdown
            self._handle_packet(data)

    def _handle_packet(self, data):
        try:
            d = json.loads(data.decode("utf-8"))
            px, py, pz = float(d["px"]), float(d["py"]), float(d["pz"])
            qx, qy, qz, qw = (float(d["qx"]), float(d["qy"]),
                              float(d["qz"]), float(d["qw"]))
        except (ValueError, KeyError, UnicodeDecodeError) as exc:
            self.get_logger().warn(f"dropping malformed packet: {exc}")
            return

        self.broadcaster.sendTransform(
            self._pose_to_transform(px, py, pz, qx, qy, qz, qw)
        )

        self._count += 1
        if self._count == 1 or self._count % 300 == 0:
            self.get_logger().info(
                f"broadcast {self.parent_frame} -> {self.child_frame} "
                f"#{self._count}"
            )

    # ----------------------------------------------- THE CONVERSION LOGIC ----
    # Pose -> Transform is a field copy: odometry already expresses base_link's
    # pose in odom, which IS the odom->base_link transform. The only real work
    # is (1) restamp with the local clock, (2) name the frames, (3) NO remap.
    def _pose_to_transform(self, px, py, pz, qx, qy, qz, qw):
        if self.normalize:
            n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if n > 0.0:
                qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

        tf = TransformStamped()
        # (1) RESTAMP locally — the dog's stamp is from its own clock and would
        #     cause tf2 extrapolation errors on this machine.
        tf.header.stamp = self.get_clock().now().to_msg()
        # (2) FRAME NAMES — the payload carries none; we supply them.
        tf.header.frame_id = self.parent_frame      # "odom"
        tf.child_frame_id = self.child_frame        # "base_link"
        # (3) PASS-THROUGH — dog odometry is already REP-103 (+x fwd, +y left,
        #     +z up). Copy verbatim. DO NOT add an axis/basis remap here.
        tf.transform.translation.x = px
        tf.transform.translation.y = py
        tf.transform.translation.z = pz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        return tf

    def destroy_node(self):
        try:
            self.sock.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
