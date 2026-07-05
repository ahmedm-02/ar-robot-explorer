#!/usr/bin/env python3
"""UDP odometry relay — RECEIVER (runs on the NUC/base, ROS 2 Jazzy / Python 3.12).

Receives the dog's pose datagrams sent by dog_relay/odom_relay_sender.py (which
runs on the dog's Foxy stack) and broadcasts them into the Jazzy TF tree as

    odom -> base_link

This is the Jazzy half of the Foxy<->Jazzy bridge: the dog can't reach Jazzy
over DDS, so the pose arrives as JSON over UDP and we re-publish it as TF here.

Structurally this mirrors iphone_pose_bridge.py (receive a pose, broadcast TF,
restamp with the local clock) with two deliberate differences:

  1. NO COORDINATE / AXIS REMAP.  iphone_pose_bridge applies an ARKit->ROS
     basis change because ARKit is not a ROS frame. The dog's /leg_odom2 is
     ALREADY in ROS REP-103 convention (+x forward, +y left, +z up), so the
     translation and quaternion are passed through COMPLETELY UNCHANGED.
     Copying the iPhone bridge's _R_BASIS conversion here would silently rotate
     the dog's pose into garbage — it is intentionally absent. Do not add one.

  2. Transport is a UDP socket, not a ROS subscription.

Frame names are HARDCODED to "odom" -> "base_link": the wire payload carries no
frame names because /leg_odom2's header fields are empty on the dog (see sender).

Run standalone (not wired into the launch file by design):
    ros2 run ar_explorer odom_relay_receiver
    ros2 run ar_explorer odom_relay_receiver --ros-args -p port:=9870
"""

from __future__ import annotations

import json
import socket
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


# Hardcoded TF frames. The payload has no frame names (the dog leaves
# /leg_odom2's frame_id / child_frame_id empty), so we name them here. These are
# the dog's own intended convention (from its firmware's commented-out TF).
PARENT_FRAME = "odom"
CHILD_FRAME = "base_link"

# How long a blocking recv waits before checking rclpy.ok() again, so Ctrl-C
# shuts the receive thread down promptly instead of hanging on recvfrom.
_RECV_TIMEOUT_SEC = 0.5


class OdomRelayReceiver(Node):
    def __init__(self) -> None:
        super().__init__("odom_relay_receiver")

        self.declare_parameter("port", 9870)            # must match the sender's --port
        self.declare_parameter("bind_address", "0.0.0.0")  # listen on all interfaces

        self.port = int(self.get_parameter("port").value)
        bind_address = self.get_parameter("bind_address").value

        self.broadcaster = TransformBroadcaster(self)

        # Bind the UDP socket. SO_REUSEADDR lets us restart the node without a
        # lingering-bind error on the same port.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_address, self.port))
        self.sock.settimeout(_RECV_TIMEOUT_SEC)

        self._count = 0
        # Receive in a background thread; rclpy.spin() keeps the node alive and
        # serviced. TransformBroadcaster publishes on a topic, so calling
        # sendTransform from this thread is fine.
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"odom_relay_receiver — listening on UDP {bind_address}:{self.port}, "
            f"broadcasting TF {PARENT_FRAME} -> {CHILD_FRAME}"
        )

    def _recv_loop(self) -> None:
        while rclpy.ok():
            try:
                data, _addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue  # just loop back and re-check rclpy.ok()
            except OSError:
                break     # socket closed during shutdown
            self._handle_packet(data)

    def _handle_packet(self, data: bytes) -> None:
        try:
            d = json.loads(data.decode("utf-8"))
            px, py, pz = float(d["px"]), float(d["py"]), float(d["pz"])
            qx, qy, qz, qw = float(d["qx"]), float(d["qy"]), float(d["qz"]), float(d["qw"])
        except (ValueError, KeyError, UnicodeDecodeError) as exc:
            self.get_logger().warn(f"Dropping malformed packet: {exc}")
            return

        out = TransformStamped()

        # RESTAMP with the local Jazzy clock. The dog stamps with a monotonic
        # (steady) clock that is meaningless on this machine, so the sender's
        # stamp_sec/stamp_nanosec (present in the payload) are intentionally
        # ignored here — restamping locally is mandatory for TF lookups to work.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = PARENT_FRAME
        out.child_frame_id = CHILD_FRAME

        # PASS-THROUGH: the dog's odometry is already REP-103, so position and
        # orientation are copied verbatim. No axis remap / basis change (see the
        # module docstring) — that would corrupt the pose.
        out.transform.translation.x = px
        out.transform.translation.y = py
        out.transform.translation.z = pz
        out.transform.rotation.x = qx
        out.transform.rotation.y = qy
        out.transform.rotation.z = qz
        out.transform.rotation.w = qw

        self.broadcaster.sendTransform(out)

        self._count += 1
        if self._count == 1 or self._count % 300 == 0:
            self.get_logger().info(
                f"Broadcast TF {PARENT_FRAME} -> {CHILD_FRAME} #{self._count}"
            )

    def destroy_node(self) -> None:
        try:
            self.sock.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelayReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
