#!/usr/bin/env python3
"""UDP odometry relay — SENDER (runs on the robot dog, ROS 2 Foxy / Python 3.8).

Bridges the dog's leg odometry across the Foxy<->Jazzy DDS boundary without a
DDS-level bridge. Subscribes to /leg_odom2 (nav_msgs/Odometry), throttles it,
and ships just the pose as a small JSON datagram over a plain UDP socket to the
NUC, where odom_relay_receiver.py reconstructs it into the Jazzy TF tree.

Why a hand-rolled UDP hop instead of a ROS bridge: Foxy and Jazzy cannot talk
over DDS directly, so we serialize the minimum we need (seven pose floats + the
original stamp) and send it ourselves. Plain `socket` + `json`, no extra deps,
so this single file can just be copied onto the dog and run.

Run on the dog (after sourcing the Foxy workspace):
    python3 odom_relay_sender.py --nuc-ip 192.168.1.50 --port 9870
or edit the DEFAULT_* constants below and run with no args.
"""

import argparse
import json
import socket
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


# ===========================================================================
# CONFIG — defaults; override on the command line with --nuc-ip / --port.
# ===========================================================================
DEFAULT_NUC_IP = "192.168.1.50"   # <-- receiver (NUC) IP. CHANGE ME or pass --nuc-ip.
DEFAULT_PORT = 9870               # <-- must match the receiver's port.
DEFAULT_RATE_HZ = 30.0            # relay rate; the dog publishes /leg_odom2 at ~180-200 Hz.
INPUT_TOPIC = "/leg_odom2"
# ===========================================================================


class OdomRelaySender(Node):
    def __init__(self, nuc_ip, port, rate_hz):
        super().__init__("odom_relay_sender")
        self.nuc_ip = nuc_ip
        self.port = port

        # Minimum wall-clock spacing between sent packets, in seconds. We throttle
        # by time (not by counting messages) so the relay rate is stable even if
        # the dog's publish rate drifts. time.monotonic() is immune to clock jumps.
        self.min_interval = (1.0 / rate_hz) if rate_hz > 0 else 0.0
        self._last_sent = 0.0

        # Connectionless UDP socket. We only ever send, so no bind() is needed;
        # the kernel picks a source port. Fire-and-forget: dropped packets are
        # fine because the next pose supersedes them ~33 ms later.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest = (nuc_ip, port)

        self.create_subscription(Odometry, INPUT_TOPIC, self._on_odom, 10)

        self._count = 0
        self.get_logger().info(
            "odom_relay_sender — %s -> UDP %s:%d at ~%.0f Hz"
            % (INPUT_TOPIC, nuc_ip, port, rate_hz)
        )

    def _on_odom(self, msg):
        # --- Throttle: drop everything that arrives inside the rate window. ---
        now = time.monotonic()
        if self.min_interval and (now - self._last_sent) < self.min_interval:
            return
        self._last_sent = now

        # --- Extract ONLY the pose. ---------------------------------------
        # NOTE: /leg_odom2 has EMPTY header.frame_id and child_frame_id (the dog
        # firmware never fills them in), so there is deliberately nothing to read
        # for frame names here. The receiver hardcodes "odom" -> "base_link".
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # --- Carry the original stamp even though the receiver won't use it ---
        # yet. The dog stamps with a monotonic (steady) clock that is meaningless
        # on the NUC, so the receiver restamps locally. We still ship sec/nanosec
        # now as cheap insurance: a future version that correlates detection and
        # odometry timestamps can use it without changing this wire format.
        payload = {
            "px": p.x, "py": p.y, "pz": p.z,
            "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w,
            "stamp_sec": msg.header.stamp.sec,
            "stamp_nanosec": msg.header.stamp.nanosec,
        }

        try:
            self.sock.sendto(json.dumps(payload).encode("utf-8"), self.dest)
        except OSError as exc:
            # Don't crash the node on a transient network error (e.g. NUC down).
            self.get_logger().warn("UDP send failed: %s" % exc)
            return

        self._count += 1
        if self._count == 1 or self._count % 300 == 0:
            self.get_logger().info("Relayed pose #%d to %s:%d" % (self._count, self.nuc_ip, self.port))

    def destroy_node(self):
        try:
            self.sock.close()
        finally:
            super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nuc-ip", default=DEFAULT_NUC_IP, help="Receiver (NUC) IP address.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Receiver UDP port.")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ, help="Relay rate in Hz.")
    # parse_known_args so ROS args (e.g. --ros-args ...) pass through to rclpy.
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = OdomRelaySender(args.nuc_ip, args.port, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
