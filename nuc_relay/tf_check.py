#!/usr/bin/env python3
"""Subscribe to /tf, print the first few transforms odom_tf_node broadcasts,
then exit. A reliable verifier (no tf2_echo CLI hangs)."""
import time

import rclpy
from tf2_msgs.msg import TFMessage

rclpy.init()
node = rclpy.create_node("tf_check")
got = []
node.create_subscription(TFMessage, "/tf", lambda m: got.extend(m.transforms), 10)

print("[tf_check] subscribed to /tf, waiting up to 10 s...", flush=True)
deadline = time.time() + 10.0
while rclpy.ok() and len(got) < 5 and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)

now = node.get_clock().now().nanoseconds
for i, t in enumerate(got[:5], 1):
    tr, q = t.transform.translation, t.transform.rotation
    stamp_ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
    age_ms = (now - stamp_ns) / 1e6
    print(f"#{i}  {t.header.frame_id} -> {t.child_frame_id}  "
          f"t=({tr.x:+.4f}, {tr.y:+.4f}, {tr.z:+.4f})  "
          f"q=({q.x:+.4f}, {q.y:+.4f}, {q.z:+.4f}, {q.w:+.4f})  "
          f"stamp_age={age_ms:+.1f} ms", flush=True)

print(f"[tf_check] total /tf transforms received: {len(got)}", flush=True)
rclpy.shutdown()
