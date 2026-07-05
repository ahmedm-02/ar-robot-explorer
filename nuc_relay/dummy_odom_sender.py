#!/usr/bin/env python3
"""Dummy odom sender — fires synthetic pose JSON at odom_tf_node for testing.

No ROS, no dog. Mimics the dog's relay wire format exactly, using the real
stationary pose we captured earlier. Use --animate to sweep x so you can see
the published TF track the input.
"""
import argparse
import json
import math
import socket
import time

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=9870)
ap.add_argument("--rate", type=float, default=30.0)
ap.add_argument("--count", type=int, default=0, help="number to send; 0 = forever")
ap.add_argument("--animate", action="store_true", help="sweep px in a sine wave")
args = ap.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dest = (args.host, args.port)

# The real stationary pose the dog sent earlier (verified on the NUC).
BASE = dict(px=0.5683, py=0.0026, pz=0.3307, qx=0.0, qy=0.0, qz=-0.0148, qw=0.9999)

period = (1.0 / args.rate) if args.rate > 0 else 0.0
print(f"[dummy] -> {args.host}:{args.port} @ {args.rate} Hz "
      f"({'animated' if args.animate else 'stationary'})", flush=True)

i = 0
t0 = time.monotonic()
while True:
    d = dict(BASE)
    if args.animate:
        d["px"] = BASE["px"] + 0.5 * math.sin((time.monotonic() - t0))
    d["stamp_sec"] = int(time.time())     # dog-style stamp (node ignores it)
    d["stamp_nanosec"] = 0
    sock.sendto(json.dumps(d).encode("utf-8"), dest)

    i += 1
    if i == 1 or i % 30 == 0:
        print(f"[dummy] sent #{i}  px={d['px']:+.4f}", flush=True)
    if args.count and i >= args.count:
        break
    if period:
        time.sleep(period)
