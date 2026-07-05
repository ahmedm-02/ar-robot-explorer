#!/usr/bin/env python3
"""Raw UDP pose listener — NO ROS, NO DDS, publishes nothing.

Verifies that the dog's relay datagrams are actually arriving on the NUC.
It only opens a UDP socket and prints what it receives, so it CANNOT touch
the ROS graph and CANNOT crash the dog. Transform conversion is a later step.
"""
import argparse
import json
import socket
import time

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=9870)
ap.add_argument("--bind", default="0.0.0.0")
ap.add_argument("--print-hz", type=float, default=4.0, help="max display updates/sec")
args = ap.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((args.bind, args.port))

print(f"[listener] bound UDP {args.bind}:{args.port} — pure socket, no ROS. Waiting for packets...",
      flush=True)

count = 0
last_print = 0.0
window_start = time.monotonic()
window_count = 0
min_print_interval = (1.0 / args.print_hz) if args.print_hz > 0 else 0.0

while True:
    try:
        data, addr = sock.recvfrom(4096)
    except KeyboardInterrupt:
        break

    count += 1
    window_count += 1

    try:
        d = json.loads(data.decode("utf-8"))
        px, py, pz = d["px"], d["py"], d["pz"]
        qx, qy, qz, qw = d["qx"], d["qy"], d["qz"], d["qw"]
    except (ValueError, KeyError, UnicodeDecodeError) as exc:
        print(f"[#{count}] from {addr[0]}:{addr[1]} — MALFORMED ({exc}): {data[:80]!r}", flush=True)
        continue

    now = time.monotonic()
    # First packet always prints immediately; after that, throttle for readability.
    if count == 1 or (now - last_print) >= min_print_interval:
        hz = window_count / (now - window_start) if now > window_start else 0.0
        print(
            f"[#{count:>6}] {hz:5.1f} Hz  from {addr[0]:<15} "
            f"pos x={px:+.4f} y={py:+.4f} z={pz:+.4f}  "
            f"quat x={qx:+.4f} y={qy:+.4f} z={qz:+.4f} w={qw:+.4f}",
            flush=True,
        )
        last_print = now
        window_start = now
        window_count = 0
