#!/usr/bin/env python3
"""Orchestrate the AprilTag calibration flow for AR Explorer.

Assumes the main pipeline (`launch/ar_explorer.launch.py`) is already running:
both apriltag_ros instances must be publishing detections (and TF) for the
shared tag. This script only computes the RealSense → iPhone transform and
starts the calibrated forwarder; bringing up the cameras + detectors is the
launch file's job.

Usage:
    python3 scripts/run_calibration.py [--tag-size 0.17] [--tag-id 0] [--duration 2.0]
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np

CALIBRATION_PATH = os.path.expanduser("~/.ros/ar_explorer_calibration.json")

REALSENSE_DETECTIONS_TOPIC = "/detections"
IPHONE_DETECTIONS_TOPIC = "/iphone/detections"

child_procs: list[subprocess.Popen] = []


def cleanup(*_):
    print("\nShutting down calibration helpers...")
    for p in child_procs:
        if p.poll() is None:
            try:
                p.terminate()
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
            except Exception:
                pass
    print("Done.")
    sys.exit(0)


signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)


def topic_has_messages(topic: str, timeout: float = 5.0) -> bool:
    """Return True iff `ros2 topic echo --once <topic>` succeeds within timeout."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", "--once", topic],
            capture_output=True, text=True, timeout=timeout,
        )
        return result.returncode == 0 and bool(result.stdout.strip())
    except subprocess.TimeoutExpired:
        return False


def run_calibration_server(tag_id: int, tag_size: float, duration: float):
    """Run calibration_server.py and return the loaded 4x4 transform matrix."""
    if os.path.exists(CALIBRATION_PATH):
        os.remove(CALIBRATION_PATH)

    proc = subprocess.Popen([
        "ros2", "run", "ar_explorer", "calibration_server",
        "--tag-id", str(tag_id),
        "--tag-size", str(tag_size),
        "--duration", str(duration),
        "--output", CALIBRATION_PATH,
    ])
    child_procs.append(proc)

    deadline = time.monotonic() + duration + 30
    while time.monotonic() < deadline:
        if os.path.exists(CALIBRATION_PATH):
            try:
                with open(CALIBRATION_PATH) as f:
                    data = json.load(f)
                if "transform" in data:
                    proc.wait(timeout=2)
                    return np.array(data["transform"])
            except (json.JSONDecodeError, ValueError):
                pass
        if proc.poll() is not None:
            print("ERROR: Calibration server exited unexpectedly.")
            return None
        time.sleep(0.5)

    print("ERROR: Calibration timed out.")
    return None


def launch_forwarder(tag_id: int, tag_size: float):
    proc = subprocess.Popen([
        "ros2", "run", "ar_explorer", "calibrated_forwarder",
        "--load", CALIBRATION_PATH,
        "--tag-ids", str(tag_id),
        "--tag-size", str(tag_size),
    ])
    child_procs.append(proc)
    return proc


def stop_existing_forwarder():
    """Stop any forwarder we previously spawned (no `pkill` system-wide sweep)."""
    for p in list(child_procs):
        if p.poll() is None and "calibrated_forwarder" in " ".join(p.args):
            try:
                p.terminate()
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()
            child_procs.remove(p)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--tag-size", type=float, default=0.120,
                        help="AprilTag edge length in meters (default: 0.120, matches 36h11.yaml).")
    parser.add_argument("--tag-id", type=int, default=17,
                        help="AprilTag ID to calibrate on (default: 17, matches 36h11.yaml).")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Seconds to collect calibration samples (default: 2.0).")
    args = parser.parse_args()

    print("=" * 50)
    print("  AR Explorer — Calibration")
    print("=" * 50)
    print()
    print(f"[1/3] Waiting for tag {args.tag_id} on both cameras...")
    print("  Point BOTH cameras at the same AprilTag (tag36h11).")
    print()

    rs_ok = ip_ok = False
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline and not (rs_ok and ip_ok):
        if not rs_ok and topic_has_messages(REALSENSE_DETECTIONS_TOPIC, timeout=3):
            rs_ok = True
            print(f"  RealSense ({REALSENSE_DETECTIONS_TOPIC}): ✓")
        if not ip_ok and topic_has_messages(IPHONE_DETECTIONS_TOPIC, timeout=3):
            ip_ok = True
            print(f"  iPhone    ({IPHONE_DETECTIONS_TOPIC}): ✓")
        time.sleep(1)

    if not (rs_ok and ip_ok):
        if not rs_ok:
            print(f"  TIMEOUT: no messages on {REALSENSE_DETECTIONS_TOPIC}.")
        if not ip_ok:
            print(f"  TIMEOUT: no messages on {IPHONE_DETECTIONS_TOPIC}.")
        print("\n  Is `ros2 launch launch/ar_explorer.launch.py iphone_ip:=...` running?")
        cleanup()

    print()
    print(f"[2/3] Computing calibration (averaging over {args.duration}s)...")
    matrix = run_calibration_server(args.tag_id, args.tag_size, args.duration)
    if matrix is None:
        cleanup()

    print()
    print("===== CALIBRATION COMPLETE =====")
    print("Transform (RealSense → iPhone) [OpenCV convention]:")
    print(np.array2string(matrix, precision=4, suppress_small=True))
    print("================================")
    print(f"Saved to: {CALIBRATION_PATH}")
    print()
    print("[3/3] Starting calibrated forwarder...")
    launch_forwarder(args.tag_id, args.tag_size)
    time.sleep(2)

    print()
    print("===== VERIFICATION =====")
    print("Look at your iPhone:")
    print("  GREEN box  = tag detected via iPhone camera (round-trip through ASUS)")
    print("  YELLOW box = tag detected via RealSense, transformed to iPhone coords")
    print("  Overlap = calibration is correct.")
    print("========================")
    print()
    print("Press 'r' to recalibrate, 'q' to quit.")
    print()

    while True:
        try:
            cmd = input("> ").strip().lower()
        except EOFError:
            break
        if cmd == "q":
            break
        if cmd == "r":
            print("Recalibrating...")
            stop_existing_forwarder()
            matrix = run_calibration_server(args.tag_id, args.tag_size, args.duration)
            if matrix is not None:
                print()
                print("===== RECALIBRATION COMPLETE =====")
                print(np.array2string(matrix, precision=4, suppress_small=True))
                print("==================================")
                launch_forwarder(args.tag_id, args.tag_size)
                print("Forwarder restarted with new calibration.")
            else:
                print("Recalibration failed — keep both cameras pointed at the tag.")
        else:
            print("Commands: 'r' = recalibrate, 'q' = quit")

    cleanup()


if __name__ == "__main__":
    main()
