#!/usr/bin/env python3
"""Orchestrate the full AprilTag calibration flow for AR Explorer.

Checks prerequisites, launches detectors if needed, computes the calibration
transform, starts the calibrated forwarder, and provides interactive
recalibration.

Usage:
    python3 scripts/run_calibration.py --url http://<iphone_ip>:8082/stream [--tag-size 0.17]
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_PATH = os.path.join(SCRIPT_DIR, "calibration.json")

REALSENSE_FRAME = "camera_color_optical_frame"
IPHONE_FRAME = "iphone_camera"

child_procs = []


def cleanup(*_):
    print("\nShutting down calibration pipeline...")
    for p in child_procs:
        try:
            p.terminate()
            p.wait(timeout=3)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
    subprocess.run(["pkill", "-f", "calibration_server.py"],
                   capture_output=True)
    subprocess.run(["pkill", "-f", "calibrated_forwarder.py"],
                   capture_output=True)
    print("Done.")
    sys.exit(0)


signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)


def check_process(name):
    """Check if a process matching 'name' is running."""
    result = subprocess.run(["pgrep", "-f", name], capture_output=True)
    return result.returncode == 0


def check_url_reachable(url):
    """Quick check if a URL is reachable. For MJPEG streams, curl may not get
    a clean HTTP code since the response streams forever. Accept any connection
    that doesn't immediately refuse — receiving any bytes counts as success."""
    try:
        result = subprocess.run(
            ["curl", "-s", "--max-time", "3", "-o", "/dev/null",
             "-w", "%{http_code}", url],
            capture_output=True, text=True, timeout=5,
        )
        code = result.stdout.strip()
        # MJPEG streams return 200 but curl may report 000 if it times out
        # mid-stream. A successful connection that transfers bytes is enough.
        if result.returncode == 0:
            return True
        # curl exit code 28 = timeout, which for a streaming endpoint means
        # we connected and received data until --max-time expired — that's OK.
        if result.returncode == 28:
            return True
        return code not in ("000", "")
    except subprocess.TimeoutExpired:
        # If subprocess itself timed out, curl was connected and streaming
        return True


def check_topic_has_messages(topic, timeout=5):
    """Check if a ROS topic is actively publishing by grabbing one message."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", "--once", topic],
            capture_output=True, text=True, timeout=timeout,
        )
        return result.returncode == 0 and len(result.stdout.strip()) > 0
    except subprocess.TimeoutExpired:
        return False


def run_calibration_server(tag_id, tag_size, duration):
    """Run the calibration server and wait for it to produce calibration.json."""
    if os.path.exists(CALIBRATION_PATH):
        os.remove(CALIBRATION_PATH)

    proc = subprocess.Popen([
        sys.executable, os.path.join(SCRIPT_DIR, "calibration_server.py"),
        "--tag-id", str(tag_id),
        "--tag-size", str(tag_size),
        "--duration", str(duration),
        "--output", CALIBRATION_PATH,
    ])
    child_procs.append(proc)

    start = time.monotonic()
    timeout = duration + 30
    while time.monotonic() - start < timeout:
        if os.path.exists(CALIBRATION_PATH):
            try:
                with open(CALIBRATION_PATH) as f:
                    data = json.load(f)
                if "transform" in data:
                    return np.array(data["transform"])
            except (json.JSONDecodeError, ValueError):
                pass
        if proc.poll() is not None:
            print("ERROR: Calibration server exited unexpectedly.")
            return None
        time.sleep(0.5)

    print("ERROR: Calibration timed out.")
    return None


def launch_forwarder(tag_id, tag_size):
    """Launch the calibrated forwarder as a background process."""
    proc = subprocess.Popen([
        sys.executable, os.path.join(SCRIPT_DIR, "calibrated_forwarder.py"),
        "--load", CALIBRATION_PATH,
        "--tag-ids", str(tag_id),
        "--tag-size", str(tag_size),
    ])
    child_procs.append(proc)
    return proc


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", required=True,
                        help="iPhone MJPEG stream URL, e.g. http://<iphone_ip>:8082/stream")
    parser.add_argument("--tag-size", type=float, default=0.17,
                        help="AprilTag edge length in meters (default: 0.17).")
    parser.add_argument("--tag-id", type=int, default=0,
                        help="AprilTag ID to calibrate on (default: 0).")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Seconds to collect calibration samples (default: 2.0).")
    args = parser.parse_args()

    rs_tag_frame = f"tag_{args.tag_id}"
    ip_tag_frame = f"iphone_tag_{args.tag_id}"

    # ── Step 1: Check prerequisites ──────────────────────────────────────────

    print("=" * 50)
    print("  AR Explorer — Calibration Pipeline")
    print("=" * 50)
    print()

    print("[1/6] Checking prerequisites...")

    if check_process("rosbridge_websocket"):
        print("  rosbridge:     ✓")
    else:
        print("  rosbridge:     ✗  (run start_ar_explorer.sh first)")
        sys.exit(1)

    if check_process("realsense2_camera"):
        print("  RealSense:     ✓")
    else:
        print("  RealSense:     ✗  (no camera node running)")
        sys.exit(1)

    print(f"  iPhone stream: checking {args.url}...")
    if check_url_reachable(args.url):
        print(f"  iPhone stream: ✓")
    else:
        print(f"  iPhone stream: ✗  (cannot reach {args.url})")
        print("  Make sure the iPhone app is running and streaming on port 8082.")
        sys.exit(1)

    print()

    # ── Step 2: Launch RealSense detector if needed ──────────────────────────

    print("[2/6] RealSense AprilTag detector...")
    if check_process("apriltag_detector.py"):
        print("  Already running.")
    else:
        print("  Starting apriltag_detector.py...")
        proc = subprocess.Popen([
            sys.executable, os.path.join(SCRIPT_DIR, "apriltag_detector.py"),
            "--tag-size", str(args.tag_size),
        ])
        child_procs.append(proc)
        time.sleep(3)

    print()

    # ── Step 3: Launch iPhone detector if needed ─────────────────────────────

    print("[3/6] iPhone AprilTag processor...")
    if check_process("iphone_apriltag_processor.py"):
        print("  Already running.")
    else:
        print(f"  Starting iphone_apriltag_processor.py ({args.url})...")
        proc = subprocess.Popen([
            sys.executable, os.path.join(SCRIPT_DIR, "iphone_apriltag_processor.py"),
            "--url", args.url,
            "--tag-size", str(args.tag_size),
        ])
        child_procs.append(proc)
        time.sleep(3)

    print()

    # ── Step 4: Wait for both cameras to see the tag ─────────────────────────

    print(f"[4/6] Waiting for tag {args.tag_id} detection...")
    print("  Point BOTH cameras at the same AprilTag (tag36h11).")
    print()

    rs_ok = False
    ip_ok = False
    start = time.monotonic()
    timeout = 60

    while time.monotonic() - start < timeout:
        if not rs_ok:
            if check_topic_has_messages("/apriltag_detections", timeout=5):
                rs_ok = True
                print(f"  RealSense sees tag {args.tag_id}: ✓")

        if not ip_ok:
            # Check /ar_markers for iPhone-sourced detections (id >= 1000)
            if check_topic_has_messages("/ar_markers", timeout=5):
                ip_ok = True
                print(f"  iPhone sees tag {args.tag_id}:    ✓")

        if rs_ok and ip_ok:
            break

        time.sleep(1)

    if not (rs_ok and ip_ok):
        print()
        if not rs_ok:
            print("  TIMEOUT: RealSense never detected the tag.")
            print("  Check that the RealSense can see the AprilTag.")
        if not ip_ok:
            print("  TIMEOUT: iPhone never detected the tag.")
            print("  Check that the iPhone stream is working and pointing at the tag.")
        print("  Try again.")
        cleanup()

    print()
    print(f"  Both cameras see tag {args.tag_id} — calibrating...")
    print()

    # ── Step 5: Compute calibration ──────────────────────────────────────────

    print(f"[5/6] Computing calibration (averaging over {args.duration}s)...")

    matrix = run_calibration_server(args.tag_id, args.tag_size, args.duration)
    if matrix is None:
        print("  Calibration failed.")
        cleanup()

    print()
    print("===== CALIBRATION COMPLETE =====")
    print("Transform (RealSense → iPhone) [OpenCV convention]:")
    print(np.array2string(matrix, precision=4, suppress_small=True))
    print("================================")
    print()
    print(f"Saved to: {CALIBRATION_PATH}")
    print()

    # ── Step 6: Start forwarder and print instructions ───────────────────────

    print("[6/6] Starting calibrated forwarder...")
    launch_forwarder(args.tag_id, args.tag_size)
    time.sleep(2)

    print()
    print("===== VERIFICATION =====")
    print("Look at your iPhone:")
    print("  GREEN box  = tag detected via iPhone camera stream (round-trip through ASUS)")
    print("  YELLOW box = tag detected via RealSense, TRANSFORMED to iPhone coordinates")
    print("  If both boxes overlap on the physical tag, calibration is correct!")
    print("========================")
    print()
    print("ADVANCED TEST: Point iPhone away from tag. Yellow box should stay")
    print("at the tag's real-world position (ARKit tracks world coordinates).")
    print()
    print("Press 'r' to recalibrate, 'q' to quit.")
    print()

    # ── Interactive loop ─────────────────────────────────────────────────────

    while True:
        try:
            cmd = input("> ").strip().lower()
        except EOFError:
            break

        if cmd == "q":
            break
        elif cmd == "r":
            print("Recalibrating...")
            # Kill existing forwarder and calibration server
            for p in list(child_procs):
                try:
                    p.terminate()
                    p.wait(timeout=2)
                except Exception:
                    pass
            subprocess.run(["pkill", "-f", "calibrated_forwarder.py"],
                           capture_output=True)
            subprocess.run(["pkill", "-f", "calibration_server.py"],
                           capture_output=True)
            child_procs.clear()

            # Re-launch detectors if they were killed
            if not check_process("apriltag_detector.py"):
                proc = subprocess.Popen([
                    sys.executable, os.path.join(SCRIPT_DIR, "apriltag_detector.py"),
                    "--tag-size", str(args.tag_size),
                ])
                child_procs.append(proc)

            if not check_process("iphone_apriltag_processor.py"):
                proc = subprocess.Popen([
                    sys.executable, os.path.join(SCRIPT_DIR, "iphone_apriltag_processor.py"),
                    "--url", args.url,
                    "--tag-size", str(args.tag_size),
                ])
                child_procs.append(proc)

            time.sleep(3)

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
