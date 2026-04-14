#!/usr/bin/env bash
#
# run_apriltag_pipeline.sh — one-shot launch of the full AprilTag pipeline.
#
# Starts the ROS stack (rosbridge + RealSense + MJPEG + apriltag_detector)
# and opens RViz preconfigured to show the TF tree and /apriltag_detections.
# Ctrl+C anywhere shuts the whole thing down.
#
# Usage:
#   ./scripts/run_apriltag_pipeline.sh [--tag-size METERS]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RVIZ_CONFIG="$SCRIPT_DIR/apriltag.rviz"

TAG_SIZE=""
for arg in "$@"; do
    case "$arg" in
        --tag-size) shift; TAG_SIZE="${1:-}"; shift || true ;;
        --tag-size=*) TAG_SIZE="${arg#*=}" ;;
        -h|--help)
            echo "Usage: $0 [--tag-size METERS]"
            echo "  --tag-size METERS   AprilTag edge length (default: 0.17 via detector)"
            exit 0
            ;;
    esac
done

# ── ROS env ──────────────────────────────────────────────────────────────────

set +u
source /opt/ros/jazzy/setup.bash
set -u

# ── Track child PIDs for cleanup ─────────────────────────────────────────────

STACK_PID=""
DETECTOR_PID=""
RVIZ_PID=""

cleanup() {
    echo ""
    echo "Shutting down AprilTag pipeline..."
    for pid in "$RVIZ_PID" "$DETECTOR_PID" "$STACK_PID"; do
        [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
    done
    # Sweep any stragglers spawned by the inner scripts.
    pkill -f apriltag_detector.py 2>/dev/null || true
    pkill -f stream_to_phone.py 2>/dev/null || true
    pkill -f rosbridge_websocket 2>/dev/null || true
    pkill -f realsense2_camera 2>/dev/null || true
    wait 2>/dev/null || true
    echo "All processes stopped."
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── 1. Launch the ROS stack (rosbridge + RealSense + MJPEG + detector) ──────

echo "[pipeline] Launching ROS stack..."
"$SCRIPT_DIR/start_ar_explorer.sh" --apriltag &
STACK_PID=$!

# Give it time to bring up RealSense + CameraInfo before RViz connects.
sleep 8

# If the user passed --tag-size, kill the default-sized detector and relaunch.
if [ -n "$TAG_SIZE" ]; then
    echo "[pipeline] Restarting detector with --tag-size $TAG_SIZE..."
    pkill -f apriltag_detector.py 2>/dev/null || true
    sleep 1
    python3 "$SCRIPT_DIR/apriltag_detector.py" --tag-size "$TAG_SIZE" &
    DETECTOR_PID=$!
    sleep 2
fi

# ── 2. Launch RViz ───────────────────────────────────────────────────────────

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "[pipeline] WARNING: $RVIZ_CONFIG not found; launching bare rviz2."
    rviz2 &
else
    echo "[pipeline] Launching RViz with preconfigured view..."
    rviz2 -d "$RVIZ_CONFIG" &
fi
RVIZ_PID=$!

echo ""
echo "================================================"
echo "  AprilTag pipeline running."
echo "  Point the RealSense at a tag36h11 marker."
echo "  Ctrl+C to stop everything."
echo "================================================"
echo ""

# Wait for any child to exit; cleanup trap handles the rest.
wait -n
cleanup
