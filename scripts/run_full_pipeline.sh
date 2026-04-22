#!/usr/bin/env bash
#
# run_full_pipeline.sh — One-command launch for the entire AR Explorer pipeline.
#
# Prompts for the iPhone's IP, then:
#   1. Starts the ROS stack (rosbridge, RealSense, MJPEG, both AprilTag detectors)
#   2. Opens the RealSense camera stream in a browser
#   3. Launches RViz with the AprilTag config
#   4. Runs the calibration flow
#
# Usage:
#   ./scripts/run_full_pipeline.sh [--tag-size 0.17]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TAG_SIZE="0.17"

for arg in "$@"; do
    case "$arg" in
        --tag-size=*) TAG_SIZE="${arg#*=}" ;;
        --tag-size) shift; TAG_SIZE="${1:-0.17}" ;;
        -h|--help)
            echo "Usage: $0 [--tag-size METERS]"
            exit 0
            ;;
    esac
done

# ── Source ROS 2 ─────────────────────────────────────────────────────────────

set +u
source /opt/ros/jazzy/setup.bash
set -u

# ── Prompt for iPhone IP ─────────────────────────────────────────────────────

echo "================================================"
echo "  AR Explorer — Full Pipeline Launcher"
echo "================================================"
echo ""
read -rp "Enter iPhone IP (from the app's HUD): " IPHONE_IP

if [ -z "$IPHONE_IP" ]; then
    echo "ERROR: No IP entered." >&2
    exit 1
fi

IPHONE_STREAM="http://${IPHONE_IP}:8082/stream"
echo ""
echo "  iPhone stream URL: $IPHONE_STREAM"
echo "  Tag size:          ${TAG_SIZE}m"
echo ""

# ── Track child PIDs for cleanup ─────────────────────────────────────────────

PIDS=()

cleanup() {
    echo ""
    echo "Shutting down full pipeline..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    pkill -f start_ar_explorer.sh 2>/dev/null || true
    pkill -f apriltag_detector.py 2>/dev/null || true
    pkill -f iphone_apriltag_processor.py 2>/dev/null || true
    pkill -f calibration_server.py 2>/dev/null || true
    pkill -f calibrated_forwarder.py 2>/dev/null || true
    pkill -f run_calibration.py 2>/dev/null || true
    pkill -f rosbridge_websocket 2>/dev/null || true
    pkill -f realsense2_camera 2>/dev/null || true
    pkill -f stream_to_phone.py 2>/dev/null || true
    wait 2>/dev/null || true
    echo "All processes stopped."
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── 1. Start the ROS stack ───────────────────────────────────────────────────

echo "[1/4] Starting ROS stack (rosbridge + RealSense + MJPEG + AprilTag detectors)..."
"$SCRIPT_DIR/start_ar_explorer.sh" --apriltag --iphone-stream "$IPHONE_STREAM" &
PIDS+=($!)
sleep 8

# ── 2. Open RealSense stream in browser ─────────────────────────────────────

LOCAL_IP=$(hostname -I | awk '{print $1}')
STREAM_URL="http://localhost:8081/stream"

echo "[2/4] Opening RealSense camera stream in browser..."
echo "  Stream URL: $STREAM_URL (also http://${LOCAL_IP}:8081/stream)"
xdg-open "$STREAM_URL" 2>/dev/null || echo "  Could not open browser — open $STREAM_URL manually."

# ── 3. Launch RViz ───────────────────────────────────────────────────────────

RVIZ_CONFIG="$SCRIPT_DIR/apriltag.rviz"
echo "[3/4] Launching RViz..."
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" &
else
    rviz2 &
fi
PIDS+=($!)
sleep 2

# ── 4. Run calibration ──────────────────────────────────────────────────────

echo "[4/4] Starting calibration flow..."
echo ""
python3 "$SCRIPT_DIR/run_calibration.py" --url "$IPHONE_STREAM" --tag-size "$TAG_SIZE"

# If calibration script exits, clean up everything
cleanup
