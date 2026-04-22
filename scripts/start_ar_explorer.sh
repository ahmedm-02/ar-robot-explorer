#!/usr/bin/env bash
#
# start_ar_explorer.sh — One-command startup for the AR Explorer ROS infrastructure.
#
# Launches rosbridge, RealSense camera (if connected), and MJPEG stream.
# Ctrl+C cleanly shuts down all background processes.
#
# Usage:
#   ./scripts/start_ar_explorer.sh [--apriltag] [--iphone-stream URL]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

RUN_APRILTAG=false
IPHONE_STREAM_URL=""
while [ $# -gt 0 ]; do
    case "$1" in
        --apriltag) RUN_APRILTAG=true; shift ;;
        --iphone-stream)
            shift
            IPHONE_STREAM_URL="${1:-}"
            if [ -z "$IPHONE_STREAM_URL" ]; then
                echo "ERROR: --iphone-stream requires a URL argument." >&2
                exit 1
            fi
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--apriltag] [--iphone-stream URL]"
            echo "  --apriltag              Also launch apriltag_detector.py against the RealSense RGB stream."
            echo "  --iphone-stream URL     Launch iphone_apriltag_processor.py consuming the iPhone's MJPEG stream."
            exit 0
            ;;
        *) echo "Unknown argument: $1" >&2; exit 1 ;;
    esac
done

# ── Source ROS 2 ──────────────────────────────────────────────────────────────

set +u
source /opt/ros/jazzy/setup.bash
set -u

# ── Gather info ───────────────────────────────────────────────────────────────

LOCAL_IP=$(hostname -I | awk '{print $1}')
PIDS=()

# ── Cleanup on exit ──────────────────────────────────────────────────────────

cleanup() {
    echo ""
    echo "Shutting down AR Explorer..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null || true
        fi
    done
    echo "All processes stopped."
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── 1. Rosbridge ──────────────────────────────────────────────────────────────

if pgrep -f "rosbridge_websocket" > /dev/null 2>&1; then
    echo "[rosbridge] Already running."
else
    echo "[rosbridge] Starting rosbridge_websocket on port 9090..."
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /dev/null 2>&1 &
    PIDS+=($!)
    sleep 3
    if kill -0 "${PIDS[-1]}" 2>/dev/null; then
        echo "[rosbridge] Running."
    else
        echo "[rosbridge] ERROR: Failed to start. Check 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml' manually."
        exit 1
    fi
fi

# ── 2. RealSense camera ──────────────────────────────────────────────────────

REALSENSE_CONNECTED=false
if lsusb | grep -qi "intel.*realsense\|8086:0b3a\|8086:0b07"; then
    REALSENSE_CONNECTED=true
    if pgrep -f "realsense2_camera" > /dev/null 2>&1; then
        echo "[realsense] Camera node already running."
    else
        echo "[realsense] Camera detected. Starting ROS camera node..."
        ros2 launch realsense2_camera rs_launch.py > /dev/null 2>&1 &
        PIDS+=($!)
        sleep 3
        if kill -0 "${PIDS[-1]}" 2>/dev/null; then
            echo "[realsense] Camera node running."
        else
            echo "[realsense] WARNING: Camera node failed to start."
            REALSENSE_CONNECTED=false
        fi
    fi
else
    echo "[realsense] No RealSense detected — skipping camera."
fi

# ── 3. MJPEG stream ──────────────────────────────────────────────────────────

if $REALSENSE_CONNECTED; then
    if pgrep -f "stream_to_phone" > /dev/null 2>&1; then
        echo "[stream] MJPEG stream already running."
    else
        echo "[stream] Starting MJPEG camera stream on port 8081..."
        python3 "$SCRIPT_DIR/stream_to_phone.py" > /dev/null 2>&1 &
        PIDS+=($!)
        sleep 2
        if kill -0 "${PIDS[-1]}" 2>/dev/null; then
            echo "[stream] MJPEG stream running."
        else
            echo "[stream] WARNING: MJPEG stream failed to start."
        fi
    fi
fi

# ── 4. AprilTag detector (opt-in) ─────────────────────────────────────────────

if $RUN_APRILTAG; then
    if ! $REALSENSE_CONNECTED; then
        echo "[apriltag] WARNING: --apriltag requested but RealSense not running; skipping."
    elif pgrep -f "apriltag_detector.py" > /dev/null 2>&1; then
        echo "[apriltag] Detector already running."
    else
        echo "[apriltag] Starting AprilTag detector (tag36h11)..."
        python3 "$SCRIPT_DIR/apriltag_detector.py" &
        PIDS+=($!)
        sleep 2
        if kill -0 "${PIDS[-1]}" 2>/dev/null; then
            echo "[apriltag] Detector running."
        else
            echo "[apriltag] WARNING: Detector failed to start (check pupil-apriltags install)."
        fi
    fi
fi

# ── 5. iPhone AprilTag processor (opt-in) ────────────────────────────────────

if [ -n "$IPHONE_STREAM_URL" ]; then
    if pgrep -f "iphone_apriltag_processor.py" > /dev/null 2>&1; then
        echo "[iphone-apriltag] Processor already running."
    else
        echo "[iphone-apriltag] Starting iPhone AprilTag processor (consuming $IPHONE_STREAM_URL)..."
        python3 "$SCRIPT_DIR/iphone_apriltag_processor.py" --url "$IPHONE_STREAM_URL" &
        PIDS+=($!)
        sleep 2
        if kill -0 "${PIDS[-1]}" 2>/dev/null; then
            echo "[iphone-apriltag] iPhone AprilTag processor running."
        else
            echo "[iphone-apriltag] WARNING: Processor failed to start."
        fi
    fi
fi

# ── Summary ───────────────────────────────────────────────────────────────────

echo ""
echo "================================================"
echo "  AR Explorer — ROS Infrastructure Running"
echo "================================================"
echo "  ASUS IP:              $LOCAL_IP"
echo "  iPhone ROS bridge:    $LOCAL_IP:9090"
if $REALSENSE_CONNECTED; then
echo "  Robot camera stream:  http://$LOCAL_IP:8081/stream"
fi
echo "================================================"
echo ""
echo "Key topics:"
echo "  /ar_markers                          — visualization_msgs/Marker"
echo "  /ar_marker_position                  — geometry_msgs/PointStamped (legacy)"
if $REALSENSE_CONNECTED; then
echo "  /camera/camera/color/image_raw       — sensor_msgs/Image (RGB)"
echo "  /camera/camera/depth/image_rect_raw  — sensor_msgs/Image (depth)"
fi
if $RUN_APRILTAG; then
echo "  /apriltag_detections                 — visualization_msgs/Marker (RViz)"
echo "  /tf                                  — camera_color_optical_frame → tag_<id>"
echo ""
echo "RViz: to visualize AprilTag detections, run: rviz2"
echo "Then set Fixed Frame to 'camera_color_optical_frame' and add a TF display"
echo "+ Marker display on /apriltag_detections"
fi
if [ -n "$IPHONE_STREAM_URL" ]; then
echo "  /ar_markers (iphone_apriltag ns)     — iPhone-detected tags → green markers"
echo "  /tf                                  — iphone_camera → iphone_tag_<id>"
echo ""
echo "iPhone AprilTag processor:  consuming $IPHONE_STREAM_URL"
fi
echo ""
echo "Press Ctrl+C to stop all processes."
echo ""

# ── Keep running ──────────────────────────────────────────────────────────────

wait
