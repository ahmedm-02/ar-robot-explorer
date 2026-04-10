#!/usr/bin/env bash
#
# start_ar_explorer.sh — One-command startup for the AR Explorer ROS infrastructure.
#
# Launches rosbridge, RealSense camera (if connected), and MJPEG stream.
# Ctrl+C cleanly shuts down all background processes.
#
# Usage:
#   ./scripts/start_ar_explorer.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ── Source ROS 2 ──────────────────────────────────────────────────────────────

source /opt/ros/jazzy/setup.bash

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
echo ""
echo "Press Ctrl+C to stop all processes."
echo ""

# ── Keep running ──────────────────────────────────────────────────────────────

wait
