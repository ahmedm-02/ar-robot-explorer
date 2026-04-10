#!/usr/bin/env python3
"""
stream_to_phone.py — MJPEG HTTP stream of the RealSense RGB camera.

Subscribes to /camera/camera/color/image_raw, converts each frame to JPEG,
and serves them as a multipart MJPEG stream on port 8081.

View in any browser:  http://<ASUS_IP>:8081/stream
Display on iPhone:    Load the URL in a WKWebView or UIImageView.

The MJPEG format is universally supported — no special client library needed.

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 scripts/stream_to_phone.py
"""

import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# ── Shared state ──────────────────────────────────────────────────────────────

# The latest JPEG-encoded frame, protected by a lock.
_frame_lock = threading.Lock()
_latest_jpeg: bytes | None = None

PORT = 8081
TARGET_FPS = 12  # skip frames to stay around this rate


# ── ROS 2 subscriber node ────────────────────────────────────────────────────

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__("mjpeg_streamer")
        self._last_time = 0.0
        self._min_interval = 1.0 / TARGET_FPS

        # Try the topic — camera node may not be ready yet.
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._on_image,
            1,  # queue depth 1 — always use latest frame
        )
        self.get_logger().info(
            f"Subscribed to /camera/camera/color/image_raw (target {TARGET_FPS} fps)"
        )

    def _on_image(self, msg: Image):
        global _latest_jpeg

        # Frame-rate throttle: skip frames that arrive too fast.
        now = time.monotonic()
        if now - self._last_time < self._min_interval:
            return
        self._last_time = now

        # Convert ROS Image → numpy → JPEG.
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1
            )
            if msg.encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgra8":
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            elif msg.encoding == "rgba8":
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            # bgr8 → no conversion needed

            _, jpeg = cv2.imencode(
                ".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70]
            )
            with _frame_lock:
                _latest_jpeg = jpeg.tobytes()
        except Exception as e:
            self.get_logger().warn(f"Frame conversion error: {e}")


# ── HTTP MJPEG server ────────────────────────────────────────────────────────

class MJPEGHandler(BaseHTTPRequestHandler):
    """Serves /stream as a multipart MJPEG stream."""

    def do_GET(self):
        if self.path != "/stream":
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Use /stream endpoint")
            return

        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=--frame",
        )
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        interval = 1.0 / TARGET_FPS
        try:
            while True:
                with _frame_lock:
                    jpeg = _latest_jpeg
                if jpeg is None:
                    time.sleep(0.1)
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n".encode())
                self.wfile.write(b"\r\n")
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                time.sleep(interval)
        except (BrokenPipeError, ConnectionResetError):
            pass  # client disconnected

    def log_message(self, format, *args):
        """Suppress per-request log spam."""
        pass


def run_http_server():
    server = HTTPServer(("0.0.0.0", PORT), MJPEGHandler)
    server.serve_forever()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = CameraSubscriber()

    # Get IP for display
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = "localhost"
    finally:
        s.close()

    print(f"\nRobot camera stream available at: http://{local_ip}:{PORT}/stream\n")

    # Start HTTP server in a background thread.
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
