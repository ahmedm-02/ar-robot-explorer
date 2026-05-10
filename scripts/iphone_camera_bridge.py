#!/usr/bin/env python3
"""iPhone MJPEG → ROS 2 image bridge.

Pulls the iPhone's MJPEG stream and republishes each frame as
sensor_msgs/Image on `image_raw` plus sensor_msgs/CameraInfo on `camera_info`.
The downstream apriltag_ros node consumes those topics like any other camera.

Camera intrinsics are loaded from a standard `camera_info_manager` YAML file
(camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix).
Calibrated values should replace the placeholders in
config/iphone_camera_info.yaml — the defaults are rough estimates only.
"""

from __future__ import annotations

import sys
import threading
import time
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


RETRY_INTERVAL_SEC = 3.0


def load_camera_info(path: str | None, frame_id: str) -> CameraInfo:
    """Load a camera_info_manager-format YAML file into a CameraInfo message.

    Returns a CameraInfo with placeholder intrinsics (fx=fy=500, cx=320, cy=240,
    no distortion) if `path` is None or empty.
    """
    info = CameraInfo()
    info.header.frame_id = frame_id

    if not path:
        info.width = 640
        info.height = 480
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [500.0, 0.0, 320.0,
                  0.0, 500.0, 240.0,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [500.0, 0.0, 320.0, 0.0,
                  0.0, 500.0, 240.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    with open(path) as f:
        data = yaml.safe_load(f)

    info.width = int(data["image_width"])
    info.height = int(data["image_height"])
    info.distortion_model = data.get("distortion_model", "plumb_bob")
    info.d = [float(x) for x in data["distortion_coefficients"]["data"]]
    info.k = [float(x) for x in data["camera_matrix"]["data"]]
    info.r = [float(x) for x in data["rectification_matrix"]["data"]]
    info.p = [float(x) for x in data["projection_matrix"]["data"]]
    return info


class IPhoneCameraBridge(Node):
    def __init__(self) -> None:
        super().__init__("iphone_camera_bridge")

        self.declare_parameter("mjpeg_url", "")
        self.declare_parameter("frame_id", "iphone_camera")
        self.declare_parameter("camera_info_url", "")
        self.declare_parameter("publish_rate_hz", 0.0)  # 0 = publish every frame

        self.mjpeg_url = self.get_parameter("mjpeg_url").value
        self.frame_id = self.get_parameter("frame_id").value
        info_url = self.get_parameter("camera_info_url").value
        rate = float(self.get_parameter("publish_rate_hz").value)

        if not self.mjpeg_url:
            raise RuntimeError("mjpeg_url parameter is required (e.g. http://<iphone_ip>:8082/stream)")

        # `file:///abs/path` is the camera_info_manager convention; accept either form.
        if info_url.startswith("file://"):
            info_path = urlparse(info_url).path
        else:
            info_path = info_url

        self.camera_info = load_camera_info(info_path, self.frame_id)
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "camera_info", 10)

        self.min_interval = (1.0 / rate) if rate > 0 else 0.0
        self._last_pub_time = 0.0
        self._shutdown = False

        self.get_logger().info(
            f"iPhone camera bridge — url={self.mjpeg_url}, frame_id={self.frame_id}, "
            f"intrinsics=({self.camera_info.k[0]:.1f}, {self.camera_info.k[4]:.1f}, "
            f"{self.camera_info.k[2]:.1f}, {self.camera_info.k[5]:.1f}), "
            f"size={self.camera_info.width}x{self.camera_info.height}"
        )

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._shutdown = True

    def _capture_loop(self) -> None:
        while not self._shutdown and rclpy.ok():
            cap = cv2.VideoCapture(self.mjpeg_url)
            if not cap.isOpened():
                self.get_logger().warn(
                    f"Cannot open iPhone stream ({self.mjpeg_url}) — retrying in {RETRY_INTERVAL_SEC:.0f}s"
                )
                self._sleep_interruptible(RETRY_INTERVAL_SEC)
                continue

            self.get_logger().info(f"Connected to iPhone stream: {self.mjpeg_url}")
            while not self._shutdown and rclpy.ok():
                ok, frame = cap.read()
                if not ok:
                    self.get_logger().warn(
                        f"iPhone stream disconnected — retrying in {RETRY_INTERVAL_SEC:.0f}s"
                    )
                    cap.release()
                    self._sleep_interruptible(RETRY_INTERVAL_SEC)
                    break
                self._publish(frame)

    def _sleep_interruptible(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self._shutdown:
                return
            time.sleep(0.1)

    def _publish(self, frame: np.ndarray) -> None:
        now = time.monotonic()
        if self.min_interval and (now - self._last_pub_time) < self.min_interval:
            return
        self._last_pub_time = now

        stamp = self.get_clock().now().to_msg()
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id

        info = self.camera_info
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id

        self.image_pub.publish(img_msg)
        self.info_pub.publish(info)


def main() -> None:
    rclpy.init()
    node = IPhoneCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
