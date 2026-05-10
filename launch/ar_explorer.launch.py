"""AR Explorer — top-level ROS 2 launch file.

Brings up the full pipeline:

  rosbridge ──┐
              │   /camera/camera/color/image_raw + camera_info
  RealSense ──┼─────────────────────────────────────────► apriltag_ros (RealSense)
              │                                              │
              │                                              ├─► /realsense/detections
              │                                              └─► TF: camera_color_optical_frame → tag_<id>
              │
              │   http://<asus>:8081/stream  (iPhone in-app WKWebView)
  stream_to_phone (RealSense RGB → MJPEG)
              │
              │   http://<iphone_ip>:8082/stream
  iphone_camera_bridge (iPhone MJPEG → ROS)
              │   /iphone_camera/image_raw + camera_info
              ▼
              apriltag_ros (iPhone) ─────► /iphone/detections
                                           TF: iphone_camera → iphone_tag_<id>
              │
              ▼
              tag_to_marker ─► /ar_markers (green CUBE, iPhone AR convention)

Launch arguments:

  iphone_ip       (string, default '')        Empty disables the iPhone branch.
  iphone_port     (string, default '8082')    iPhone MJPEG stream port.
  realsense       (bool,   default true)      Enable RealSense + its detector.
  apriltag        (bool,   default true)      Enable AprilTag detection.
  tag_size        (float,  default 0.17)      AprilTag edge length (meters).
  rosbridge       (bool,   default true)      Enable rosbridge_websocket on 9090.

Examples:

  # Everything on (RealSense + iPhone @ 192.168.1.42 + apriltag detection)
  ros2 launch launch/ar_explorer.launch.py iphone_ip:=192.168.1.42

  # RealSense only (no iPhone)
  ros2 launch launch/ar_explorer.launch.py

  # iPhone only (no RealSense USB device)
  ros2 launch launch/ar_explorer.launch.py iphone_ip:=192.168.1.42 realsense:=false
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(REPO_DIR, "scripts")
CONFIG_DIR = os.path.join(REPO_DIR, "config")


def generate_launch_description() -> LaunchDescription:
    iphone_ip = LaunchConfiguration("iphone_ip")
    iphone_port = LaunchConfiguration("iphone_port")
    realsense = LaunchConfiguration("realsense")
    apriltag = LaunchConfiguration("apriltag")
    tag_size = LaunchConfiguration("tag_size")
    rosbridge = LaunchConfiguration("rosbridge")

    has_iphone = PythonExpression(["'", iphone_ip, "' != ''"])
    realsense_and_apriltag = PythonExpression(
        ["'", realsense, "'.lower() == 'true' and '", apriltag, "'.lower() == 'true'"]
    )
    iphone_and_apriltag = PythonExpression(
        ["'", iphone_ip, "' != '' and '", apriltag, "'.lower() == 'true'"]
    )

    iphone_url = ["http://", iphone_ip, ":", iphone_port, "/stream"]

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare("rosbridge_server"),
            "/launch/rosbridge_websocket_launch.xml",
        ]),
        condition=IfCondition(rosbridge),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("realsense2_camera"),
            "/launch/rs_launch.py",
        ]),
        condition=IfCondition(realsense),
    )

    realsense_mjpeg = ExecuteProcess(
        cmd=["python3", "-u", os.path.join(SCRIPTS_DIR, "stream_to_phone.py")],
        name="realsense_mjpeg_stream",
        output="screen",
        condition=IfCondition(realsense),
    )

    iphone_bridge = ExecuteProcess(
        cmd=[
            "python3", "-u", os.path.join(SCRIPTS_DIR, "iphone_camera_bridge.py"),
            "--ros-args",
            "-r", "__node:=iphone_camera_bridge",
            "-r", "image_raw:=/iphone_camera/image_raw",
            "-r", "camera_info:=/iphone_camera/camera_info",
            "-p", ["mjpeg_url:=", *iphone_url],
            "-p", "frame_id:=iphone_camera",
            "-p", ["camera_info_url:=file://", os.path.join(CONFIG_DIR, "iphone_camera_info.yaml")],
        ],
        name="iphone_camera_bridge",
        output="screen",
        condition=IfCondition(has_iphone),
    )

    apriltag_realsense = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_realsense",
        namespace="realsense",
        parameters=[
            os.path.join(CONFIG_DIR, "tags_realsense.yaml"),
            {"size": tag_size},
        ],
        remappings=[
            ("image_rect", "/camera/camera/color/image_raw"),
            ("camera_info", "/camera/camera/color/camera_info"),
        ],
        output="screen",
        condition=IfCondition(realsense_and_apriltag),
    )

    apriltag_iphone = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_iphone",
        namespace="iphone",
        parameters=[
            os.path.join(CONFIG_DIR, "tags_iphone.yaml"),
            {"size": tag_size},
        ],
        remappings=[
            ("image_rect", "/iphone_camera/image_raw"),
            ("camera_info", "/iphone_camera/camera_info"),
        ],
        output="screen",
        condition=IfCondition(iphone_and_apriltag),
    )

    iphone_marker_bridge = ExecuteProcess(
        cmd=[
            "python3", "-u", os.path.join(SCRIPTS_DIR, "tag_to_marker.py"),
            "--ros-args",
            "-r", "__node:=iphone_tag_to_marker",
            "-p", "detections_topic:=/iphone/detections",
            "-p", "marker_topic:=/ar_markers",
            "-p", "tag_frame_prefix:=iphone_tag_",
            "-p", "namespace:=iphone_apriltag",
            "-p", "marker_id_offset:=1000",
            "-p", ["tag_size:=", tag_size],
        ],
        name="iphone_tag_to_marker",
        output="screen",
        condition=IfCondition(iphone_and_apriltag),
    )

    return LaunchDescription([
        DeclareLaunchArgument("iphone_ip", default_value="",
                              description="iPhone IP address (empty disables iPhone branch)."),
        DeclareLaunchArgument("iphone_port", default_value="8082",
                              description="iPhone MJPEG stream port."),
        DeclareLaunchArgument("realsense", default_value="true",
                              description="Whether to launch the RealSense camera."),
        DeclareLaunchArgument("apriltag", default_value="true",
                              description="Whether to launch AprilTag detectors."),
        DeclareLaunchArgument("tag_size", default_value="0.17",
                              description="AprilTag edge length (meters)."),
        DeclareLaunchArgument("rosbridge", default_value="true",
                              description="Whether to launch rosbridge_websocket on port 9090."),
        LogInfo(msg=["AR Explorer launching — iPhone IP: '", iphone_ip,
                     "', RealSense: ", realsense, ", AprilTag: ", apriltag]),
        rosbridge_launch,
        GroupAction([realsense_launch, realsense_mjpeg]),
        iphone_bridge,
        apriltag_realsense,
        apriltag_iphone,
        iphone_marker_bridge,
    ])
