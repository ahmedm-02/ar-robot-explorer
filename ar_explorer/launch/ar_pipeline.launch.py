"""AR Explorer — calibration handshake pipeline.

Built on the ASUS-tested baseline (RealSense + apriltag_ros). Adds an iPhone
branch (camera bridge + AprilTag + tag-to-marker) and calibration handshake
(calibration_server + calibrated_forwarder) when an iPhone IP is provided.

Topology (symmetric by design):

  RealSense                                iPhone
  ─────────                                ──────
  realsense2_camera_node                   iphone_camera_bridge
    ns=camera, name=camera                   ns=iphone, name=iphone
    /camera/camera/color/image_raw           /iphone/iphone/image_raw
    /camera/camera/color/camera_info         /iphone/iphone/camera_info
                │                                       │
                ▼                                       ▼
  apriltag_node (name=apriltag)            apriltag_node (ns=iphone, name=apriltag)
    /detections                              /iphone/detections
    TF: camera_color_optical_frame           TF: iphone_camera
        → tag_17                                 → iphone_tag_17
                │                                       │
                └────────────┬──────────────────────────┘
                             ▼
                  calibration_server (computes RS ↔ iPhone transform once)
                  calibrated_forwarder (RS detections → /ar_markers, yellow)
                  tag_to_marker       (iPhone detections → /ar_markers, green)
                             │
                             ▼
                  rosbridge :9090 → iPhone Swift app

Launch arguments:

  machine      (default 'all') Which machine this launch runs on. 'all' is
                                single-machine (default — every node runs, same
                                behavior as before this arg existed); 'dog'
                                brings up only the RealSense camera + its
                                AprilTag detector, intended for the NUC mounted
                                on the robot dog; 'base' brings up everything
                                else (iPhone camera bridge, iPhone AprilTag,
                                iPhone ARKit→TF pose bridge, calibration
                                handshake, rosbridge), intended for the base
                                computer talking to the iPhone.
  realsense    (default true)   Bring up RealSense + its AprilTag detector.
  iphone_ip    (default '')     iPhone IP. Non-empty activates the iPhone
                                bridge, iPhone AprilTag, tag_to_marker, and
                                (when calibration:=true) the calibration nodes.
  iphone_port  (default 8082)   iPhone MJPEG stream port.
  apriltag     (default true)   Run AprilTag detection on enabled cameras.
  rosbridge    (default true)   Run rosbridge_websocket on port 9090.
  calibration  (default true)   Run calibration_server + calibrated_forwarder
                                (only effective when iphone_ip is non-empty).

Examples:

  # Single-machine baseline — every node on one box (default machine:='all')
  ros2 launch ar_explorer ar_pipeline.launch.py

  # On the NUC mounted on the robot dog — RealSense + its AprilTag only,
  # publishing /camera/... onto the shared ROS network for the base to consume.
  ros2 launch ar_explorer ar_pipeline.launch.py machine:=dog

  # On the base computer — iPhone bridge, calibration handshake, rosbridge.
  ros2 launch ar_explorer ar_pipeline.launch.py machine:=base iphone_ip:=192.168.1.42
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('ar_explorer')
    config_dir = os.path.join(pkg_share, 'config')
    realsense_tag_config = os.path.join(config_dir, '36h11.yaml')
    iphone_tag_config = os.path.join(config_dir, '36h11_iphone.yaml')
    iphone_cam_info = 'file://' + os.path.join(config_dir, 'iphone_camera_info.yaml')

    machine = LaunchConfiguration('machine')
    realsense = LaunchConfiguration('realsense')
    iphone_ip = LaunchConfiguration('iphone_ip')
    iphone_port = LaunchConfiguration('iphone_port')
    apriltag = LaunchConfiguration('apriltag')
    rosbridge = LaunchConfiguration('rosbridge')
    calibration = LaunchConfiguration('calibration')

    has_iphone = PythonExpression(["'", iphone_ip, "' != ''"])
    realsense_and_apriltag = PythonExpression(
        ["'", realsense, "'.lower() == 'true' and '", apriltag, "'.lower() == 'true'"]
    )
    iphone_and_apriltag = PythonExpression(
        ["'", iphone_ip, "' != '' and '", apriltag, "'.lower() == 'true'"]
    )
    iphone_and_calibration = PythonExpression(
        ["'", iphone_ip, "' != '' and '", calibration, "'.lower() == 'true'"]
    )

    # ── Machine gates ────────────────────────────────────────────────────────
    # 'all' matches both, so the single-machine default keeps every node on
    # exactly as before. 'dog' / 'base' each match only their own group.
    is_dog  = PythonExpression(["'", machine, "' in ('all', 'dog')"])
    is_base = PythonExpression(["'", machine, "' in ('all', 'base')"])

    # Per-node conditions: existing logic AND the relevant machine gate. The
    # inner PythonExpressions perform() to the strings 'True'/'False' before
    # the outer eval, so the composed expression is a valid Python boolean.
    realsense_on_dog           = PythonExpression(
        ["'", realsense, "'.lower() == 'true' and ", is_dog])
    realsense_apriltag_on_dog  = PythonExpression(
        [realsense_and_apriltag, " and ", is_dog])
    rosbridge_on_base          = PythonExpression(
        ["'", rosbridge, "'.lower() == 'true' and ", is_base])
    iphone_on_base             = PythonExpression(
        [has_iphone, " and ", is_base])
    iphone_apriltag_on_base    = PythonExpression(
        [iphone_and_apriltag, " and ", is_base])
    iphone_calibration_on_base = PythonExpression(
        [iphone_and_calibration, " and ", is_base])

    iphone_url = ['http://', iphone_ip, ':', iphone_port, '/stream']

    # ── rosbridge ────────────────────────────────────────────────────────────
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('rosbridge_server'),
            '/launch/rosbridge_websocket_launch.xml',
        ]),
        condition=IfCondition(rosbridge_on_base),
    )

    # ── RealSense camera (ns=camera, name=camera → /camera/camera/...) ──────
    realsense_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_gyro': False,
            'enable_accel': False,
        }],
        output='screen',
        condition=IfCondition(realsense_on_dog),
    )

    # ── RealSense AprilTag detector (ASUS-tested setup, preserved) ──────────
    apriltag_realsense = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[realsense_tag_config],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        output='screen',
        condition=IfCondition(realsense_apriltag_on_dog),
    )

    # ── iPhone camera bridge (ns=iphone, name=iphone → /iphone/iphone/...) ──
    iphone_bridge = Node(
        package='ar_explorer',
        executable='iphone_camera_bridge',
        name='iphone',
        namespace='iphone',
        parameters=[{
            'mjpeg_url': iphone_url,
            'frame_id': 'iphone_camera',
            'camera_info_url': iphone_cam_info,
        }],
        output='screen',
        condition=IfCondition(iphone_on_base),
    )

    # ── iPhone AprilTag detector ────────────────────────────────────────────
    apriltag_iphone = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='iphone',
        parameters=[iphone_tag_config],
        remappings=[
            ('image_rect', '/iphone/iphone/image_raw'),
            ('camera_info', '/iphone/iphone/camera_info'),
        ],
        output='screen',
        condition=IfCondition(iphone_apriltag_on_base),
    )

    # ── iPhone pose → TF arkit_world → iphone_camera ────────────────────────
    # base-only: consumes the iPhone's ARKit pose for the calibration handshake.
    iphone_pose_bridge = Node(
        package='ar_explorer',
        executable='iphone_pose_bridge',
        name='iphone_pose_bridge',
        output='screen',
        condition=IfCondition(iphone_on_base),
    )

    # ── iPhone tag → /ar_markers overlay bridge ─────────────────────────────
    iphone_tag_to_marker = Node(
        package='ar_explorer',
        executable='tag_to_marker',
        name='iphone_tag_to_marker',
        parameters=[{
            'detections_topic': '/iphone/detections',
            'marker_topic': '/ar_markers',
            'tag_frame_prefix': 'iphone_tag_',
            'namespace': 'iphone_apriltag',
            'marker_id_offset': 1000,
            'tag_size': 0.120,
        }],
        output='screen',
        condition=IfCondition(iphone_apriltag_on_base),
    )

    # ── Calibration handshake ───────────────────────────────────────────────
    calibration_server = Node(
        package='ar_explorer',
        executable='calibration_server',
        name='calibration_server',
        arguments=['--tag-id', '17', '--tag-size', '0.120'],
        output='screen',
        condition=IfCondition(iphone_calibration_on_base),
    )

    calibrated_forwarder = Node(
        package='ar_explorer',
        executable='calibrated_forwarder',
        name='calibrated_forwarder',
        arguments=['--tag-ids', '17', '--tag-size', '0.120'],
        output='screen',
        condition=IfCondition(iphone_calibration_on_base),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'machine',
            default_value='all',
            choices=['all', 'dog', 'base'],
            description=(
                'Which machine this launch runs on. '
                '"all" = single-machine (default, current behavior); '
                '"dog" = RealSense nodes only (NUC on the robot); '
                '"base" = iPhone + calibration + rosbridge (base computer).'
            ),
        ),
        DeclareLaunchArgument('realsense',   default_value='true'),
        DeclareLaunchArgument('iphone_ip',   default_value=''),
        DeclareLaunchArgument('iphone_port', default_value='8082'),
        DeclareLaunchArgument('apriltag',    default_value='true'),
        DeclareLaunchArgument('rosbridge',   default_value='true'),
        DeclareLaunchArgument('calibration', default_value='true'),

        LogInfo(msg=['AR Explorer launching — machine=', machine,
                     ', realsense=', realsense,
                     ", iphone_ip='", iphone_ip, "', apriltag=", apriltag,
                     ', calibration=', calibration]),

        rosbridge_launch,

        realsense_camera,
        apriltag_realsense,

        iphone_bridge,
        apriltag_iphone,
        iphone_pose_bridge,
        iphone_tag_to_marker,

        calibration_server,
        calibrated_forwarder,
    ])
