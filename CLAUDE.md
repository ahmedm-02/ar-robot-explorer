# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## System architecture

AR Explorer is a three-machine search-and-rescue AR system. Data flows in one direction for sensor feeds and bidirectionally for control:

```
Robot dog ── USB ──▶ ASUS Linux (ROS 2 Jazzy) ── Wi-Fi ──▶ iPhone (ARKit/RealityKit)
                           ▲                                      ▲
                           │                                      │
                           └──────── rosbridge (9090) ────────────┘
                                                                  │
                                                MacBook Tkinter GUI (mac_client.py)
                                                  via iPhone's NWListener (8080)
```

Two independent control paths reach the iPhone concurrently, and both must keep working:

1. **ROS path** — scripts on the ASUS publish `visualization_msgs/Marker` to `/ar_markers` (and legacy `geometry_msgs/PointStamped` on `/ar_marker_position`). The iPhone is a rosbridge **client** (`ROSBridgeClient.swift`) that subscribes over `ws://<asus-ip>:9090`.
2. **Direct path** — the iPhone itself runs an `NWListener`-based WebSocket **server** (`WebSocketServer.swift`) on port 8080. The MacBook Tkinter GUI (`ARExplorer/mac_client.py`) connects to it to place/clear markers and receive the list of bundled USDZ models.

Both paths funnel into `ARSessionManager`, which owns all AR state as an `@Observable` and is assumed to run on the main thread (ARKit delegate callbacks already are). Don't add `@MainActor` or background dispatch to it without revisiting that invariant.

### Coordinate convention

All marker coordinates are **iPhone camera-relative**: `+x` right, `+y` up, `-z` forward. `(0, 0, -3)` means 3 m directly ahead of the phone. The ROS marker `frame_id` is `"camera"` — this is not a TF frame published by the robot, it is a convention the iPhone interprets directly.

### Camera video

iPhone publishes its camera as MJPEG on `http://<iphone-ip>:8082/stream`; the ASUS-side `iphone_camera_bridge` node republishes it as `sensor_msgs/Image` + `CameraInfo` for the AprilTag detector. MJPEG is the wire format because it works in any client with no library.

## Common commands

### ASUS Linux (ROS 2 Jazzy) — this machine

The pipeline is an ament_python package at `ar_explorer/`. Build it once, then launch.

```bash
# One-time setup
sudo apt install -y ros-jazzy-apriltag-ros ros-jazzy-rosbridge-server \
    ros-jazzy-realsense2-camera python3-yaml python3-opencv

# Build & source
mkdir -p ~/ros2_ws/src && ln -snf ~/ar-robot-explorer/ar_explorer ~/ros2_ws/src/ar_explorer
cd ~/ros2_ws && colcon build --packages-select ar_explorer && source install/setup.bash

# RealSense only (matches ASUS-tested baseline)
ros2 launch ar_explorer ar_pipeline.launch.py

# Full handshake with iPhone
ros2 launch ar_explorer ar_pipeline.launch.py iphone_ip:=192.168.1.42
```

Launch args: `realsense` (true), `iphone_ip` (`''`; non-empty enables iPhone branch), `iphone_port` (`8082`), `apriltag` (true), `rosbridge` (true), `calibration` (true; gated on `iphone_ip`).

```bash
# Pre-flight check that both cameras see the same tag
ros2 run ar_explorer calibration_check

# Recompute calibration interactively (uses ~/.ros/ar_explorer_calibration.json)
ros2 run ar_explorer run_calibration

# Interactive marker publisher
ros2 run ar_explorer ar_marker_publisher
ros2 run ar_explorer ar_marker_publisher --legacy   # also publish PointStamped
```

### iPhone app (must be built on macOS)

Open `ARExplorer/ARExplorer.xcodeproj` in Xcode. The project uses Xcode 26's `SWIFT_DEFAULT_ACTOR_ISOLATION = MainActor` setting — keep that in mind when touching concurrency (see the header comment in `WebSocketServer.swift`).

### MacBook GUI

```bash
pip install websockets
python ARExplorer/mac_client.py <iphone-ip> [port]   # port defaults to 8080
```

The iPhone's IP + port is shown in the app's HUD.

## Key topics and ports

| Channel | Direction | Payload |
|---|---|---|
| `/ar_markers` (rosbridge :9090) | ASUS → iPhone | `visualization_msgs/Marker` (ADD / DELETE / DELETEALL) |
| `/ar_marker_position` (rosbridge :9090) | ASUS → iPhone | `geometry_msgs/PointStamped` (legacy) |
| `/camera/camera/color/image_raw` + `/camera_info` | ASUS internal | RealSense RGB |
| `/iphone/iphone/image_raw` + `/iphone/iphone/camera_info` | ASUS internal | iPhone camera (via `iphone_camera_bridge`) |
| `/detections` | ASUS internal | `apriltag_msgs/AprilTagDetectionArray` (RealSense) |
| `/iphone/detections` | ASUS internal | `apriltag_msgs/AprilTagDetectionArray` (iPhone) |
| iPhone NWListener :8080 | MacBook → iPhone | JSON commands: `place`, `placeModel`, `clear` |
| MJPEG :8082 `/stream` | iPhone → ASUS | multipart JPEG of iPhone camera (consumed by `iphone_camera_bridge`) |

TF frames published by the apriltag_ros instances (per `36h11.yaml` / `36h11_iphone.yaml`):
- RealSense: `camera_color_optical_frame` → `tag_17`
- iPhone:    `iphone_camera`               → `iphone_tag_17`

## Conventions worth preserving

- **No third-party iOS dependencies.** Networking uses `URLSessionWebSocketTask` (client) and `Network.framework` `NWListener` (server). Don't introduce SwiftPM packages for WebSocket/JSON work.
- **Main-thread callbacks.** Both `ROSBridgeClient` and `WebSocketServer` dispatch `onPosition` / `onMarker` / `onCommand` to the main queue so SwiftUI/`@Observable` state can be mutated directly. Preserve this contract when adding new callbacks.
- **USDZ assets live in the iOS bundle.** The ROS `mesh_resource` field contains just the base name (e.g. `fender_stratocaster`), not a URL — the iPhone looks it up in the app bundle. `ar_marker_publisher.py`'s `add model …` command uses the same convention.
- **Marker shape/action enum values** in `ROSBridgeClient.swift` mirror `visualization_msgs/Marker` constants exactly (ADD=0, DELETE=2, DELETEALL=3; SPHERE=2, CUBE=1, CYLINDER=3, MESH_RESOURCE=10). Don't renumber.

## Project status and roadmap

### Completed

- **Phase 1** — iPhone AR display. Tap-to-place markers (red boxes, blue spheres) anchored in ARKit world space. Tracking status HUD, coaching overlay, plane detection.
- **Phase 2** — Remote marker placement via the direct path. MacBook Tkinter GUI connects to iPhone's `NWListener` on port 8080, sends camera-relative coordinates. Auto-populated model picker queries the iPhone for bundled USDZ names on connect.
- **Phase 2.5** — USDZ 3D model display. Long-press to place bundled models, remote placement via `place_model` JSON command, billboard labels, world-space anchoring.
- **Phase 3** — ROS bridge integration. iPhone subscribes to rosbridge as a WebSocket client. Supports both `visualization_msgs/Marker` (full-featured: position, color, text label, shape type, mesh_resource for USDZ names, ADD/DELETE/DELETEALL actions) and legacy `geometry_msgs/PointStamped` on parallel topics.
- **RealSense integration** — D435 streaming in ROS 2 via `realsense2_camera` launch; AprilTag detection via upstream `apriltag_ros`.

### Current focus

**Phase 4 — AprilTag shared coordinate frame.** Replaces the camera-relative coordinate convention with true world-frame coordinates. Both the RealSense (robot's eyes) and the iPhone detect the same physical AprilTag; exchanged poses yield `T_world_from_robot = T_world_from_tag · inverse(T_robot_from_tag)`. After calibration, robot-frame detections map to correct ARKit world positions regardless of where the iPhone is pointing. This is the critical enabling piece for the robot's observations to appear in the correct physical location in the first responder's AR view.

### Next up

- **RealSense → marker pipeline.** Once AprilTag calibration is working, build a ROS node that processes RealSense RGB+depth frames, detects objects of interest, and publishes `visualization_msgs/Marker` messages in the shared world frame.
- **Phase 5 — Unitree dog integration.** Replace stationary RealSense with camera-on-robot. ROS version bridging between the dog (Foxy), NUC (Jazzy), and Emolga (Humble) is the known pain point — team has been debugging topic discovery and serialization across distributions.
- **Phase 6 — Autonomous exploration with VLMaps.** Vision-language maps enable natural-language goal specification for robot navigation.
- **Phase 7 — Multi-robot support.** Multiple robots feeding detections into one first responder's AR view, with deconfliction.

### Gaussian Splatting + SAM3 pipeline

Separate workstream generating USDZ files from robot-scanned objects. Outputs are bundled into the iOS app for now; future work will stream USDZs over the network and reference them via the ROS `mesh_resource` field.
