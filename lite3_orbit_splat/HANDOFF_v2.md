# Lite3 Orbit + Splat + AR — Lab Partner Handoff

**Author:** Ayaan
**Last updated:** 2026-06-04
**Purpose:** Exact runbook for the same computers, containers, accounts, paths, and commands used in this lab's setup.

---

## How to use this document

This is structured in three parts:

- **Part I — Overview**: read this first to get your bearings. Architecture, machine map, what files live where, and the small set of gotchas that will burn you if you don't know them.
- **Part II — Runbook**: copy-paste exact procedures for boot, capture, Nav2, AR, splat training, and shutdown.
- **Part III — Reference**: lookup tables for UDP command codes, frame IDs, troubleshooting, and external docs.

When a command depends on which machine you're on, the section header tells you. Commands are exact unless a line says to substitute a value.

---

# Part I — Overview

## 0. Read this first

Commands are written for the same accounts, Docker container names, ROS domains, paths, and network setup we used.

- Foxy robot/orbit/splat stack lives in Docker container `foxy_ros` on the NUC. Use `ROS_DOMAIN_ID=0`.
- Jazzy AR/iPhone/AprilTag stack lives in Docker container `jazzy_ar` on the NUC. Use `ROS_DOMAIN_ID=42`.
- The RealSense camera can only be owned by one stack at a time. **Do not run Foxy capture and Jazzy AR camera simultaneously.**
- The dog only obeys `/cmd_vel` after the dog-side navigation + move mode script is run. The mode session times out; rerun it before every motion session.
- If a Docker container already exists, do not run `docker run` again with the same name. Use `docker exec` or `docker start`.

## 1. System architecture

### 1.1 Topology diagram

```
                ┌─────────────────────┐
                │  Developer laptop   │
                └──┬──────────────┬───┘
                   │              │
                SSH│              │SSH/Tailscale
                   ▼              ▼
       ┌─────────────────┐   ┌───────────────────────┐
       │  Lite3 dog      │   │  Emolga (GPU)         │
       │  Jetson         │   │  100.110.138.76       │
       │  ysc@<dog-ip>   │   │  Nerfstudio,          │
       │                 │   │   splatfacto          │
       │  ~/start_       │   └───────────────────────┘
       │   transfer.sh   │              ▲
       │  publishes      │              │
       │   /leg_odom2    │              │ video pulled
       │                 │              │ from NUC via
       │  QNX motion     │              │ docker cp + scp
       │  host           │              │
       │  127.0.0.1:43893│   ┌──────────┴────────────────────┐
       └────────┬────────┘   │  NUC (locobot2)               │
                │            │  100.72.194.78                │
   Direct Eth   │            │                               │
   192.168.186.x│            │  ┌────────────┐ ┌───────────┐ │
                ▼            │  │ foxy_ros   │ │ jazzy_ar  │ │
       ┌────────────────┐    │  │ DOMAIN=0   │ │ DOMAIN=42 │ │
       │ dog eth1:      │────┼─▶│ Nav2,      │ │ AprilTag, │ │
       │ 192.168.186.2  │    │  │ orbit,     │ │ rosbridge │ │
       └────────────────┘    │  │ splat cap. │ │           │ │
                             │  │ RealSense* │ │RealSense* │ │
                             │  └────────────┘ └───────────┘ │
                             │   * mutually exclusive        │
                             │     time-share, NEVER both    │
                             └────────────┬──────────────────┘
                                          │ ARMLab/ARMLab-5G LAN
                                          │ (DDS multicast)
                                          ▼
                              ┌────────────────────────┐
                              │  AR base computer      │
                              │  native ROS2 Jazzy     │
                              │  DOMAIN=42             │
                              └──────────┬─────────────┘
                                         │ rosbridge ws
                                         ▼
                              ┌────────────────────────┐
                              │  iPhone (ARExplorer    │
                              │   app, MJPEG :8082)    │
                              └────────────────────────┘
```

The dog talks to the NUC over direct Ethernet for the ROS2 transfer stack. The QNX motion host inside the dog is reachable only from the dog itself at `127.0.0.1` — the Jetson does not route between the direct-Ethernet subnet and the motion host's internal network.

Emolga is reached over Tailscale from the NUC and laptop. The AR base ↔ NUC link is on the ARMLab LAN, **not** Tailscale, because raw DDS multicast does not traverse Tailscale.

### 1.2 Machine map and exact identities

| Machine | Account / address | Purpose / exact notes |
|---|---|---|
| **NUC / locobot2** | `ssh dog@100.72.194.78` | Main robot computer. Runs `foxy_ros` and `jazzy_ar` Docker containers. Tailscale IP confirmed as 100.72.194.78. |
| **Dog / Lite3 Jetson** | `ssh ysc@192.168.0.129` OR `ssh ysc@<current dog IP>` | Dog-side stack. Runs `~/start_transfer.sh`. Dog had `wlan0=192.168.0.129/24` in one confirmed network state. If it changes, use current dog IP. |
| **Dog direct Ethernet** | dog eth1: 192.168.186.2 / NUC eno1: 192.168.186.3 | NUC-to-dog link. Ping from NUC to 192.168.186.2 worked. |
| **Dog QNX motion host** | 127.0.0.1:43893 from dog | Endpoint that accepted mode switch and speed commands. **Use from dog only.** |
| **Emolga** | `ssh robodog@100.110.138.76` | GPU workstation for Nerfstudio / splatfacto. Tailscale IP confirmed as 100.110.138.76. |
| **Base computer** | native ROS2 Jazzy, ARMLab LAN | Runs iPhone/base side AR pipeline, RQT image viewer, rosbridge, and calibration tools. |
| **iPhone** | `iphone_ip` from app/HUD | Runs ARExplorer app. Connects to base rosbridge `ws://<base-ip>:9090`. Serves MJPEG at `http://<iphone-ip>:8082/stream`. |

### 1.3 Software file tree

```
NUC: /root/                            (inside foxy_ros container)
├── rplidar_ws_foxy/install/           # RPLIDAR A2M8 driver
├── lite3_ws/
│   ├── src/
│   │   ├── lite3_bringup/             # Nav stack + odom bridge
│   │   │   ├── lite3_bringup/
│   │   │   │   ├── odom_bridge_node.py    # /leg_odom2 → /odom + TF
│   │   │   │   └── motion_host_cli.py     # one-shot UDP commands
│   │   │   ├── launch/bringup.launch.py
│   │   │   ├── config/nav2_params.yaml    # CRITICAL: controller_server.odom_topic: /odom
│   │   │   ├── config/mapper_async.yaml
│   │   │   └── urdf/lite3_minimal.urdf.xacro
│   │   └── lite3_capture/             # Orbit + video capture
│   │       ├── lite3_capture/
│   │       │   ├── orbit_node.py
│   │       │   ├── motion_test_node.py
│   │       │   └── capture_orbit_node.py
│   │       ├── launch/orbit.launch.py
│   │       ├── launch/orbit_capture.launch.py
│   │       └── config/orbit_params.yaml   # radius, period, ramp, yaw_sign=-1
│   └── install/
└── captures/                          # video output (auto-created)

NUC: /home/dog/                        (on NUC host, outside containers)
├── jazzy_ar/Dockerfile                # Jazzy container build recipe
└── ar-robot-explorer/                 # cloned repo, mounted into jazzy_ar

Emolga: ~/                             (robodog@100.110.138.76)
├── splat_pipeline/
│   ├── splat_pipeline.sh
│   ├── fetch_and_splat.sh
│   └── README.md
├── gaussian_captures/                 # videos pulled from NUC
└── splat_runs/                        # per-run training outputs
```

## 2. Current status

| Component | Current state | What to do next |
|---|---|---|
| **Foxy dog/Nav2** | Structurally operational. Nav2 launch comes up and action server accepted goals. Physical Nav2 goal execution still needs final careful test. | Run dog mode switch immediately before sending Nav2 goal. Watch `/cmd_vel`. |
| **Foxy `/cmd_vel` + orbit** | Working. Dog moved after dog-side mode switch. Open-loop orbit was validated. | Use `orbit_capture` for splat capture only when dog is in clear area. |
| **Foxy RealSense passthrough** | Original blocker fixed by recreating `foxy_ros` with `--privileged` and `-v /dev:/dev`. `/dev/video0..5` visible inside container. | Make sure Jazzy is not holding camera first. |
| **Foxy orbit capture** | Smoke-tested. A run recorded `/root/captures/smoke_02_20260529_035742.mp4` and JSON with 336 frames at observed 29.7 fps. | Run full capture and then send video to Emolga. |
| **Emolga splat pipeline** | Scripts expected under `~/splat_pipeline`; outputs under `~/splat_runs` and `~/gaussian_captures`. | Search for `config.yml`, `.ply`, `.splat`, `.ksplat`. Verify nerfstudio commands. |
| **Jazzy AR image** | Built. Packages RealSense, AprilTag, rosbridge verified. `/camera/camera/color/image_raw` publishes around 12-16 Hz with intermittent incomplete-frame warnings. | Run RQT on base, not inside Docker. Then verify `/detections`. |
| **Base to NUC Jazzy DDS** | Verified after moving NUC to ARMLab/ARMLab-5G network. Tailscale alone was not enough for raw DDS multicast. | Keep both on ARMLab for ROS2 DDS. Tailscale is fine for SSH/SCP. |
| **iPhone AR** | Next validation step. | Launch base pipeline with `iphone_ip`, run `calibration_check` and `run_calibration`, verify `/ar_markers` on iPhone. |

## 3. Golden rule: Foxy stack vs Jazzy stack

These two stacks should not be mixed. The same RealSense camera is shared by time, not by simultaneous access.

| Task | Container / machine | ROS domain | Camera owner |
|---|---|---|---|
| Dog Nav2 / orbit / splat video capture | `foxy_ros` on NUC | 0 | Foxy |
| AR / AprilTag / iPhone calibration | `jazzy_ar` on NUC + base computer | 42 | Jazzy |
| Nerfstudio training/export | Emolga | not ROS-dependent | none |

Before launching any RealSense driver, run this on the NUC host:

```bash
sudo lsof /dev/video0 2>/dev/null
```

If anything is listed, stop the stack currently holding the camera before proceeding.

## 4. Critical gotchas to know before editing

Five things that look incidental but will silently break the system if you touch them wrong. Read this section before modifying any config or source.

### 4.1 Nav2 DWB starvation bug

**What it is.** Nav2's `controller_server` (running DWB) subscribes to a topic named by `controller_server.odom_topic`, which defaults to `odom`. If no `nav_msgs/Odometry` is being published on that topic, DWB silently starves: goals are accepted, the behavior tree runs, the planner produces a path, but **no `/cmd_vel` ever leaves controller_server.** No error is logged; it just doesn't work.

**The trap.** Some Nav2 docs suggest setting `bt_navigator.odom_topic`. That parameter does **not** propagate to `controller_server`. Setting it alone does nothing useful.

**What this repo does.** The `odom_bridge_node` publishes both:
1. The TF `odom → base_link`
2. A `nav_msgs/Odometry` message on the `/odom` topic

And `nav2_params.yaml` explicitly sets:

```yaml
controller_server:
  ros__parameters:
    odom_topic: /odom
```

**Implications for editing.**
- If you refactor `odom_bridge_node`, the `/odom` Odometry topic must remain (not just TF).
- If you edit `nav2_params.yaml`, the `controller_server.odom_topic` line must stay set to `/odom`.
- If goals stop executing despite being accepted, check `ros2 topic hz /odom` first. If it's zero, that's the problem.

### 4.2 Foxy `robot_description` must be wrapped in `ParameterValue`

**What it is.** When `bringup.launch.py` passes a URDF to `robot_state_publisher` via `Command(['xacro ', urdf_file])`, Foxy's `launch_ros` tries to YAML-parse the resulting XML as a parameter value. Any colon inside an XML comment (e.g. "What this file provides:") gets interpreted as a YAML mapping and the launch fails with a `yaml.scanner.ScannerError`.

**The fix.** Wrap the Command output explicitly as a string:

```python
from launch_ros.parameter_descriptions import ParameterValue

parameters=[{
    'robot_description': ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
    ),
}]
```

Galactic+ handles this automatically. Foxy does not.

**Implication for editing.** If you replace the URDF or change the `robot_state_publisher` block, keep the `ParameterValue` wrapper. If you see "Unable to parse the value of parameter robot_description as yaml", this is the cause.

### 4.3 Navigation mode session timeout

The dog's QNX motion host requires heartbeats at ≥2 Hz to maintain a control session. The dog-side mode switch script (Appendix A.1) sends heartbeats, then mode commands, then more heartbeats, then exits. The session expires shortly after heartbeats stop.

**Practical consequence.** Between two consecutive `/cmd_vel` sessions, the dog will silently revert to manual mode. If you ran the mode switch ten minutes ago and the dog won't move, run it again. This is normal.

### 4.4 RealSense is a single-owner USB device

The Foxy container and the Jazzy container cannot both run the RealSense driver at the same time. There is no software enforcement of this — it's operational discipline. If you accidentally start both, you'll get cryptic device-busy errors. Always check `sudo lsof /dev/video0` before launching a camera node.

### 4.5 Browser-rename gotcha

When downloading the same zip filename multiple times, browsers append ` (1)`, ` (2)`, etc. instead of overwriting. This bit us twice — once with `lite3_bringup.zip` and once with `lite3_capture_v2.zip`. The receiving machine ended up with a stale file under the expected name and a new file under `<name> (1).zip`.

**Defense.** Before scp-ing a file you just downloaded, verify the contents:

```bash
python3 -c "
import zipfile
z = zipfile.ZipFile('/home/robodog/Downloads/<file>.zip')
print('files:', len(z.namelist()))
"
```

If renaming for clarity, use `_v2`, `_v3` suffixes deliberately, and scp with the explicit filename as the destination:

```bash
scp "/home/robodog/Downloads/file (1).zip" nuc:~/file_v2.zip
```

---

# Part II — Runbook

## 5. NUC boot / reconnect and container basics

### 5.1 Reconnect to the NUC

```bash
ssh dog@100.72.194.78
```

Check you are on the right machine:

```bash
hostname
ip -br addr
sudo docker ps -a
```

### 5.2 If NUC Wi-Fi is wrong, switch to ARMLab

This was required for Jazzy DDS between base and NUC. SSH may drop; reconnect with Tailscale after switching.

```bash
nmcli connection show

sudo nmcli connection up "ARM_LAB" ifname wlp0s20f3
# If that does not work:
sudo nmcli connection up "ARMLab-5G" ifname wlp0s20f3

# Then reconnect:
ssh dog@100.72.194.78
```

### 5.3 Container name conflict fix

If you see `Conflict. The container name "/jazzy_ar" is already in use`, use `exec`/`start`, not `docker run`.

```bash
sudo docker ps -a | grep jazzy_ar

# If running:
sudo docker exec -it jazzy_ar bash

# If Exited:
sudo docker start jazzy_ar
sudo docker exec -it jazzy_ar bash

# If you really want to recreate it:
sudo docker stop jazzy_ar
sudo docker rm jazzy_ar
```

## 6. Foxy robot / orbit / splat workflow

### 6.1 Start dog-side transfer stack

On laptop/base terminal, SSH into the dog and leave this running:

```bash
ssh ysc@192.168.0.129
# If that IP is unavailable, use the current dog IP:
# ssh ysc@<dog-ip>

~/start_transfer.sh
```

This brings up the dog-side stack that publishes `/leg_odom2` and runs `motion_sender` subscribing to `/cmd_vel`.

### 6.2 Enter `foxy_ros` and source exact setup

On NUC host:

```bash
ssh dog@100.72.194.78
sudo docker start foxy_ros
sudo docker exec -it foxy_ros bash
```

Inside `foxy_ros`:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/foxy/setup.bash
source /root/rplidar_ws_foxy/install/setup.bash
source /root/lite3_ws/install/setup.bash
```

Optional one-time convenience inside `foxy_ros` to skip the export/source lines on every shell:

```bash
cat >> /root/.bashrc << 'EOF'
# ROS2 + Lite3 workspaces
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/foxy/setup.bash
source /root/rplidar_ws_foxy/install/setup.bash
source /root/lite3_ws/install/setup.bash 2>/dev/null
EOF
```

### 6.3 Preflight checks inside `foxy_ros`

```bash
ros2 topic hz /leg_odom2
ls /dev/ttyUSB0
ls /dev/video*
ros2 pkg executables lite3_bringup
ros2 pkg executables lite3_capture
```

Expected: `/leg_odom2` around 180-200 Hz, LiDAR at `/dev/ttyUSB0`, and RealSense devices `/dev/video0` through `/dev/video5` if Foxy owns camera.

### 6.4 If Foxy does not see RealSense, recreate `foxy_ros` exactly this way

This is the exact fix that preserved the existing workspace and gave the container access to `/dev/video*`. Run on NUC host, not inside container.

```bash
sudo docker commit foxy_ros foxy_ros_with_ws:latest
sudo docker stop foxy_ros
sudo docker rename foxy_ros foxy_ros_old

sudo docker run -it \
  --name foxy_ros \
  --network host \
  --ipc host \
  --privileged \
  -v /dev:/dev \
  foxy_ros_with_ws:latest \
  bash
```

Inside the recreated container, source the Foxy setup and verify:

```bash
source /opt/ros/foxy/setup.bash
source /root/rplidar_ws_foxy/install/setup.bash
source /root/lite3_ws/install/setup.bash
ls /dev/video*
```

Rollback if needed:

```bash
sudo docker stop foxy_ros
sudo docker rm foxy_ros
sudo docker rename foxy_ros_old foxy_ros
sudo docker start foxy_ros
```

### 6.5 Run the dog-side mode switch immediately before motion

Open a second terminal to the dog and run **Appendix A.1**. Do this right before `/cmd_vel`, Nav2, or orbit. The session expires after heartbeats stop (see §4.3).

### 6.6 Exact `/cmd_vel` pulse that worked

Inside `foxy_ros` after dog is in navigation + move mode:

```bash
timeout 1 ros2 topic pub -r 10 --qos-reliability best_effort /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.04, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

ros2 topic pub --once --qos-reliability best_effort /cmd_vel geometry_msgs/msg/Twist \
'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

**Important:** paste the full `ros2` command. Do not paste only the YAML body; the shell will report `command not found`.

### 6.7 Exact orbit + RealSense capture command that worked

Inside `foxy_ros`, after dog stack is running and dog-side mode switch has just been run:

```bash
sleep 5 && ros2 launch lite3_capture orbit_capture.launch.py basename:=smoke_02
```

Observed successful output from one smoke run: RealSense D435I detected, color stream RGB8 640x480 @ 30 fps, recording to `/root/captures/smoke_02_20260529_035742.mp4`, finalized with 336 frames and observed 29.7 fps.

After launch exits or after Ctrl-C, check output:

```bash
ls -lh /root/captures/smoke_02_*.mp4
cat /root/captures/smoke_02_*.json
```

### 6.8 Full 360° orbit parameter edit

The default orbit settings give only about 132° of arc, not a full revolution. The math:

```
total_angle = (2π / orbit_period_s) × (ramp_seconds + orbit_duration_s)
```

With defaults `period=60, duration=20, ramp=2`: `(2π/60) × 22 = 2.30 rad ≈ 132°`.

(Why `ramp + duration` and not `2 × ramp + duration`: the trapezoidal envelope's ramps each integrate to half their full-speed arc, so two 2-second ramps together cover the same arc as one 2-second full-speed segment.)

For near full 360°, edit orbit params inside `foxy_ros`:

```bash
nano /root/lite3_ws/src/lite3_capture/config/orbit_params.yaml
```

Set:

```yaml
orbit_period_s: 30.0
orbit_duration_s: 28.0
ramp_seconds: 2.0
yaw_sign: -1
```

Verify: `(2π/30) × 30 = 2π ≈ 360°`. ✓

Because symlink install is used for config, no rebuild is normally needed. Launch with a longer capture duration if the launch arg exists:

```bash
ros2 launch lite3_capture orbit_capture.launch.py \
  basename:=full_orbit_01 \
  capture_duration_s:=36
```

**Drift caveat.** At 0.75 m radius without closed-loop feedback, radius wanders 10-20 cm over a full 360°. Fine for first-pass splats; the reconstruction will show fuzzier results at the perimeter than a perfectly circular orbit would.

## 7. Nav2 physical execution test

This is independent of the splat demo. Use this only when the dog is in a safe open area.

1. Start dog stack: ssh into dog and run `~/start_transfer.sh`.
2. Enter `foxy_ros` and source Foxy/lite3 setup.
3. Run **Appendix A.1** on dog immediately before sending the goal.
4. Launch Nav2 inside `foxy_ros`:

   ```bash
   ros2 launch lite3_bringup bringup.launch.py use_nav2:=true
   ```

5. In a second `foxy_ros` shell, verify odom and TF:

   ```bash
   ros2 topic info -v /odom
   ros2 topic hz /odom
   timeout 5 ros2 run tf2_ros tf2_echo map base_link
   ros2 action list | grep navigate_to_pose
   ```

6. In a third `foxy_ros` shell, watch the command output:

   ```bash
   ros2 topic echo /cmd_vel
   ```

7. Send the small goal:

   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
   "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.3, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
   ```

Expected: goal accepted, `/cmd_vel` streams, dog physically moves. If accepted but no movement, rerun the dog mode switch (Appendix A.1) and retry immediately.

If `/cmd_vel` stays silent even after accepting the goal, that's the DWB starvation symptom from §4.1 — check `ros2 topic hz /odom` and the `controller_server.odom_topic` setting.

## 8. Emolga Nerfstudio / splat workflow

### 8.1 Connect and verify nerfstudio

```bash
ssh robodog@100.110.138.76
conda activate nerfstudio
which ns-process-data ns-train ns-export
ns-train --help | head
```

### 8.2 Search for generated Nerfstudio / splat outputs

Use this to find exactly what already exists on Emolga:

```bash
find ~/splat_runs ~/outputs ~ -type f \( \
  -name "config.yml" -o \
  -name "transforms.json" -o \
  -name "dataparser_transforms.json" -o \
  -iname "*.ply" -o \
  -iname "*.splat" -o \
  -iname "*.ksplat" \
\) -printf "%TY-%Tm-%Td %TH:%TM %s %p\n" 2>/dev/null | sort | tail -100
```

Find the latest Nerfstudio training config:

```bash
find ~ -path "*/outputs/*/splatfacto/*/config.yml" \
  -printf "%TY-%Tm-%Td %TH:%TM %p\n" 2>/dev/null | sort | tail -20

CONFIG=$(find ~ -path "*/outputs/*/splatfacto/*/config.yml" 2>/dev/null | sort | tail -1)
echo "$CONFIG"
```

### 8.3 View / export latest Nerfstudio run

```bash
ns-viewer --load-config "$CONFIG"
```

Export Gaussian splat:

```bash
mkdir -p ~/splat_exports/latest

ns-export gaussian-splat \
  --load-config "$CONFIG" \
  --output-dir ~/splat_exports/latest

find ~/splat_exports ~/splat_runs ~ -type f \( -iname "*.ply" -o -iname "*.splat" -o -iname "*.ksplat" \) \
  -printf "%TY-%Tm-%Td %TH:%TM %s %p\n" 2>/dev/null | sort | tail -50
```

### 8.4 End-to-end orchestrator command

Once `orbit_capture` works, run this on Emolga. This command is the intended one-command path from capture to splat.

```bash
~/splat_pipeline/fetch_and_splat.sh demo_01 28
```

It SSHes to NUC, runs `orbit_capture` in `foxy_ros`, copies MP4+JSON out, SCPs to Emolga, then runs `ns-process-data`, `ns-train splatfacto`, and `ns-export`. It assumes the dog is already in navigation/move mode.

## 9. Jazzy AR / iPhone workflow

### 9.1 Build image — already done, but exact command is here

On NUC host:

```bash
mkdir -p ~/jazzy_ar
cd ~/jazzy_ar

cat > Dockerfile << 'EOF'
FROM ros:jazzy
RUN apt-get update && apt-get install -y \
    ros-jazzy-apriltag-ros \
    ros-jazzy-rosbridge-server \
    ros-jazzy-realsense2-camera \
    ros-jazzy-tf2-tools \
    python3-scipy \
    python3-colcon-common-extensions \
    python3-opencv \
    git lsof \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ros2_ws
CMD ["bash"]
EOF

sudo docker build -t jazzy_ar .
```

Verify:

```bash
sudo docker run --rm jazzy_ar bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 pkg list | grep -E 'realsense|apriltag|rosbridge'"
```

Expected packages: `apriltag_msgs`, `apriltag_ros`, `realsense2_camera`, `realsense2_camera_msgs`, `rosbridge_library`, `rosbridge_msgs`, `rosbridge_server`.

### 9.2 Load `ar-robot-explorer` repo — exact fix if folder is bad

We hit a case where `~/ar-robot-explorer` existed but was not a git repo. Fix on NUC host:

```bash
cd ~
mv ar-robot-explorer ar-robot-explorer_old_$(date +%Y%m%d_%H%M%S)
git clone https://github.com/ahmedm-02/ar-robot-explorer.git
cd ~/ar-robot-explorer
git remote -v
ls ar_explorer/package.xml
ls ar_explorer/setup.py
```

### 9.3 Start / restart `jazzy_ar` exactly

If recreating from scratch:

```bash
sudo docker stop jazzy_ar
sudo docker rm jazzy_ar

sudo docker run -it --name jazzy_ar \
  --network host \
  --privileged \
  -v /dev:/dev \
  -v ~/ar-robot-explorer/ar_explorer:/ros2_ws/src/ar_explorer \
  -e ROS_DOMAIN_ID=42 \
  jazzy_ar bash
```

If it already exists, use:

```bash
sudo docker start jazzy_ar
sudo docker exec -it jazzy_ar bash
```

### 9.4 Build / source `ar_explorer` inside `jazzy_ar`

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
ls /dev/video*

cd /ros2_ws
colcon build --packages-select ar_explorer
source install/setup.bash

ros2 pkg list | grep ar_explorer
```

Expected `/dev/video0..5` if RealSense is plugged into NUC and container has `/dev` mount.

### 9.5 Launch NUC-side AR camera / AprilTag pipeline

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch ar_explorer ar_pipeline.launch.py machine:=dog
```

Known warning: RealSense may print `Incomplete video frame detected` / `Frame Corrupted`. Continue if the image topic publishes and detections work.

### 9.6 In a second `jazzy_ar` shell, verify image and detections

```bash
sudo docker exec -it jazzy_ar bash
```

Inside:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 topic list | grep -E "camera|image|detections"
ros2 topic hz /camera/camera/color/image_raw
```

Observed image rate was about 12-16 Hz after USB 3 replug. That is lower than ideal but enough for smoke testing. Then put AprilTag 17 in view:

```bash
ros2 topic echo /detections --once
ros2 topic info /detections -v

# Optional TF check if tag frame exists:
timeout 5 ros2 run tf2_ros tf2_echo camera_color_optical_frame tag_17
```

### 9.7 RQT image viewer — do this on base, not inside Docker

Inside `jazzy_ar`, `rqt_image_view` failed with "could not connect to display" because the Docker container had no GUI display. Run RQT on the base computer instead.

On base computer:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash

ros2 topic list | grep image
ros2 topic hz /camera/camera/color/image_raw

sudo apt update
sudo apt install -y ros-jazzy-rqt-image-view

ros2 run rqt_image_view rqt_image_view
```

In the RQT dropdown, select `/camera/camera/color/image_raw`.

### 9.8 Base-side iPhone pipeline

On base computer, after NUC-side Jazzy pipeline is running and iPhone app is open. Replace `<iphone-ip>` with the IP shown in the iPhone app/HUD.

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ar_explorer ar_pipeline.launch.py \
  machine:=base \
  iphone_ip:=<iphone-ip>
```

In another base terminal, confirm base sees NUC detections:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic list | grep detections
ros2 topic echo /detections --once
```

### 9.9 Calibration and iPhone marker smoke test

Put AprilTag 17 where both RealSense and iPhone can see it. On base:

```bash
ros2 run ar_explorer calibration_check
ros2 run ar_explorer run_calibration
ls -lh ~/.ros/ar_explorer_calibration.json
ros2 topic echo /ar_markers --once
ros2 run ar_explorer ar_marker_publisher
```

Expected: calibration JSON is written, `/ar_markers` publishes, and a marker appears in iPhone AR view.

## 10. DDS / network verification

Use this if the base cannot see NUC topics. Both machines must be on ARMLab/ARMLab-5G or another multicast-friendly LAN. Tailscale is not enough for raw DDS discovery.

On NUC host, listener:

```bash
sudo docker run -it --rm --network host -e ROS_DOMAIN_ID=42 ros:jazzy bash -lc \
"apt update >/dev/null && apt install -y ros-jazzy-demo-nodes-cpp >/dev/null && source /opt/ros/jazzy/setup.bash && ros2 run demo_nodes_cpp listener"
```

On base computer, talker:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

Expected NUC output:

```
[listener]: I heard: [Hello World: 12]
[listener]: I heard: [Hello World: 13]
[listener]: I heard: [Hello World: 14]
```

## 11. Shutdown procedure

1. Stop ROS launches first with Ctrl-C in `foxy_ros` and/or `jazzy_ar`.
2. Return the dog to manual/controller mode using **Appendix A.2** on the dog.
3. Then Ctrl-C `~/start_transfer.sh` on the dog.
4. Optionally stop containers on NUC host:

   ```bash
   sudo docker stop jazzy_ar
   sudo docker stop foxy_ros
   ```

## 12. Quick task recipes

### 12.1 AR camera smoke test only

```bash
# NUC host
ssh dog@100.72.194.78
sudo docker start jazzy_ar
sudo docker exec -it jazzy_ar bash

# inside jazzy_ar
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch ar_explorer ar_pipeline.launch.py machine:=dog

# second terminal into jazzy_ar
sudo docker exec -it jazzy_ar bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /detections --once
```

### 12.2 Splat capture only

```bash
# Dog terminal
ssh ysc@192.168.0.129
~/start_transfer.sh

# Dog second terminal: run Appendix A.1 mode switch

# NUC / Foxy terminal
ssh dog@100.72.194.78
sudo docker start foxy_ros
sudo docker exec -it foxy_ros bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/foxy/setup.bash
source /root/rplidar_ws_foxy/install/setup.bash
source /root/lite3_ws/install/setup.bash

sleep 5 && ros2 launch lite3_capture orbit_capture.launch.py basename:=smoke_02

# Check output
ls -lh /root/captures/smoke_02_*.mp4
cat /root/captures/smoke_02_*.json
```

### 12.3 Latest splat on Emolga only

```bash
ssh robodog@100.110.138.76
conda activate nerfstudio

find ~/splat_runs ~/outputs ~ -type f \( \
  -name "config.yml" -o -iname "*.ply" -o -iname "*.splat" -o -iname "*.ksplat" \
\) -printf "%TY-%Tm-%Td %TH:%TM %s %p\n" 2>/dev/null | sort | tail -100

CONFIG=$(find ~ -path "*/outputs/*/splatfacto/*/config.yml" 2>/dev/null | sort | tail -1)
echo "$CONFIG"

ns-viewer --load-config "$CONFIG"
```

---

# Part III — Reference

## 13. Troubleshooting — exact errors we saw

| Symptom | Meaning | Exact fix |
|---|---|---|
| `docker: Conflict. The container name "/jazzy_ar" is already in use` | Container already exists. | `sudo docker exec -it jazzy_ar bash`, or `sudo docker start jazzy_ar` first if stopped. |
| `could not connect to display` from `rqt_image_view` inside Docker | Container was started without GUI DISPLAY/X11. | Run `rqt_image_view` on base computer instead. |
| `Incomplete video frame detected` / `Frame Corrupted` | RealSense stream has dropped/corrupt frames, usually USB bandwidth/cable/port. | Use USB 3, check `ros2 topic hz`. Continue if stream stays alive and detections work. |
| `/cmd_vel` publishes but dog does not move | Dog not in navigation + move mode, or mode session expired. | Rerun Appendix A.1 on dog immediately before the motion command. |
| Base cannot see NUC Jazzy topics | DDS multicast not working across networks/Tailscale. | Put NUC and base on ARMLab/ARMLab-5G and rerun `demo_nodes_cpp` test. |
| No `/dev/video*` inside container | Container lacks `/dev` passthrough or camera not plugged at start. | Recreate with `--privileged -v /dev:/dev`; verify host sees `/dev/video*` first. |
| Shell says `command not found` after pasting Twist YAML | Only YAML was pasted, not the `ros2 topic pub` command. | Paste the complete `ros2 topic pub` command including message type and quoted YAML. |
| Nav2 goal accepted but no Twists | DWB starving on `/odom` (see §4.1). | Verify `controller_server.odom_topic: /odom`, `ros2 topic hz /odom`, and `/cmd_vel` echo. |
| `Unable to parse the value of parameter robot_description as yaml` | Foxy YAML-parsing URDF (see §4.2). | Wrap with `ParameterValue(Command(['xacro ', urdf_file]), value_type=str)`. |
| Browser-downloaded zip has stale content | Browser saved a duplicate as `file (1).zip` (see §4.5). | Verify contents with `python3 -m zipfile -l <file>`; use the right path on scp. |

## 14. UDP command reference for the motion host

From "Jueying Lite3 Motion Host Communication Interface v1.0.7-0" (DEEP Robotics, 2024-05-15).

### 14.1 Packet format

**Simple commands** (12 bytes, little-endian):

```c
struct CommandHead {
    uint32_t code;            // command code
    uint32_t paramters_size;  // 0 for simple commands
    uint32_t type;            // 0 for simple commands
};
```

**Complex commands** (used by motion_sender for speed):

```c
struct Command {
    CommandHead head;            // type = 1
    uint32_t data[kDataSize];    // payload (e.g. double-precision speed)
};
```

UDP port: `43893`. Target IP **from the dog itself**: `127.0.0.1`. From a host on segment 1: `192.168.1.120`.

### 14.2 Common simple command codes

| Command | Code | Spec section |
|---|---|---|
| Heartbeat (≥2 Hz to maintain session) | `0x21040001` | 1.2.1 |
| Navigation mode (enables `/cmd_vel`) | `0x21010C03` | 1.2.7 |
| Manual mode (joystick takeover) | `0x21010C02` | 1.2.7 |
| Move mode (axis commands move robot) | `0x21010D06` | 1.2.4 |
| Pose mode (axis commands rotate body) | `0x21010D05` | 1.2.4 |
| Stand/sit toggle | `0x21010202` | 1.2.2 |
| Soft emergency stop | `0x21020C0E` | 1.2.2 |
| Reset to zero | `0x21010C05` | 1.2.2 |

### 14.3 Speed commands (complex, used by motion_sender)

These are sent at ≥20 Hz by motion_sender on the dog as it forwards `/cmd_vel`. You normally won't send them yourself, but Appendix A.3 has a working example.

| Command | Code | Type | Range |
|---|---|---|---|
| Linear velocity X (m/s, forward) | `0x0140` | complex | [-1.0, 1.0] |
| Linear velocity Y (m/s, lateral) | `0x0145` | complex | [-0.5, 0.5] |
| Angular velocity Z (rad/s, yaw) | `0x0141` | complex | [-1.5, 1.5] |

## 15. Frame IDs and TF tree

```
map
 └── odom              (slam_toolbox)
      └── base_link    (odom_bridge_node, from /leg_odom2)
           └── laser   (robot_state_publisher from URDF)
```

**Frames:**

- `map`: SLAM-managed world frame. Subject to discrete loop-closure jumps.
- `odom`: Continuous odometric frame. Published by `odom_bridge_node`. Drifts but never jumps.
- `base_link`: Robot body frame, conventionally at the body centerpoint.
- `laser`: RPLIDAR A2M8 sensor frame.

**Measured `base_link → laser` static offset:** x = −0.060 m, y = 0.000 m, z = 0.097 m (LiDAR is 6 cm behind body center, 9.7 cm above). Defined in `urdf/lite3_minimal.urdf.xacro`. Update this if the LiDAR is remounted.

## 16. Open-loop orbit drift

At 0.75 m radius with default orbit settings, radius drift over a full 360° is typically 10–20 cm.

**Sources of drift:**
- Open-loop velocity commands — no feedback from `/leg_odom2` to correct radius
- Trapezoidal envelope has discrete ramp transitions that the dog smooths via its own dynamics
- Lateral and yaw aren't perfectly orthogonal in the dog's gait

**Implications:**
- First-pass splats work but reconstruct fuzzier at the perimeter than ideal
- Visual hand-aiming the dog at the start is worth doing
- Closed-loop mitigation (radius correction using `/leg_odom2`) is a planned improvement

## 17. External references

- **Jueying Lite3 Motion Host Communication Interface v1.0.7-0** (DEEP Robotics, 2024-05-15). PDF in project docs. Sections 1.2.7 (control modes), 1.2.12 (speed commands), 1.1 (packet format).
- **Lite3_MotionSDK** on GitHub. For motion host IP determination and reference C++ client.
- **ROS2 Foxy documentation**: <https://docs.ros.org/en/foxy/>
- **Nav2 (Foxy) documentation**: <https://navigation.ros.org/>
- **Nerfstudio documentation**: <https://docs.nerf.studio/>
- **splatfacto method**: linked from Nerfstudio docs.

---

# Appendix A — Exact dog-side mode scripts

Run these **on the dog itself.** These are the scripts that worked. Do not run them from the NUC container — the motion host is reachable only from the dog, at `127.0.0.1:43893`.

## A.1 Put dog into navigation + move mode

```bash
python3 - <<'PY'
import socket
import struct
import time

HOST = "127.0.0.1"
PORT = 43893
HEARTBEAT = 0x21040001
NAVIGATION_MODE = 0x21010C03
MOVE_MODE = 0x21010D06

def simple(code):
    return struct.pack("<III", code, 0, 0)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send(code, name):
    payload = simple(code)
    print(f"sent {name}: code=0x{code:08X}, bytes={' '.join(f'{b:02x}' for b in payload)}")
    sock.sendto(payload, (HOST, PORT))

for _ in range(5):
    send(HEARTBEAT, "heartbeat")
    time.sleep(0.2)

for _ in range(5):
    send(NAVIGATION_MODE, "NAVIGATION / automatic mode")
    time.sleep(0.2)

for _ in range(5):
    send(MOVE_MODE, "MOVE mode")
    time.sleep(0.2)

for _ in range(5):
    send(HEARTBEAT, "heartbeat")
    time.sleep(0.2)

sock.close()
print("Dog should now be in automatic/navigation + move mode.")
PY
```

## A.2 Return dog to manual / controller mode

```bash
python3 - <<'PY'
import socket
import struct
import time

HOST = "127.0.0.1"
PORT = 43893
HEARTBEAT = 0x21040001
MANUAL_MODE = 0x21010C02

def simple(code):
    return struct.pack("<III", code, 0, 0)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

for _ in range(3):
    sock.sendto(simple(HEARTBEAT), (HOST, PORT))
    time.sleep(0.2)

for _ in range(5):
    payload = simple(MANUAL_MODE)
    print(f"sent MANUAL mode: {' '.join(f'{b:02x}' for b in payload)}")
    sock.sendto(payload, (HOST, PORT))
    time.sleep(0.2)

sock.close()
print("Dog should now be back in manual/controller mode.")
PY
```

## A.3 Direct UDP forward speed test that moved the dog

Use only in a clear area. This sends `SPEED_X` directly to the dog motion host after switching modes.

```bash
python3 - <<'PY'
import socket
import struct
import time

HOST = "127.0.0.1"
PORT = 43893
HEARTBEAT = 0x21040001
NAVIGATION_MODE = 0x21010C03
MOVE_MODE = 0x21010D06
SPEED_X = 0x0140

def simple(code, value=0):
    return struct.pack("<III", code, value, 0)

def complex_double(code, value):
    return struct.pack("<III", code, 8, 1) + struct.pack("<d", float(value))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

for _ in range(5):
    sock.sendto(simple(HEARTBEAT), (HOST, PORT))
    time.sleep(0.2)

for _ in range(5):
    sock.sendto(simple(NAVIGATION_MODE), (HOST, PORT))
    time.sleep(0.2)

for _ in range(5):
    sock.sendto(simple(MOVE_MODE), (HOST, PORT))
    time.sleep(0.2)

t0 = time.time()
while time.time() - t0 < 1.0:
    sock.sendto(simple(HEARTBEAT), (HOST, PORT))
    sock.sendto(complex_double(SPEED_X, 0.04), (HOST, PORT))
    time.sleep(0.05)

for _ in range(10):
    sock.sendto(complex_double(SPEED_X, 0.0), (HOST, PORT))
    time.sleep(0.05)

sock.close()
print("Done.")
PY
```

---

*Exact commands for Ayaan/Ahmed lab setup. Use commands exactly unless a line says to substitute a value.*
