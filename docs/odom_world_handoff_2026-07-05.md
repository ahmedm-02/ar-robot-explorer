# Odom → ARKit World Handoff — 2026-07-05

**Goal:** Put the dog's live pose into the iPhone's ARKit world so RealSense/dog
detections appear in the first responder's AR view and **track the dog as it
walks** — across the Foxy(dog)↔Jazzy(NUC/ASUS) boundary.

Continues [odom_relay_handoff_2026-06-28.md](odom_relay_handoff_2026-06-28.md).

---

## TL;DR — where we are

- ✅ **UDP odom relay** (dog Foxy → NUC Jazzy) — dog's `/leg_odom2` arrives as
  JSON/UDP `:9870`, republished as TF `odom → base_link`. Integrated into the
  launch, verified live ~28 Hz.
- ✅ **Full TF tree wired & resolves:**
  `arkit_world → odom → base_link → camera_link → camera_color_optical_frame → tag_17`.
- ✅ **Calibration is robust & auto-starting** (see Architecture): settle+converge
  gate, single persistent publisher, no more duplicate-publisher conflicts.
- ✅ **iPhone visual verification** via `origin_markers`: 🔴 arkit_world origin,
  🔵 odom origin, 🟢 live `base_link` (dog) tracker.
- 🐞 **OPEN BUG (where we stopped):** the 🟢 `base_link` marker moves with the
  dog but **in the wrong direction** (dog moves forward → marker goes
  backward/up), and sits far from the dog. Prime suspect: the **identity
  placeholder** for the `base_link → camera_link` mount. See "Open bug" below.

---

## The open bug — green tracker moves the wrong direction

**Symptom:** As the dog walks forward, the green `base_link` sphere moves along
the wrong axis (e.g. backward or up), and is offset from the physical dog. The
🔵 odom-origin sphere being far is likely **expected** (it marks where the dog
booted; the dog has walked away from it) — the green direction is the real issue.

**Leading hypothesis (unconfirmed): the identity mount `base_link → camera_link`.**
The mount is currently an identity placeholder (see launch). If the RealSense is
physically mounted rotated relative to the dog's body, that rotation error gets
**baked into `arkit_world → odom`** during calibration. The tag handshake still
pins the pose correctly *at the instant of calibration*, but the dog's motion
(correctly expressed in `odom`) is then pushed through the mis-rotated
`arkit_world → odom`, so the marker moves along the wrong axis. Translation error
would only offset position; a **rotation** error flips direction — matching the
symptom.

**Caveat:** the settle/converge gate does NOT catch this. Convergence means
low-noise samples, not accuracy — a systematic mount error converges cleanly and
is still wrong. Also, a full ~90–180° flip implies a *large* mount rotation; if
the RS is actually mounted forward-facing and level, the mount is ~identity and
the flip would instead point at the dog-odometry convention or the two apriltag
tag-frame conventions.

**Next steps to resolve (pick up here):**
1. **Determine the physical RealSense mounting** on the dog (facing forward?
   tilted down? rotated 90°? upside down?). This decides whether a big mount
   rotation is even plausible.
2. **Run the direction diagnostic** (read-only, not yet run): from the current
   `arkit_world → odom`, compute where the dog's forward (+x) and up (+z) axes
   point in the phone's view. A clean 90/180° error ⇒ a convention bug; an
   arbitrary angle ⇒ mount tilt.
3. **Fix:** replace the identity `base_link → camera_link` with the real mount
   **rotation** (translation matters less for direction) in
   `ar_pipeline.launch.py`, then recalibrate
   (`ros2 service call /odom_world_calibrator/recalibrate std_srvs/srv/Trigger`)
   and retest the walk.
4. If it's NOT the mount, investigate: dog `/leg_odom2` convention (we pass it
   verbatim as REP-103, no remap) and whether `36h11.yaml` vs `36h11_iphone.yaml`
   give matching tag-frame conventions.

---

## Architecture — calibration (the part we hardened this session)

Earlier design had `odom_world_calibrator` both compute AND publish
`arkit_world → odom` via a latched broadcaster tied to its own lifetime. That
caused a real bug: running it twice (or an auto + a manual run) put **two
conflicting `arkit_world → odom` on `/tf_static`**; tf2 used whichever arrived
last → spheres landed in the wrong place ("calibration reset" symptom). Ctrl-C
also silently dropped the edge.

**Fixed by splitting compute from publish + guaranteeing a single instance:**

- **`odom_world_calibrator`** — COMPUTES + SAVES only; never publishes TF.
  - Computes `T_arkit_from_odom = T_arkit_from_tag @ inv(T_odom_from_tag)` from
    the two-camera tag-17 handshake (same physical tag in both trees).
  - **Settle + converge gate:** discards a warm-up window of AprilTag startup
    jitter, then requires the tag-pose spread (translation stddev + max rotation
    deviation) to be within tolerance in BOTH trees before accepting. If it never
    converges it writes **nothing** — no calibration beats a wrong one.
  - Saves atomically to `~/.ros/ar_explorer_odom_world.json`.
  - Stays alive offering a `~/recalibrate` (`std_srvs/Trigger`) service.
  - Params: `tag_id`(17), `settle_time`(2.0s), `window`(2.0s), `min_samples`(15),
    `trans_tol`(0.01 m), `rot_tol_deg`(2.0), `output`.
- **`odom_world_tf`** — the SOLE persistent publisher of `arkit_world → odom`.
  Loads the JSON, broadcasts to `/tf_static`, reloads on file change. One source
  of truth ⇒ conflicting publishers are impossible.
- **Auto-start:** the launch base group runs `odom_world_calibrator` +
  `odom_world_tf` + `origin_markers` (single instance) — no manual terminal step.
  Recalibrate = the service, or relaunch.

---

## Launch topology (`ar_explorer/launch/ar_pipeline.launch.py`)

- **`machine:=dog`** (NUC on the robot): RealSense + AprilTag, `odom_relay_receiver`
  (UDP→`odom→base_link`), `base_link_to_camera` static mount (**identity
  placeholder — the suspect**).
- **`machine:=base`** (ASUS): iPhone bridge + AprilTag, `iphone_pose_bridge`
  (`arkit_world→iphone_camera`), calibration_server/calibrated_forwarder,
  rosbridge, and NEW `odom_world_calibrator` + `odom_world_tf` + `origin_markers`.
- Args added: `odom_relay`(true), `origin_markers`(true).
- **Both machines must be `ROS_DOMAIN_ID=42`** (dog stays 0) so their TF trees
  merge into one graph — required for calibration. Dog isolation depends on this.

---

## Convention gotcha (important, already handled in `origin_markers`)

The iPhone renders `frame_id:"arkit_world"` markers in **raw ARKit** convention
(+X right, +Y up, +Z toward viewer), but our TF `arkit_world` frame is **ROS
REP-103** (+X fwd, +Y left, +Z up) — `iphone_pose_bridge` applies the basis
change. So `origin_markers` converts ROS→ARKit with `R_BASIS.T` (inverse of
`iphone_pose_bridge._R_BASIS`), i.e. `(px,py,pz)→(-py, pz, -px)`, before
publishing the blue/green spheres. Verified exact for the static blue sphere.

---

## How to bring it up & test

```bash
# both machines, domain 42, after `git pull` + colcon build:
#   NUC:
ros2 launch ar_explorer ar_pipeline.launch.py machine:=dog
#   ASUS:
ros2 launch ar_explorer ar_pipeline.launch.py machine:=base iphone_ip:=<iphone-ip>
```
Point BOTH cameras at tag 17; watch the ASUS `odom_world_calibrator` log for
`Settling… → Collecting… → CALIBRATED` (or `Not converged … retrying`). On
success `odom_world_tf` publishes and the 🔴🔵🟢 spheres appear on the iPhone.

**Walk test:** dog walks → 🟢 should track it; you walk → all spheres stay
world-anchored. (Currently 🟢 tracks in the WRONG direction — the open bug.)

Recalibrate without relaunch:
`ros2 service call /odom_world_calibrator/recalibrate std_srvs/srv/Trigger`

---

## Deferred / still TODO

- 🐞 Fix the green-tracker direction bug (measure real mount rotation → update
  `base_link → camera_link` → recalibrate). **Start here next session.**
- Measure the real RealSense mount offset (translation too).
- **Part 3:** rewrite `calibrated_forwarder` to use
  `lookup_transform(arkit_world, tag)` so detections place from the TF tree (and
  track the dog) instead of the old frozen-anchor path.

---

## Key files & entry points

| File | Role |
|---|---|
| `ar_explorer/ar_explorer/odom_relay_receiver.py` | UDP `:9870` → `odom→base_link` |
| `ar_explorer/ar_explorer/odom_world_calibrator.py` | compute+save `arkit_world→odom` (settle/converge, `~/recalibrate` svc) |
| `ar_explorer/ar_explorer/odom_world_tf.py` | sole persistent publisher of `arkit_world→odom` from file |
| `ar_explorer/ar_explorer/origin_markers.py` | 🔴 arkit origin, 🔵 odom origin, 🟢 live base_link → `/ar_markers` |
| `ar_explorer/launch/ar_pipeline.launch.py` | topology; `base_link→camera_link` identity mount (**suspect**) |
| `~/.ros/ar_explorer_odom_world.json` | saved calibration (written by calibrator, read by odom_world_tf) |

Recent commits on `main`: `7e3cd39` (robust auto-start calibration), `9bd6bca`
(origin_markers), `5e67983` (mount + arkit_world→odom bridge), `fd5e95a`
(odom node in launch).
