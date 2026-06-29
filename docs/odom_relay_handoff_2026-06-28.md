# Odometry Relay — Handoff / Session Summary
**Date:** 2026-06-28
**Goal:** Get the dog's `/leg_odom2` pose onto the Jazzy ROS graph as a TF
(`odom -> base_link`), across the Foxy ↔ Jazzy boundary, without crashing the dog.

---

## TL;DR — where we are

- ✅ **Raw data hop fully verified** — the dog's pose reaches the NUC live over the
  ethernet cable, confirmed **even with the dog powered and moving**.
- ✅ **The TF-conversion node is written** (`~/odom_tf_node.py`) and passes a syntax
  check.
- ⛔ **Blocker:** the node can't run yet — `python3 ~/odom_tf_node.py` fails with
  `ModuleNotFoundError: No module named 'yaml'` (a Python-environment problem, not a
  code problem). Must be fixed before anything else.
- 🔜 **Decided plan:** move all **Jazzy** machines to `ROS_DOMAIN_ID=42` (dog stays
  `0`) to isolate them, then run the node and publish the transform.

---

## What was verified

A pure-UDP listener on the NUC (`~/udp_pose_listen.py`, **no ROS**, cannot crash the
dog) received the dog's relay datagrams on `:9870`:

- Source `192.168.186.2` (the dog) over the **ethernet cable**, ~30 Hz, zero malformed.
- **Stationary** readout matched the dog's own output (user-confirmed).
- **Live + moving** confirmed later: values track as the dog moves; the `from` address
  distinguishes real dog (`192.168.186.2`) from any local/test sender (`127.0.0.1`).

**Key insight — odometry is relative, not absolute:** after the dog power-cycled
(battery died → charged), its odometry **reset to `0,0,0`** at boot. `/leg_odom2` reports
displacement from the odom-frame origin set at boot, not an absolute world position.
(That's why an early reading was `x=0.5683` and a post-reboot reading was `0,0,0`.)

---

## The core gotcha — Foxy ↔ Jazzy DDS crash

- The dog runs **ROS 2 Foxy** (old Fast-RTPS); the NUC/ASUS run **ROS 2 Jazzy** (newer
  Fast DDS). **They cannot share DDS** — when a Jazzy participant tries to match a Foxy
  one, the **Foxy node crashes** with `deserialization of data failed`, taking the dog's
  motion bringup down.
- That's why the relay uses **plain UDP/JSON** (version-agnostic) instead of a ROS
  topic across the boundary.
- **`/tf` is the specific collision point** — the dog both publishes and listens to
  `/tf`, so any Jazzy `/tf` publisher it can see kills it. (The NUC's RealSense pipeline
  coexists only because it doesn't share a topic with the dog.)

### Why this keeps coming up: the domain situation (CONFIRMED)

- **Everything is currently `ROS_DOMAIN_ID=0`** — the dog AND the Jazzy graph
  (ASUS `echo $ROS_DOMAIN_ID` → `0`). They are **not** domain-separated.
- They coexist only because the **dog is on a different subnet** (`eno1` cable
  `192.168.186.x`) than the ASUS (wifi). DDS discovery is per-subnet multicast.
- **The NUC is the unique leak point:** it is dual-homed (cable + wifi), so any Jazzy
  DDS node on the NUC on domain 0 announces on **both** subnets and reaches the dog.
- This is exactly why our earlier `odom_relay_receiver` (run on the NUC, unset domain
  = 0) crashed the dog.

---

## The fix we chose: domain separation on 42

Move **all Jazzy** machines to `ROS_DOMAIN_ID=42`; **dog stays on 0**. Different domains
never discover each other, even on the same cable — so the dog can't see the Jazzy
`/tf`, and can't crash. Bonus: it also fixes the latent risk that the NUC's `ar_pipeline`
(also Jazzy on domain 0) poses to the dog.

- Make it robust by setting `export ROS_DOMAIN_ID=42` **persistently in `~/.bashrc`** on
  the NUC and the ASUS, so no terminal ever forgets and falls back to 0.
- ⚠️ The UDP relay is **domain-agnostic** — setting the NUC to 42 does **not** affect
  reception of the dog's packets (the socket ignores `ROS_DOMAIN_ID`). Domain only
  changes *who sees the node's `/tf`*.

(Alternative considered: run the node on the ASUS + a dumb UDP forwarder on the NUC —
structurally crash-proof but more moving parts. We went with domain 42 for simplicity
and because it keeps the node on the NUC where the data already lands.)

---

## The conversion logic (pose → transform)

A pose **is** a transform: `/leg_odom2` already expresses base_link's pose in the odom
frame, which **is** the `odom -> base_link` transform. So conversion is a field copy:

| TransformStamped field | Source | Note |
|---|---|---|
| `header.stamp` | **local NUC clock `now()`** | restamp — dog's stamp is its own clock, would break tf2 |
| `header.frame_id` | hardcode `"odom"` | payload carries no frame names |
| `child_frame_id` | hardcode `"base_link"` | " |
| `translation.x/y/z` | `px/py/pz` | direct copy |
| `rotation.x/y/z/w` | `qx/qy/qz/qw` | direct copy |

Three real rules: (1) restamp locally, (2) name the frames, (3) **NO axis remap** — the
dog's odom is already REP-103 (`+x` fwd, `+y` left, `+z` up). (Optional: normalize the
quaternion.)

---

## The TF tree design (for later, when wiring in)

**Persistent operational tree:**
```
arkit_world ──(fixed, from calibration)── odom ──(our node, ~30Hz)── base_link ──(static mount)── camera
     └──(ARKit, live)── iphone
```
- `odom -> base_link` ← **our node** (dynamic, `/tf`).
- `base_link -> camera` ← static mount (`/tf_static`, separate).
- `arkit_world -> odom` ← calibration bridge (fixed; re-calibrated periodically as both
  frames drift).

**Calibration is a one-time computation, not a tree.** The chain
`odom → base_link → camera → tag → iphone → arkit_world` is the *path you multiply
through once* (both cameras see the same physical tag) to solve for the fixed
`odom ↔ arkit_world` bridge. It can't be a live tree (it would form a cycle). In the
codebase the tag is two separate frames — `tag_17` (under RealSense `camera`) and
`iphone_tag_17` (under `iphone_camera`) — because a frame can't have two parents;
calibration equates them mathematically. Our node owns **only** `odom -> base_link`.

---

## Files created this session (on the NUC, in `~`)

| File | Purpose | Status |
|---|---|---|
| `~/udp_pose_listen.py` | Pure-UDP listener; prints incoming pose. No ROS → can't crash the dog. | ✅ verified working |
| `~/odom_tf_node.py` | **The node.** UDP in → `/tf` `odom->base_link` out. Standalone, not in the launch file. | ⛔ blocked on `yaml` env |
| `~/dummy_odom_sender.py` | Replays the captured pose to `127.0.0.1:9870` for testing (`--animate` to sweep). | ✅ works |
| `~/tf_check.py` | Subscribes to `/tf`, prints first few transforms, exits. Reliable verifier. | ready (untested) |

Repo: `~/ar-robot-explorer`, branch **`odom-relay`** (NOT `main`). Contains the dog
sender `dog_relay/odom_relay_sender.py` and the original
`ar_explorer/ar_explorer/odom_relay_receiver.py` (same logic as `odom_tf_node.py` but
⚠️ it crashes the dog if run on the NUC on domain 0).

---

## Network facts

| Thing | Value |
|---|---|
| Dog (Foxy, domain 0), cable side | `192.168.186.2` on the NUC's `eno1` |
| Dog SSH (from personal laptop only) | `ysc@192.168.1.120` |
| NUC (Jazzy) cable side | `192.168.186.3` (`eno1`) — use as the sender's `--nuc-ip` |
| NUC wifi (toward ASUS) | `192.168.0.116` (`wlp0s20f3`) |
| NUC tailscale | `100.72.194.78` |
| ASUS (Jazzy, domain 0) | base computer (reach/IP TBD) |
| Relay UDP port | `9870` |
| Domains | dog `0`; Jazzy currently `0` → **plan: move Jazzy to `42`** |

---

## NEXT STEPS (ordered for safety)

**0. Fix the Python environment** *(zero risk)* — find which `python3` has `rclpy` +
`yaml` (is a conda/venv shadowing the system Python? is `python3-yaml` installed?), so
`python3 ~/odom_tf_node.py` imports cleanly.

**1. Isolated test** *(zero risk — DDS stays on loopback)*:
```bash
# terminal A (node, localhost-only so it can't reach the dog):
source /opt/ros/jazzy/setup.bash && ROS_LOCALHOST_ONLY=1 python3 ~/odom_tf_node.py
# terminal B (dummy data):
python3 ~/dummy_odom_sender.py --host 127.0.0.1 --port 9870
# terminal C (verify the TF):
source /opt/ros/jazzy/setup.bash && ROS_LOCALHOST_ONLY=1 python3 ~/tf_check.py
```
Confirm `tf_check` prints `odom -> base_link` with the dummy's translation/rotation.

**2. Set the Jazzy domain to 42** — add `export ROS_DOMAIN_ID=42` to `~/.bashrc` on the
**NUC** and on the **ASUS** (do the ASUS manually). Re-source. **Do NOT touch the dog.**

**3. Run the node live on domain 42** — first `echo $ROS_DOMAIN_ID` → must read `42`.
Then run `python3 ~/odom_tf_node.py` (it receives the dog's real UDP). **Tripwire:** keep
`~/udp_pose_listen.py` running; if the dog's `192.168.186.2` stream stutters, kill the
node immediately.

**4. Verify end-to-end** — on the NUC (domain 42): `tf_check.py` or
`ros2 run tf2_ros tf2_echo odom base_link`. On the ASUS (domain 42): same → confirms it
propagates Jazzy → Jazzy. Dog still streaming = isolation held. ✅

**5. Wire into the pipeline (later)** — add `base_link -> camera` (static mount) and
`arkit_world -> odom` (calibration bridge) so the dog's pose actually drives marker
placement.

---

## Quick re-verify (the safe raw check, anytime)
```bash
# NUC:
python3 ~/udp_pose_listen.py --port 9870
# Dog (ssh ysc@192.168.1.120, Foxy sourced, bringup running):
python3 ~/odom_relay_sender.py --nuc-ip 192.168.186.3 --port 9870
```
Rule of thumb reading the feed: `from 192.168.186.2` = real dog; any other source
(e.g. `127.0.0.1`) = a local/test sender.
