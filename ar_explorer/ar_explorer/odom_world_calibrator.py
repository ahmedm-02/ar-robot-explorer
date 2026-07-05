#!/usr/bin/env python3
"""Compute the fixed arkit_world → odom calibration and SAVE it to a file.

This node ONLY computes and writes the calibration; it never publishes TF.
The single persistent publisher of arkit_world → odom is `odom_world_tf`, which
loads the file this node writes. Splitting compute from publish is deliberate:
running this twice can no longer create conflicting /tf_static publishers — it
just rewrites the file.

WHY the settle + converge gate: AprilTag pose is noisy right after the pipeline
starts (exposure/focus settling, first detections jittering). A naive "first
handshake → compute" would lock in a bad transform and then silently show wrong
coordinates. So we only accept a reading that has settled AND converged:

  1. WARM-UP  — once the tag is visible in BOTH trees, discard the first
                `settle_time` seconds of samples (the jittery ones).
  2. CONVERGE — then collect over `window` seconds and measure the SPREAD
                (translation stddev + max rotation deviation) of the tag pose in
                each tree. Only finalize if both are within tolerance.
  3. SAFE     — if it never converges, write NOTHING (odom_world_tf then
                publishes no edge) → no calibration rather than a wrong one.

After a successful save the node stays alive (idle) and offers a `recalibrate`
std_srvs/Trigger service to redo the settle+converge cycle without relaunching.
Because it only writes the file, staying alive creates ZERO duplicate-publisher
risk.

Formula (same physical tag in both trees → tag_<id> ≡ iphone_tag_<id>):
    T_arkit_from_odom = T_arkit_from_tag @ inverse(T_odom_from_tag)

Usage:
    ros2 run ar_explorer odom_world_calibrator                # auto-calibrates
    ros2 service call /odom_world_calibrator/recalibrate std_srvs/srv/Trigger
"""

from __future__ import annotations

import json
import os
import sys
import time
from datetime import datetime, timezone

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_srvs.srv import Trigger

try:
    import tf2_ros
    from tf2_ros import TransformException
except ImportError:
    print("ERROR: tf2_ros not available. Source /opt/ros/jazzy/setup.bash first.",
          file=sys.stderr)
    sys.exit(1)


DEFAULT_OUTPUT = os.path.expanduser("~/.ros/ar_explorer_odom_world.json")


def transform_to_matrix(transform) -> np.ndarray:
    t = transform.translation
    q = transform.rotation
    M = np.eye(4)
    M[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def average_matrices(samples: list[np.ndarray]) -> np.ndarray:
    """Mean translation + hemisphere-corrected quaternion mean."""
    translations = np.array([T[:3, 3] for T in samples])
    avg_t = translations.mean(axis=0)
    quats = _aligned_quats(samples)
    avg_q = quats.mean(axis=0)
    avg_q /= np.linalg.norm(avg_q)
    M = np.eye(4)
    M[:3, :3] = R.from_quat(avg_q).as_matrix()
    M[:3, 3] = avg_t
    return M


def _aligned_quats(samples: list[np.ndarray]) -> np.ndarray:
    """Quaternions with the q/-q double cover resolved against the first sample."""
    quats = np.array([R.from_matrix(T[:3, :3]).as_quat() for T in samples])
    ref = quats[0]
    for i in range(1, len(quats)):
        if np.dot(quats[i], ref) < 0:
            quats[i] = -quats[i]
    return quats


def spread(samples: list[np.ndarray]) -> tuple[float, float]:
    """Return (translation spread [m], rotation spread [deg]) of a sample set.
    Translation spread = ||per-axis stddev||; rotation spread = max angular
    deviation from the mean rotation."""
    trans = np.array([T[:3, 3] for T in samples])
    trans_spread = float(np.linalg.norm(trans.std(axis=0)))

    quats = _aligned_quats(samples)
    mean_q = quats.mean(axis=0)
    mean_q /= np.linalg.norm(mean_q)
    dots = np.clip(np.abs(quats @ mean_q), 0.0, 1.0)
    rot_spread = float(np.degrees(2.0 * np.arccos(dots)).max())
    return trans_spread, rot_spread


class OdomWorldCalibrator(Node):
    # State machine
    WAITING = "waiting"      # tag not visible in both trees
    SETTLING = "settling"    # both visible, discarding warm-up jitter
    COLLECTING = "collecting"  # gathering a convergence window
    DONE = "done"            # calibrated; idle until recalibrate

    def __init__(self) -> None:
        super().__init__("odom_world_calibrator")

        self.declare_parameter("tag_id", 17)
        self.declare_parameter("world_frame", "arkit_world")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("settle_time", 2.0)     # warm-up discard, s
        self.declare_parameter("window", 2.0)          # convergence sample window, s
        self.declare_parameter("min_samples", 15)
        self.declare_parameter("trans_tol", 0.01)      # max translation spread, m
        self.declare_parameter("rot_tol_deg", 2.0)     # max rotation spread, deg
        self.declare_parameter("output", DEFAULT_OUTPUT)

        self.tag_id = int(self.get_parameter("tag_id").value)
        self.world_frame = self.get_parameter("world_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.window = float(self.get_parameter("window").value)
        self.min_samples = int(self.get_parameter("min_samples").value)
        self.trans_tol = float(self.get_parameter("trans_tol").value)
        self.rot_tol_deg = float(self.get_parameter("rot_tol_deg").value)
        self.output = self.get_parameter("output").value

        self.odom_tag_frame = f"tag_{self.tag_id}"
        self.arkit_tag_frame = f"iphone_tag_{self.tag_id}"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.srv = self.create_service(Trigger, "~/recalibrate", self._on_recalibrate)

        self._state = self.WAITING
        self._t0 = None
        self._odom_samples: list[np.ndarray] = []
        self._arkit_samples: list[np.ndarray] = []
        self._logged_wait = False

        self.create_timer(0.05, self._tick)   # 20 Hz
        self.get_logger().info(
            f"odom_world_calibrator — auto-calibrating on tag {self.tag_id} "
            f"(settle {self.settle_time:.1f}s, window {self.window:.1f}s, "
            f"tol {self.trans_tol*100:.1f}cm/{self.rot_tol_deg:.1f}deg). "
            f"Saves to {self.output}"
        )

    # ---------------------------------------------------------------- service
    def _on_recalibrate(self, request, response):
        self.get_logger().info("Recalibration requested — restarting settle+converge.")
        self._reset(self.WAITING)
        response.success = True
        response.message = "Recalibration started."
        return response

    def _reset(self, state: str) -> None:
        self._state = state
        self._t0 = None
        self._odom_samples.clear()
        self._arkit_samples.clear()

    # ------------------------------------------------------------------- loop
    def _lookup(self, parent: str, child: str):
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return transform_to_matrix(tf.transform)
        except TransformException:
            return None

    def _tick(self) -> None:
        if self._state == self.DONE:
            return

        T_odom = self._lookup(self.odom_frame, self.odom_tag_frame)
        T_arkit = self._lookup(self.world_frame, self.arkit_tag_frame)

        # Both trees must currently see the tag; losing it resets the cycle so we
        # never mix pre- and post-occlusion samples.
        if T_odom is None or T_arkit is None:
            if self._state != self.WAITING:
                self.get_logger().warn("Lost the tag in one tree — restarting.")
            self._reset(self.WAITING)
            if not self._logged_wait:
                self._logged_wait = True
                self.get_logger().info(
                    f"Waiting for tag {self.tag_id} in BOTH trees "
                    f"({self.odom_frame}→{self.odom_tag_frame}, "
                    f"{self.world_frame}→{self.arkit_tag_frame})..."
                )
            return
        self._logged_wait = False

        now = time.monotonic()

        if self._state == self.WAITING:
            self._state = self.SETTLING
            self._t0 = now
            self.get_logger().info(
                f"Both trees see tag {self.tag_id} — settling for "
                f"{self.settle_time:.1f}s (discarding startup jitter)..."
            )
            return

        if self._state == self.SETTLING:
            if now - self._t0 < self.settle_time:
                return
            self._state = self.COLLECTING
            self._t0 = now
            self._odom_samples.clear()
            self._arkit_samples.clear()
            self.get_logger().info(f"Settled — collecting for {self.window:.1f}s...")
            return

        # COLLECTING
        self._odom_samples.append(T_odom)
        self._arkit_samples.append(T_arkit)
        if now - self._t0 < self.window:
            return
        self._evaluate()

    def _evaluate(self) -> None:
        n = min(len(self._odom_samples), len(self._arkit_samples))
        if n < self.min_samples:
            self.get_logger().warn(
                f"Only {n} samples (<{self.min_samples}) — recollecting.")
            self._reset(self.SETTLING)
            self._t0 = time.monotonic()   # re-settle
            return

        odom_ts, odom_rs = spread(self._odom_samples)
        arkit_ts, arkit_rs = spread(self._arkit_samples)
        converged = (odom_ts <= self.trans_tol and arkit_ts <= self.trans_tol
                     and odom_rs <= self.rot_tol_deg and arkit_rs <= self.rot_tol_deg)

        if not converged:
            self.get_logger().warn(
                f"Not converged (spread — RS {odom_ts*100:.1f}cm/{odom_rs:.1f}deg, "
                f"iPhone {arkit_ts*100:.1f}cm/{arkit_rs:.1f}deg > tol "
                f"{self.trans_tol*100:.1f}cm/{self.rot_tol_deg:.1f}deg). "
                "Detections still noisy — retrying, NOT saving."
            )
            # Re-collect a fresh window (tag may still be settling).
            self._state = self.COLLECTING
            self._t0 = time.monotonic()
            self._odom_samples.clear()
            self._arkit_samples.clear()
            return

        T_odom_from_tag = average_matrices(self._odom_samples)
        T_arkit_from_tag = average_matrices(self._arkit_samples)
        T_arkit_from_odom = T_arkit_from_tag @ np.linalg.inv(T_odom_from_tag)

        self._save(T_arkit_from_odom, n, max(odom_ts, arkit_ts), max(odom_rs, arkit_rs))
        t = T_arkit_from_odom[:3, 3]
        self.get_logger().info(
            f"===== CALIBRATED ({n} samples, spread "
            f"{max(odom_ts, arkit_ts)*100:.1f}cm/{max(odom_rs, arkit_rs):.1f}deg) "
            f"=====\n{self.world_frame} → {self.odom_frame}  "
            f"t=({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})\n"
            f"Saved to {self.output}. odom_world_tf will publish it. Idle now "
            "(call ~/recalibrate to redo).\n"
            "=================================================="
        )
        self._state = self.DONE

    def _save(self, M: np.ndarray, n: int, ts: float, rs: float) -> None:
        q = R.from_matrix(M[:3, :3]).as_quat()  # (x, y, z, w)
        t = M[:3, 3]
        data = {
            "parent_frame": self.world_frame,
            "child_frame": self.odom_frame,
            "translation": [float(t[0]), float(t[1]), float(t[2])],
            "quaternion": [float(q[0]), float(q[1]), float(q[2]), float(q[3])],
            "tag_id": self.tag_id,
            "n_samples": n,
            "trans_spread_m": ts,
            "rot_spread_deg": rs,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        # Atomic write so odom_world_tf never reads a half-written file.
        tmp = self.output + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, self.output)


def main(args=None):
    rclpy.init(args=args)
    node = OdomWorldCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
