"""Guided physical-controller experiment; observes the existing mapping only."""
import json
from pathlib import Path
import time

import numpy as np
import pinocchio as pin
from config import teleop_config as cfg


TRIALS = [(side, motion) for side in ("left", "right")
          for motion in ("twist / 비틀기", "bend up-down / 위아래 꺾기",
                         "bend left-right / 좌우 꺾기")]


def rotation_vectors(reference, current):
    """Return signed rotation vectors in degrees, in local and world axes."""
    local = np.rad2deg(pin.log3(reference.T @ current))
    return local, reference @ local


class WristAxisTest:
    def __init__(self, directory):
        directory = Path(directory) / time.strftime("%Y%m%d_%H%M%S")
        directory.mkdir(parents=True, exist_ok=False)
        self.directory = directory
        self.trial_index = 0
        self.pending = False
        self.active = False
        self.announced = False
        self.rows = []
        self.summaries = []
        self.duration = 12.
        print(f"[WRIST TEST] Results: {directory}", flush=True)

    def prompt(self):
        if self.trial_index >= len(TRIALS):
            return
        side, motion = TRIALS[self.trial_index]
        print(f"\n[WRIST TEST] Next: {side} {motion}. 머리와 반대 손을 고정하세요.\n"
              "중립에서 로봇도 멈춘 뒤 N + Enter: 12초 기록 시작.\n"
              "한 방향으로 10~15도 → 1~2초 유지 → 중립 → 반대 방향 → 유지 → 중립.\n"
              "처음 움직인 방향도 메모하세요. 회전축 부호는 그 방향을 기준으로 해석합니다.", flush=True)

    def request_start(self):
        if not self.active and self.trial_index < len(TRIALS):
            self.pending = True

    def interrupt(self, reason):
        self.pending = False
        if self.active:
            self.finish(reason)
        self.announced = False

    def update(self, controllers, targets, measured, ik_success, feedback_age, now=None):
        now = time.monotonic() if now is None else now
        if not self.announced:
            self.prompt()
            self.announced = True
        if self.pending:
            self.pending, self.active = False, True
            self.started = now
            self.last_print = -float("inf")
            self.rows = []
            self.references = ([p[:3, :3].copy() for p in controllers],
                               [p.rotation.copy() for p in targets],
                               [p.rotation.copy() for p in measured])
            print("[WRIST TEST] START — 12초 동안 천천히 움직이세요.", flush=True)
        if not self.active:
            return
        side, motion = TRIALS[self.trial_index]
        index = 0 if side == "left" else 1
        row = {"time_s": now - self.started, "side": side, "motion": motion,
               "ik_success": bool(ik_success), "feedback_age_s": float(feedback_age)}
        for name, reference, rotation in zip(
                ("controller", "target", "measured"),
                [r[index] for r in self.references],
                (controllers[index][:3, :3], targets[index].rotation, measured[index].rotation)):
            local, world = rotation_vectors(reference, rotation)
            row[f"{name}_local_deg"] = local.tolist()
            row[f"{name}_world_deg"] = world.tolist()
        row["orientation_error_deg"] = float(np.rad2deg(np.linalg.norm(
            pin.log3(measured[index].rotation.T @ targets[index].rotation))))
        basis = np.asarray(getattr(cfg, f"{side.upper()}_CONTROLLER_TO_EE_ROT"))
        for name in ("target", "measured"):
            row[f"{name}_hand_deg"] = (basis.T @ np.array(row[f"{name}_local_deg"])).tolist()
        self.rows.append(row)
        if now - self.last_print >= .5:
            values = " ".join(f"{name}={np.round(row[name + '_world_deg'], 1)}"
                              for name in ("controller", "target", "measured"))
            print(f"[WRIST TEST] {side} {row['time_s']:.1f}s WORLD[X,Y,Z]deg "
                  f"{values} IK={'OK' if ik_success else 'FAIL'} "
                  f"error={row['orientation_error_deg']:.1f}deg", flush=True)
            print("[WRIST TEST] HAND[twist,bend,sideways]deg " + " ".join(
                f"{name}={np.round(row[key], 1)}" for name, key in (
                    ("controller", "controller_local_deg"), ("target", "target_hand_deg"),
                    ("measured", "measured_hand_deg"))), flush=True)
            self.last_print = now
        if now - self.started >= self.duration:
            self.finish()
            self.prompt()

    def finish(self, interrupted=None):
        side, motion = TRIALS[self.trial_index]
        path = self.directory / f"{self.trial_index + 1}_{side}_record_{len(self.summaries) + 1}.json"
        path.write_text(json.dumps({"side": side, "motion": motion,
                                   "interrupted": interrupted,
                                   "reference_rotations": [[r.tolist() for r in group]
                                                           for group in self.references],
                                   "samples": self.rows}, indent=2) + "\n")
        summary = {"side": side, "motion": motion, "interrupted": interrupted,
                   "samples_file": path.name,
                   "ik_failed_samples": sum(not r["ik_success"] for r in self.rows)}
        if self.rows:
            peak = max(self.rows, key=lambda r: np.linalg.norm(r["controller_world_deg"]))
            summary["controller_peak_sample"] = peak
            summary["max_orientation_error_deg"] = max(r["orientation_error_deg"] for r in self.rows)
            print("[WRIST TEST] Saved. Controller peak WORLD degrees:",
                  np.round(peak["controller_world_deg"], 1),
                  "IK failed samples:", summary["ik_failed_samples"], flush=True)
        self.summaries.append(summary)
        (self.directory / "summary.json").write_text(json.dumps({
            "axes": "+X forward, +Y left, +Z up; positive rotation uses right-hand rule",
            "note": "Local vectors use each pose's trial-start axes. World vectors share wrapper/robot axes."
                    " Hand vectors express robot local rotations in the physical hand frame; compare to controller local."
                    " Measured FK is from Isaac feedback (up to 60 Hz); axes/sign need the operator's motion labels.",
            "trials": self.summaries}, indent=2) + "\n")
        self.active = False
        if interrupted is None:
            self.trial_index += 1
        if self.trial_index == len(TRIALS):
            print(f"[WRIST TEST] COMPLETE: {self.directory / 'summary.json'}", flush=True)
