"""Slow HOME return between episodes; no robot joint-state teleportation."""
import numpy as np
from collections import deque

from config import recording_config as cfg
from robot.f14_config import HOME_ACTION


class EpisodeReset:
    def __init__(self):
        self.elapsed = 0.
        self.stable = 0.
        self.phase = "OPEN"
        self.error = None
        self.positions = deque(maxlen=120)
        self.gripper_command = None

    def goal(self, measured, dt):
        self.elapsed += dt
        if self.elapsed > cfg.RESET_TIMEOUT_SECONDS:
            self.error = "HOME return timed out; check contact or joint obstruction. No robot state was forced."
        goal = HOME_ACTION.copy()
        # In the current USD/protocol 1 opens the fingers. Release any held cube
        # before returning HOME, while leaving the operator's mapping unchanged.
        if self.gripper_command is None:
            self.gripper_command = measured[14:].copy()
        from teleop.input_timing import timed_filter_alpha
        from config import teleop_config
        alpha = timed_filter_alpha(teleop_config.GRIPPER_FILTER_ALPHA, dt, teleop_config.FILTER_REFERENCE_HZ)
        self.gripper_command += alpha * (1. - self.gripper_command)
        goal[14:] = self.gripper_command
        enabled = self.elapsed >= cfg.RESET_OPEN_SECONDS and self.error is None
        if not enabled:
            goal[:14] = measured[:14]
        return goal, enabled

    def update(self, measured, reference_velocity, dt):
        if self.phase == "OPEN" and self.elapsed >= cfg.RESET_OPEN_SECONDS:
            self.phase = "HOME"
        if self.phase == "HOME":
            self.positions.append(measured[:14].copy())
            reached = (np.max(np.abs(measured[:14] - HOME_ACTION[:14])) < .015
                       and np.max(np.abs(reference_velocity)) < .025)
            self.stable = self.stable + dt if reached else 0.
            window = max(2, round(cfg.RESET_SETTLE_SECONDS / dt))
            # PhysX constraint impulses can report ~0.35 rad/s even when the
            # joint positions are steady within a milliradian. Require a slow
            # drive reference AND stable actual positions over the full window.
            samples = list(self.positions)[-window:]
            measured_stable = len(samples) >= window and np.max(np.ptp(samples, axis=0)) < .003
            if self.stable >= cfg.RESET_SETTLE_SECONDS and measured_stable:
                self.phase = "RESPAWN"
        elif self.phase == "SETTLE":
            self.stable += dt
            if self.stable >= cfg.RESET_SETTLE_SECONDS:
                self.phase = "COMPLETE"

    def cubes_respawned(self):
        self.phase, self.stable = "SETTLE", 0.
