"""Follow XYZ requests by bounded, validated Cartesian steps from measured FK.

Backtracking searches along the requested direction, not a global nearest pose.
Unconverged solver outputs are never accepted. Receiver drive limits remain in
charge of physical motion; this helper only chooses a reachable destination.
"""
from dataclasses import dataclass
import time

import numpy as np
import pinocchio as pin


@dataclass
class ReachableResult:
    q: np.ndarray
    success: bool
    mode: str
    fraction: float
    attempts: int
    remaining_mm: list
    diagnostics: object = None


class ReachableIK:
    def __init__(self, ik, max_step=.04, max_joint_step=.20, retries=4,
                 retry_iterations=50, budget_ms=12.):
        self.ik = ik
        self.max_step = max_step
        self.max_joint_step = max_joint_step
        self.retries = retries
        self.retry_iterations = retry_iterations
        self.budget_ms = budget_ms

    def solve(self, target_left, target_right, measured_q, **solver_options):
        measured_q = np.asarray(measured_q, dtype=float).copy()
        current = self.ik.forward_kinematics(measured_q)
        requested = [target_left, target_right]
        deltas = [target.translation - pose.translation for target, pose in zip(requested, current)]
        distance = max(np.linalg.norm(delta) for delta in deltas)
        fraction = min(1., self.max_step / distance) if distance > 0 else 1.
        started = time.perf_counter()
        failure = None
        attempts = 0
        lower = self.ik.model.lowerPositionLimit[self.ik.arm_q_indices]
        upper = self.ik.model.upperPositionLimit[self.ik.arm_q_indices]
        for attempt in range(self.retries + 1):
            if attempt and (time.perf_counter() - started) * 1000 >= self.budget_ms:
                break
            targets = [pin.SE3(target.rotation.copy(), pose.translation + fraction * delta)
                       for target, pose, delta in zip(requested, current, deltas)]
            options = dict(solver_options)
            if attempt:
                options['max_iter'] = min(options.get('max_iter', 100), self.retry_iterations)
            attempts += 1
            q, ok = self.ik.solve(*targets, measured_q, **options)
            failure = self.ik.last_diagnostics
            valid = (ok and np.isfinite(q).all() and np.all(q >= lower) and np.all(q <= upper)
                     and np.max(np.abs(q - measured_q)) <= self.max_joint_step)
            if valid:
                # Independently check both full poses, including fixed rotation.
                actual = self.ik.forward_kinematics(q)
                eps = options.get('eps', 1e-4)
                valid = np.linalg.norm(np.concatenate([
                    pin.log6(pose.inverse() * target).vector for pose, target in zip(actual, targets)])) < eps
                if valid:
                    mode = 'PROJECTED' if attempt else 'STEP' if fraction < 1. else 'FULL'
                    remaining = [float(np.linalg.norm(pose.translation - target.translation) * 1000)
                                 for pose, target in zip(actual, requested)]
                    return ReachableResult(q.copy(), True, mode, fraction, attempts, remaining)
            fraction *= .5
        return ReachableResult(measured_q, False, 'HOLD', 0., attempts,
                               [float(np.linalg.norm(d) * 1000) for d in deltas], failure)
