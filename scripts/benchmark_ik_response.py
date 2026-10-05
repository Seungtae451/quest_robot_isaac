"""Compare reachable full-pose IK responses with ideal measured joint tracking.

Uses the current HOME and gripper-middle TCP. No Isaac, recording, or user
dataset is started. Reported times are controller time, not wall-clock time.
"""
import json
from pathlib import Path
import sys
import time

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import teleop_config as cfg
from robot.f14_config import HOME_Q, F14_URDF_PATH, GRIPPER_CONTROL_OFFSET
from robot.f14_ik import F14IK
from robot.differential_ik import DifferentialIK

OLD = dict(QP_POSITION_GAIN=3., QP_ORIENTATION_GAIN=5.,
           QP_MAX_CARTESIAN_SPEED=.25, QP_ORIENTATION_WEIGHT=3.,
           QP_NONLINEAR_ROTATION_TOLERANCE=cfg.IK_EPS,
           QP_LAG_HORIZON=.5, ARM_MAX_ACCELERATION=.5)


def measure(ik, targets):
    solver = DifferentialIK(ik, rotating_geometry=True)
    q = HOME_Q.copy()
    previous = np.zeros(14)
    elapsed = []
    reached = None
    failures = 0
    dt = 1 / 60
    for step in range(1800):
        start = time.perf_counter()
        result = solver.solve(*targets, q, dt=dt)
        elapsed.append((time.perf_counter() - start) * 1000)
        failures += not all(result.diagnostics['arm_success'])
        velocity = (result.q - q) / dt
        assert np.max(np.abs(velocity)) <= cfg.ARM_MAX_VELOCITY + 1e-6
        assert np.max(np.abs(velocity - previous)) <= cfg.ARM_MAX_ACCELERATION * dt + 1e-6
        q, previous = result.q, velocity
        actual = ik.forward_kinematics(q)
        pos = [np.linalg.norm(a.translation - t.translation) for a, t in zip(actual, targets)]
        rot = [np.linalg.norm(pin.log3(a.rotation.T @ t.rotation)) for a, t in zip(actual, targets)]
        if max(pos) < .002 and max(rot) < np.deg2rad(1) and reached is None:
            reached = (step + 1) * dt
        if max(pos) < cfg.IK_EPS and max(rot) < cfg.IK_EPS:
            break
    return dict(reach_2mm_1deg_seconds=reached, elapsed_controller_seconds=(step+1)*dt,
                final_position_mm=(np.array(pos)*1000).tolist(),
                final_rotation_deg=np.rad2deg(rot).tolist(),
                solve_p95_ms=float(np.percentile(elapsed,95)), rejected_steps=failures)


def main():
    ik = F14IK(F14_URDF_PATH, ee_offset=GRIPPER_CONTROL_OFFSET)
    current = {key: getattr(cfg, key) for key in OLD}
    scenarios = {}
    for name, delta in [('small_pose',[.15,.12,-.1,.1,.08,0.,0.]),
                        ('large_pose',[.4,.25,-.2,.3,.15,.1,0.])]:
        goal = HOME_Q.copy()
        goal[:7] += delta
        full = ik.arm_to_full_q(goal)
        assert np.all(full >= ik.model.lowerPositionLimit) and np.all(full <= ik.model.upperPositionLimit)
        targets = ik.forward_kinematics(goal)  # reachable position AND orientation
        values = {}
        try:
            for profile, settings in [('previous', OLD), ('fast', current)]:
                for key, value in settings.items():
                    setattr(cfg, key, value)
                values[profile] = measure(ik, targets)
        finally:
            for key, value in current.items():
                setattr(cfg, key, value)
        scenarios[name] = values
    output = ROOT / 'outputs/ik_diagnosis/fast_response_benchmark.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(dict(ideal_measured_tracking=True, scenarios=scenarios),indent=2)+'\n')
    print(output)
    print(json.dumps(scenarios, indent=2))


if __name__ == '__main__':
    main()
