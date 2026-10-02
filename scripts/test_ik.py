"""Standalone synthetic F14 IK acceptance checks; no Quest or Isaac runtime."""
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot.f14_config import HOME_Q, F14_URDF_PATH
from robot.f14_ik import F14IK
from config import teleop_config as cfg


def main():
    ik = F14IK(F14_URDF_PATH)
    left, right = ik.forward_kinematics(HOME_Q)
    for pose in (left, right):
        pose.translation[0] += .05
        pose.rotation = pose.rotation @ pin.exp3(np.array([0., 0., .02]))
    q, ok = ik.solve(left, right, HOME_Q, max_iter=cfg.IK_MAX_ITER, eps=cfg.IK_EPS,
                     dt=cfg.IK_DT, damping=cfg.IK_DAMPING)
    assert ok, f"IK failed: {ik.last_error}"
    for actual, desired in zip(ik.forward_kinematics(q), (left, right)):
        error = pin.log6(actual.inverse() * desired).vector
        print("6DoF residual [metres, radians]:", error)
        assert np.linalg.norm(error) < cfg.IK_EPS
    print("IK PASS: HOME +5 cm X and relative rotation, both arms")


if __name__ == "__main__":
    main()
