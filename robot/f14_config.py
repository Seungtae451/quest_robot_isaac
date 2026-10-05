"""Canonical, simulator-independent F14 names, HOME, and existing asset paths.

Semantic ordering is left arm (7), right arm (7), left/right closure (2).
The simulator's interleaved joint storage must never leak into that ordering.
"""
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
F14_URDF_PATH = PROJECT_ROOT / "assets/F14_URDF_rev_2_0_1 2/urdf/FlaminGO_14Dof_Arm_Robot_v2.urdf"
F14_USD_PATH = PROJECT_ROOT / "assets/f14/FlaminGO_14Dof_Arm_Robot_v2_isaac/FlaminGO_14Dof_Arm_Robot_v2_isaac.usd"
# Legacy offline downward-pose tools: wrist-local -Y (finger origins
# y=-0.1275) maps to world -Z, with the jaws closing along world Y.
# These rotations are not constraints in live XYZ-only TCP control.
GRIPPER_FORWARD_AXIS = np.array([0., -1., 0.])
# TCP at the midpoint of the two finger ends. URDF finger origins have
# y=-0.1275 and mesh tips y=-0.107498; their symmetric z midpoint is -0.0225.
# Jaw opening changes the gap, not this nominal grasp centre.
GRIPPER_TIP_OFFSET = np.array([0., -0.235, -0.0225])
# Control TCP: midpoint along the finger length, from y=-0.1275 to -0.235.
GRIPPER_CONTROL_OFFSET = np.array([0., -0.18125, -0.0225])
LEFT_EE_DOWN_ROT = np.array([[-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
RIGHT_EE_DOWN_ROT = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
# Mirrored HOME in rev 2.0.1 URDF semantic joint coordinates.
# Left joints 2/3/5: +0.47/-0.11/-0.58; right uses opposite signs.
LEFT_HOME = np.array([
    0., 0.47, 0.11, -1.68, -0.43, 0., 0.])
RIGHT_HOME = np.array([
    0., -0.47, -0.11, -1.68, 0.43, 0., 0.])
HOME_Q = np.concatenate((LEFT_HOME, RIGHT_HOME))
HOME_ACTION = np.concatenate((HOME_Q, [0., 0.]))
LEFT_JOINT_NAMES = [f"left_dof{i}_joint" for i in range(1, 8)]
RIGHT_JOINT_NAMES = [f"right_dof{i}_joint" for i in range(1, 8)]
ARM_JOINT_NAMES = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
# Actions/HOME use rev 2.0.1 URDF angles. The existing rev 2.0.0 USD has
# opposite right dof3/dof4 axes; multiply by these signs at the USD boundary.
USD_ARM_SIGNS = np.array([1., 1., 1., 1., 1., 1., 1., 1., 1., -1., -1., 1., 1., 1.])
GRIPPER_JOINT_NAMES = [f"{side}_{finger}_gripper_joint" for side in ("left", "right") for finger in ("l", "r")]
EE_BODY_NAMES = ["left_dof7_link", "right_dof7_link"]
GRIPPER_WIDTH = 0.0425
