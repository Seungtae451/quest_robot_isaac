"""Canonical, simulator-independent F14 names, HOME, and existing asset paths.

Semantic ordering is left arm (7), right arm (7), left/right closure (2).
The simulator's interleaved joint storage must never leak into that ordering.
"""
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
F14_URDF_PATH = PROJECT_ROOT / "assets/f14/F14_URDF_rev_2_0_0/urdf/FlaminGO_14Dof_Arm_Robot_v2_mujoco.urdf"
F14_USD_PATH = PROJECT_ROOT / "assets/f14/FlaminGO_14Dof_Arm_Robot_v2_isaac/FlaminGO_14Dof_Arm_Robot_v2_isaac.usd"
LEFT_HOME = np.array([-0.57, 0.40, 0.22, -0.95, 0.24, -0.79, -0.27])
RIGHT_HOME = np.array([-0.57, -0.40, 0.22, 0.95, -0.24, 0.79, -0.27])
HOME_Q = np.concatenate((LEFT_HOME, RIGHT_HOME))
HOME_ACTION = np.concatenate((HOME_Q, [0., 0.]))
LEFT_JOINT_NAMES = [f"left_dof{i}_joint" for i in range(1, 8)]
RIGHT_JOINT_NAMES = [f"right_dof{i}_joint" for i in range(1, 8)]
ARM_JOINT_NAMES = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
GRIPPER_JOINT_NAMES = [f"{side}_{finger}_gripper_joint" for side in ("left", "right") for finger in ("l", "r")]
EE_BODY_NAMES = ["left_dof7_link", "right_dof7_link"]
GRIPPER_WIDTH = 0.0425
