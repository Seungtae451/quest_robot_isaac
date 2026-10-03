"""Canonical, simulator-independent F14 names, HOME, and existing asset paths.

Semantic ordering is left arm (7), right arm (7), left/right closure (2).
The simulator's interleaved joint storage must never leak into that ordering.
"""
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
F14_URDF_PATH = PROJECT_ROOT / "assets/F14_URDF_rev_2_0_1 2/urdf/FlaminGO_14Dof_Arm_Robot_v2.urdf"
F14_USD_PATH = PROJECT_ROOT / "assets/f14/FlaminGO_14Dof_Arm_Robot_v2_isaac/FlaminGO_14Dof_Arm_Robot_v2_isaac.usd"
# Both grippers extend along wrist-local -Y (finger origins y=-0.1275).
# Fix that axis to world -Z, with the jaws closing along world Y. Mirrored
# link frames need different rotations; an identity EE rotation is NOT down.
GRIPPER_FORWARD_AXIS = np.array([0., -1., 0.])
LEFT_EE_DOWN_ROT = np.array([[-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
RIGHT_EE_DOWN_ROT = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
# Collision-free downward HOME wrist positions: (0.40, +/-0.18, 0.57) m.
# Fingers extend 0.235 m below the wrist, leaving 35 mm over the z=0.30 table.
# Compared to the previous HOME, raise ~10 cm, move forward ~10 cm and out
# ~5 cm. This yaw/IK branch leaves wrist joint-limit margin for XYZ movement.
# Start/reset here so enabling XYZ control does not first reorient the wrist.
LEFT_HOME = np.array([
   -1.0714420570, 0.7260463689, 1.5511059018, -1.4016318491,
    1.5896682282, -0.6753974941, -1.0963101219])
RIGHT_HOME = np.array([
   -1.0714388313, -0.7260551171, -1.5511024793, -1.4016360913,
   -1.5896669913, 0.6753994612, -1.0963084577])
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
