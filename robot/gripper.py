"""Convert controller input -> semantic closure -> four physical finger joints.

The scalar always means 0=open, 1=closed. Trigger encoding is chosen once for
the installed wrapper, never guessed per sample (legacy fully pressed is zero,
which is indistinguishable from a standard released sample without context).
"""
import numpy as np

from robot.f14_config import GRIPPER_WIDTH


def normalize_input(raw: float, encoding: str = "standard") -> float:
    if not np.isfinite(raw):
        raise ValueError("Non-finite controller gripper input")
    if encoding == "legacy-inverted-10":
        raw = 1.0 - raw / 10.0
    elif encoding != "standard":
        raise ValueError(f"Unknown gripper encoding: {encoding}")
    return float(np.clip(raw, 0., 1.))


def closure_to_joints(left: float, right: float) -> np.ndarray:
    """Return [left_l, left_r, right_l, right_r] finger translations in metres."""
    closure = np.asarray([left, right], dtype=float)
    if not np.isfinite(closure).all():
        raise ValueError("Non-finite gripper closure")
    opening = GRIPPER_WIDTH * (1.0 - np.clip(closure, 0., 1.))
    return np.array([-opening[0], opening[0], -opening[1], opening[1]])


def joints_to_closure(joints: np.ndarray) -> np.ndarray:
    """Measured state: average the two finger openings for each gripper.

Use measured joint positions here, not the command; contact may prevent a
gripper from reaching its requested closure. Tiny limit/physics errors clip.
"""
    fingers = np.asarray(joints).reshape(2, 2)
    return np.clip(1.0 - (fingers[:, 1] - fingers[:, 0]) / (2 * GRIPPER_WIDTH), 0., 1.)
