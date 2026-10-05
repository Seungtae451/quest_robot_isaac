"""Relative XYZ grasp-tip control, plus legacy wrist/rotation diagnostics.

No new axis swaps, translation clamps, or angle clamps occur here. Calibration
uses an SO(3) projection of the rotation sum; filtering follows the SO(3)
geodesic with log3/exp3 rather than averaging rotation matrix components.
"""
import numpy as np
import pinocchio as pin

from config import teleop_config as cfg
from robot.f14_config import LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT
from robot.tool_frame import controller_ee_basis
from teleop.input_timing import timed_filter_alpha
from types import SimpleNamespace


def front_facing_controls(data):
    """Reorder hands into ROBOT L/R; do not mirror any world coordinate axis.

    Physical A/B/X button handling stays in the adapter, before this mapping.
    """
    result=vars(data).copy()
    result['left_wrist_pose']=data.right_wrist_pose
    result['right_wrist_pose']=data.left_wrist_pose
    for key in ('triggerValue','squeezeValue'):
        result[f'left_ctrl_{key}']=getattr(data,f'right_ctrl_{key}')
        result[f'right_ctrl_{key}']=getattr(data,f'left_ctrl_{key}')
    return SimpleNamespace(**result)


def valid_pose(pose) -> bool:
    pose = np.asarray(pose)
    return (pose.shape == (4, 4) and np.isfinite(pose).all()
            and np.allclose(pose[3], [0, 0, 0, 1], atol=1e-3)
            and np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=0.03)
            and abs(np.linalg.det(pose[:3, :3]) - 1) < 0.03)


def project_so3(rotation):
    u, _, vt = np.linalg.svd(rotation)
    # Correct a possible reflection so det(R)=+1, not merely |det(R)|=1.
    return u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt


def average_pose(samples):
    if not samples or not all(valid_pose(p) for p in samples):
        raise ValueError("Calibration requires valid SE(3) samples")
    samples = np.asarray(samples)
    pose = np.eye(4)
    pose[:3, 3] = samples[:, :3, 3].mean(axis=0)
    pose[:3, :3] = project_so3(samples[:, :3, :3].sum(axis=0))
    return pose


def samples_are_still(samples, *, check_rotation=True) -> bool:
    mean = average_pose(samples)
    return all(np.linalg.norm(p[:3, 3] - mean[:3, 3]) <= cfg.CALIBRATION_POSITION_TOLERANCE
               and (not check_rotation or np.linalg.norm(pin.log3(mean[:3, :3].T @ p[:3, :3])) <= cfg.CALIBRATION_ROTATION_TOLERANCE)
               for p in samples)


class TranslationPoseMapper:
    """XYZ displacement only. Target rotation is unused by position-only IK."""
    def __init__(self, controller_start, robot_anchor, use_filter=True, *, side):
        if side not in ('left','right') or not valid_pose(controller_start):
            raise ValueError('Invalid arm or neutral controller pose')
        self.start=controller_start.copy()
        self.anchor=robot_anchor.copy()
        self.alpha_p=cfg.POSITION_FILTER_ALPHA if use_filter else 1.
        self.delta=np.zeros(3)
        self.raw_delta=np.zeros(3)

    def target(self,current,dt=None):
        if not valid_pose(current):
            raise ValueError('Invalid current controller pose')
        self.raw_delta=(current[:3,3]-self.start[:3,3])*np.asarray(cfg.TRANSLATION_AXIS_SIGNS)
        delta=np.where(np.abs(self.raw_delta)<cfg.POSITION_DEADBAND,0.,self.raw_delta)
        alpha=self.alpha_p if dt is None else timed_filter_alpha(self.alpha_p,dt,cfg.FILTER_REFERENCE_HZ)
        self.delta+=alpha*(delta-self.delta)
        target=self.anchor.copy()
        target.translation=self.anchor.translation+self.delta
        return target


class WorldPoseMapper(TranslationPoseMapper):
    """Follow world XYZ and world rotation deltas, without an A-press jump."""
    def __init__(self, controller_start, robot_anchor, use_filter=True, *, side):
        super().__init__(controller_start, robot_anchor, use_filter, side=side)
        self.alpha_r=cfg.ROTATION_FILTER_ALPHA if use_filter else 1.
        self.world_rotation=np.eye(3)
        self.controller_basis=controller_ee_basis(side)

    def target(self, current, dt=None):
        target=super().target(current, dt)
        mapped_current=current[:3,:3]@self.controller_basis
        mapped_start=self.start[:3,:3]@self.controller_basis
        relative=project_so3(mapped_current@mapped_start.T)
        alpha=self.alpha_r if dt is None else timed_filter_alpha(self.alpha_r,dt,cfg.FILTER_REFERENCE_HZ)
        self.world_rotation=project_so3(self.world_rotation@pin.exp3(alpha*pin.log3(self.world_rotation.T@relative)))
        target.rotation=self.world_rotation@self.anchor.rotation
        return target


class DownwardPoseMapper:
    """Controller translation only; EE orientation is fixed in robot/world axes.

    Calibration/recentering changes the position anchor, never the downward
    rotation. Controller orientation is validated as tracking data but is not
    used in the target or its filter. IK still solves the complete EE pose.
    """
    def __init__(self, controller_start, robot_anchor, use_filter=True, *, side):
        if side not in ("left", "right"):
            raise ValueError("Expected left or right arm")
        if not valid_pose(controller_start):
            raise ValueError("Invalid neutral controller pose")
        self.start = controller_start.copy()
        self.anchor = robot_anchor.copy()
        self.anchor.rotation = (LEFT_EE_DOWN_ROT if side == "left" else RIGHT_EE_DOWN_ROT).copy()
        self.alpha_p = cfg.POSITION_FILTER_ALPHA if use_filter else 1.
        self.delta = np.zeros(3)
        self.raw_delta = np.zeros(3)

    def target(self, current, dt=None):
        if not valid_pose(current):
            raise ValueError("Invalid current controller pose")
        self.raw_delta = current[:3, 3] - self.start[:3, 3]
        delta = np.where(np.abs(self.raw_delta) < cfg.POSITION_DEADBAND, 0., self.raw_delta)
        alpha = self.alpha_p if dt is None else timed_filter_alpha(self.alpha_p, dt, cfg.FILTER_REFERENCE_HZ)
        self.delta += alpha * (delta - self.delta)
        target = self.anchor.copy()
        target.translation = self.anchor.translation + self.delta
        return target


class RelativePoseMapper:
    """Legacy 6DoF mapper for offline wrist diagnostics, not live teleoperation.

    Recalibration anchors at the measured robot pose. Rotation stays wrist-local:
    controller twist/bend axes map to the mirrored F14 hand frames, not world axes.
    """
    def __init__(self, controller_start, robot_anchor, use_filter=True, *, side=None):
        self.start = controller_start.copy()
        self.anchor = robot_anchor.copy()
        self.alpha_p = cfg.POSITION_FILTER_ALPHA if use_filter else 1.
        self.alpha_r = cfg.ROTATION_FILTER_ALPHA if use_filter else 1.
        if side not in (None, "left", "right"):
            raise ValueError("Expected left or right arm")
        # Generic callers retain their configurable basis; teleoperation always
        # identifies its arm and uses the matching physical hand axes.
        basis = cfg.CONTROLLER_TO_EE_ROT if side is None else getattr(cfg, f"{side.upper()}_CONTROLLER_TO_EE_ROT")
        self.basis = np.asarray(basis, dtype=float)
        if (self.basis.shape != (3, 3) or not np.isfinite(self.basis).all()
                or not np.allclose(self.basis.T @ self.basis, np.eye(3))
                or not np.isclose(np.linalg.det(self.basis), 1.)):
            raise ValueError("Controller-to-EE basis must be a proper rotation")
        self.delta = np.zeros(3)
        self.raw_delta = np.zeros(3)
        self.relative_rotation = np.eye(3)

    def target(self, current, dt=None):
        if not valid_pose(current):
            raise ValueError("Invalid current controller pose")
        self.raw_delta = current[:3, 3] - self.start[:3, 3]
        # Preserve the validated 5 mm deadband around neutral. Outside it the
        # full displacement is used (no threshold subtraction, no max clamp).
        delta = np.where(np.abs(self.raw_delta) < cfg.POSITION_DEADBAND, 0., self.raw_delta)
        alpha_p = self.alpha_p if dt is None else timed_filter_alpha(self.alpha_p, dt, cfg.FILTER_REFERENCE_HZ)
        alpha_r = self.alpha_r if dt is None else timed_filter_alpha(self.alpha_r, dt, cfg.FILTER_REFERENCE_HZ)
        self.delta += alpha_p * (delta - self.delta)
        # Preserve relative LOCAL wrist rotations, then express their axes in
        # the physical gripper frame. B changes axes, never the rotation angle.
        relative = self.start[:3, :3].T @ current[:3, :3]
        relative = self.basis @ relative @ self.basis.T
        omega = pin.log3(self.relative_rotation.T @ project_so3(relative))
        self.relative_rotation = project_so3(self.relative_rotation @ pin.exp3(alpha_r * omega))
        target = self.anchor.copy()
        target.translation = self.anchor.translation + self.delta
        target.rotation = self.anchor.rotation @ self.relative_rotation
        return target

    @property
    def angle_degrees(self):
        return float(np.rad2deg(np.linalg.norm(pin.log3(self.relative_rotation))))
