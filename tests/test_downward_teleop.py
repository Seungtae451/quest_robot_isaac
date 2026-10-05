"""XYZ-only commands still solve the full pose of physically downward grippers."""
import numpy as np
import pinocchio as pin
import pytest

from config import teleop_config as cfg
from robot.f14_config import (
    F14_URDF_PATH, GRIPPER_FORWARD_AXIS, LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT,
)
from robot.f14_ik import F14IK
from teleop.xr_pose import DownwardPoseMapper, samples_are_still


@pytest.mark.parametrize("side,rotation", [("left", LEFT_EE_DOWN_ROT), ("right", RIGHT_EE_DOWN_ROT)])
def test_controller_rotation_is_ignored_even_at_rotated_calibration(side, rotation):
    start = np.eye(4)
    start[:3, :3] = pin.exp3(np.array([.3, -.4, .6]))
    start[:3, 3] = [.2, -.1, .8]
    anchor = pin.SE3(pin.exp3(np.array([-.6, .1, .3])), np.array([.3, .13, .47]))
    mapper = DownwardPoseMapper(start, anchor, use_filter=False, side=side)
    for angles in ([0, 0, 0], [1.2, 0, 0], [0, -1.1, 0], [0, 0, 1.5], [-.7, .8, -1.]):
        current = start.copy()
        current[:3, :3] = start[:3, :3] @ pin.exp3(np.array(angles))
        target = mapper.target(current)
        np.testing.assert_array_equal(target.rotation, rotation)
        np.testing.assert_array_equal(target.translation, anchor.translation)
        current[:3, 3] += [.05, -.03, .02]
        target = mapper.target(current)
        np.testing.assert_allclose(target.translation, anchor.translation + [.05, -.03, .02])
        np.testing.assert_allclose(target.rotation @ GRIPPER_FORWARD_AXIS, [0, 0, -1])
    # Creating or using the mapper must not overwrite measured FK/calibration.
    np.testing.assert_allclose(anchor.rotation, pin.exp3(np.array([-.6, .1, .3])))
    np.testing.assert_allclose(start[:3, 3], [.2, -.1, .8])


def test_xyz_calibration_accepts_rotation_but_rejects_translation():
    start, rotated = np.eye(4), np.eye(4)
    rotated[:3, :3] = pin.exp3(np.array([.8, -.2, .4]))
    assert samples_are_still([start, rotated], check_rotation=False)
    assert not samples_are_still([start, rotated])
    rotated[0, 3] = .04
    assert not samples_are_still([start, rotated], check_rotation=False)


def test_downward_position_filter_is_independent_of_input_rotation_and_rate():
    for side in ("left", "right"):
        mappers = [DownwardPoseMapper(np.eye(4), pin.SE3.Identity(), side=side) for _ in range(2)]
        poses = [np.eye(4), np.eye(4)]
        for pose in poses:
            pose[:3, 3] = [.08, -.06, .003]  # Z stays within the 5 mm deadband.
        poses[1][:3, :3] = pin.exp3(np.array([.9, .7, -.4]))
        for i, hz in enumerate((30, 60)):
            for _ in range(hz):
                targets = mappers[i].target(poses[i], dt=1 / hz)
            np.testing.assert_allclose(targets.translation[2], 0.)
        np.testing.assert_allclose(mappers[0].delta, mappers[1].delta, atol=1e-12)


def test_legacy_downward_pose_and_xyz_ik_keep_the_physical_grasp_axis_down(downward_home_q):
    ik = F14IK(F14_URDF_PATH)
    home = ik.forward_kinematics(downward_home_q)
    for pose, rotation, position in zip(home, (LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT),
            ([.4, .18, .57], [.4, -.18, .57])):
        np.testing.assert_allclose(pose.translation, position, atol=1e-8)
        np.testing.assert_allclose(pose.rotation, rotation, atol=1e-8)
        np.testing.assert_allclose(pose.rotation @ GRIPPER_FORWARD_AXIS, [0, 0, -1], atol=1e-8)
    q = downward_home_q.copy()
    for left_delta, right_delta in (
        ([.03, 0, 0], [-.02, 0, 0]),
        ([.06, .03, -.05], [.05, -.025, -.04]),
        ([0, 0, -.06], [0, 0, -.06]),
        ([0, 0, 0], [0, 0, 0]),
    ):
        targets = [pin.SE3(rotation, anchor.translation + delta)
            for rotation, anchor, delta in zip((LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT), home, (left_delta, right_delta))]
        q, ok = ik.solve(*targets, q, max_iter=cfg.IK_MAX_ITER, eps=cfg.IK_EPS,
                        dt=cfg.IK_DT, damping=cfg.IK_DAMPING)
        assert ok, ik.last_diagnostics
        for actual, target in zip(ik.forward_kinematics(q), targets):
            assert np.linalg.norm(pin.log6(actual.inverse() * target).vector) < cfg.IK_EPS
            np.testing.assert_allclose(actual.rotation @ GRIPPER_FORWARD_AXIS, [0, 0, -1], atol=cfg.IK_EPS)
