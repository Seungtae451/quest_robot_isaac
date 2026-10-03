"""Physical hand-axis matching with differing controller/robot orientations."""
import numpy as np
import pinocchio as pin
import pytest

from teleop.xr_pose import RelativePoseMapper


@pytest.mark.parametrize("side,axes", [
    ("left", [(0., -1., 0.), (0., 0., -1.), (1., 0., 0.)]),
    ("right", [(0., -1., 0.), (0., 0., 1.), (-1., 0., 0.)])])
@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("sign", [-1, 1])
def test_twist_pitch_yaw_follow_physical_hand_axes(side, axes, axis, sign):
    start = np.eye(4)
    start[:3, :3] = pin.exp3(np.array([.4, -.3, .2]))
    anchor = pin.SE3(pin.exp3(np.array([-.8, .5, -1.])), np.array([.3, .2, .5]))
    mapper = RelativePoseMapper(start, anchor, use_filter=False, side=side)
    np.testing.assert_allclose(mapper.target(start).homogeneous, anchor.homogeneous, atol=1e-12)
    angle = sign * np.deg2rad(15)
    current = start.copy()
    current[:3, :3] = start[:3, :3] @ pin.exp3(np.eye(3)[axis] * angle)
    target = mapper.target(current)
    # Twist follows finger extension (-Y link), bends follow mirrored left/up.
    actual = pin.log3(anchor.rotation.T @ target.rotation)
    np.testing.assert_allclose(actual, np.array(axes[axis]) * angle, atol=1e-12)
    np.testing.assert_allclose(target.translation, anchor.translation, atol=1e-12)


@pytest.mark.parametrize("side", ["left", "right"])
def test_combined_rotation_and_filter_preserve_hand_motion(side):
    start = np.eye(4)
    start[:3, :3] = pin.exp3(np.array([.3, .1, -.6]))
    anchor = pin.SE3(pin.exp3(np.array([-.3, .7, .8])), np.zeros(3))
    mapper = RelativePoseMapper(start, anchor, side=side)
    current = start.copy()
    delta = pin.exp3(np.array([.15, 0., 0.])) @ pin.exp3(np.array([0., -.12, .08]))
    current[:3, :3] = start[:3, :3] @ delta
    for _ in range(120):
        target = mapper.target(current)
    recovered = mapper.basis.T @ anchor.rotation.T @ target.rotation @ mapper.basis
    np.testing.assert_allclose(recovered, delta, atol=1e-10)
