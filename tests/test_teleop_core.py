"""Regression checks for mapping, IK, safety, and latest-frame transports.

Run with env_isaaclab Python. No Isaac application or physical Quest required.
Live WSS and real GPU cameras have separate bounded smoke-test entry points.
"""
from contextlib import closing
import struct
import time

import numpy as np
import pinocchio as pin
import pytest

from robot.f14_config import HOME_Q, HOME_ACTION, F14_URDF_PATH, PROJECT_ROOT, USD_ARM_SIGNS
from robot.f14_ik import F14IK
from robot.gripper import closure_to_joints, joints_to_closure, normalize_input
from teleop.action_protocol import ActionReceiver, ActionSender, pack_action, unpack_action, PACKET_SIZE
from teleop.xr_pose import RelativePoseMapper, average_pose, samples_are_still, valid_pose
from simulation.quest_view_compositor import compose_quest_view


@pytest.mark.parametrize("distance", [.1, .2, .5, 1.])
def test_unit_translation_scale_without_clamp(distance):
    start = np.eye(4)
    mapper = RelativePoseMapper(start, pin.SE3.Identity(), use_filter=False)
    current = start.copy()
    current[0, 3] = distance
    np.testing.assert_allclose(mapper.target(current).translation, [distance, 0, 0])


@pytest.mark.parametrize("degrees", [30., 90., 170.])
def test_relative_orientation_at_nonidentity_neutral(degrees):
    start = np.eye(4)
    start[:3, :3] = pin.exp3(np.array([.3, -.1, .2]))
    anchor = pin.SE3(pin.exp3(np.array([.1, .2, -.3])), np.ones(3))
    mapper = RelativePoseMapper(start, anchor, use_filter=False)
    np.testing.assert_allclose(mapper.target(start).homogeneous, anchor.homogeneous, atol=1e-12)
    current = start.copy()
    rotation = pin.exp3(np.array([0., np.deg2rad(degrees), 0.]))
    current[:3, :3] = start[:3, :3] @ rotation
    np.testing.assert_allclose(mapper.target(current).rotation, anchor.rotation @ rotation, atol=1e-10)


def test_filter_steady_state_and_rotation_mean():
    start = np.eye(4)
    mapper = RelativePoseMapper(start, pin.SE3.Identity())
    current = start.copy()
    current[0, 3] = .2
    current[:3, :3] = pin.exp3(np.array([.4, -.2, .1]))
    for _ in range(120):
        target = mapper.target(current)
    np.testing.assert_allclose(target.homogeneous, current, atol=1e-10)
    plus, minus = np.eye(4), np.eye(4)
    plus[:3, :3] = pin.exp3(np.array([0., 0., np.deg2rad(179)]))
    minus[:3, :3] = pin.exp3(np.array([0., 0., np.deg2rad(-179)]))
    mean = average_pose([plus, minus])
    assert valid_pose(mean)
    assert abs(np.linalg.norm(pin.log3(mean[:3, :3])) - np.pi) < 1e-6
    assert not samples_are_still([start, current])


def test_gripper_encoding_independence_and_inverse():
    # Inverted legacy values below 1.5 must NOT switch to standard encoding.
    for raw, expected in [(10., 0.), (5., .5), (1., .9), (0., 1.)]:
        assert normalize_input(raw, "legacy-inverted-10") == pytest.approx(expected)
    assert normalize_input(.25) == .25
    np.testing.assert_allclose(closure_to_joints(0., 1.), [-.0425, .0425, 0, 0])
    np.testing.assert_allclose(joints_to_closure(closure_to_joints(.25, .75)), [.25, .75])


def test_ik_matches_home_and_rejects_unreachable():
    ik = F14IK(F14_URDF_PATH)
    # Keep the known FK reference independent of user-editable HOME. Right
    # dof3/dof4 signs below are expressed in the rev 2.0.1 URDF convention.
    reference = np.array([-.57, .40, .22, -.95, .24, -.79, -.27,
                          -.57, -.40, -.22, -.95, -.24, .79, -.27])
    left, right = ik.forward_kinematics(reference)
    np.testing.assert_allclose(left.translation, [.42102236, .26728693, .53370183], atol=1e-8)
    np.testing.assert_allclose(right.translation, [.42102386, -.26728430, .53370183], atol=1e-8)
    left, right = ik.forward_kinematics(HOME_Q)
    left.translation[0] += .05
    right.translation[0] += .05
    q, ok = ik.solve(left, right, HOME_Q, max_iter=30, eps=2e-4, dt=.3, damping=1e-4)
    assert ok
    for actual, target in zip(ik.forward_kinematics(q), (left, right)):
        assert np.linalg.norm(pin.log6(actual.inverse() * target).vector) < 2e-4
    left.translation[0] += 10.
    original = q.copy()
    _, ok = ik.solve(left, right, q, max_iter=30)
    assert not ok
    assert ik.last_diagnostics["iterations"] == 30
    assert ik.last_diagnostics["position_error_mm"][0] > 1000
    np.testing.assert_array_equal(q, original)


def test_rev201_angles_match_existing_usd_kinematics():
    current = F14IK(F14_URDF_PATH)
    original = F14IK(PROJECT_ROOT / "assets/f14/F14_URDF_rev_2_0_0/urdf/FlaminGO_14Dof_Arm_Robot_v2_mujoco.urdf")
    # The axis conversion must preserve every link, including camera mounts
    # and intermediate arm links, both at HOME and after movement.
    for q in (HOME_Q, HOME_Q + np.tile([.02, -.01, .03, .02, -.01, .02, .01], 2)):
        full_new = current.arm_to_full_q(q)
        full_old = original.arm_to_full_q(q * USD_ARM_SIGNS)
        pin.forwardKinematics(current.model, current.data, full_new)
        pin.forwardKinematics(original.model, original.data, full_old)
        pin.updateFramePlacements(current.model, current.data)
        pin.updateFramePlacements(original.model, original.data)
        for frame in current.model.frames:
            new_id = current.model.getFrameId(frame.name, frame.type)
            old_id = original.model.getFrameId(frame.name, frame.type)
            np.testing.assert_allclose(current.data.oMf[new_id].homogeneous,
                                       original.data.oMf[old_id].homogeneous, atol=1e-12)
    old_limits = np.column_stack((original.model.lowerPositionLimit[original.arm_q_indices],
                                 original.model.upperPositionLimit[original.arm_q_indices]))
    new_limits = np.sort(old_limits * USD_ARM_SIGNS[:, None], axis=1)
    np.testing.assert_allclose(new_limits[:, 0], current.model.lowerPositionLimit[current.arm_q_indices])
    np.testing.assert_allclose(new_limits[:, 1], current.model.upperPositionLimit[current.arm_q_indices])


def test_protocol_rejects_bad_size_magic_and_nonfinite():
    assert PACKET_SIZE == 80
    packet = pack_action(HOME_ACTION, 42)
    decoded = unpack_action(packet)
    assert decoded.sequence == 42
    np.testing.assert_allclose(decoded.action, HOME_ACTION)
    for bad in (packet[:-1], packet + b"x", b"F14Q" + packet[4:], packet[:-4] + struct.pack("!f", float("nan"))):
        with pytest.raises(ValueError):
            unpack_action(bad)
    action = HOME_ACTION.copy()
    action[-1] = 1.1
    with pytest.raises(ValueError):
        pack_action(action, 1)


def test_udp_freshness_reordering_timeout_and_sender_restart():
    with closing(ActionReceiver("127.0.0.1", 0)) as receiver:
        with closing(ActionSender("127.0.0.1", receiver.socket.getsockname()[1])) as sender:
            assert receiver.state == "WAITING"
            sender.poll_feedback(); receiver.poll()
            assert sender.poll_feedback() and sender.ready
            now = time.monotonic()
            newer = HOME_ACTION.copy(); newer[14:] = [.25, .75]
            for payload in [pack_action(newer, 20, now), pack_action(HOME_ACTION, 19, now - .01),
                            pack_action(HOME_ACTION, 21, now - 10), pack_action(HOME_ACTION, 22, now + 10), b"bad"]:
                sender.socket.sendto(payload, sender.destination)
            assert receiver.poll()
            np.testing.assert_allclose(receiver.action, newer)
            assert receiver.rejected == 4 and receiver.state == "ACTIVE"
            # New sender sequence restarts at 1, monotonic timestamp still rises.
            sender.send(newer)
            receiver.poll()
            assert receiver.latest.sequence == 1
            time.sleep(.52)
            assert receiver.state == "HOLD"
            np.testing.assert_allclose(receiver.action, newer)


def test_compositor_keeps_raw_rgb_unchanged():
    images = {"front": np.full((480, 640, 3), (0, 255, 0), np.uint8),
              "left_wrist": np.full((240, 320, 3), (255, 0, 0), np.uint8),
              "right_wrist": np.full((240, 320, 3), (0, 0, 255), np.uint8)}
    before = {k: v.copy() for k, v in images.items()}
    composite = compose_quest_view(images, "test")
    assert composite.shape == (720, 1280, 3)
    np.testing.assert_array_equal(composite[360, 50], [255, 0, 0])
    np.testing.assert_array_equal(composite[360, 640], [0, 255, 0])
    np.testing.assert_array_equal(composite[360, 1200], [0, 0, 255])
    for key in images:
        np.testing.assert_array_equal(images[key], before[key])


def test_zmq_video_roundtrip_latest_and_color(tmp_path):
    from teleop.xr_video import VideoPublisher, VideoSubscriber
    endpoint = f"ipc://{tmp_path}/video.sock"
    with closing(VideoPublisher(endpoint)) as publisher, closing(VideoSubscriber(endpoint)) as sub:
        image = np.full((720, 1280, 3), (235, 20, 10), np.uint8)
        until = time.monotonic() + 3.
        received = None
        while time.monotonic() < until and received is None:
            publisher.submit(image, time.monotonic())
            time.sleep(.03)
            received = sub.receive()
        assert received is not None
        np.testing.assert_allclose(received[350, 600], [10, 20, 235], atol=4)
        assert sub.age < 1.
    assert not publisher.thread.is_alive()
