"""Retargeting, failed-IK hold/recovery and stalled drives must not jump."""
from contextlib import closing
from dataclasses import replace
import time

import numpy as np
import pytest

from config import teleop_config as cfg
from robot.f14_config import HOME_ACTION
from robot.joint_motion import JointMotionLimiter
from teleop.action_protocol import ActionReceiver, ActionSender, pack_action, unpack_action

DT = 1 / 60


def limiter():
    return JointMotionLimiter(np.zeros(14), cfg.ARM_MAX_VELOCITY,
                              cfg.ARM_MAX_ACCELERATION, cfg.ARM_MAX_TRACKING_ERROR)


def checked_step(motion, goal, measured=None, enabled=True):
    q, v = motion.position.copy(), motion.velocity.copy()
    result = motion.step(goal, q if measured is None else measured, DT, enabled)
    assert np.max(np.abs(result - q)) <= cfg.ARM_MAX_VELOCITY * DT + 1e-12
    assert np.max(np.abs(motion.velocity - v)) <= cfg.ARM_MAX_ACCELERATION * DT + 1e-12
    return result


def test_large_ik_jump_is_reached_gradually_with_bounded_velocity_and_acceleration():
    motion = limiter()
    goal = np.linspace(-1., 1., 14)
    first = checked_step(motion, goal)
    assert np.max(np.abs(first)) < .001
    for _ in range(600):
        checked_step(motion, goal)
    np.testing.assert_allclose(motion.position, goal, atol=1e-4)


def test_failed_ik_brakes_and_recovery_starts_at_stopped_position():
    motion = limiter()
    for _ in range(60):
        checked_step(motion, np.ones(14))
    previous = motion.position.copy()
    for _ in range(120):
        checked_step(motion, np.ones(14), enabled=False)
    stopped = motion.position.copy()
    assert np.max(stopped - previous) <= cfg.ARM_MAX_VELOCITY**2 / (2 * cfg.ARM_MAX_ACCELERATION) + 1e-6
    assert np.max(stopped) < .5  # did not continue to the stale 1-rad IK goal
    for _ in range(60):
        checked_step(motion, -np.ones(14), enabled=False)
    np.testing.assert_array_equal(motion.position, stopped)
    recovered = checked_step(motion, -np.ones(14))
    assert np.max(np.abs(recovered - stopped)) < .001
    for _ in range(720):
        checked_step(motion, -np.ones(14))
    np.testing.assert_allclose(motion.position, -np.ones(14), atol=1e-4)


def test_target_reversal_preserves_reference_and_decelerates_before_reversing():
    motion = limiter()
    for _ in range(60):
        checked_step(motion, np.ones(14))
    q = motion.position.copy()
    next_q = checked_step(motion, -np.ones(14))
    assert np.all(next_q > q)  # momentum decelerates; no snap to opposite target
    for _ in range(600):
        checked_step(motion, -np.ones(14))
    np.testing.assert_allclose(motion.position, -np.ones(14), atol=1e-4)


def test_blocked_measured_joint_does_not_build_up_a_distant_drive_target():
    motion = limiter()
    for _ in range(600):
        checked_step(motion, np.ones(14), measured=np.zeros(14))
    assert np.max(motion.position) < cfg.ARM_MAX_TRACKING_ERROR + .001 + cfg.ARM_MAX_VELOCITY**2 / (2 * cfg.ARM_MAX_ACCELERATION)
    np.testing.assert_allclose(motion.velocity, 0., atol=1e-12)
    assert motion.tracking_limited.all()


def test_hold_protocol_and_feedback_report_current_measured_state():
    goal = HOME_ACTION.copy()
    goal[0] += .3
    packet = unpack_action(pack_action(goal, 1, hold_arms=True))
    assert packet.hold_arms
    with closing(ActionReceiver("127.0.0.1", 0)) as receiver:
        with closing(ActionSender("127.0.0.1", receiver.socket.getsockname()[1])) as sender:
            sender.send(goal)
            receiver.poll()
            assert receiver.arms_enabled
            sender.send(goal, hold_arms=True)
            receiver.poll()
            assert not receiver.arms_enabled
            measured = HOME_ACTION.copy()
            measured[0] += .02
            receiver.update_feedback(measured)
            sender.poll_feedback(); receiver.poll(); sender.poll_feedback()
            np.testing.assert_allclose(sender.measured_state, measured, atol=1e-7)
            # Goal stays far away, but feedback must never present it as the
            # current pose used for IK warm starts/recalibration.
            np.testing.assert_allclose(receiver.action, goal, atol=1e-7)
            sender.send(goal)
            receiver.poll()
            assert receiver.arms_enabled
            receiver.latest = replace(receiver.latest, timestamp=time.monotonic() - cfg.COMMAND_TIMEOUT - .1)
            assert not receiver.arms_enabled


def test_bad_motion_input_is_rejected():
    motion = limiter()
    with pytest.raises(ValueError):
        motion.step(np.ones(14), np.zeros(14), float("nan"))
    with pytest.raises(ValueError):
        motion.step(np.full(14, np.nan), np.zeros(14), DT)
