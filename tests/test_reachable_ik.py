"""Feasible projection, downward pose validation and rejection of failed outputs."""
import numpy as np
import pinocchio as pin

from config import teleop_config as cfg
from robot.f14_config import F14_URDF_PATH, HOME_Q, LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT
from robot.f14_ik import F14IK
from robot.reachable_ik import ReachableIK

OPTIONS = dict(max_iter=cfg.IK_MAX_ITER, eps=cfg.IK_EPS, dt=cfg.IK_DT, damping=cfg.IK_DAMPING)


def make_targets(ik, delta):
    home = ik.forward_kinematics(HOME_Q)
    return [pin.SE3(rot, pose.translation + delta)
            for rot, pose in zip((LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT), home)]


def test_far_unreachable_request_follows_only_verified_nearby_steps_and_can_retreat():
    ik = F14IK(F14_URDF_PATH)
    follow = ReachableIK(ik, budget_ms=1000.)
    targets = make_targets(ik, np.array([1., 0., 0.]))
    _, ok = ik.solve(*targets, HOME_Q, **OPTIONS)
    assert not ok
    q = HOME_Q.copy(); successful = 0
    for _ in range(25):
        old = ik.forward_kinematics(q)
        result = follow.solve(*targets, q, **OPTIONS)
        if not result.success:
            np.testing.assert_array_equal(result.q, q)
            break
        successful += 1
        assert np.max(np.abs(result.q-q)) <= follow.max_joint_step
        for actual, previous, rotation in zip(ik.forward_kinematics(result.q), old, (LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT)):
            assert np.linalg.norm(actual.translation-previous.translation) <= follow.max_step + cfg.IK_EPS
            assert np.linalg.norm(pin.log3(actual.rotation.T @ rotation)) < cfg.IK_EPS
        q = result.q
    assert successful >= 2
    assert np.max(np.abs(q-HOME_Q)) > .05
    result = follow.solve(*make_targets(ik, np.zeros(3)), q, **OPTIONS)
    assert result.success  # feasible inward direction resumes without recentering


def test_failed_partial_candidate_is_never_sent_and_shrinking_keeps_rotation(monkeypatch):
    ik = F14IK(F14_URDF_PATH); real_solve = ik.solve
    requested = make_targets(ik, np.array([.03, 0., 0.]))
    calls = []
    def first_fails(left, right, q, **kwargs):
        calls.append((left.copy(), right.copy(), q.copy()))
        if len(calls) == 1:
            return np.full(14, 999.), False
        return real_solve(left, right, q, **kwargs)
    monkeypatch.setattr(ik, 'solve', first_fails)
    result = ReachableIK(ik, budget_ms=1000.).solve(*requested, HOME_Q, **OPTIONS)
    assert result.success and result.mode == 'PROJECTED' and result.attempts == 2
    assert result.fraction == .5
    for pose in ik.forward_kinematics(result.q):
        assert np.isfinite(pose.translation).all()
    np.testing.assert_array_equal(calls[1][2], HOME_Q)  # do not seed from failed output
    np.testing.assert_allclose(calls[1][0].translation, calls[0][0].translation-[.015,0,0])
    np.testing.assert_array_equal(calls[1][0].rotation, LEFT_EE_DOWN_ROT)


def test_no_feasible_step_holds_measured_position_and_has_bounded_attempts(monkeypatch):
    ik = F14IK(F14_URDF_PATH)
    monkeypatch.setattr(ik, 'solve', lambda *a, **k: (np.full(14, np.nan), False))
    follow = ReachableIK(ik, budget_ms=1000.)
    result = follow.solve(*make_targets(ik, np.ones(3)), HOME_Q, **OPTIONS)
    assert not result.success and result.mode == 'HOLD'
    assert result.attempts == follow.retries+1
    np.testing.assert_array_equal(result.q, HOME_Q)


def test_budget_stops_additional_search_without_using_failed_output(monkeypatch):
    ik = F14IK(F14_URDF_PATH); calls = []
    def fails(*args, **kwargs):
        calls.append(1)
        return HOME_Q+1, False
    monkeypatch.setattr(ik, 'solve', fails)
    result = ReachableIK(ik, budget_ms=0.).solve(*make_targets(ik, np.ones(3)), HOME_Q, **OPTIONS)
    assert len(calls) == 1 and not result.success
    np.testing.assert_array_equal(result.q, HOME_Q)


def test_limit_aware_ik_finds_downward_pose_that_old_limit_clipping_misses():
    ik = F14IK(F14_URDF_PATH)
    targets = make_targets(ik, np.array([-.07434536875244477, .1602743635840903, .007675769839950795]))
    _, old_ok = ik.solve(*targets, HOME_Q, **OPTIONS)
    assert not old_ok
    q, ok = ik.solve(*targets, HOME_Q, **OPTIONS,
                     limit_margin=cfg.IK_LIMIT_MARGIN, limit_gain=cfg.IK_LIMIT_GAIN)
    assert ok
    assert np.all(q >= ik.model.lowerPositionLimit[ik.arm_q_indices])
    assert np.all(q <= ik.model.upperPositionLimit[ik.arm_q_indices])
    for actual, target in zip(ik.forward_kinematics(q), targets):
        assert np.linalg.norm(pin.log6(actual.inverse()*target).vector) < cfg.IK_EPS
    # The teleop follower still bounds the next joint destination; a distant
    # alternative solution must not bypass branch continuity or drive limits.
    result=ReachableIK(ik,budget_ms=1000.).solve(*targets,HOME_Q,**OPTIONS,
                     limit_margin=cfg.IK_LIMIT_MARGIN,limit_gain=cfg.IK_LIMIT_GAIN)
    assert result.success
    assert np.max(np.abs(result.q-HOME_Q)) <= cfg.IK_FOLLOW_MAX_JOINT_STEP
