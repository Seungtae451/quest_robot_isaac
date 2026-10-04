"""Real F14 Jacobian/QP regression, independent of the Isaac application."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pinocchio as pin
import proxsuite

from config import teleop_config as cfg
from robot.differential_ik import DifferentialIK
from robot.f14_config import F14_URDF_PATH, HOME_Q, LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT
from robot.f14_ik import F14IK
from robot.joint_motion import JointMotionLimiter
from teleop.action_protocol import pack_action, unpack_action, PACKET_SIZE


def setup():
    ik = F14IK(F14_URDF_PATH)
    return ik, DifferentialIK(ik), list(ik.forward_kinematics(HOME_Q))


def test_convergence_speed_acceleration_limits_and_fixed_full_rotation():
    ik, controller, targets = setup()
    targets[0].translation += [.02, .01, .015]
    targets[1].translation += [-.015, -.01, .015]
    q = HOME_Q.copy(); previous = np.zeros(14); dt = 1 / 60
    for _ in range(600):
        result = controller.solve(*targets, q, dt=dt)
        assert result.diagnostics['arm_success'] == [True, True]
        velocity = (result.q - q) / dt
        assert np.max(np.abs(velocity)) <= cfg.ARM_MAX_VELOCITY + 2e-7
        assert np.max(np.abs(velocity - previous)) <= cfg.ARM_MAX_ACCELERATION * dt + 2e-7
        assert np.all(result.q >= controller.lower) and np.all(result.q <= controller.upper)
        for actual, target in zip(ik.forward_kinematics(result.q), targets):
            assert np.linalg.norm(pin.log3(actual.rotation.T @ target.rotation)) <= cfg.IK_EPS + 2e-6
        q, previous = result.q, velocity
    assert max(result.remaining_mm) < .2  # measured Cartesian accuracy remains 0.2 mm


def test_unreachable_arm_does_not_stop_other_arm_and_can_retreat():
    ik, controller, home = setup(); targets = [p.copy() for p in home]
    targets[0].translation += [1., 0., .3]
    targets[1].translation += [0., -.015, .02]
    q = HOME_Q.copy()
    for _ in range(900):
        result = controller.solve(*targets, q, dt=1/60); q = result.q
        assert np.isfinite(q).all() and np.all(q >= controller.lower) and np.all(q <= controller.upper)
    right = ik.forward_kinematics(q)[1]
    assert np.linalg.norm(right.translation - targets[1].translation) < cfg.IK_EPS
    assert np.linalg.norm(ik.forward_kinematics(q)[0].translation - targets[0].translation) > .5
    distance = np.linalg.norm(q[:7] - HOME_Q[:7])
    for _ in range(900):
        result = controller.solve(*home, q, dt=1/60); q = result.q
    assert np.linalg.norm(q[:7] - HOME_Q[:7]) < distance


def test_infeasible_output_is_rejected_for_only_one_arm():
    _, controller, targets = setup()
    targets[1].translation += [.01, 0., .01]
    failed = SimpleNamespace(init=lambda *a, **k: None, update=lambda *a, **k: None,
        settings=SimpleNamespace(initial_guess=None),
        solve=lambda: None, results=SimpleNamespace(x=np.full(7, 999.),
            info=SimpleNamespace(status=proxsuite.proxqp.QPSolverOutput.PROXQP_PRIMAL_INFEASIBLE, iter=1)))
    controller.solvers[0] = failed
    result = controller.solve(*targets, HOME_Q, dt=1/60)
    assert result.success and result.mode == 'QP_PARTIAL'
    np.testing.assert_array_equal(result.q[:7], HOME_Q[:7])
    assert np.max(np.abs(result.q[7:] - HOME_Q[7:])) > 0


def test_blocked_measured_drive_is_bounded_and_retreat_is_available():
    ik, controller, home = setup(); targets = [p.copy() for p in home]
    targets[0].translation += [.06, 0., .03]
    for _ in range(600):
        result = controller.solve(*targets, HOME_Q, dt=1/60)
    assert np.max(np.abs(result.q - HOME_Q)) <= cfg.ARM_MAX_TRACKING_ERROR + .001
    old_lag = np.linalg.norm(result.q - HOME_Q)
    for _ in range(300):
        result = controller.solve(*home, HOME_Q, dt=1/60)
    assert np.linalg.norm(result.q - HOME_Q) < old_lag / 2


def test_cold_retry_does_not_integrate_nonconverged_warm_result():
    _, controller, targets = setup()
    targets[0].translation += [.01, 0., .01]

    class FirstFails:
        def __init__(self, real):
            self.real = real
            self.settings = real.settings
            self.calls = 0

        def init(self, *args, **kwargs):
            return self.real.init(*args, **kwargs)

        def solve(self):
            self.calls += 1
            if self.calls > 1:
                self.real.solve()

        @property
        def results(self):
            if self.calls == 1:
                return SimpleNamespace(x=np.full(7, 999.), info=SimpleNamespace(
                    status=proxsuite.proxqp.QPSolverOutput.PROXQP_MAX_ITER_REACHED, iter=100))
            return self.real.results

    first = FirstFails(controller.solvers[0])
    controller.solvers[0] = first
    result = controller.solve(*targets, HOME_Q, dt=1/60)
    assert result.success and first.calls == 2 and result.attempts == 3
    assert result.diagnostics['arm_success'] == [True, True]
    assert np.max(np.abs(result.q - HOME_Q)) <= cfg.ARM_MAX_ACCELERATION / 60**2 + 1e-9


def test_open_home_overlap_can_lift_and_table_descent_is_limited():
    ik, controller, targets = setup()
    feedback = json.loads((Path(__file__).parent / 'fixtures/open_home_collision_feedback.json').read_text())
    for pose in targets:
        pose.translation[2] += .04
    q = HOME_Q.copy()
    for _ in range(600):
        result = controller.solve(*targets, q, dt=1/60, feedback=feedback); q = result.q
    for actual, target in zip(ik.forward_kinematics(q), targets):
        assert np.linalg.norm(actual.translation - target.translation) < cfg.IK_EPS
    for pose in targets:
        pose.translation[2] -= .14
    for _ in range(900):
        result = controller.solve(*targets, q, dt=1/60, feedback=feedback); q = result.q
    for side, pose in enumerate(ik.forward_kinematics(q)):
        assert pose.translation[2] + feedback['arms'][side]['offset_bounds'][0][2] > feedback['surface_z']


def test_differential_packet_and_receiver_reference_are_bounded_on_timeout():
    action = np.r_[HOME_Q, 1., 1.]
    packed = pack_action(action, 1, direct_reference=True)
    assert len(packed) == PACKET_SIZE and packed[:4] == b'F16D'
    assert unpack_action(packed).direct_reference and not unpack_action(packed).hold_arms
    assert not unpack_action(pack_action(action, 2, hold_arms=True, direct_reference=True)).direct_reference
    motion = JointMotionLimiter(HOME_Q, .25, .5, .05, allow_tracking_retreat=True)
    position = HOME_Q.copy(); dt = 1/60
    # Feed a bounded moving reference, then simulate a packet timeout.
    for step in range(120):
        goal = HOME_Q + min(.1, step * dt * .1)
        old_velocity = motion.velocity.copy()
        result = motion.step(goal, position, dt, reference_mode=True)
        assert np.max(np.abs(motion.velocity)) <= .25
        assert np.max(np.abs(motion.velocity - old_velocity)) <= .5 * dt + 1e-12
        position = result
    assert np.max(np.abs(position - goal)) < .002
    for _ in range(60):
        position = motion.step(goal, position, dt, enabled=False, reference_mode=True)
    np.testing.assert_array_equal(motion.velocity, np.zeros(14))


def test_measured_gpu_home_recovers_rotation_with_changing_collision_envelopes():
    ik, controller, _ = setup()
    # Real PhysX HOME differs slightly from the nominal URDF HOME. This exposed
    # slow QP convergence with redundant collision planes and fixed rotation.
    q = np.array([-1.0719156265,.7261884212,1.5506101847,-1.4016547203,1.5895093679,-.6753546000,-1.0974546671,
                  -1.0719118118,-.7262005806,-1.5506044626,-1.4016593695,-1.5895102024,.6753527522,-1.0974551439])
    initial = list(ik.forward_kinematics(q))
    targets = [pin.SE3(rot, pose.translation + [0.,0.,.04])
               for rot, pose in zip((LEFT_EE_DOWN_ROT, RIGHT_EE_DOWN_ROT), initial)]
    feedback = json.loads((Path(__file__).parent / 'fixtures/open_home_collision_feedback.json').read_text())
    original = [np.array(arm['part_bounds']) for arm in feedback['arms']]
    failed = 0
    for step in range(400):
        for arm, parts in zip(feedback['arms'], original):
            arm['part_bounds'] = (parts + 1e-7 * np.sin(step)).tolist()
        result = controller.solve(*targets, q, dt=1/60, feedback=feedback)
        failed += not all(result.diagnostics['arm_success'])
        q = result.q
    assert failed < 40
    for pose, start, target in zip(ik.forward_kinematics(q), initial, targets):
        assert pose.translation[2] - start.translation[2] > .035
        assert np.linalg.norm(pin.log3(pose.rotation.T @ target.rotation)) < cfg.IK_EPS
