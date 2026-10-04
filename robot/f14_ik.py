"""Pinocchio dual-arm LOCAL-Jacobian damped least-squares IK.

Quest SE(3) targets -> 14 arm angles; simulator ordering and finger actuation
are intentionally outside this solver. This retains the previously validated
log6 / LOCAL Jacobian / integrate algorithm and the same URDF. The original
implementation is preserved in f14_ik_legacy.py for numerical comparison.
"""
import numpy as np
import pinocchio as pin

# The fallback preserves old scripts that insert only robot/ into sys.path.
try:
    from robot.f14_config import LEFT_JOINT_NAMES, RIGHT_JOINT_NAMES
except ModuleNotFoundError:
    from f14_config import LEFT_JOINT_NAMES, RIGHT_JOINT_NAMES


class F14IK:
    def __init__(self, urdf_path):
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.left_ee_frame = self.model.getFrameId("left_dof7_link")
        self.right_ee_frame = self.model.getFrameId("right_dof7_link")
        for frame in (self.left_ee_frame, self.right_ee_frame):
            if frame >= len(self.model.frames):
                raise ValueError("F14 end-effector frame not found")
        joints = []
        for name in LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES:
            if not self.model.existJointName(name):
                raise ValueError(f"Joint not found: {name}")
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError(f"Expected scalar F14 arm joint: {name}")
            joints.append(joint)
        self.arm_q_indices = np.array([j.idx_q for j in joints])
        self.arm_v_indices = np.array([j.idx_v for j in joints])
        self.last_error = float("inf")
        self.last_diagnostics = None
        print(f"Pinocchio nq={self.model.nq}, nv={self.model.nv}; arm indices={self.arm_q_indices.tolist()}")

    def arm_to_full_q(self, arm_q):
        arm_q = np.asarray(arm_q, dtype=float)
        if arm_q.shape != (14,) or not np.isfinite(arm_q).all():
            raise ValueError("Expected 14 finite arm angles")
        q = pin.neutral(self.model)
        q[self.arm_q_indices] = arm_q
        return q

    def full_to_arm_q(self, q):
        return q[self.arm_q_indices].copy()

    def forward_kinematics(self, arm_q):
        q = self.arm_to_full_q(arm_q)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.left_ee_frame].copy(), self.data.oMf[self.right_ee_frame].copy()

    def solve(self, target_left, target_right, arm_q_init, max_iter=200,
              eps=1e-4, dt=0.2, damping=1e-5, verbose=False,
              limit_margin=0., limit_gain=0.):
        q = self.arm_to_full_q(arm_q_init)
        low = self.model.lowerPositionLimit[self.arm_q_indices]
        high = self.model.upperPositionLimit[self.arm_q_indices]
        self.last_diagnostics = None
        for i in range(max_iter + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            # current^-1 * target expresses the desired displacement in each
            # CURRENT EE's local frame. log6 yields [linear, angular] error.
            error = np.concatenate([
                pin.log6(self.data.oMf[frame].inverse() * target).vector
                for frame, target in ((self.left_ee_frame, target_left), (self.right_ee_frame, target_right))
            ])
            self.last_error = float(np.linalg.norm(error))
            if np.isfinite(self.last_error) and self.last_error < eps:
                return self.full_to_arm_q(q), True
            # Check the final integrated configuration too: the legacy loop
            # could report failure just as the last step reached tolerance.
            if i == max_iter or not np.isfinite(self.last_error):
                break
            jacobian = np.vstack([
                pin.computeFrameJacobian(self.model, self.data, q, frame, pin.ReferenceFrame.LOCAL)
                for frame in (self.left_ee_frame, self.right_ee_frame)
            ])[:, self.arm_v_indices]
            # Explicitly exclude the four fingers. Damping regularizes near
            # singular arm postures; dt damps the nonlinear iterative update.
            arm_velocity = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * np.eye(12), error)
            if limit_margin > 0:
                arm_q = self.full_to_arm_q(q)
                lower_distance = arm_q - low
                upper_distance = high - arm_q
                # Penalise outward motion near a limit, not a safe retreat.
                outward_distance = np.where(arm_velocity < 0, lower_distance, upper_distance)
                mobility = np.clip(outward_distance / limit_margin, .01, 1.) ** 2
                weighted_jt = mobility[:, None] * jacobian.T
                arm_velocity = weighted_jt @ np.linalg.solve(
                    jacobian @ weighted_jt + damping * np.eye(12), error)
                if limit_gain > 0:
                    repel = limit_gain * (
                        np.clip(1. - lower_distance / limit_margin, 0., 1.)
                        - np.clip(1. - upper_distance / limit_margin, 0., 1.))
                    # Redundant elbow/arm motion relieves wrist saturation
                    # without relaxing any Cartesian or orientation tolerance.
                    nullspace = np.eye(14) - np.linalg.pinv(jacobian, rcond=1e-6) @ jacobian
                    arm_velocity += nullspace @ repel
                # Bound each numerical update as well as the receiver motion.
                arm_velocity *= min(1., .15 / max(np.max(np.abs(arm_velocity * dt)), 1e-12))
            velocity = np.zeros(self.model.nv)
            velocity[self.arm_v_indices] = arm_velocity
            q = pin.integrate(self.model, q, velocity * dt)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        # Record the final failed candidate only. These observations do not
        # change goals, convergence tolerance, or the caller's braking policy.
        placements = [self.data.oMf[f] for f in (self.left_ee_frame, self.right_ee_frame)]
        arm_q = self.full_to_arm_q(q)
        low = self.model.lowerPositionLimit[self.arm_q_indices]
        high = self.model.upperPositionLimit[self.arm_q_indices]
        names = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
        self.last_diagnostics = {
            "iterations": i,
            "position_error_mm": [float(np.linalg.norm(actual.translation - target.translation) * 1000)
                                  for actual, target in zip(placements, (target_left, target_right))],
            "rotation_error_deg": [float(np.rad2deg(np.linalg.norm(
                pin.log3(actual.rotation.T @ target.rotation))))
                                   for actual, target in zip(placements, (target_left, target_right))],
            "near_limits": [names[k] for k in range(14)
                            if min(arm_q[k] - low[k], high[k] - arm_q[k]) <= np.deg2rad(.1)],
        }
        if verbose:
            print(f"IK unreachable/not converged: residual={self.last_error:.6g}")
        # Caller MUST retain its previous valid q on failure. A partial solution
        # is returned for diagnostics, never silently accepted as a command.
        return self.full_to_arm_q(q), False
