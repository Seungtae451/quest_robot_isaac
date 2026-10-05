"""Collection-only, independently constrained Pinocchio/ProxQP arm control.

The optimization variable is seven joint velocities per arm, not a distant
IK destination. Keep a continuous reference; measured joints constrain drive
lag. No unverified/infeasible QP result is used as a command.
"""
import numpy as np
import pinocchio as pin
import proxsuite
import time

from config import teleop_config as cfg
from config import table_assist_config as assist_cfg
from robot.reachable_ik import ReachableResult
from robot.table_approach import limit_obstacle_path
from robot.f14_config import LEFT_JOINT_NAMES, RIGHT_JOINT_NAMES


class DifferentialIK:
    ROWS = 13  # 7 joint bounds + 3 angular bounds + 3 merged collision axes

    def __init__(self, ik, *, position_only=False, rotating_geometry=False):
        self.ik = ik
        self.position_only = position_only
        self.rotating_geometry = position_only or rotating_geometry
        self.rows = 256 if self.rotating_geometry else self.ROWS
        self.data = ik.model.createData()
        self.lower = ik.model.lowerPositionLimit[ik.arm_q_indices].copy()
        self.upper = ik.model.upperPositionLimit[ik.arm_q_indices].copy()
        self.reference = None
        self.velocity = np.zeros(14)
        self.solvers = [proxsuite.proxqp.dense.QP(7, 0, self.rows) for _ in range(2)]
        self.initialized = [False, False]
        for qp in self.solvers:
            qp.settings.eps_abs = 1e-7
            qp.settings.eps_rel = 0.
            qp.settings.max_iter = cfg.QP_MAX_ITER
            qp.settings.max_iter_in = cfg.QP_MAX_INNER_ITER
            qp.settings.initial_guess = proxsuite.proxqp.InitialGuess.WARM_START_WITH_PREVIOUS_RESULT

    def reset(self, measured_q):
        measured_q = np.asarray(measured_q, float)
        self.ik.arm_to_full_q(measured_q)  # shape/finite validation
        self.reference = measured_q.copy()
        self.velocity.fill(0.)
        self.initialized = [False, False]

    def _geometry(self, feedback, side, pose):
        if feedback is None or not feedback.get('enabled'):
            return None
        arm = feedback['arms'][side]
        parts = np.asarray(arm.get('part_bounds', [arm['offset_bounds']]), float)
        obstacles = feedback['obstacles']
        local = None
        if self.rotating_geometry:
            local = [np.asarray(p,float) for p in arm['part_local_points']]
            if not local or any(p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.isfinite(p).all() for p in local):
                raise ValueError('Invalid rotating gripper collision envelopes')
            offsets = [p @ pose.rotation.T for p in local]
            parts = np.asarray([[p.min(0),p.max(0)] for p in offsets])
        if (parts.ndim != 3 or parts.shape[1:] != (2, 3) or not len(parts)
                or not np.isfinite(parts).all() or np.any(parts[:, 1] < parts[:, 0])):
            raise ValueError('Invalid QP collision envelopes')
        for obstacle in obstacles:
            bounds = np.asarray(obstacle['bounds'], float)
            if (bounds.shape != (2, 3) or not np.isfinite(bounds).all()
                    or np.any(bounds[1] <= bounds[0])):
                raise ValueError('Invalid QP obstacle')
        return parts, obstacles, local

    @staticmethod
    def _rotating_collision_rows(pose, jacobian, geometry):
        """Constrain corner velocities, including passive wrist rotation.

        Choose one separating face per part/solid, retaining the box's open top.
        Every corner must remain on that safe side of the selected face.
        """
        if geometry is None:
            return []
        _, obstacles, local = geometry
        rows=[]
        for part in local:
            offsets=part@pose.rotation.T
            points=offsets+pose.translation
            for obstacle in obstacles:
                bounds=np.asarray(obstacle['bounds'],float)
                low=bounds[0]-assist_cfg.MINIMUM_CLEARANCE
                high=bounds[1]+assist_cfg.MINIMUM_CLEARANCE
                gaps=np.r_[low-points.max(0),points.min(0)-high]
                face=int(np.argmax(gaps)); axis=face%3
                positive=face>=3
                for point,offset in zip(points,offsets):
                    # v(point) = v(TCP) + omega x (point - TCP).
                    cross=np.array([[0,-offset[2],offset[1]],
                                    [offset[2],0,-offset[0]],[-offset[1],offset[0],0]])
                    row=(jacobian[:3]-cross@jacobian[3:])[axis]
                    gap=point[axis]-high[axis] if positive else low[axis]-point[axis]
                    speed=max(float(gap),0.)/assist_cfg.LOOKAHEAD_SECONDS
                    rows.append((row,-speed,np.inf) if positive else (row,-np.inf,speed))
        return rows

    @staticmethod
    def _collision_rows(position, world_jacobian, geometry):
        if geometry is None:
            return []
        low_velocity = np.full(3, -np.inf)
        high_velocity = np.full(3, np.inf)
        parts, obstacles = geometry
        for part in parts:
            for obstacle in obstacles:
                bounds = np.asarray(obstacle['bounds'], float)
                low = bounds[0] - part[1] - assist_cfg.MINIMUM_CLEARANCE
                high = bounds[1] - part[0] + assist_cfg.MINIMUM_CLEARANCE
                if np.all(position >= low) and np.all(position <= high):
                    # Existing margin overlap: allow ANY outward escape,
                    # including lifting over the open box rim.
                    for axis, direction in enumerate(np.sign(position - (low + high) / 2)):
                        if direction:
                            if direction > 0:
                                low_velocity[axis] = max(low_velocity[axis], 0.)
                            else:
                                high_velocity[axis] = min(high_velocity[axis], 0.)
                else:
                    gaps = np.maximum(low - position, position - high)
                    axis = int(np.argmax(gaps))
                    direction = -1. if position[axis] < low[axis] else 1.
                    speed = gaps[axis] / assist_cfg.LOOKAHEAD_SECONDS
                    if direction > 0:
                        low_velocity[axis] = max(low_velocity[axis], -speed)
                    else:
                        high_velocity[axis] = min(high_velocity[axis], speed)
        # Identical separating normals share one inequality. This preserves
        # their exact intersection but avoids ill-conditioned duplicate rows.
        return [(world_jacobian[axis], low_velocity[axis], high_velocity[axis])
                for axis in range(3) if np.isfinite(low_velocity[axis]) or np.isfinite(high_velocity[axis])]

    def _valid_step(self, before, after, target, geometry):
        # Validate the nonlinear FK too, rather than relying solely on the
        # first-order rotation/collision constraints of the QP.
        old_error = np.linalg.norm(pin.log3(before.rotation.T @ target.rotation))
        new_error = np.linalg.norm(pin.log3(after.rotation.T @ target.rotation))
        if not self.position_only and new_error > max(cfg.QP_NONLINEAR_ROTATION_TOLERANCE, old_error) + 2e-6:
            return False
        if geometry is not None:
            if self.rotating_geometry:
                _, obstacles, local = geometry
                # Check the actual rotated envelopes, not a downward template.
                # Margin overlap may escape, but cannot deepen. Intermediate
                # poses prevent a thin wall from being crossed in one step.
                delta=pin.log3(before.rotation.T@after.rotation)
                for part in local:
                    old=part@before.rotation.T+before.translation
                    for obstacle in obstacles:
                        bounds=np.asarray(obstacle['bounds'],float)
                        def gap(points):
                            return float(np.max(np.r_[bounds[0]-points.max(0),points.min(0)-bounds[1]]))-assist_cfg.MINIMUM_CLEARANCE
                        minimum=min(0.,gap(old))-2e-7
                        for fraction in (.25,.5,.75,1.):
                            rot=before.rotation@pin.exp3(delta*fraction)
                            points=part@rot.T+before.translation+fraction*(after.translation-before.translation)
                            if gap(points)<minimum:
                                return False
                return True
            corrected, blocked = limit_obstacle_path(
                before.translation, after.translation, *geometry[:2], np.zeros((2, 3)))
            if blocked and np.linalg.norm(corrected - after.translation) > 1e-7:
                return False
        return True

    def solve(self, target_left, target_right, measured_q, *, dt, feedback=None):
        measured_q = np.asarray(measured_q, float)
        self.ik.arm_to_full_q(measured_q)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Invalid differential IK timestep')
        dt = min(float(dt), cfg.QP_MAX_DT)
        if self.reference is None:
            self.reset(measured_q)
        full_q = self.ik.arm_to_full_q(self.reference)
        pin.forwardKinematics(self.ik.model, self.data, full_q)
        pin.updateFramePlacements(self.ik.model, self.data)
        frames = (self.ik.left_ee_frame, self.ik.right_ee_frame)
        before = [self.data.oMf[f].copy() for f in frames]
        targets = (target_left, target_right)
        statuses, accepted, iterations = [], [], []
        solves = 0
        for side, (frame, pose, target) in enumerate(zip(frames, before, targets)):
            section = slice(side * 7, (side + 1) * 7)
            q = self.reference[section]
            previous = self.velocity[section].copy()
            geometry = self._geometry(feedback, side, pose)
            jac = pin.computeFrameJacobian(self.ik.model, self.data, full_q, frame,
                pin.ReferenceFrame.LOCAL)[:, self.ik.arm_v_indices[section]]
            world_jac = pose.rotation @ jac[:3]
            error = pin.log3(pose.rotation.T @ target.rotation)
            desired_xyz = cfg.QP_POSITION_GAIN * (target.translation - pose.translation)
            desired_xyz *= min(1., cfg.QP_MAX_CARTESIAN_SPEED / max(np.linalg.norm(desired_xyz), 1e-12))
            desired = np.r_[pose.rotation.T @ desired_xyz, cfg.QP_ORIENTATION_GAIN * error]
            task_jac = jac[:3] if self.position_only else jac
            weighted = jac[:3] if self.position_only else jac * np.array([1., 1., 1., cfg.QP_ORIENTATION_WEIGHT, cfg.QP_ORIENTATION_WEIGHT, cfg.QP_ORIENTATION_WEIGHT])[:, None]
            desired = desired[:3] if self.position_only else desired * np.array([1., 1., 1., cfg.QP_ORIENTATION_WEIGHT, cfg.QP_ORIENTATION_WEIGHT, cfg.QP_ORIENTATION_WEIGHT])
            distance_low, distance_high = q - self.lower[section], self.upper[section] - q
            repel = cfg.QP_LIMIT_GAIN * (np.clip(1. - distance_low / cfg.IK_LIMIT_MARGIN, 0., 1.)
                                      - np.clip(1. - distance_high / cfg.IK_LIMIT_MARGIN, 0., 1.))
            # Only redundant motion is used for posture relief.
            repel = (np.eye(7) - np.linalg.pinv(task_jac, rcond=1e-6) @ task_jac) @ repel
            if self.position_only:
                # Do not rotate idle hands just to optimise posture at A/HOME.
                repel *= min(1.,np.linalg.norm(target.translation-pose.translation)/.01)
            regularization = cfg.QP_REGULARIZATION
            h = weighted.T @ weighted + regularization * np.eye(7)
            g = -weighted.T @ desired - regularization * repel
            # A positive scalar does not change the minimizer. Normalize the
            # objective so the proximal solver's dual tolerance is meaningful
            # even with redundant collision planes and angular task weights.
            objective_scale = float(np.trace(h))
            h /= objective_scale
            g /= objective_scale
            acceleration = cfg.ARM_MAX_ACCELERATION
            vmax = cfg.ARM_MAX_VELOCITY
            braking = lambda d: np.sqrt(2 * acceleration * np.maximum(d, 0.) + (acceleration * dt)**2) - acceleration * dt
            low = np.maximum.reduce([np.full(7, -vmax), previous - acceleration * dt,
                (self.lower[section] - q) / dt, -braking(distance_low)])
            high = np.minimum.reduce([np.full(7, vmax), previous + acceleration * dt,
                (self.upper[section] - q) / dt, braking(distance_high)])
            lag = q - measured_q[section]
            # Predictively slow increasing lag; retreat remains available even
            # when the measured actuator is blocked past the permitted lag.
            lag_horizon = max(dt, cfg.QP_LAG_HORIZON)
            low = np.maximum(low, np.minimum(0., (-cfg.ARM_MAX_TRACKING_ERROR - lag) / lag_horizon))
            high = np.minimum(high, np.maximum(0., (cfg.ARM_MAX_TRACKING_ERROR - lag) / lag_horizon))
            # With fast acceleration, also reserve stopping distance before
            # the drive-lag boundary. Keeps a blocked actuator's reference bounded.
            low = np.maximum(low, -braking(cfg.ARM_MAX_TRACKING_ERROR + lag))
            high = np.minimum(high, braking(cfg.ARM_MAX_TRACKING_ERROR - lag))
            allowance = np.maximum(np.abs(error), cfg.IK_EPS / np.sqrt(3))
            rows = [(np.eye(7)[i], low[i], high[i]) for i in range(7)]
            if not self.position_only:
                rows += [(jac[3+i], (error[i] - allowance[i]) / dt,
                           (error[i] + allowance[i]) / dt) for i in range(3)]
            if self.rotating_geometry:
                world_full_jac=pose.rotation@jac.reshape(2,3,7)
                world_full_jac=world_full_jac.reshape(6,7)
                rows += self._rotating_collision_rows(pose,world_full_jac,geometry)
            else:
                rows += self._collision_rows(pose.translation, world_jac, geometry[:2] if geometry else None)
            if len(rows) > self.rows:
                raise ValueError('Too many collision constraints')
            c = np.zeros((self.rows, 7)); l = np.full(self.rows, -np.inf); u = np.full(self.rows, np.inf)
            for i, (row, lo, hi) in enumerate(rows):
                c[i], l[i], u[i] = row, lo, hi
            qp = self.solvers[side]
            valid = False
            status = 'CONSTRAINT_CONFLICT'
            if np.all(l <= u):
                if not self.initialized[side]:
                    qp.settings.initial_guess = proxsuite.proxqp.InitialGuess.NO_INITIAL_GUESS
                    qp.init(h, g, None, None, c, l, u)
                    self.initialized[side] = True
                else:
                    qp.settings.initial_guess = proxsuite.proxqp.InitialGuess.WARM_START_WITH_PREVIOUS_RESULT
                    # Jacobians and active separating faces change. Reusing
                    # the previous Ruiz scaling can cause false nonconvergence.
                    qp.update(H=h, g=g, C=c, l=l, u=u, update_preconditioner=True)
                qp_started = time.perf_counter()
                qp.solve()
                solves += 1
                if (qp.results.info.status == proxsuite.proxqp.QPSolverOutput.PROXQP_MAX_ITER_REACHED
                        and (time.perf_counter() - qp_started) * 1000 < cfg.QP_COLD_RETRY_BUDGET_MS):
                    # One cold restart for an obsolete warm active set. Never
                    # accept the failed result or relax physical constraints.
                    qp.settings.initial_guess = proxsuite.proxqp.InitialGuess.NO_INITIAL_GUESS
                    qp.init(h, g, None, None, c, l, u)
                    qp.solve()
                    solves += 1
                status = str(qp.results.info.status).split('.')[-1]
                v = np.asarray(qp.results.x).copy()
                cv = c @ v
                valid = (qp.results.info.status == proxsuite.proxqp.QPSolverOutput.PROXQP_SOLVED
                    and np.isfinite(v).all() and np.all(cv >= l - 2e-7) and np.all(cv <= u + 2e-7))
                if valid:
                    # Remove only floating-point bound leakage, then verify
                    # constraints again using the command actually integrated.
                    v = (np.clip(q + v * dt, self.lower[section], self.upper[section]) - q) / dt
                    cv = c @ v
                    valid = np.all(cv >= l - 2e-7) and np.all(cv <= u + 2e-7)
                    candidate = self.reference.copy(); candidate[section] = q + v * dt
                    after = self.ik.forward_kinematics(candidate)[side]
                    valid = valid and self._valid_step(pose, after, target, geometry)
                    if not valid:
                        status = 'NONLINEAR_GUARD'
            if not valid:
                # Bounded deceleration if still geometrically admissible;
                # otherwise hold this arm's reference and let the receiver's
                # independent acceleration limiter brake the physical drive.
                v = previous + np.clip(-previous, -acceleration * dt, acceleration * dt)
                candidate = self.reference.copy(); candidate[section] = q + v * dt
                safe_brake = (np.all(candidate[section] >= self.lower[section])
                    and np.all(candidate[section] <= self.upper[section])
                    and self._valid_step(pose, self.ik.forward_kinematics(candidate)[side], target, geometry))
                if not safe_brake:
                    v = np.zeros(7)
                self.initialized[side] = False
            self.reference[section] += v * dt
            self.velocity[section] = v
            statuses.append(status); accepted.append(bool(valid))
            iterations.append(int(qp.results.info.iter))
        actual = self.ik.forward_kinematics(self.reference)
        measured = self.ik.forward_kinematics(measured_q)
        self.ik.last_error = float(np.linalg.norm(np.concatenate([
            t.translation-a.translation if self.position_only else pin.log6(a.inverse() * t).vector
            for a, t in zip(actual, targets)])))
        remaining = [float(np.linalg.norm(a.translation - t.translation) * 1000) for a, t in zip(measured, targets)]
        diagnostics = {'arm_status': statuses, 'arm_success': accepted, 'position_only': self.position_only,
            'iterations': max(iterations), 'position_error_mm': remaining,
            'reference_position_m': [a.translation.tolist() for a in actual],
            'measured_position_m': [a.translation.tolist() for a in measured],
            'max_joint_speed_rad_s': [float(np.max(np.abs(self.velocity[s*7:(s+1)*7]))) for s in range(2)],
            'max_joint_lag_rad': [float(np.max(np.abs(self.reference[s*7:(s+1)*7] - measured_q[s*7:(s+1)*7]))) for s in range(2)],
            'rotation_error_deg': [float(np.rad2deg(np.linalg.norm(pin.log3(a.rotation.T @ t.rotation)))) for a, t in zip(actual, targets)],
            'near_limits': [name for i, name in enumerate(LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES)
                if min(self.reference[i] - self.lower[i], self.upper[i] - self.reference[i]) < np.deg2rad(1.)]}
        return ReachableResult(self.reference.copy(), any(accepted), 'QP' if all(accepted) else 'QP_PARTIAL' if any(accepted) else 'HOLD',
                               1., solves, remaining, diagnostics)
