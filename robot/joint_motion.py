"""Continuous arm drive reference with bounded speed and acceleration.

Initialise once from measured joints. Retargeting (including an IK branch
change) never resets position or velocity. Missing/failed IK requests brake.
"""
import numpy as np


class JointMotionLimiter:
    def __init__(self, measured_q, max_velocity, max_acceleration, max_tracking_error,
                 allow_tracking_retreat=False):
        self.position = np.asarray(measured_q, dtype=float).copy()
        if self.position.shape != (14,) or not np.isfinite(self.position).all():
            raise ValueError("Expected 14 finite measured arm angles")
        for value in (max_velocity, max_acceleration, max_tracking_error):
            if not np.isfinite(value) or value <= 0:
                raise ValueError("Motion limits must be finite and positive")
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.max_tracking_error = float(max_tracking_error)
        self.allow_tracking_retreat = bool(allow_tracking_retreat)
        self.velocity = np.zeros(14)
        self.tracking_limited = np.zeros(14, dtype=bool)

    def step(self, goal, measured_q, dt, enabled=True, *, reference_mode=False):
        goal = np.asarray(goal, dtype=float)
        measured_q = np.asarray(measured_q, dtype=float)
        if (goal.shape != (14,) or measured_q.shape != (14,)
                or not np.isfinite(goal).all() or not np.isfinite(measured_q).all()
                or not np.isfinite(dt) or dt <= 0):
            raise ValueError("Invalid joint motion update")
        # This gain starts deceleration early enough for a stationary target:
        # at vmax the proportional controller asks for at most amax deceleration.
        # Differential IK already integrates a continuous joint reference.
        # Catch up to it without another slow destination low-pass; velocity,
        # acceleration, tracking and timeout braking still apply locally.
        gain = 1. / dt if reference_mode else self.max_acceleration / self.max_velocity
        desired_velocity = np.clip(gain * (goal - self.position),
                                   -self.max_velocity, self.max_velocity)
        if reference_mode:
            # Start braking before a stopped reference, avoiding the limit
            # cycle produced by high-gain catch-up plus acceleration clipping.
            distance = np.abs(goal - self.position)
            stopping_speed = np.sqrt(2 * self.max_acceleration * distance
                + (self.max_acceleration * dt)**2) - self.max_acceleration * dt
            desired_velocity = np.clip(desired_velocity, -stopping_speed, stopping_speed)
        self.tracking_limited = np.abs(self.position - measured_q) > self.max_tracking_error
        # Brake further separation, but permit a retreat reducing drive lag.
        # Keep the continuous reference and acceleration bounds; never snap
        # the target to measured joints when contact blocks an actuator.
        increases_lag = desired_velocity * (self.position - measured_q) >= 0.
        blocked = self.tracking_limited & increases_lag if self.allow_tracking_retreat else self.tracking_limited
        desired_velocity[blocked] = 0.
        if not enabled:
            desired_velocity.fill(0.)
        self.velocity += np.clip(desired_velocity - self.velocity,
                                 -self.max_acceleration * dt, self.max_acceleration * dt)
        self.position += self.velocity * dt
        return self.position.copy()
