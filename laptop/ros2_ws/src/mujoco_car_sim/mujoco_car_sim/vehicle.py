"""Vehicle control: Twist -> Ackermann wheel targets -> PID wheel torques.

Two layers, like a real RC car / F1TENTH stack:
  1. AckermannKinematics turns a body command (forward speed v, yaw rate w) into
     a steering angle for each front wheel and a target spin rate for every wheel.
  2. WheelPID runs one PID loop per wheel, every physics step, turning the error
     between target and measured wheel speed into motor torque.
No ROS in here, so it can be unit-tested or reused on its own.
"""
from dataclasses import dataclass

import numpy as np

WHEELS = ('fl', 'fr', 'rl', 'rr')  # front-left, front-right, rear-left, rear-right


@dataclass
class Geometry:
    wheelbase: float = 0.32     # front axle to rear axle (m)
    track: float = 0.26         # left wheel centre to right wheel centre (m)
    wheel_radius: float = 0.05  # (m)
    max_steer: float = 0.45     # max front-wheel angle (rad), ~26 deg
    max_speed: float = 3.0      # (m/s)


class AckermannKinematics:
    def __init__(self, geom: Geometry):
        self.g = geom

    def __call__(self, v, w):
        """Return (steer_left, steer_right, wheel_rates[fl, fr, rl, rr] in rad/s).

        v is forward speed (m/s) and w is yaw rate (rad/s, +left), both at the rear axle.
        A car can't turn on the spot, so w only has an effect while v != 0.
        """
        g = self.g
        v = float(np.clip(v, -g.max_speed, g.max_speed))

        if abs(v) < 1e-3:
            # Stationary: still point the wheels the way the driver is asking,
            # so they're already turned when the car starts moving.
            steer = float(np.sign(w)) * g.max_steer if abs(w) > 1e-3 else 0.0
            return steer, steer, np.zeros(4)

        # Bicycle model: curvature k = w / v, centre steering angle = atan(L * k).
        # Clamp to the steering limit, then recompute the curvature we can actually do.
        delta = np.clip(np.arctan(g.wheelbase * w / v), -g.max_steer, g.max_steer)
        k = np.tan(delta) / g.wheelbase
        half = g.track / 2

        # Ackermann: each front wheel points at the same turning centre,
        # so the inner wheel turns more sharply than the outer one.
        steer_l = np.arctan(g.wheelbase * k / (1 - k * half))
        steer_r = np.arctan(g.wheelbase * k / (1 + k * half))

        # Each wheel's speed is proportional to its distance from the turning centre
        # (an "electronic differential"), so no tyre has to scrub.
        speeds = v * np.array([
            np.hypot(g.wheelbase * k, 1 - k * half),  # fl
            np.hypot(g.wheelbase * k, 1 + k * half),  # fr
            1 - k * half,                              # rl
            1 + k * half,                              # rr
        ])
        return float(steer_l), float(steer_r), speeds / g.wheel_radius


class WheelPID:
    """Vectorised PID, one loop per wheel, with torque limits and anti-windup."""

    def __init__(self, kp, ki, kd, max_torque, d_filter=0.9, n=4):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_torque = max_torque
        self.d_filter = d_filter  # 0 = raw derivative, closer to 1 = smoother
        self.integral = np.zeros(n)
        self.d_meas = np.zeros(n)
        self.prev_measured = None
        self.last_torque = np.zeros(n)

    def reset(self):
        self.integral[:] = 0.0
        self.d_meas[:] = 0.0
        self.prev_measured = None

    def __call__(self, target, measured, dt):
        error = target - measured

        # Derivative on the measurement (not the error), so a sudden change in the
        # setpoint doesn't cause a torque spike ("derivative kick").
        # It is low-pass filtered because wheel speed is noisy at a 2 ms timestep.
        if self.prev_measured is not None:
            raw = (measured - self.prev_measured) / dt
            self.d_meas = self.d_filter * self.d_meas + (1 - self.d_filter) * raw
        self.prev_measured = measured.copy()

        candidate_integral = self.integral + error * dt
        torque = self.kp * error + self.ki * candidate_integral - self.kd * self.d_meas
        saturated = np.abs(torque) > self.max_torque

        # Anti-windup: only keep integrating on wheels whose motor isn't maxed out.
        self.integral = np.where(saturated, self.integral, candidate_integral)
        torque = np.clip(torque, -self.max_torque, self.max_torque)
        self.last_torque = torque
        return torque
