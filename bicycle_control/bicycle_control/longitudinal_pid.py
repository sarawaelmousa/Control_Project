"""
Low-Level Powertrain Cruise Controller (Longitudinal PID).
Regulates vehicle speed via normalized throttle/braking effort.
"""

import numpy as np  # noqa: F401


class PIDLongitudinalController:
    """Low-Level Powertrain Cruise Controller / Electronic Speed Control (ESC).

    Translates high-level velocity requests into normalized throttle/brake effort.
    Because physical vehicles experience friction and speed-squared aerodynamic drag,
    a closed-loop speed regulator is required to maintain target velocity.
    """

    def __init__(self, kp=1.0, ki=0.2, kd=0.05, dt=0.1,
                 max_throttle=1.0, max_brake=1.0, integral_limit=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_vel = None   

    def compute(self, target_vel, current_vel):
        """Computes normalized throttle/braking effort in [-1.0, 1.0]."""
        error = target_vel - current_vel

        # Proportional
        p = self.kp * error

        # Derivative on the measured speed (no kick when the target jumps)
        if self.prev_vel is None:
            d = 0.0
        else:
            d = -self.kd * (current_vel - self.prev_vel) / self.dt

        # Integral
        integral_new = float(np.clip(self.integral + error * self.dt,
                                     -self.integral_limit, self.integral_limit))

        # Unsaturated and saturated output
        u_unsat = p + self.ki * integral_new + d
        u = float(np.clip(u_unsat, -self.max_brake, self.max_throttle))

        # Conditional integration 
        # don't accumulate while saturated and the error pushes further into saturation
        pushing_high = u_unsat > self.max_throttle and error > 0
        pushing_low = u_unsat < -self.max_brake and error < 0
        if not (pushing_high or pushing_low):
            self.integral = integral_new

        self.prev_error = error
        self.prev_vel = current_vel
        return u
    
        pass

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_vel = None
