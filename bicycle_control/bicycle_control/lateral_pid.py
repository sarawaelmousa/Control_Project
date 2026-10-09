"""
High-Level Lateral Steering Controller: Reactive Lateral PID.
Steers based on instantaneous Cross-Track Error (CTE) and Heading Error.
"""

import math
import numpy as np  # noqa: F401


class LateralPIDController:
    """Lateral PID steering controller based on Cross-Track Error (CTE) and Heading Error.

    Commands front wheel steering based on instantaneous lateral offset (cross-track error)
    and orientation error relative to the nearest path waypoint.
    """

    def __init__(self, kp=0.5, ki=0.2, kd=0.5, k_yaw=0.15 ,dt=0.1,
                 max_steer_rad=math.radians(35.0), integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.k_yaw = k_yaw
        self.dt = dt
        self.max_steer_rad = max_steer_rad
        self.integral_limit = integral_limit

        self.integral_cte = 0.0
        self.prev_cte = 0.0
        self._first = True   # avoids a derivative spike on the first call

    def compute_steering(self, cte, heading_err):
        """Computes front wheel steering angle delta in radians.

        Args:
            cte: Signed cross-track error in meters (positive = vehicle is left of path).
            heading_err: Heading error in radians (psi_vehicle - psi_path).

        Returns:
            delta_rad: Commanded front steering angle in radians [-max_steer_rad, max_steer_rad].
        """
        # Wrap heading error to [-pi, pi]
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

        # P and D on cross-track error
        p = self.kp * cte
        d = 0.0 if self._first else self.kd * (cte - self.prev_cte) / self.dt

        # Integral with clamp (anti-windup layer 1)
        integral_new = float(np.clip(self.integral_cte + cte * self.dt,
                                     -self.integral_limit, self.integral_limit))

        # Negative sign: left of path -> steer right
        delta_unsat = -(p + self.ki * integral_new + d) - self.k_yaw * heading_err
        delta = float(np.clip(delta_unsat, -self.max_steer_rad, self.max_steer_rad))

        # Conditional integration (anti-windup layer 2)
        pushing_high = delta_unsat > self.max_steer_rad and cte < 0
        pushing_low = delta_unsat < -self.max_steer_rad and cte > 0
        if not (pushing_high or pushing_low):
            self.integral_cte = integral_new

        self.prev_cte = cte
        self._first = False
        return delta
    
        pass

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral_cte = 0.0
        self.prev_cte = 0.0
