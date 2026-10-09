"""
High-Level Lateral Steering Controller: Extended Kinematic Bicycle MPC.
Solves a constrained non-linear program over prediction horizon N using SciPy,
optimizing steering angle and longitudinal acceleration (mapped to throttle).
"""

import math  # noqa: F401
import numpy as np  # noqa: F401
from scipy.optimize import minimize  # noqa: F401


class KinematicBicycleMPC:
    """Nonlinear Model Predictive Control for an Extended Kinematic Bicycle Model.

    Optimizes future control sequences u = [delta_k, a_k] where steering angle delta_k
    and longitudinal acceleration a_k (mapped to throttle effort) are the control inputs,
    forward-simulating a 4-state extended kinematic bicycle model x = [x, y, theta, v]^T.
    """

    def __init__(self, wheelbase=1.25, dt=0.1, horizon=10,
                 max_steer_rad=math.radians(35.0), k_a=4.0,
                 max_accel=None, max_brake=None):
        self.L = wheelbase
        self.dt = dt
        self.N = horizon
        self.max_steer_rad = max_steer_rad
        self.k_a = float(max_accel if max_accel is not None else k_a)

        # Weights: heavily penalize lateral CTE, heading error, and steering rate
        self.w_lat = 30.0
        self.w_long = 1.0
        self.w_yaw = 10.0
        self.w_v = 5.0
        self.w_steer = 0.2
        self.w_dsteer = 6.0
        self.w_accel = 0.1

        self.last_u = np.zeros(2 * self.N)  # warm-start [delta_0, a_0, delta_1, a_1, ...]
        # Powertrain resistance, same values as the simulator
        self.c_drag = 0.005
        self.c_roll = 0.05

    def solve(self, x0, ref_trajectory, current_steer=0.0):
        """Solves MPC optimization problem over horizon N.

        x0: [x, y, yaw, v]
        ref_trajectory: list of length N containing [x_ref, y_ref, yaw_ref, v_ref]
        current_steer: actual current steering angle in radians
        Returns: (steer_rad, throttle_cmd in [-1.0, 1.0])
        """
        # 1. Horizon and bounds
        N = min(self.N, len(ref_trajectory))
        if N < 2:
            return 0.0, 0.0

        ref = np.asarray(ref_trajectory, dtype=float)[:N]
        lb = np.tile([-self.max_steer_rad, -self.k_a], N)
        ub = np.tile([self.max_steer_rad, self.k_a], N)
        bounds = list(zip(lb, ub))

        x_init, y_init, th_init, v_init = [float(s) for s in x0]
        dt, L = self.dt, self.L

        # 2. Objective: roll the model forward and sum Frenet-frame costs
        def objective(u):
            x, y, th, v = x_init, y_init, th_init, v_init
            prev_delta = current_steer
            cost = 0.0
            for k in range(N):
                delta = u[2 * k]
                a = u[2 * k + 1]

                # Extended kinematic bicycle step (all derivatives use the old state)
                x_n = x + v * math.cos(th) * dt
                y_n = y + v * math.sin(th) * dt
                th_n = th + (v / L) * math.tan(delta) * dt
                v_n = v + (a - self.c_drag * v * v - self.c_roll * v) * dt
                x, y, th, v = x_n, y_n, th_n, max(v_n, 0.0)

                # Error in the path-aligned (Frenet) frame of reference point k
                xr, yr, psir, vr = ref[k]
                dx, dy = x - xr, y - yr
                e_lat = -math.sin(psir) * dx + math.cos(psir) * dy
                e_long = math.cos(psir) * dx + math.sin(psir) * dy
                e_yaw = math.atan2(math.sin(th - psir), math.cos(th - psir))
                e_v = v - vr

                cost += (self.w_lat * e_lat ** 2
                         + self.w_long * e_long ** 2
                         + self.w_yaw * e_yaw ** 2
                         + self.w_v * e_v ** 2
                         + self.w_steer * delta ** 2
                         + self.w_dsteer * (delta - prev_delta) ** 2
                         + self.w_accel * a ** 2)
                prev_delta = delta
            return cost

        # 3. Warm start: shift the previous solution by one step (2 numbers)
        last = self.last_u[:2 * N]
        u_init = np.empty(2 * N)
        u_init[:-2] = last[2:]
        u_init[-2:] = last[-2:]          # repeat the final step
        u_init = np.clip(u_init, lb, ub)

        # 4. Optimise and extract the first control
        res = minimize(objective, u_init, bounds=bounds, method='SLSQP',
                       options={'maxiter': 25, 'ftol': 1e-3})
        u_opt = res.x if np.all(np.isfinite(res.x)) else u_init

        self.last_u = np.zeros(2 * self.N)
        self.last_u[:2 * N] = u_opt

        delta_cmd = float(u_opt[0])
        throttle_cmd = float(np.clip(u_opt[1] / self.k_a, -1.0, 1.0))
        return delta_cmd, throttle_cmd
    
        pass
