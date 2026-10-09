"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math  # noqa: F401
import numpy as np  # noqa: F401


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        return float(np.clip(self.kv * v + self.l_min, self.l_min, self.l_max))
        pass

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance."""
        n = len(path_points)

        # 1. Nearest waypoint to the rear axle
        nearest = min(range(n), key=lambda i: (path_points[i][0] - x) ** 2
                                              + (path_points[i][1] - y) ** 2)

        # 2. Walk forward (wrapping around the closed loop) until a waypoint is >= lookahead away
        idx = nearest
        for _ in range(n):
            px, py = path_points[idx][0], path_points[idx][1]
            if math.hypot(px - x, py - y) >= lookahead:
                return idx, path_points[idx]
            idx = (idx + 1) % n

        return idx, path_points[idx]   # fallback: track shorter than lookahead
        pass

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        dx = target_pt[0] - x
        dy = target_pt[1] - y

        # Rotate the world-frame offset into the car frame
        x_local =  math.cos(yaw) * dx + math.sin(yaw) * dy
        y_local = -math.sin(yaw) * dx + math.cos(yaw) * dy
        alpha = math.atan2(y_local, x_local)

        # Use the real distance to the chosen waypoint (it is slightly more than `lookahead`)
        ld = max(math.hypot(dx, dy), 1e-3)

        delta = math.atan2(2.0 * self.L * math.sin(alpha), ld)
        return float(np.clip(delta, -self.max_steer_rad, self.max_steer_rad))
        pass
