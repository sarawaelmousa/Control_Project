"""
Target Velocity Profiler based on track curvature.
Calculates maximum safe cornering speeds subject to lateral acceleration limits.
"""

import math  # noqa: F401


class VelocityProfiler:
    """Generates target speed profiles based on track curvature or precomputed data."""

    def __init__(self, default_speed=4.0, max_speed=8.0, max_lat_accel=5.0 , min_speed=1.0 ):
        self.default_speed = default_speed
        self.max_speed = max_speed
        self.max_lat_accel = max_lat_accel
        self.min_speed = min_speed

    def compute_target_speed(self, kappa, fallback_speed=None):
        """Calculates curvature-limited velocity: v_max = sqrt(a_lat_max / |kappa|)."""
        # No usable curvature: fall back
        if kappa is None or not math.isfinite(kappa):
            return fallback_speed if fallback_speed is not None else self.default_speed

        k = abs(kappa)
        if k < 1e-6:
            v = self.max_speed                       # straight line
        else:
            v = math.sqrt(self.max_lat_accel / k)    # from a_lat = v^2 * kappa

        return float(min(max(v, self.min_speed), self.max_speed))
        pass
