from __future__ import annotations

import math


def combined_tire_force(
    slip_ratio: float,
    slip_angle: float,
    normal_load: float,
    friction: float,
    longitudinal_stiffness: float = 9000.0,
    cornering_stiffness: float = 70000.0,
) -> tuple[float, float]:
    if normal_load <= 0 or friction <= 0 or longitudinal_stiffness <= 0 or cornering_stiffness <= 0:
        raise ValueError("tire parameters must be positive")
    fx = longitudinal_stiffness * slip_ratio
    fy = -cornering_stiffness * math.tan(slip_angle)
    limit = friction * normal_load
    norm = math.hypot(fx, fy)
    if norm > limit:
        scale = limit / norm
        fx, fy = fx * scale, fy * scale
    return float(fx), float(fy)
