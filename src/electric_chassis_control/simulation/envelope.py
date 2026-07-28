from __future__ import annotations

import numpy as np


def compute_stability_envelope(friction_values: np.ndarray, speed: float, gravity: float = 9.81) -> np.ndarray:
    friction_values = np.asarray(friction_values, dtype=float)
    if friction_values.ndim != 1 or np.any(friction_values <= 0) or speed <= 0 or gravity <= 0:
        raise ValueError("friction values, speed and gravity must be positive")
    return friction_values * gravity / speed
