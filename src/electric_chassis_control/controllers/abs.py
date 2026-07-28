from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class ABSController:
    wheel_radius: float = 0.32
    target_slip: float = 0.15
    max_pressure: float = 1.0

    def compute(self, wheel_speeds: np.ndarray, vehicle_speed: float, requested_pressure: float) -> np.ndarray:
        wheels = np.asarray(wheel_speeds, dtype=float)
        if wheels.shape != (4,) or vehicle_speed <= 0 or requested_pressure < 0:
            raise ValueError("wheel speeds, vehicle speed and requested pressure are invalid")
        slip = (wheels * self.wheel_radius - vehicle_speed) / max(vehicle_speed, 0.5)
        reduction = np.clip(np.abs(slip) / max(self.target_slip, 1e-6), 0.0, 1.0)
        pressure = requested_pressure * (1.0 - 0.75 * reduction)
        return np.clip(pressure, 0.0, self.max_pressure)
