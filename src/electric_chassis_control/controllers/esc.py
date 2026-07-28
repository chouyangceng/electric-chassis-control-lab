from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ESCOutput:
    yaw_moment: float
    brake_pressures: np.ndarray
    intervention: bool


@dataclass
class ESCController:
    yaw_gain: float = 1000.0
    sideslip_gain: float = 350.0
    yaw_threshold: float = 0.04
    max_moment: float = 2500.0
    max_brake_pressure: float = 1.0

    def compute(self, speed: float, yaw_rate: float, target_yaw_rate: float, sideslip: float) -> ESCOutput:
        values = np.asarray([speed, yaw_rate, target_yaw_rate, sideslip], dtype=float)
        if not np.all(np.isfinite(values)) or speed < 0:
            raise ValueError("ESC inputs must be finite and speed must be non-negative")
        error = target_yaw_rate - yaw_rate
        moment = float(np.clip(self.yaw_gain * error - self.sideslip_gain * sideslip, -self.max_moment, self.max_moment))
        intervention = abs(error) > self.yaw_threshold or abs(sideslip) > self.yaw_threshold
        pressure = np.zeros(4)
        if intervention:
            pressure[np.argmax(np.abs([moment, -moment, moment, -moment]))] = min(
                self.max_brake_pressure, abs(moment) / self.max_moment
            )
        return ESCOutput(moment if intervention else 0.0, pressure, intervention)
