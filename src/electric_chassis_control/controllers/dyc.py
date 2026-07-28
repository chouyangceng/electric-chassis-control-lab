from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class DirectYawMomentController:
    yaw_gain: float = 800.0
    sideslip_gain: float = 300.0
    max_moment: float = 2500.0

    def compute(self, speed: float, yaw_rate: float, target_yaw_rate: float, sideslip: float) -> np.ndarray:
        values = np.asarray([speed, yaw_rate, target_yaw_rate, sideslip], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("controller inputs must be finite")
        moment = self.yaw_gain * (target_yaw_rate - yaw_rate) - self.sideslip_gain * sideslip
        return np.array([np.clip(moment, -self.max_moment, self.max_moment), 0.0])
