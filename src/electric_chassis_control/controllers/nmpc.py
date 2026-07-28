from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class NMPCController:
    horizon: int = 8
    max_moment: float = 2500.0
    last_command: float = 0.0

    def compute(self, yaw_error: float, sideslip: float, dt: float = 0.02) -> float:
        if self.horizon <= 0 or dt <= 0 or not np.all(np.isfinite([yaw_error, sideslip])):
            raise ValueError("NMPC inputs and horizon must be valid")
        command = 700.0 * yaw_error - 260.0 * sideslip
        self.last_command = float(np.clip(command, -self.max_moment, self.max_moment))
        return self.last_command
