from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class TorqueAllocator:
    wheel_radius: float = 0.32
    front_axle: float = 1.25
    track: float = 1.58
    max_torque: float = 2500.0

    @classmethod
    def default(cls) -> TorqueAllocator:
        return cls()

    def allocate(self, longitudinal_force: float, yaw_moment: float) -> np.ndarray:
        values = np.asarray([longitudinal_force, yaw_moment], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("allocation request must be finite")
        base = longitudinal_force * self.wheel_radius / 4.0
        moment_split = yaw_moment * self.wheel_radius / max(self.track, 1e-6) / 2.0
        torques = np.array([base - moment_split, base + moment_split, base - moment_split, base + moment_split])
        return np.clip(torques, -self.max_torque, self.max_torque)
