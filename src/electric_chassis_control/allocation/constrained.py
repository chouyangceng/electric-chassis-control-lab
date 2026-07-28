from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AllocationResult:
    torques: np.ndarray
    residual_norm: float
    max_torque: float


@dataclass
class ConstrainedTorqueAllocator:
    wheel_radius: float = 0.32
    track: float = 1.58
    max_torque: float = 2500.0

    @classmethod
    def default(cls) -> ConstrainedTorqueAllocator:
        return cls()

    def allocate(self, longitudinal_force: float, yaw_moment: float, friction: float = 0.85) -> AllocationResult:
        if friction <= 0 or not np.all(np.isfinite([longitudinal_force, yaw_moment, friction])):
            raise ValueError("allocation inputs must be finite and friction must be positive")
        matrix = np.array([[1, 1, 1, 1], [-self.track, self.track, -self.track, self.track]], dtype=float) / self.wheel_radius
        desired = np.array([longitudinal_force, yaw_moment])
        raw, *_ = np.linalg.lstsq(matrix, desired, rcond=None)
        limit = min(self.max_torque, friction * 3500.0 * self.wheel_radius)
        torques = np.clip(raw, -limit, limit)
        residual = matrix @ torques - desired
        return AllocationResult(torques, float(np.linalg.norm(residual)), float(limit))
