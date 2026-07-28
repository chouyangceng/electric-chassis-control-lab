from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class LQRController:
    gain: np.ndarray = field(default_factory=lambda: np.array([500.0, 120.0]))
    max_moment: float = 2500.0

    def compute(self, error: np.ndarray) -> float:
        error = np.asarray(error, dtype=float)
        if error.shape != (2,) or not np.all(np.isfinite(error)):
            raise ValueError("LQR error must be a finite vector with shape (2,)")
        return float(np.clip(self.gain @ error, -self.max_moment, self.max_moment))
