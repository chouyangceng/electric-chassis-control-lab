from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BrakeCommand:
    regenerative_force: float
    mechanical_force: float


@dataclass
class RegenerativeBrakeCoordinator:
    max_regen_force: float = 4000.0
    min_soc: float = 0.1
    max_soc: float = 0.95

    def allocate(self, total_brake_force: float, battery_soc: float) -> BrakeCommand:
        if total_brake_force < 0 or not 0 <= battery_soc <= 1:
            raise ValueError("brake force and battery SOC are invalid")
        if battery_soc >= self.max_soc:
            regen_ratio = 0.0
        else:
            regen_ratio = max(0.0, min(1.0, (self.max_soc - battery_soc) / (self.max_soc - self.min_soc)))
        regenerative = min(total_brake_force, self.max_regen_force * regen_ratio)
        return BrakeCommand(float(regenerative), float(total_brake_force - regenerative))
