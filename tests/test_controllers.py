import numpy as np

from electric_chassis_control.allocation.qp_allocator import TorqueAllocator
from electric_chassis_control.controllers.dyc import DirectYawMomentController


def test_dyc_is_finite_at_zero_speed():
    cmd = DirectYawMomentController().compute(0.0, 0.0, 0.2, 0.0)
    assert np.all(np.isfinite(cmd))


def test_allocator_returns_four_bounded_torques():
    allocator = TorqueAllocator.default()
    torques = allocator.allocate(1000.0, 0.0)
    assert torques.shape == (4,)
    assert np.all(np.abs(torques) <= allocator.max_torque + 1e-8)
