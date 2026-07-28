import numpy as np

from electric_chassis_control.models.vehicle7dof import Vehicle7DOF


def test_four_wheel_torque_is_clipped_to_limits():
    model = Vehicle7DOF.default()
    result = model.clip_torques(np.array([5000.0, -5000.0, 100.0, -100.0]))
    assert np.all(np.abs(result) <= model.max_torque)


def test_tire_force_respects_friction_circle():
    model = Vehicle7DOF.default()
    fx, fy = model.tire_force(0.5, 0.8, 3500.0, 0.7)
    assert np.hypot(fx, fy) <= 0.7 * 3500.0 + 1e-8
