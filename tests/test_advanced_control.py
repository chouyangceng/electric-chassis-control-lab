import numpy as np


def test_esc_only_intervenes_when_yaw_error_is_large():
    from electric_chassis_control.controllers.esc import ESCController

    controller = ESCController(yaw_threshold=0.03)
    calm = controller.compute(15.0, 0.10, 0.105, 0.0)
    emergency = controller.compute(15.0, 0.0, 0.25, 0.08)
    assert calm.intervention is False
    assert emergency.intervention is True
    assert calm.brake_pressures.shape == (4,)


def test_abs_reduces_pressure_on_excessive_slip():
    from electric_chassis_control.controllers.abs import ABSController

    controller = ABSController(target_slip=0.15)
    pressure = controller.compute(np.array([20.0, 20.0, 20.0, 20.0]), 15.0, 1000.0)
    assert pressure.shape == (4,)
    assert np.all(pressure <= 1000.0)
    assert np.all(pressure < 1000.0)


def test_regenerative_braking_respects_soc_and_force_balance():
    from electric_chassis_control.allocation.energy import RegenerativeBrakeCoordinator

    coordinator = RegenerativeBrakeCoordinator(max_regen_force=4000.0)
    command = coordinator.allocate(3000.0, battery_soc=0.5)
    assert np.isclose(command.regenerative_force + command.mechanical_force, 3000.0)
    assert command.regenerative_force > 0
    saturated = coordinator.allocate(3000.0, battery_soc=0.99)
    assert saturated.regenerative_force < command.regenerative_force


def test_constrained_allocator_reports_residual():
    from electric_chassis_control.allocation.constrained import ConstrainedTorqueAllocator

    result = ConstrainedTorqueAllocator.default().allocate(1200.0, 300.0, friction=0.7)
    assert result.torques.shape == (4,)
    assert np.all(np.abs(result.torques) <= result.max_torque + 1e-9)
    assert result.residual_norm >= 0.0


def test_stability_envelope_is_monotonic_in_friction():
    from electric_chassis_control.simulation.envelope import compute_stability_envelope

    envelope = compute_stability_envelope(np.linspace(0.2, 1.0, 5), speed=15.0)
    assert envelope.shape == (5,)
    assert np.all(np.diff(envelope) >= -1e-9)
