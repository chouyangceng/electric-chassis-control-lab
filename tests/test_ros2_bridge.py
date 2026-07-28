"""Contract tests for the optional ROS 2 bridge.

The tests intentionally exercise the bridge without requiring a ROS 2 install;
the package provides tiny message fallbacks for CI and algorithm development.
"""

import numpy as np
import pytest

from electric_chassis_control.models.state import ChassisCommand, ChassisState
from ros2.electric_chassis_control_ros.electric_chassis_control_ros.bridge import (
    ROS2_AVAILABLE,
    Ros2CommandBridge,
)


def test_bridge_clips_torque_and_brake_before_publishing() -> None:
    bridge = Ros2CommandBridge(max_torque=100.0, max_brake_pressure=0.8)
    command = ChassisCommand(
        steering=0.12,
        wheel_torques=np.array([250.0, -180.0, 20.0, -1.0]),
        brake_pressures=np.array([1.0, 0.2, 0.9, -0.5]),
        diagnostics={"allocator_residual": 0.1},
    )

    messages = bridge.command_to_messages(command)

    assert np.allclose(messages.torque.data, [100.0, -100.0, 20.0, -1.0])
    assert np.allclose(messages.brake.data, [0.8, 0.2, 0.8, 0.0])
    assert messages.twist.linear.x == pytest.approx(0.12)
    assert messages.diagnostics.status[0].level >= 0


def test_bridge_builds_safe_command_from_standard_twist() -> None:
    bridge = Ros2CommandBridge(max_torque=120.0)
    request = bridge.make_twist(steering=0.05, longitudinal_force=900.0, yaw_moment=500.0, brake=0.3)

    command = bridge.command_from_twist(request)

    assert command.steering == pytest.approx(0.05)
    assert command.wheel_torques.shape == (4,)
    assert np.max(np.abs(command.wheel_torques)) <= 120.0 + 1e-9
    assert np.allclose(command.brake_pressures, 0.3)


def test_bridge_rejects_non_finite_ros_inputs() -> None:
    bridge = Ros2CommandBridge()
    request = bridge.make_twist(steering=float("nan"), longitudinal_force=0.0, yaw_moment=0.0, brake=0.0)

    with pytest.raises(ValueError, match="finite"):
        bridge.command_from_twist(request)


def test_bridge_rejects_non_finite_steering_on_output() -> None:
    bridge = Ros2CommandBridge()
    command = ChassisCommand(
        steering=float("nan"),
        wheel_torques=np.zeros(4),
        brake_pressures=np.zeros(4),
        diagnostics={},
    )

    with pytest.raises(ValueError, match="finite"):
        bridge.command_to_messages(command)


def test_bridge_warns_when_only_brake_pressure_is_clipped() -> None:
    bridge = Ros2CommandBridge(max_brake_pressure=0.8)
    command = ChassisCommand(
        steering=0.0,
        wheel_torques=np.zeros(4),
        brake_pressures=np.full(4, 1.0),
        diagnostics={},
    )

    messages = bridge.command_to_messages(command)

    assert messages.diagnostics.status[0].level == 1
    assert "brake" in messages.diagnostics.status[0].message


def test_state_round_trip_uses_standard_odometry_message() -> None:
    bridge = Ros2CommandBridge()
    state = ChassisState(
        vx=12.0,
        vy=0.4,
        yaw_rate=0.08,
        sideslip=0.02,
        wheel_speeds=np.array([37.0, 37.1, 36.8, 36.9]),
    )

    odometry = bridge.state_to_odometry(state, frame_id="base_link")
    restored = bridge.odometry_to_state(odometry)

    assert odometry.header.frame_id == "base_link"
    assert restored.vx == pytest.approx(state.vx)
    assert restored.vy == pytest.approx(state.vy)
    assert restored.yaw_rate == pytest.approx(state.yaw_rate)
    assert restored.sideslip == pytest.approx(state.sideslip)
    assert np.allclose(restored.wheel_speeds, state.wheel_speeds)


def test_ros_dependency_is_optional() -> None:
    # Importing the bridge is always valid on a plain Python developer machine.
    assert isinstance(ROS2_AVAILABLE, bool)
