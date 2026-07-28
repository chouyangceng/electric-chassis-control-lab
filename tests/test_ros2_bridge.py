"""Contract tests for the optional ROS 2 bridge.

These tests run without ROS 2 so the safety boundary remains testable in CI.
"""

from pathlib import Path

import numpy as np
import pytest

from electric_chassis_control.models.state import ChassisCommand, ChassisState
from ros2.electric_chassis_control_ros.electric_chassis_control_ros import bridge as bridge_module

ROS2_AVAILABLE = bridge_module.ROS2_AVAILABLE
Ros2CommandBridge = bridge_module.Ros2CommandBridge


def _command(*, steering: float = 0.1, torque: float = 20.0, brake: float = 0.2) -> ChassisCommand:
    return ChassisCommand(
        steering=steering,
        wheel_torques=np.full(4, torque),
        brake_pressures=np.full(4, brake),
        diagnostics={},
    )


def test_bridge_clips_actuators_and_keeps_output_messages_semantic() -> None:
    bridge = Ros2CommandBridge(max_torque=100.0, max_brake_pressure=0.8)
    command = ChassisCommand(
        steering=0.12,
        wheel_torques=np.array([250.0, -180.0, 20.0, -1.0]),
        brake_pressures=np.array([1.0, 0.2, 0.9, -0.5]),
        diagnostics={"allocator_residual": 0.0},
    )

    messages = bridge.command_to_messages(command)

    assert np.allclose(messages.torque.data, [100.0, -100.0, 20.0, -1.0])
    assert np.allclose(messages.brake.data, [0.8, 0.2, 0.8, 0.0])
    assert messages.steering.data == pytest.approx(0.12)
    assert not hasattr(messages, "twist")
    assert messages.diagnostics.status[0].level == 1


def test_bridge_builds_command_from_wrench_and_scalar_inputs() -> None:
    bridge = Ros2CommandBridge(max_torque=120.0)
    wrench = bridge.make_wrench(longitudinal_force=900.0, yaw_moment=500.0)

    command = bridge.command_from_messages(wrench, steering=0.05, brake=0.3)

    assert wrench.force.x == pytest.approx(900.0)
    assert wrench.torque.z == pytest.approx(500.0)
    assert command.steering == pytest.approx(0.05)
    assert command.wheel_torques.shape == (4,)
    assert np.max(np.abs(command.wheel_torques)) <= 120.0 + 1e-9
    assert np.allclose(command.brake_pressures, 0.3)


def test_bridge_rejects_non_finite_ros_inputs() -> None:
    bridge = Ros2CommandBridge()
    wrench = bridge.make_wrench(longitudinal_force=float("nan"), yaw_moment=0.0)

    with pytest.raises(ValueError, match="finite"):
        bridge.command_from_messages(wrench, steering=0.0, brake=0.0)


@pytest.mark.parametrize("maximum", [1.01, 2.0, float("inf")])
def test_bridge_rejects_brake_limit_above_physical_range(maximum: float) -> None:
    with pytest.raises(ValueError, match=r"\(0, 1\]"):
        Ros2CommandBridge(max_brake_pressure=maximum)


def test_bridge_warns_on_allocator_residual_or_saturation() -> None:
    bridge = Ros2CommandBridge(max_torque=50.0, residual_warn_threshold=1.0)
    wrench = bridge.make_wrench(longitudinal_force=100_000.0, yaw_moment=20_000.0)
    command = bridge.command_from_messages(wrench, steering=0.0, brake=0.0)

    messages = bridge.command_to_messages(command)

    status = messages.diagnostics.status[0]
    assert status.level == 1
    assert "allocator" in status.message
    assert any(value.key == "allocator_saturated" and value.value == "true" for value in status.values)


def test_watchdog_returns_deterministic_failsafe_for_stale_or_rejected_command() -> None:
    watchdog = bridge_module.CommandWatchdog(timeout_s=0.2, safe_brake_pressure=0.7)
    watchdog.accept(_command(), timestamp=10.0)

    active = watchdog.evaluate(timestamp=10.1)
    assert not active.is_failsafe
    assert np.allclose(active.command.wheel_torques, 20.0)

    stale = watchdog.evaluate(timestamp=10.21)
    assert stale.is_failsafe
    assert stale.reason == "command timeout"
    assert np.allclose(stale.command.wheel_torques, 0.0)
    assert np.allclose(stale.command.brake_pressures, 0.7)
    assert stale.command.steering == 0.0

    watchdog.accept(_command(), timestamp=11.0)
    watchdog.reject("malformed command")
    rejected = watchdog.evaluate(timestamp=11.0)
    assert rejected.is_failsafe
    assert rejected.reason == "malformed command"
    assert np.allclose(rejected.command.wheel_torques, 0.0)
    assert np.allclose(rejected.command.brake_pressures, 0.7)
    assert rejected.command.steering == 0.0
    rejected_messages = Ros2CommandBridge().command_to_messages(
        rejected.command, failsafe_reason=rejected.reason
    )
    assert rejected_messages.steering.data == 0.0


def test_command_input_cache_requires_fresh_complete_set_after_clear() -> None:
    cache = bridge_module.CommandInputCache()
    wrench = bridge_module.Wrench()
    cache.update("wrench", wrench, timestamp=1.0)
    cache.update("steering", 0.1, timestamp=1.0)
    cache.update("brake", 0.2, timestamp=1.0)
    assert cache.complete(now=1.1, timeout_s=0.2) is not None

    cache.clear()
    assert cache.complete(now=1.1, timeout_s=0.2) is None
    cache.update("steering", 0.4, timestamp=1.2)
    assert cache.complete(now=1.2, timeout_s=0.2) is None


def test_state_messages_keep_odometry_covariance_and_wheel_speeds_separate() -> None:
    bridge = Ros2CommandBridge()
    state = ChassisState(
        vx=12.0,
        vy=0.4,
        yaw_rate=0.08,
        sideslip=0.02,
        wheel_speeds=np.array([37.0, 37.1, 36.8, 36.9]),
    )

    messages = bridge.state_to_messages(state, timestamp_s=12.345)
    restored = bridge.messages_to_state(messages.odometry, messages.wheel_speeds, messages.sideslip)

    assert messages.odometry.header.frame_id == "odom"
    assert messages.odometry.child_frame_id == "base_link"
    assert messages.odometry.header.stamp.sec == 12
    assert messages.odometry.header.stamp.nanosec == 345_000_000
    assert np.allclose(messages.odometry.twist.covariance, 0.0)
    assert np.allclose(messages.wheel_speeds.data, state.wheel_speeds)
    assert messages.sideslip.data == pytest.approx(state.sideslip)
    assert restored.vx == pytest.approx(state.vx)
    assert restored.vy == pytest.approx(state.vy)
    assert restored.yaw_rate == pytest.approx(state.yaw_rate)
    assert restored.sideslip == pytest.approx(state.sideslip)
    assert np.allclose(restored.wheel_speeds, state.wheel_speeds)


def test_launch_loads_yaml_and_manifest_declares_launch_runtime_dependencies() -> None:
    root = Path(__file__).parents[1]
    launch_source = (root / "ros2/electric_chassis_control_ros/launch/controller.launch.py").read_text(
        encoding="utf-8"
    )
    manifest = (root / "ros2/electric_chassis_control_ros/package.xml").read_text(encoding="utf-8")
    setup_cfg = (root / "ros2/electric_chassis_control_ros/setup.cfg").read_text(encoding="utf-8")

    assert "controller.yaml" in launch_source
    assert "get_package_share_directory" in launch_source
    assert "<exec_depend>launch</exec_depend>" in manifest
    assert "<exec_depend>launch_ros</exec_depend>" in manifest
    assert "script_dir=$base/lib/electric_chassis_control_ros" in setup_cfg
    assert "install_scripts=$base/lib/electric_chassis_control_ros" in setup_cfg


def test_ros_dependency_is_optional() -> None:
    assert isinstance(ROS2_AVAILABLE, bool)
