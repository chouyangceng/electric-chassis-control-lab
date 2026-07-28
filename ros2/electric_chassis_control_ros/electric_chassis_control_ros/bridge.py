"""ROS-independent message conversion and safety boundary for the ROS 2 adapter."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from electric_chassis_control.allocation.constrained import ConstrainedTorqueAllocator
from electric_chassis_control.models.state import ChassisCommand, ChassisState

try:  # pragma: no cover - exercised only in a sourced ROS 2 environment
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Wrench
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64, Float64MultiArray

    ROS2_AVAILABLE = True
except ImportError:  # pragma: no cover - fallback classes are covered by project CI
    ROS2_AVAILABLE = False

    @dataclass
    class _Vector3:
        x: float = 0.0
        y: float = 0.0
        z: float = 0.0

    @dataclass
    class Wrench:
        force: _Vector3 = field(default_factory=_Vector3)
        torque: _Vector3 = field(default_factory=_Vector3)

    @dataclass
    class Float64:
        data: float = 0.0

    @dataclass
    class Float64MultiArray:
        data: list[float] = field(default_factory=list)

    @dataclass
    class KeyValue:
        key: str = ""
        value: str = ""

    @dataclass
    class DiagnosticStatus:
        OK: int = 0
        WARN: int = 1
        ERROR: int = 2
        level: int = 0
        name: str = ""
        message: str = ""
        values: list[KeyValue] = field(default_factory=list)

    @dataclass
    class DiagnosticArray:
        status: list[DiagnosticStatus] = field(default_factory=list)

    @dataclass
    class _Header:
        frame_id: str = ""

    @dataclass
    class _Twist:
        linear: _Vector3 = field(default_factory=_Vector3)
        angular: _Vector3 = field(default_factory=_Vector3)

    @dataclass
    class _TwistWithCovariance:
        twist: _Twist = field(default_factory=_Twist)
        covariance: list[float] = field(default_factory=lambda: [0.0] * 36)

    @dataclass
    class Odometry:
        header: _Header = field(default_factory=_Header)
        child_frame_id: str = ""
        twist: _TwistWithCovariance = field(default_factory=_TwistWithCovariance)


@dataclass(frozen=True)
class BridgeMessages:
    """Actuator and diagnostic messages emitted for one chassis command."""

    torque: Float64MultiArray
    brake: Float64MultiArray
    diagnostics: DiagnosticArray


@dataclass(frozen=True)
class StateMessages:
    """Standard odometry plus explicit non-odometry chassis state topics."""

    odometry: Odometry
    wheel_speeds: Float64MultiArray
    sideslip: Float64


@dataclass(frozen=True)
class WatchdogDecision:
    """Result of evaluating the most recently accepted actuator command."""

    command: ChassisCommand
    is_failsafe: bool
    reason: str


class CommandWatchdog:
    """Pure-Python command timeout and reject latch.

    The helper deliberately has no ROS clock or node dependency, so timeout and
    malformed-command behaviour can be tested deterministically.
    """

    def __init__(self, *, timeout_s: float, safe_brake_pressure: float) -> None:
        if not np.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be finite and positive")
        if not np.isfinite(safe_brake_pressure) or not 0.0 <= safe_brake_pressure <= 1.0:
            raise ValueError("safe_brake_pressure must be within [0, 1]")
        self.timeout_s = float(timeout_s)
        self.safe_brake_pressure = float(safe_brake_pressure)
        self._command: ChassisCommand | None = None
        self._accepted_at: float | None = None
        self._rejection_reason: str | None = "no command received"

    def accept(self, command: ChassisCommand, *, timestamp: float) -> None:
        if not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        self._command = command
        self._accepted_at = float(timestamp)
        self._rejection_reason = None

    def reject(self, reason: str) -> None:
        self._rejection_reason = reason.strip() or "malformed command"

    def evaluate(self, *, timestamp: float) -> WatchdogDecision:
        if not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if self._rejection_reason is not None:
            return self._failsafe(self._rejection_reason)
        if self._command is None or self._accepted_at is None:
            return self._failsafe("no command received")
        if float(timestamp) - self._accepted_at > self.timeout_s:
            return self._failsafe("command timeout")
        return WatchdogDecision(self._command, False, "command active")

    def _failsafe(self, reason: str) -> WatchdogDecision:
        command = ChassisCommand(
            steering=0.0,
            wheel_torques=np.zeros(4),
            brake_pressures=np.full(4, self.safe_brake_pressure),
            diagnostics={"failsafe": 1.0},
        )
        return WatchdogDecision(command, True, reason)


class Ros2CommandBridge:
    """Convert semantically correct ROS messages while enforcing actuator limits.

    A ``Wrench`` carries longitudinal force in ``force.x`` [N] and requested
    yaw moment in ``torque.z`` [N m]. Steering [rad] and normalized brake
    pressure are separate scalar topics.
    """

    def __init__(
        self,
        *,
        max_torque: float = 2500.0,
        max_brake_pressure: float = 1.0,
        residual_warn_threshold: float = 1.0,
        allocator: ConstrainedTorqueAllocator | None = None,
    ) -> None:
        if not np.isfinite(max_torque) or max_torque <= 0:
            raise ValueError("max_torque must be finite and positive")
        if not np.isfinite(max_brake_pressure) or not 0.0 < max_brake_pressure <= 1.0:
            raise ValueError("max_brake_pressure must be within (0, 1]")
        if not np.isfinite(residual_warn_threshold) or residual_warn_threshold < 0:
            raise ValueError("residual_warn_threshold must be finite and non-negative")
        self.max_torque = float(max_torque)
        self.max_brake_pressure = float(max_brake_pressure)
        self.residual_warn_threshold = float(residual_warn_threshold)
        self.allocator = allocator or ConstrainedTorqueAllocator(max_torque=self.max_torque)

    @staticmethod
    def make_wrench(*, longitudinal_force: float, yaw_moment: float) -> Wrench:
        """Create a request with standard force and torque semantics."""
        message = Wrench()
        message.force.x = float(longitudinal_force)
        message.torque.z = float(yaw_moment)
        return message

    def command_from_messages(self, request: Wrench, *, steering: float, brake: float) -> ChassisCommand:
        """Allocate a safe four-wheel command from explicit high-level inputs."""
        values = np.asarray([request.force.x, request.torque.z, steering, brake], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("ROS command fields must be finite")
        bounded_brake = float(np.clip(brake, 0.0, self.max_brake_pressure))
        result = self.allocator.allocate(float(request.force.x), float(request.torque.z))
        torques = np.clip(np.asarray(result.torques, dtype=float), -self.max_torque, self.max_torque)
        saturated = bool(result.residual_norm > self.residual_warn_threshold)
        return ChassisCommand(
            steering=float(steering),
            wheel_torques=torques,
            brake_pressures=np.full(4, bounded_brake),
            diagnostics={
                "allocator_residual": float(result.residual_norm),
                "allocator_saturated": float(saturated),
            },
        )

    def command_to_messages(
        self, command: ChassisCommand, *, failsafe_reason: str | None = None
    ) -> BridgeMessages:
        """Convert a command to actuator messages after safety clipping."""
        torques = np.asarray(command.wheel_torques, dtype=float)
        brakes = np.asarray(command.brake_pressures, dtype=float)
        if not np.isfinite(command.steering):
            raise ValueError("steering command must be finite")
        if torques.shape != (4,) or brakes.shape != (4,):
            raise ValueError("wheel command arrays must contain four values")
        if not np.isfinite(torques).all() or not np.isfinite(brakes).all():
            raise ValueError("wheel command arrays must contain finite values")
        torque_clipped = bool(np.any(np.abs(torques) > self.max_torque))
        brake_clipped = bool(np.any((brakes < 0.0) | (brakes > self.max_brake_pressure)))
        torques = np.clip(torques, -self.max_torque, self.max_torque)
        brakes = np.clip(brakes, 0.0, self.max_brake_pressure)

        torque_message = Float64MultiArray()
        torque_message.data = torques.tolist()
        brake_message = Float64MultiArray()
        brake_message.data = brakes.tolist()

        residual = float(command.diagnostics.get("allocator_residual", 0.0))
        allocator_saturated = bool(command.diagnostics.get("allocator_saturated", 0.0))
        allocator_warning = allocator_saturated or residual > self.residual_warn_threshold
        warnings = []
        if torque_clipped:
            warnings.append("torque clipped")
        if brake_clipped:
            warnings.append("brake clipped")
        if allocator_warning:
            warnings.append("allocator residual/saturation")

        status = DiagnosticStatus()
        status.name = "electric_chassis_control/command_safety"
        if failsafe_reason is not None:
            status.level = DiagnosticStatus.ERROR
            status.message = f"failsafe active: {failsafe_reason}"
        elif warnings:
            status.level = DiagnosticStatus.WARN
            status.message = "; ".join(warnings)
        else:
            status.level = DiagnosticStatus.OK
            status.message = "command within limits"
        status.values = [
            KeyValue(key="allocator_residual", value=f"{residual:.6g}"),
            KeyValue(key="allocator_saturated", value=str(allocator_warning).lower()),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.status = [status]
        return BridgeMessages(torque=torque_message, brake=brake_message, diagnostics=diagnostics)

    def state_to_messages(self, state: ChassisState, *, frame_id: str = "base_link") -> StateMessages:
        """Encode state without overloading odometry covariance or velocity fields."""
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError("frame_id must be a non-empty string")
        odometry = Odometry()
        odometry.header.frame_id = frame_id
        odometry.child_frame_id = frame_id
        odometry.twist.twist.linear.x = float(state.vx)
        odometry.twist.twist.linear.y = float(state.vy)
        odometry.twist.twist.angular.z = float(state.yaw_rate)

        wheel_speeds = Float64MultiArray()
        wheel_speeds.data = np.asarray(state.wheel_speeds, dtype=float).tolist()
        sideslip = Float64()
        sideslip.data = float(state.sideslip)
        return StateMessages(odometry, wheel_speeds, sideslip)

    @staticmethod
    def messages_to_state(
        odometry: Odometry, wheel_speeds: Float64MultiArray, sideslip: Float64
    ) -> ChassisState:
        """Decode odometry and the explicit wheel-speed/sideslip topics."""
        twist = odometry.twist.twist
        wheels = np.asarray(wheel_speeds.data, dtype=float)
        values = np.asarray([twist.linear.x, twist.linear.y, twist.angular.z, sideslip.data], dtype=float)
        if wheels.shape != (4,) or not np.all(np.isfinite(wheels)):
            raise ValueError("wheel speed message must contain four finite values")
        if not np.all(np.isfinite(values)):
            raise ValueError("state message fields must be finite")
        return ChassisState(
            vx=float(twist.linear.x),
            vy=float(twist.linear.y),
            yaw_rate=float(twist.angular.z),
            sideslip=float(sideslip.data),
            wheel_speeds=wheels.copy(),
        )
