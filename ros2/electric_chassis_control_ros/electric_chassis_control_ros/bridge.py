"""Message conversion and safety boundary for the ROS 2 adapter."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from electric_chassis_control.allocation.constrained import ConstrainedTorqueAllocator
from electric_chassis_control.models.state import ChassisCommand, ChassisState

try:  # pragma: no cover - exercised only in a sourced ROS 2 environment
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64MultiArray

    ROS2_AVAILABLE = True
except ImportError:  # pragma: no cover - fallback is covered by project CI
    ROS2_AVAILABLE = False

    @dataclass
    class _Vector3:
        x: float = 0.0
        y: float = 0.0
        z: float = 0.0

    @dataclass
    class Twist:
        linear: _Vector3 = field(default_factory=_Vector3)
        angular: _Vector3 = field(default_factory=_Vector3)

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
    class _TwistWithCovariance:
        twist: Twist = field(default_factory=Twist)
        covariance: list[float] = field(default_factory=lambda: [0.0] * 36)

    @dataclass
    class Odometry:
        header: _Header = field(default_factory=_Header)
        child_frame_id: str = ""
        twist: _TwistWithCovariance = field(default_factory=_TwistWithCovariance)


@dataclass(frozen=True)
class BridgeMessages:
    """ROS messages emitted for one safe chassis command."""

    twist: Twist
    torque: Float64MultiArray
    brake: Float64MultiArray
    diagnostics: DiagnosticArray


class Ros2CommandBridge:
    """Convert standard ROS messages while enforcing actuator limits.

    ``Twist`` request convention: ``linear.x`` steering [rad], ``linear.y``
    longitudinal force [N], ``linear.z`` uniform brake pressure [0, 1], and
    ``angular.z`` requested yaw moment [N m].
    """

    def __init__(
        self,
        *,
        max_torque: float = 2500.0,
        max_brake_pressure: float = 1.0,
        allocator: ConstrainedTorqueAllocator | None = None,
    ) -> None:
        if not np.isfinite([max_torque, max_brake_pressure]).all() or max_torque <= 0:
            raise ValueError("max_torque must be finite and positive")
        if max_brake_pressure <= 0:
            raise ValueError("max_brake_pressure must be finite and positive")
        self.max_torque = float(max_torque)
        self.max_brake_pressure = float(max_brake_pressure)
        self.allocator = allocator or ConstrainedTorqueAllocator(max_torque=self.max_torque)

    @staticmethod
    def make_twist(*, steering: float, longitudinal_force: float, yaw_moment: float, brake: float) -> Twist:
        """Create a standard ``Twist`` request using the documented convention."""
        msg = Twist()
        msg.linear.x = float(steering)
        msg.linear.y = float(longitudinal_force)
        msg.linear.z = float(brake)
        msg.angular.z = float(yaw_moment)
        return msg

    def command_from_twist(self, request: Twist) -> ChassisCommand:
        """Allocate a safe four-wheel command from a standard ROS request."""
        values = np.asarray(
            [request.linear.x, request.linear.y, request.linear.z, request.angular.z], dtype=float
        )
        if not np.all(np.isfinite(values)):
            raise ValueError("ROS command fields must be finite")
        brake = float(np.clip(request.linear.z, 0.0, self.max_brake_pressure))
        result = self.allocator.allocate(request.linear.y, request.angular.z)
        torques = np.clip(np.asarray(result.torques, dtype=float), -self.max_torque, self.max_torque)
        return ChassisCommand(
            steering=float(request.linear.x),
            wheel_torques=torques,
            brake_pressures=np.full(4, brake),
            diagnostics={"allocator_residual": float(result.residual_norm)},
        )

    def command_to_messages(self, command: ChassisCommand) -> BridgeMessages:
        """Convert a command to standard ROS messages after safety clipping."""
        torques = np.asarray(command.wheel_torques, dtype=float)
        brakes = np.asarray(command.brake_pressures, dtype=float)
        if not np.isfinite(command.steering):
            raise ValueError("steering command must be finite")
        if torques.shape != (4,) or brakes.shape != (4,) or not np.isfinite(torques).all():
            raise ValueError("wheel command arrays must contain four finite values")
        if not np.isfinite(brakes).all():
            raise ValueError("wheel command arrays must contain finite values")
        torque_clipped = bool(np.any(np.abs(torques) > self.max_torque))
        brake_clipped = bool(np.any((brakes < 0.0) | (brakes > self.max_brake_pressure)))
        torques = np.clip(torques, -self.max_torque, self.max_torque)
        brakes = np.clip(brakes, 0.0, self.max_brake_pressure)

        twist = Twist()
        twist.linear.x = float(command.steering)
        twist.linear.y = float(np.sum(torques))
        twist.angular.z = float(torques[1] + torques[3] - torques[0] - torques[2])
        torque_msg = Float64MultiArray()
        torque_msg.data = torques.tolist()
        brake_msg = Float64MultiArray()
        brake_msg.data = brakes.tolist()

        residual = float(command.diagnostics.get("allocator_residual", 0.0))
        status = DiagnosticStatus()
        status.name = "electric_chassis_control/actuator_limits"
        status.level = DiagnosticStatus.WARN if torque_clipped or brake_clipped else DiagnosticStatus.OK
        clipped = []
        if torque_clipped:
            clipped.append("torque")
        if brake_clipped:
            clipped.append("brake")
        status.message = f"{' and '.join(clipped)} command clipped" if clipped else "command within limits"
        status.values = [KeyValue(key="allocator_residual", value=f"{residual:.6g}")]
        diagnostics = DiagnosticArray()
        diagnostics.status = [status]
        return BridgeMessages(twist=twist, torque=torque_msg, brake=brake_msg, diagnostics=diagnostics)

    def state_to_odometry(self, state: ChassisState, *, frame_id: str = "base_link") -> Odometry:
        """Encode a chassis state in standard ``nav_msgs/msg/Odometry``.

        Linear velocity and yaw rate use their conventional fields.  Sideslip
        is carried in ``twist.twist.angular.x`` and four wheel speeds occupy the
        first four entries of the twist covariance array, preserving the full
        research state without introducing a custom message type.
        """
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError("frame_id must be a non-empty string")
        odometry = Odometry()
        odometry.header.frame_id = frame_id
        twist = odometry.twist.twist
        twist.linear.x = float(state.vx)
        twist.linear.y = float(state.vy)
        twist.angular.x = float(state.sideslip)
        twist.angular.z = float(state.yaw_rate)
        covariance = np.zeros(36, dtype=float)
        covariance[:4] = np.asarray(state.wheel_speeds, dtype=float)
        odometry.twist.covariance = covariance.tolist()
        return odometry

    @staticmethod
    def odometry_to_state(odometry: Odometry) -> ChassisState:
        """Decode the documented ``Odometry`` representation into ``ChassisState``."""
        twist = odometry.twist.twist
        covariance = np.asarray(odometry.twist.covariance, dtype=float)
        if covariance.shape != (36,) or not np.all(np.isfinite(covariance[:4])):
            raise ValueError("Odometry twist covariance must contain four finite wheel speeds")
        values = np.asarray([twist.linear.x, twist.linear.y, twist.angular.z, twist.angular.x], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("Odometry twist fields must be finite")
        return ChassisState(
            vx=float(twist.linear.x),
            vy=float(twist.linear.y),
            yaw_rate=float(twist.angular.z),
            sideslip=float(twist.angular.x),
            wheel_speeds=covariance[:4].copy(),
        )
