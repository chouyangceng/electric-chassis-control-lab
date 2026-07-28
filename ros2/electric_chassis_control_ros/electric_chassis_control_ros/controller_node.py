"""ROS 2 node wrapping the safe electric chassis command bridge."""

from __future__ import annotations

import math
import time

from .bridge import ROS2_AVAILABLE, CommandWatchdog, Ros2CommandBridge, WatchdogDecision

try:  # Messages may be importable where rclpy is absent.
    if not ROS2_AVAILABLE:
        raise ImportError
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Wrench
    from rclpy.node import Node
    from std_msgs.msg import Float64, Float64MultiArray
except ImportError:  # pragma: no cover - exercised on non-ROS machines
    ROS_NODE_AVAILABLE = False
else:
    ROS_NODE_AVAILABLE = True


if ROS_NODE_AVAILABLE:  # pragma: no cover - requires a sourced ROS 2 environment
    class ControllerNode(Node):
        """Allocate wheel commands and fail safely when command inputs are invalid or stale."""

        def __init__(self) -> None:
            super().__init__("electric_chassis_controller")
            max_torque = float(self.declare_parameter("max_torque", 2500.0).value)
            max_brake = float(self.declare_parameter("max_brake_pressure", 1.0).value)
            safe_brake = float(self.declare_parameter("safe_brake_pressure", 0.7).value)
            timeout_s = float(self.declare_parameter("command_timeout_s", 0.25).value)
            period_s = float(self.declare_parameter("watchdog_period_s", 0.05).value)
            residual_threshold = float(
                self.declare_parameter("allocator_residual_warn_threshold", 1.0).value
            )
            if safe_brake > max_brake:
                raise ValueError("safe_brake_pressure cannot exceed max_brake_pressure")
            if not math.isfinite(period_s) or period_s <= 0:
                raise ValueError("watchdog_period_s must be finite and positive")

            self.bridge = Ros2CommandBridge(
                max_torque=max_torque,
                max_brake_pressure=max_brake,
                residual_warn_threshold=residual_threshold,
            )
            self.watchdog = CommandWatchdog(
                timeout_s=timeout_s,
                safe_brake_pressure=safe_brake,
            )
            self._wrench: Wrench | None = None
            self._steering: float | None = None
            self._brake: float | None = None
            self._input_times: dict[str, float] = {}

            self.torque_pub = self.create_publisher(Float64MultiArray, "~/wheel_torques", 10)
            self.brake_pub = self.create_publisher(Float64MultiArray, "~/brake_pressures", 10)
            self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self.create_subscription(Wrench, "~/force_request", self._on_wrench, 10)
            self.create_subscription(Float64, "~/steering_angle", self._on_steering, 10)
            self.create_subscription(Float64, "~/brake_request", self._on_brake, 10)
            self.create_timer(period_s, self._on_watchdog)

        def _on_wrench(self, request: Wrench) -> None:
            if not math.isfinite(request.force.x) or not math.isfinite(request.torque.z):
                self._reject_and_publish("malformed force request")
                return
            self._wrench = request
            self._input_times["wrench"] = time.monotonic()
            self._try_accept_and_publish()

        def _on_steering(self, request: Float64) -> None:
            if not math.isfinite(request.data):
                self._reject_and_publish("malformed steering request")
                return
            self._steering = float(request.data)
            self._input_times["steering"] = time.monotonic()
            self._try_accept_and_publish()

        def _on_brake(self, request: Float64) -> None:
            if not math.isfinite(request.data):
                self._reject_and_publish("malformed brake request")
                return
            self._brake = float(request.data)
            self._input_times["brake"] = time.monotonic()
            self._try_accept_and_publish()

        def _try_accept_and_publish(self) -> None:
            if self._wrench is None or self._steering is None or self._brake is None:
                return
            now = time.monotonic()
            if len(self._input_times) != 3 or now - min(self._input_times.values()) > self.watchdog.timeout_s:
                return
            try:
                command = self.bridge.command_from_messages(
                    self._wrench,
                    steering=self._steering,
                    brake=self._brake,
                )
            except ValueError as exc:
                self._reject_and_publish(f"malformed command: {exc}")
                return
            self.watchdog.accept(command, timestamp=now)
            self._publish(self.watchdog.evaluate(timestamp=now))

        def _reject_and_publish(self, reason: str) -> None:
            self.get_logger().error(reason)
            self.watchdog.reject(reason)
            self._publish(self.watchdog.evaluate(timestamp=time.monotonic()))

        def _on_watchdog(self) -> None:
            self._publish(self.watchdog.evaluate(timestamp=time.monotonic()))

        def _publish(self, decision: WatchdogDecision) -> None:
            messages = self.bridge.command_to_messages(
                decision.command,
                failsafe_reason=decision.reason if decision.is_failsafe else None,
            )
            self.torque_pub.publish(messages.torque)
            self.brake_pub.publish(messages.brake)
            self.diag_pub.publish(messages.diagnostics)

else:

    class ControllerNode:  # pragma: no cover - trivial guard
        """Placeholder that provides an actionable error without ROS 2."""

        def __init__(self, *_: object, **__: object) -> None:
            raise RuntimeError("ROS 2 is not installed; source a ROS 2 environment to run ControllerNode")


def main(args: list[str] | None = None) -> int:
    """Run the node when ROS 2 is available."""
    if not ROS_NODE_AVAILABLE:
        raise RuntimeError("ROS 2 is not installed; install rclpy and source your ROS 2 workspace")
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":  # pragma: no cover
    main()
