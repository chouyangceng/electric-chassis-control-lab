"""ROS 2 node wrapping :class:`Ros2CommandBridge`."""

from __future__ import annotations

from .bridge import ROS2_AVAILABLE, Ros2CommandBridge

try:  # Messages may be importable in a Python shell where rclpy is absent.
    if not ROS2_AVAILABLE:
        raise ImportError
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
except ImportError:  # pragma: no cover - exercised on non-ROS machines
    ROS_NODE_AVAILABLE = False
else:
    ROS_NODE_AVAILABLE = True


if ROS_NODE_AVAILABLE:  # pragma: no cover - requires a sourced ROS 2 environment
    class ControllerNode(Node):
        """Subscribe to a high-level Twist and publish safe wheel commands."""

        def __init__(self) -> None:
            super().__init__("electric_chassis_controller")
            max_torque = float(self.declare_parameter("max_torque", 2500.0).value)
            max_brake = float(self.declare_parameter("max_brake_pressure", 1.0).value)
            self.bridge = Ros2CommandBridge(max_torque=max_torque, max_brake_pressure=max_brake)
            self.torque_pub = self.create_publisher(Float64MultiArray, "~/wheel_torques", 10)
            self.brake_pub = self.create_publisher(Float64MultiArray, "~/brake_pressures", 10)
            self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self.create_subscription(Twist, "~/command", self._on_command, 10)

        def _on_command(self, request: Twist) -> None:
            try:
                command = self.bridge.command_from_twist(request)
                messages = self.bridge.command_to_messages(command)
            except ValueError as exc:
                self.get_logger().error(str(exc))
                return
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
