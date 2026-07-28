from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="electric_chassis_control_ros",
                executable="controller_node",
                name="electric_chassis_controller",
                output="screen",
                parameters=[{"max_torque": 2500.0, "max_brake_pressure": 1.0}],
            )
        ]
    )
