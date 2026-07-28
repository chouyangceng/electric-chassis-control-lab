from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("electric_chassis_control_ros")) / "config" / "controller.yaml"
    return LaunchDescription(
        [
            Node(
                package="electric_chassis_control_ros",
                executable="controller_node",
                name="electric_chassis_controller",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )
