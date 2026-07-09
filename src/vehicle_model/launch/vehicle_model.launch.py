from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='vehicle_model',
                executable='vehicle_model',
                name='vehicle_model',
                output='screen',
            ),
        ]
    )
