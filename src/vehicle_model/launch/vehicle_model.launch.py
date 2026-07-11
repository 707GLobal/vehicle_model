from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('wheel_base', default_value='1.53'),
            DeclareLaunchArgument('max_steer_angle', default_value='25.0'),
            DeclareLaunchArgument('dt', default_value='0.02'),
            DeclareLaunchArgument('start_x', default_value='0.0'),
            DeclareLaunchArgument('start_y', default_value='0.0'),
            DeclareLaunchArgument('start_yaw', default_value='0.0'),
            Node(
                package='vehicle_model',
                executable='vehicle_model',
                name='vehicle_model',
                output='screen',
                parameters=[{
                    'wheel_base': ParameterValue(
                        LaunchConfiguration('wheel_base'), value_type=float),
                    'max_steer_angle': ParameterValue(
                        LaunchConfiguration('max_steer_angle'), value_type=float),
                    'dt': ParameterValue(
                        LaunchConfiguration('dt'), value_type=float),
                    'start_x': ParameterValue(
                        LaunchConfiguration('start_x'), value_type=float),
                    'start_y': ParameterValue(
                        LaunchConfiguration('start_y'), value_type=float),
                    'start_yaw': ParameterValue(
                        LaunchConfiguration('start_yaw'), value_type=float),
                }],
            ),
        ]
    )
