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
            # 动力学参数
            DeclareLaunchArgument('mass', default_value='250.0'),
            DeclareLaunchArgument('iz', default_value='300.0'),
            DeclareLaunchArgument('lf', default_value='0.77'),
            DeclareLaunchArgument('cf', default_value='200000.0'),
            DeclareLaunchArgument('cr', default_value='200000.0'),
            DeclareLaunchArgument('friction_coeff', default_value='1.2'),
            DeclareLaunchArgument('tau_steer', default_value='0.1'),
            DeclareLaunchArgument('speed_gain', default_value='2.0'),
            DeclareLaunchArgument('max_accel', default_value='3.0'),
            DeclareLaunchArgument('max_decel', default_value='5.0'),
            DeclareLaunchArgument('v_kin_thresh', default_value='1.0'),
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
                    'mass': ParameterValue(
                        LaunchConfiguration('mass'), value_type=float),
                    'iz': ParameterValue(
                        LaunchConfiguration('iz'), value_type=float),
                    'lf': ParameterValue(
                        LaunchConfiguration('lf'), value_type=float),
                    'cf': ParameterValue(
                        LaunchConfiguration('cf'), value_type=float),
                    'cr': ParameterValue(
                        LaunchConfiguration('cr'), value_type=float),
                    'friction_coeff': ParameterValue(
                        LaunchConfiguration('friction_coeff'), value_type=float),
                    'tau_steer': ParameterValue(
                        LaunchConfiguration('tau_steer'), value_type=float),
                    'speed_gain': ParameterValue(
                        LaunchConfiguration('speed_gain'), value_type=float),
                    'max_accel': ParameterValue(
                        LaunchConfiguration('max_accel'), value_type=float),
                    'max_decel': ParameterValue(
                        LaunchConfiguration('max_decel'), value_type=float),
                    'v_kin_thresh': ParameterValue(
                        LaunchConfiguration('v_kin_thresh'), value_type=float),
                }],
            ),
        ]
    )
