from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('gps_noise_std', default_value='2.0'),
            Node(
                package='ekf_localization',
                executable='ekf_localization_node',
                name='ekf_localization',
                parameters=[
                    {
                        'gps_noise_std': LaunchConfiguration('gps_noise_std'),
                        'process_noise': [0.01, 0.01, 0.001, 0.05],
                    }
                ],
                output='screen',
            ),
        ]
    )
