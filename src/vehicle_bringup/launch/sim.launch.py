from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    path_profile = LaunchConfiguration('path_profile')
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('vehicle_bringup'), 'config', 'ekf_sim.rviz']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('path_profile', default_value='circle'),
            DeclareLaunchArgument('gps_outage_start', default_value='-1.0'),
            DeclareLaunchArgument('gps_outage_duration', default_value='0.0'),
            Node(
                package='vehicle_sim',
                executable='simulator_node',
                name='vehicle_simulator',
                parameters=[
                    {
                        'path_profile': path_profile,
                        'gps_outage_start': LaunchConfiguration('gps_outage_start'),
                        'gps_outage_duration': LaunchConfiguration('gps_outage_duration'),
                    }
                ],
                output='screen',
            ),
            Node(
                package='ekf_localization',
                executable='ekf_localization_node',
                name='ekf_localization',
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
            ),
        ]
    )
