from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('vehicle_bringup'), 'config', 'ekf_sim.rviz']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('gps_outage_start', default_value='30.0'),
            DeclareLaunchArgument('gps_outage_duration', default_value='15.0'),
            DeclareLaunchArgument('gps_noise_std', default_value='3.0'),
            DeclareLaunchArgument('output_csv', default_value='ekf_error_log.csv'),
            Node(
                package='vehicle_sim',
                executable='simulator_node',
                name='vehicle_simulator',
                parameters=[
                    {
                        'path_profile': 'figure8',
                        'gps_position_std': LaunchConfiguration('gps_noise_std'),
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
                parameters=[
                    {
                        'gps_noise_std': LaunchConfiguration('gps_noise_std'),
                        'use_imu_accel': True,
                        'fuse_gps_yaw': True,
                        'process_noise': [0.02, 0.02, 0.002, 0.08],
                    }
                ],
                output='screen',
            ),
            Node(
                package='vehicle_sim',
                executable='error_logger_node',
                name='error_logger',
                parameters=[{'output_csv': LaunchConfiguration('output_csv')}],
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
