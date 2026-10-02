import os
import signal
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def _stop_previous_demo() -> None:
    """Kill leftover demo processes so RViz does not keep an older path."""
    needles = (
        'odom_ground_truth_node',
        'ekf_localization_node',
        'gazebo_fake_gps_node',
        'cmd_vel_demo_node',
        'error_logger_node',
        'delete_gazebo_entity_node',
        'gzserver',
        'gzclient',
        'ekf_gazebo.rviz',
    )
    me = os.getpid()
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        ipid = int(pid)
        if ipid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as cmd_file:
                cmd = cmd_file.read().replace(b'\x00', b' ').decode(errors='ignore')
        except OSError:
            continue
        if 'ros2' in cmd and 'launch' in cmd:
            continue
        if any(needle in cmd for needle in needles):
            try:
                os.kill(ipid, signal.SIGKILL)
            except OSError:
                pass
    time.sleep(1.0)


def generate_launch_description():
    _stop_previous_demo()
    pkg_desc = get_package_share_directory('vehicle_description')
    default_world = os.path.join(pkg_desc, 'worlds', 'empty.world')
    urdf_file = os.path.join(pkg_desc, 'urdf', 'vehicle.urdf')

    with open(urdf_file, 'r', encoding='utf-8') as urdf_stream:
        robot_description = urdf_stream.read()

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('vehicle_bringup'), 'config', 'ekf_gazebo.rviz']
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments={'world': default_world}.items(),
    )

    delete_old_vehicle = Node(
        package='vehicle_sim',
        executable='delete_gazebo_entity_node',
        name='delete_gazebo_vehicle_once',
        parameters=[{'entity_name': 'vehicle'}],
        output='screen',
    )

    spawn_vehicle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            'vehicle',
            '-file',
            urdf_file,
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.05',
            '-Y',
            '0.0',
            '-reference_frame',
            'world',
            '-unpause',
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            SetParameter(name='use_sim_time', value=True),
            DeclareLaunchArgument('world', default_value=default_world),
            gazebo_launch,
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
            ),
            TimerAction(period=5.0, actions=[delete_old_vehicle]),
            TimerAction(period=5.5, actions=[spawn_vehicle]),
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package='vehicle_sim',
                        executable='cmd_vel_demo_node',
                        name='cmd_vel_demo',
                        parameters=[
                            {
                                'use_sim_time': True,
                                'linear_speed': 0.40,
                                'angular_speed': 0.10,
                            },
                        ],
                        output='screen',
                    ),
                ],
            ),
            Node(
                package='vehicle_sim',
                executable='odom_ground_truth_node',
                name='odom_ground_truth',
                parameters=[{
                    'input_topic': '/odom',
                    'path_jump_reset_m': 0.4,
                }],
            ),
            Node(
                package='vehicle_sim',
                executable='gazebo_fake_gps_node',
                name='gazebo_fake_gps',
                parameters=[
                    {
                        'input_topic': '/odom',
                        'frame_id': 'odom',
                        'gps_position_std': 0.35,
                        'gps_yaw_std': 0.08,
                    }
                ],
            ),
            Node(
                package='ekf_localization',
                executable='ekf_localization_node',
                name='ekf_fusion',
                parameters=[
                    {
                        'frame_id': 'odom',
                        'gps_noise_std': 0.35,
                        'gps_yaw_noise_std': 0.08,
                        'fuse_gps_yaw': True,
                        'use_imu_accel': False,
                        'wheel_odom_topic': '/odom',
                        'wheel_speed_noise_std': 0.05,
                        'predict_from_wheel_odom': True,
                        'gps_measurement_scale': 8.0,
                        'path_jump_reset_m': 0.4,
                        'path_min_step_m': 0.05,
                        'process_noise': [0.001, 0.001, 0.0005, 0.01],
                    }
                ],
                output='screen',
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
            ),
        ]
    )
