from setuptools import find_packages, setup

package_name = 'vehicle_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ademkas',
    maintainer_email='ademkas@todo.todo',
    description='2D kinematic vehicle simulator with IMU and GPS for EKF learning',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'simulator_node = vehicle_sim.simulator_node:main',
            'error_logger_node = vehicle_sim.error_logger_node:main',
            'gazebo_fake_gps_node = vehicle_sim.gazebo_fake_gps_node:main',
            'cmd_vel_demo_node = vehicle_sim.cmd_vel_demo_node:main',
            'odom_ground_truth_node = vehicle_sim.odom_ground_truth_node:main',
            'delete_gazebo_entity_node = vehicle_sim.delete_gazebo_entity_node:main',
            'plot_error_log = vehicle_sim.plot_error_log:main',
        ],
    },
)
