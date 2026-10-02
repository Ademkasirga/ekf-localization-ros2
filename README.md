
# ekf_ws — 2D EKF Localization (ROS 2 Humble + Gazebo)

Educational **Extended Kalman Filter** project for planar vehicle localization: fuse noisy GPS-like measurements with wheel odometry (and IMU where appropriate), compare **ground truth** vs **EKF** in RViz.

<img width="1656" height="922" alt="rviz_demo" src="https://github.com/user-attachments/assets/9f6fcc02-8c7b-4b98-935a-2ac8a21ed80c" />

## Features

- Custom **4-state EKF** (`x`, `y`, `yaw`, `v`) in Python/NumPy
- Predict: kinematic model (yaw rate + velocity)
- Updates: GPS position, optional GPS heading, wheel speed
- **Kinematic simulator** (circle, figure-8, GPS outage scenarios)
- **Gazebo** diff-drive robot with IMU and fake noisy GPS
- Unit tests (`colcon test --packages-select ekf_localization`)

## Tech stack

- **ROS 2 Humble** · Python 3 · NumPy
- **Gazebo** + `gazebo_ros` · URDF/xacro
- **RViz2** · colcon / ament_python

## Packages

| Package | Description |
|---------|-------------|
| `ekf_localization` | EKF core + ROS node |
| `vehicle_sim` | Simulators, fake GPS, logging, path helpers |
| `vehicle_description` | Robot model & Gazebo plugins |
| `vehicle_bringup` | Launches & RViz configs |

## Quick start

```bash
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions
cd ekf_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Gazebo + EKF + RViz (~4 m radius circle)
ros2 launch vehicle_bringup gazebo_ekf.launch.py

# Kinematic sim only
ros2 launch vehicle_bringup sim.launch.py
```

## Main topics (Gazebo)

| Topic | Type | Role |
|-------|------|------|
| `/gps/pose` | `PoseWithCovarianceStamped` | Noisy pseudo-GPS |
| `/odometry/filtered` | `Odometry` | EKF output |
| `/ground_truth/path` | `Path` | Reference path |
| `/ekf/path` | `Path` | Estimated path |

## Optional benchmark

```bash
chmod +x scripts/run_ekf_gazebo_benchmark.sh
./scripts/run_ekf_gazebo_benchmark.sh 75
```

## License

Add your license (e.g. MIT) if you publish publicly.
