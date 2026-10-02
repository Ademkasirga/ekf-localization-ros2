#!/usr/bin/env bash
# Tüm demo süreçlerini kapat (çift node + eski Gazebo modeli önlemek için)
killall gzserver gzclient rviz2 2>/dev/null
pkill -f ekf_localization_node 2>/dev/null
pkill -f ekf_fusion 2>/dev/null
pkill -f gazebo_ekf.launch 2>/dev/null
pkill -f cmd_vel_demo 2>/dev/null
ros2 daemon stop 2>/dev/null || true
echo "Demo durduruldu. Yeniden: source install/setup.bash && ros2 launch ..."
