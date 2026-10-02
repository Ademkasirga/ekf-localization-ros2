#!/usr/bin/env python3
"""Republish Gazebo /odom as /ground_truth for EKF comparison in RViz."""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from vehicle_sim.path_reset import PathRecorder


class OdomGroundTruthNode(Node):

    def __init__(self) -> None:
        super().__init__('odom_ground_truth')
        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('max_path_poses', 2000)
        self.declare_parameter('path_jump_reset_m', 1.5)
        self.declare_parameter('path_min_step_m', 0.02)
        self.declare_parameter('path_min_start_motion_m', 0.04)

        jump = float(self.get_parameter('path_jump_reset_m').value)
        min_step = float(self.get_parameter('path_min_step_m').value)
        min_start = float(self.get_parameter('path_min_start_motion_m').value)
        self._path_rec = PathRecorder(
            'odom',
            jump_reset_m=jump,
            min_step_m=min_step,
            min_start_motion_m=min_start,
        )
        self._startup_path_cleared = False

        in_topic = self.get_parameter('input_topic').value
        self._gt_pub = self.create_publisher(Odometry, 'ground_truth', 10)
        self._path_pub = self.create_publisher(Path, 'ground_truth/path', 10)
        self.create_subscription(Odometry, in_topic, self._cb, 20)
        self.create_timer(0.2, self._startup_clear_path_cb)

    def _startup_clear_path_cb(self) -> None:
        if self._startup_path_cleared:
            return
        self._startup_path_cleared = True
        self._path_rec.clear(self.get_clock().now().to_msg())
        self._path_pub.publish(self._path_rec.message)

    def _cb(self, msg: Odometry) -> None:
        self._gt_pub.publish(msg)

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._path_rec.set_frame_id(msg.header.frame_id)
        max_n = int(self.get_parameter('max_path_poses').value)
        self._path_rec.append(msg.header.stamp, pose, max_n)
        self._path_pub.publish(self._path_rec.message)


def main(args=None):
    rclpy.init(args=args)
    node = OdomGroundTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
