#!/usr/bin/env python3
"""Remove a Gazebo model so the next spawn starts at the origin."""

import rclpy
from gazebo_msgs.srv import DeleteEntity
from rclpy.node import Node


class DeleteGazeboEntityNode(Node):

    def __init__(self) -> None:
        super().__init__('delete_gazebo_entity')
        self.declare_parameter('entity_name', 'vehicle')
        self.declare_parameter('timeout_sec', 30.0)

        name = self.get_parameter('entity_name').value
        timeout = self.get_parameter('timeout_sec').value
        client = self.create_client(DeleteEntity, '/delete_entity')

        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn('delete_entity service not available; skipping')
            return

        req = DeleteEntity.Request()
        req.name = name
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Deleted Gazebo entity [{name}]')
        else:
            self.get_logger().info(f'No entity to delete or delete failed for [{name}]')


def main(args=None):
    rclpy.init(args=args)
    node = DeleteGazeboEntityNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
