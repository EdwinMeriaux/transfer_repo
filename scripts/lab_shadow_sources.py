#!/usr/bin/env python3
"""Publish isolated stationary odometry and a zero desired command for lab shadow tests."""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class LabShadowSources(Node):
    def __init__(self):
        super().__init__('blueboat_lab_shadow_sources')
        self.odom = self.create_publisher(Odometry, '/blueboat/lab/odometry', 10)
        self.command = self.create_publisher(Twist, '/blueboat/cmd_vel', 10)
        self.create_timer(0.05, self.publish_odometry)
        self.create_timer(0.2, self.publish_command)

    def publish_odometry(self):
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.child_frame_id = 'base_link'
        message.pose.pose.orientation.w = 1.0
        self.odom.publish(message)

    def publish_command(self):
        self.command.publish(Twist())


def main():
    rclpy.init()
    node = LabShadowSources()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

