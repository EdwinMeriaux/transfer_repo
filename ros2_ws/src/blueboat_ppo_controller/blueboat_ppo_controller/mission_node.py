"""ROS 2 wrapper for the two simple field missions."""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .mission_control import MissionController, yaw_from_quaternion


class MissionNode(Node):
    def __init__(self, arguments):
        super().__init__('blueboat_simple_mission')
        self.arguments = arguments
        self.controller = MissionController(
            arguments.mission,
            distance_m=arguments.distance,
            return_distance_m=arguments.return_distance,
            center_distance_m=arguments.center_distance,
            return_offset_m=arguments.return_offset,
            diameter_m=arguments.diameter,
            speed_m_s=arguments.speed,
            max_yaw_rate_rad_s=arguments.max_yaw_rate,
            arrival_radius_m=arguments.arrival_radius)
        self.publisher = self.create_publisher(Twist, arguments.command_topic, 1)
        self.create_subscription(Odometry, arguments.odometry_topic, self.odometry_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.control)
        self.pose = None
        self.pose_receipt = None
        self.started_at = time.monotonic()
        self.last_stage = None
        self.complete_since = None
        self.finished = False
        self.error = None
        self.get_logger().info(
            f'Waiting for odometry; mission={arguments.mission}, command={arguments.command_topic}')

    def odometry_callback(self, message):
        try:
            if message.header.frame_id != self.arguments.world_frame or message.child_frame_id != self.arguments.body_frame:
                raise ValueError(
                    f'Expected {self.arguments.world_frame}/{self.arguments.body_frame}, got '
                    f'{message.header.frame_id}/{message.child_frame_id}')
            position = np.array([message.pose.pose.position.x, message.pose.pose.position.y], dtype=float)
            q = message.pose.pose.orientation
            yaw = yaw_from_quaternion([q.x, q.y, q.z, q.w])
            if not np.isfinite(position).all():
                raise ValueError('Nonfinite position')
            self.pose = position, yaw
            self.pose_receipt = time.monotonic()
        except ValueError as error:
            self.pose = None
            self.error = str(error)

    def publish(self, surge=0.0, yaw_rate=0.0):
        message = Twist()
        message.linear.x = float(surge)
        message.angular.z = float(yaw_rate)
        self.publisher.publish(message)

    def control(self):
        now = time.monotonic()
        if self.finished:
            self.publish()
            return
        if now - self.started_at > self.arguments.timeout:
            self.error = 'Mission timeout'
            self.finished = True
            self.publish()
            return
        if self.pose is None or self.pose_receipt is None:
            self.publish()
            if now - self.started_at > self.arguments.odometry_wait:
                self.error = self.error or 'No valid odometry received'
                self.finished = True
            return
        if now - self.pose_receipt > self.arguments.odometry_timeout:
            self.error = 'Odometry became stale'
            self.finished = True
            self.publish()
            return
        position, yaw = self.pose
        try:
            command = self.controller.update(position, yaw)
        except ValueError as error:
            self.error = str(error)
            self.finished = True
            self.publish()
            return
        self.publish(command.surge_m_s, command.yaw_rate_rad_s)
        if command.stage != self.last_stage:
            self.get_logger().info(
                f'stage={command.stage}, position=({position[0]:.2f},{position[1]:.2f}), '
                f'yaw={math.degrees(yaw):.1f} deg')
            self.last_stage = command.stage
        if command.complete:
            if self.complete_since is None:
                self.complete_since = now
                self.get_logger().info('Mission complete; publishing stop command')
            elif now - self.complete_since >= 2.0:
                self.finished = True

    def stop(self):
        for _ in range(5):
            self.publish()
            rclpy.spin_once(self, timeout_sec=0.02)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description='Publish a simple position-feedback BlueBoat mission')
    parser.add_argument('--mission', choices=('out-and-back', 'center-circle'), required=True)
    parser.add_argument('--distance', type=float, default=20.0)
    parser.add_argument('--return-distance', type=float, default=18.0)
    parser.add_argument('--center-distance', type=float, default=25.0)
    parser.add_argument('--return-offset', type=float, default=2.0)
    parser.add_argument('--diameter', type=float, default=12.0)
    parser.add_argument('--speed', type=float, default=0.4)
    parser.add_argument('--max-yaw-rate', type=float, default=0.3)
    parser.add_argument('--arrival-radius', type=float, default=0.6)
    parser.add_argument('--command-topic', default='/blueboat/cmd_vel')
    parser.add_argument('--odometry-topic', default='/mavros/local_position/odom')
    parser.add_argument('--world-frame', default='map')
    parser.add_argument('--body-frame', default='base_link')
    parser.add_argument('--odometry-wait', type=float, default=15.0)
    parser.add_argument('--odometry-timeout', type=float, default=0.5)
    parser.add_argument('--timeout', type=float, default=1200.0)
    return parser.parse_args(argv)


def main(argv=None):
    parsed = arguments(argv)
    rclpy.init(args=[])
    node = MissionNode(parsed)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.error = node.error or 'Interrupted by operator'
    finally:
        if rclpy.ok():
            node.stop()
        error = node.error
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if error:
        print(error, file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
