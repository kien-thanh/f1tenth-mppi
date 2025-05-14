#!/usr/bin/env python3
from pathlib import Path

import rclpy
from rclpy.node import Node
import numpy as np
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from time import gmtime, strftime

class WaypointsLogger(Node):
    """
    Record waypoints using the real car.
    """
    def __init__(self):
        super().__init__('waypoints_logger')
        target_dir = Path("~/f1Tenth/waypoints").expanduser()
        target_dir.mkdir(parents=True, exist_ok=True)
        file_path = target_dir / strftime('wp-%Y-%m-%d-%H-%M-%S.csv', gmtime())
        self.file = open(file_path, 'w')

        self.odom_topic = '/pf/pose/odom'

        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.save_waypoint,
            10
        )
        self.get_logger().info('Saving waypoints...')

    def save_waypoint(self, data):
        quaternion = np.array([data.pose.pose.orientation.x,
                               data.pose.pose.orientation.y,
                               data.pose.pose.orientation.z,
                               data.pose.pose.orientation.w])

        euler = euler_from_quaternion(quaternion)
        # speed = LA.norm(np.array([data.twist.twist.linear.x,
        #                           data.twist.twist.linear.y,
        #                           data.twist.twist.linear.z]), 2)
        # if data.twist.twist.linear.x > 0.:
        #     self.get_logger().info(f'Speed: {data.twist.twist.linear.x}')

        # self.file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
        #                                       data.pose.pose.position.y,
        #                                       euler[2],
        #                                       speed))

        self.file.write('%f, %f, %f\n' % (data.pose.pose.position.x,
                                          data.pose.pose.position.y,
                                          euler[2]))

    def shutdown(self):
        self.file.close()
        self.get_logger().info('Goodbye')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
