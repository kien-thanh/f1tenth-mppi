#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        # want to subscribe to some odometry data and some laser scan
        self.subscription_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10) # QoS profile of 10 for reliable delivery :)
        
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # create a publisher to change how we drive 
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # make some parameters
        self.declare_parameter('reaction_time', .5)
        self.declare_parameter('safety_distance',1.0) # in meters
        # make some variables
        self.velocity = 0.0
        self.distance_threshold = 1.0 # adjust later

    def laser_callback(self,msg):
        # process the laserscan data
        ranges = np.array(msg.ranges)
        ranges[~np.isfinite(ranges)] = msg.range_max # replace NaN and infinite with max
        ittc = ranges/self.velocity

        # get the minimum iitc
        min_ittc = np.min(ittc)
        self.get_logger().info(f'Min iTTC: {min_ittc:.2f} s') # want to log for bugs

        braking_distance = self.velocity * self.get_parameter('reaction_time').value + \
                            self.get_parameter('safety_distance').value
        # TODO: find the min_ittc
        if (min_ittc*self.velocity < braking_distance) and (self.velocity>.1): 
            # possible collision check
            self.get_logger().warn("COLLISION DETECTED! braking...")
            self.publish_stop_command()

    def odom_callback(self,msg):
        self.velocity = msg.twist.twist.linear.x

    def publish_stop_command(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        