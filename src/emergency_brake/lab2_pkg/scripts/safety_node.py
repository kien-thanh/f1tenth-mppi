#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from collections import deque

safety_time = .5
# TODO - LOOKS GOOD, JUST NEED TO TUNE DOWN THE SAFETY TIME FOR ANY REAL-LIFE TESTING
# TODO - ALSO CHANGE THE FILTER ANGLES DEPENDING ON WHAT i WANT TO SHOW

# TODO probably change the safety time to about 1 second for real life
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
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        # create a publisher to change how we drive 
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # make some parameters
        # changing to unrealistic parameters (as in very quick reaction 
        # time and braking distance)
        self.declare_parameter('reaction_time', .01)
        self.declare_parameter('safety_distance',0.0001) # in meters
        # make some variables
        self.velocity = 0.0

        self.ittc_history = deque(maxlen=5)
    
    

    def laser_callback(self,msg):
        # process the laserscan data
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        ranges[~np.isfinite(ranges)] = msg.range_max

        forward_filter = np.abs(angles) <= np.pi/4
        angles = angles[forward_filter]

        ranges = ranges[forward_filter]

        range_x = ranges *np.cos(angles)
        range_y = ranges * np.sin(angles)
        velocity_vector = np.array([self.velocity, 0])

        dot_products = range_x * velocity_vector[0] + range_y * velocity_vector[1]
        relative_velocities = dot_products / ranges


        ittc = ranges/np.abs(relative_velocities)

        # get the minimum iitc
        min_ittc = np.min(abs(ittc))
        self.ittc_history.append(min_ittc)
        self.get_logger().info(f'Min iTTC: {min_ittc:.2f} s') # want to log for bugs

        braking_time = self.velocity / 4 + safety_time
        # self.get_parameter('reaction_time').value + \
        #                     self.get_parameter('safety_distance').value
        # TODO: find the min_ittc
        if ((min_ittc < (braking_time)) and self.is_rapidly_decreasing()) or (min_ittc<.1 and self.velocity >.1): 
            # possible collision check
            self.get_logger().warn("COLLISION DETECTED! braking...")
            self.publish_stop_command()
    
    
    def is_rapidly_decreasing(self):
        recent_values = np.array(self.ittc_history)
        print(f"{recent_values=}")
        differences = np.diff(recent_values)
        decreasing_trend = differences < 0
        return np.all(decreasing_trend)
    

    def odom_callback(self,msg):
        self.velocity = msg.twist.twist.linear.x
        # self.get_logger().info(f'Current Speed: {self.velocity}') # log velocity

    def publish_stop_command(self):
        self.get_logger().info("STOP FOR SAFETY REASONS")
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
        
