import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from time import time

""" 
Only test in the simulator with levine.png as map
"""
# At the beginning, the car's distance to the left wall is 0.701m, to the right wall is 1.003 m
DESIRED_DIST_LEFT = 0.85  # m
VELOCITY = 1.5  # m/s
THETA = 45  # degree, the angle between the 2 laser beams a and b
L = 0.5  # m, lookahead distance
Kp = 1
Ki = 0
Kd = 0.01
MAX_STEERING_ANGLE = 0.38  # rad, check src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml

class WallFollow(Node):
    """
    Implement Wall Following on the car
    """

    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)

        # TODO: set PID gains
        self.kp = Kp
        self.kd = Kd
        self.ki = Ki

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        # self.error =

        # TODO: store any necessary values you think you'll need
        self.desired_distance = 1.5
        self.last_time = time()

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data : single range array from the LiDAR
            angle (rad): between angle_min and angle_max of the LiDAR

        Returns:
            range (m): range measurement in meters at the given angle

        """

        #TODO: implement
        angle_min = range_data.angle_min
        angle_increment = range_data.angle_increment
        ranges = range_data.ranges
        index = int((angle - angle_min) / angle_increment)
        range_at_angle = ranges[index]

        if np.isinf(range_at_angle) or np.isnan(range_at_angle):
            return 100
        return range_at_angle

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        # TODO: implement
        # notice follow the wall to the LEFT
        # Assume that the car's x-axis is tilted to the RIGHT by an angle alpha
        # compared to the axis along the wall direction.

        # Two laser scans a and b:
        # b 90 deg to the left of the car's x-axis
        # a THETA clockwise relative to b
        angle_b = np.radians(90)
        angle_a = angle_b - np.radians(THETA)
        b = self.get_range(range_data, angle_b)
        a = self.get_range(range_data, angle_a)

        alpha = np.arctan((a * np.cos(THETA) - b)/(a * np.sin(THETA)))
        self.get_logger().info(f'alpha: {np.degrees(alpha)} degree')
        # distance between the car and the right wall at the time t
        D_t = b * np.cos(alpha)
        # at the time t+1
        D_t1 = D_t + L * np.sin(alpha)
        return dist - D_t1

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # TODO: Use kp, ki & kd to implement a PID controller
        current_time = time()
        dt = current_time - self.last_time
        self.last_time = current_time
        angle = self.compute_control(error, dt)
        # clip the angle to [-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE]
        angle = np.clip(angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)

        # the car's x-axis is tilted to the RIGHT by an angle alpha，
        # in the car's local coordinate system (x-axis forward, y-axis left),
        # the steering angle should be -alpha.
        self.get_logger().info(f'steering angle: {-np.degrees(angle)} degree')
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = -angle
        if np.radians(0) < abs(angle) <= np.radians(10):
            drive_msg.drive.speed = velocity
        elif np.radians(0) < abs(angle) <= np.radians(20):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5

        self.drive_pub.publish(drive_msg)

    def compute_control(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg, DESIRED_DIST_LEFT)  # TODO: replace with error calculated by get_error()
        self.get_logger().info(f'error: {error} m')
        # TODO: calculate desired car velocity based on error
        self.pid_control(error, VELOCITY)  # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
