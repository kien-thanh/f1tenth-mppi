#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    """
    1. Initial State:
       At the beginning, the brake is released by default (self.release_braking = True),
       allowing you to teleoperate the car using, for example, a joystick.

    2. Behavior:
       If the brake is released:
         - Check the iTTC value:
           1. If the value is lower than the threshold:
              - Engage braking.
              - The brake will remain locked (self.release_braking = False) until
                the braking status is explicitly released (e.g., by pressing special joystick buttons).
           2. Otherwise:
              - If a controller (e.g., pure pursuit) is available, use the controller to control the car.
              - If no controller is available, continue teleoperating the car with the joystick.
       Otherwise (if the brake is engaged):
         - Keep braking.
    """

    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter("is_real_car", True)  # simulator or real car
        self.declare_parameter("ttc_threshold", 0.2)  # v=2, 0.3
        self.declare_parameter("fov_half", 5.0) # half of FOV, degree, used for filtering
        # Add a margin of 0.3 to avoid collisions, as braking from the start to a complete stop requires time
        self.declare_parameter("range_margin", 0.5)

        is_real_car = self.get_parameter('is_real_car').get_parameter_value().bool_value
        self.iTTC_threshold = self.get_parameter('ttc_threshold').get_parameter_value().double_value
        self.fov_half = self.get_parameter('fov_half').get_parameter_value().double_value
        self.range_margin = self.get_parameter('range_margin').get_parameter_value().double_value

        # odom
        self.speed = 0.0
        # The brake is released by default; otherwise, teleoperation via joystick or keyboard will not be possible.
        self.release_braking = True
        # controller e.g. pure pursuit
        self.is_controller_available = False
        self.controller_speed = None
        self.controller_steering_angle = None

        # Topics
        self.lidarscan_topic = '/scan'

        # get speed from odom for calculating iTTC
        self.odom_topic = '/odom' if is_real_car else '/ego_racecar/odom'

        # Once you want the car to move after emergency braking, get a True value (e.g. press the joystick buttons) to
        # release the braking status
        self.release_braking_topic = '/release_braking'

        # The controller and the safety node work together to control the car
        self.controller_topic = '/controller_drive'
        self.drive_topic = '/drive'
        self.emergency_braking_topic = '/emergency_braking'

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, self.lidarscan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.release_braking_sub = self.create_subscription(Bool, self.release_braking_topic,
                                                            self.release_braking_callback, 10)
        self.controller_sub = self.create_subscription(AckermannDriveStamped, self.controller_topic,
                                                       self.controller_callback, 10)
        # Publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.emergency_braking_pub = self.create_publisher(Bool, self.emergency_braking_topic, 10)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def release_braking_callback(self, release_braking_msg):
        if self.release_braking != release_braking_msg.data:
            self.get_logger().info("Releasing Braking")
            self.release_braking = release_braking_msg.data

    def controller_callback(self, controller_drive_msg):
        self.is_controller_available = True
        self.controller_speed = controller_drive_msg.drive.speed
        self.controller_steering_angle = controller_drive_msg.drive.steering_angle

    def scan_callback(self, scan_msg):
        drive_msg = AckermannDriveStamped()
        if self.release_braking:
            # process the laser scan data, filter out inf or nan values, only consider the
            # scan beams within range [-10, 10] degree
            ranges = np.array(scan_msg.ranges)
            angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
            ranges = np.where(np.isfinite(ranges), ranges, scan_msg.range_max)
            forward_filter = np.abs(angles) <= self.fov_half * np.pi / 180
            ranges, angles = ranges[forward_filter], angles[forward_filter]

            range_rates = -self.speed * np.cos(angles)

            iTTC_values = np.where(range_rates < 0, (ranges - self.range_margin) / np.maximum(-range_rates, 1e-5), np.inf)
            min_iTTC_index = np.argmin(iTTC_values)
            min_iTTC = iTTC_values[min_iTTC_index]
            min_range = ranges[min_iTTC_index]
            min_angle = angles[min_iTTC_index] * 180 / np.pi  # degree

            if min_iTTC < self.iTTC_threshold:
                self.get_logger().info('Emergency braking')
                drive_msg.drive.speed = 0.0
                drive_msg.drive.steering_angle = 0.0
                self.release_braking = True # keep braking until get the True value from the /release_braking topic
                self.drive_pub.publish(drive_msg)
                self.get_logger().info(f"Min iTTC: {min_iTTC}, "
                                       f"range: {min_range} m, "
                                       f"angle: {min_angle} degree")
            else:
                if self.is_controller_available:
                    drive_msg.drive.speed = self.controller_speed
                    drive_msg.drive.steering_angle = self.controller_steering_angle
                    self.drive_pub.publish(drive_msg)
                else:
                    pass

        # if the brake isn't released, keep braking
        else:
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            self.get_logger().info("Keep braking, press the joystick buttons L1 + Circle to release")

        emergency_braking_msg = Bool()
        emergency_braking_msg.data = not self.release_braking
        self.emergency_braking_pub.publish(emergency_braking_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Safety Node Initialized")
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
