#!/usr/bin/env python3
import os
import rclpy
import yaml
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float64
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from math import atan2, sqrt, sin
from pure_pursuit.utils import segment_circle_intersection
from common_utils.utils import (init_marker, publish_marker, quaternion_to_yaw, publish_drive, WaypointsProcessor,
                                LapProcessor, TrackingLogger)
from vehicle_msgs.msg import VehicleState

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Declare and get parameters
        self.declare_parameter("is_real_car", True)  # simulator or real car
        self.declare_parameter("is_work_with_safety_node", True)  # work with safety node together or not
        self.declare_parameter('config_file',
                               "~/f1Tenth/src/pure_pursuit/pure_pursuit/config.yaml")
        self.declare_parameter('is_tracking_log', True)

        is_real_car = self.get_parameter('is_real_car').get_parameter_value().bool_value
        is_work_with_safety_node = self.get_parameter('is_work_with_safety_node').get_parameter_value().bool_value
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        config_file = os.path.expanduser(config_file)
        is_tracking_log = self.get_parameter('is_tracking_log').get_parameter_value().bool_value

        # Topics
        # sub
        traj_topic = "/planned_trajectory"
        odom_topic = '/pf/pose/odom' if is_real_car else '/ego_racecar/odom'
        # pub
        drive_topic = '/controller_drive' if is_work_with_safety_node else '/drive'
        lap_count_topic = "/lap_count"
        cross_track_error_topic = "/cross_track_error"
        lookahead_point_marker_topic = '/lookahead_point_viz'

        # Subscribers
        self.traj_sub = self.create_subscription(VehicleState, traj_topic, self.traj_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        # Publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.lap_count_pub = self.create_publisher(Int32, lap_count_topic, 10)
        self.cross_track_error_pub = self.create_publisher(Float64, cross_track_error_topic, 10)
        self.lookahead_point_marker_pub = self.create_publisher(Marker, lookahead_point_marker_topic, 10)

        # Pure pursuit setup
        with open(config_file) as f:
            config = yaml.safe_load(f)

        self.vref, self.l_wheelbase, self.lookahead_dist = config['vref'], config['l_wheelbase'], config['lookahead_dist']

        # Pure pursuit initial states
        self.xcurrent = None

        # Waypoints will be subscribed from the trajectory planning node,
        # then preprocessed to find the nearest waypoints, count laps,
        # and calculate cross-track error.
        self.waypoints = None
        self.waypoints_processor = None

        # Count the traveled progress (a full lap 0%->100%) and laps
        self.lap_processor = LapProcessor()
        self.current_progress = None
        self.current_lap = 0

        # Calculate waypoint closest to the current position, projection of the current position on the trajectory,
        # cross track error
        self.closest_waypoint = None
        self.proj = None
        self.cross_track_error = 0.0

        # Log necessary data for performance analysis
        if is_tracking_log:
            log_name = "tracking_log_pure_pursuit_real" if is_real_car else "tracking_log_pure_pursuit_sim"
            self.tracking_logger = TrackingLogger(log_name)
        else:
            self.tracking_logger = None
        self.log_start_time = None

        # Markers initialization
        self.lookahead_point_marker = init_marker("map", scale=[0.06, 0.06], color=[1.0, 0.0, 0.0, 1.0])

    def traj_callback(self, msg):
        """
        1. Get waypoints from the trajectory planning node.
        2. Preprocess waypoints to accelerate subsequent computations.

        Parameters
        ----------
        msg: vehicle_msgs.msg.VehicleState
            message from the trajectory planning node
        Returns
        -------

        """
        if self.waypoints is not None:
            return
        self.waypoints = np.array([msg.x, msg.y, msg.theta, msg.v]).T
        if not np.isclose(msg.v[0], self.vref, atol=1e-2):
            raise ValueError(
                f"Mismatch. Expected each waypoint to have vref {self.vref:.2f} in config.yaml,"
                f"but got {msg.v[0]:.2f} in the trajectory planning node.")
        self.waypoints_processor = WaypointsProcessor(self.waypoints)

    def odom_callback(self, data):
        """
        1. Get current position and heading from the odometry.
        2. Compute the lookahead point and steering angle.
        3. Publish drive control based on steering angle and reference velocity.
        4. Publish markers for waypoints and lookahead point visualization.

        Parameters
        ----------
        data: nav_msgs.msg.Odometry
            message from the odometry.
        Returns
        -------

        """
        if self.waypoints is None:
            self.get_logger().warn("Waiting for waypoints...")
            return

        # Get current state
        pose, twist = data.pose.pose, data.twist.twist
        x, y = pose.position.x, pose.position.y
        theta = quaternion_to_yaw(pose.orientation)
        v = twist.linear.x
        self.xcurrent = np.array([x, y, theta, v])

        # Compute the lookahead point
        lookahead_point, steering_angle = self.compute_lookahead()

        # Publish drive control
        publish_drive(steering_angle, self.vref, self.drive_pub)

        # Publish lap count and cross track error
        self.lap_count_pub.publish(Int32(data=self.current_lap))
        self.cross_track_error_pub.publish(Float64(data=self.cross_track_error))

        # Publish markers
        publish_marker(lookahead_point, self.lookahead_point_marker, self.lookahead_point_marker_pub)

        # Log performance metrics
        if self.tracking_logger:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            if self.log_start_time is None:
                self.log_start_time = current_time
            relative_time = current_time - self.log_start_time

            self.tracking_logger.log(
                relative_time,
                *self.xcurrent,
                *self.closest_waypoint,
                *self.proj,
                self.current_progress,
                self.current_lap,
                self.cross_track_error
            )


    def find_lookahead_point(self, current_position, closest_waypoint_idx, closest_t):
        """
        On a closed-loop trajectory, starting from the waypoint closest to the current position and traversing one full
        lap, find the first intersection between the circle centered at the current position with a radius equal to the
        lookahead distance, and the line segments connecting adjacent waypoints.

        Parameters
        ----------
        current_position: ndarray
            An (2,) array, current position of the vehicle.
        closest_waypoint_idx: int
            The index of the closest waypoint to the current position.
        closest_t: float
            The project factor t on the line segment closest_waypoint_idx -> closest_waypoint_idx + 1. It represents the
            projection of the current position onto the trajectory, which is used to ensure that if the lookahead circle
            intersects with the line segment from closest_waypoint_idx to closest_waypoint_idx + 1, the point ahead
            of the projection is selected. This guarantees that the vehicle moves forward along the path.

        Returns
        -------

        """
        waypoints_positions = self.waypoints_processor.waypoints_positions
        segment_vectors = self.waypoints_processor.segment_vectors

        n = len(waypoints_positions)

        for idx in range(closest_waypoint_idx, closest_waypoint_idx + n):
            i = idx % n  # closed-loop trajectory
            min_t = closest_t if i == closest_waypoint_idx else 0.0
            result = segment_circle_intersection(waypoints_positions[i],
                                                 segment_vectors[i],
                                                 current_position,
                                                 self.lookahead_dist,
                                                 min_t)
            if result is not None:
                return result
        raise ValueError(
            "No valid intersection found between the lookahead circle and the trajectory segments. Check if "
            "the lookahead distance or the trajectory is reasonable.")


    def compute_lookahead(self):
        """
        1. Based on current position, calculate:
        - the closest waypoint index
        - current progress and cross track error

        2. From current progress, determine the current lap count.
        3. From the closest waypoint index, get the lookahead point.

        4. Compute the angle between the vector from current position to lookahead point and the global x-axis.
        Subtract the yaw (i.e., the angle between the vehicle's x-axis and the global x-axis)
        to get the angle between the vector (current position → lookahead point) and the vehicle's x-axis,
        denoted as alpha.

        Geometric relationship:
            Turning radius R: R = (lookahead_distance/2) / sin(alpha)
            Steering angle (delta) = arctan(wheelbase/turning_radius)
                                   = arctan(2* wheelbase*sin(alpha) / lookahead_distance)

        Returns
        -------
        lookahead_point: ndarray
            lookahead point [x,y] coordinates (unit: m).
        delta: float
            steering angle (unit: rad).

        """
        current_position = self.xcurrent[:2]
        current_theta = self.xcurrent[2]

        (
            closest_waypoint_idx,
            self.current_progress,
            self.cross_track_error,
            self.proj,
            closest_t
        ) = self.waypoints_processor.find_closest_waypoint_idx(current_position)

        self.closest_waypoint = self.waypoints[closest_waypoint_idx]
        self.current_lap = self.lap_processor.update(self.current_progress)

        lookahead_point = self.find_lookahead_point(current_position, closest_waypoint_idx, closest_t)
        dx_global = lookahead_point[0] - current_position[0]
        dy_global = lookahead_point[1] - current_position[1]
        lookahead_angle = atan2(dy_global, dx_global)
        l = sqrt(dx_global ** 2 + dy_global ** 2)
        alpha = lookahead_angle - current_theta
        curvature = 2 * sin(alpha) / l
        delta = atan2(self.l_wheelbase * curvature, 1.0) # steering angle
        return lookahead_point, delta


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
