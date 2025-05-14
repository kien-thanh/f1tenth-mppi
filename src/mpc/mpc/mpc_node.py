import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int32
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
import os
from mpc.mpc_solver import MPCsolver
from mpc.utils import unwrap_traj, extract_drive_command, fill_array_message_attributes
from common_utils.utils import (init_marker, publish_marker, quaternion_to_yaw, publish_drive, WaypointsProcessor,
                                LapProcessor, TrackingLogger)
from vehicle_msgs.msg import VehicleState, VehicleInput


class MPC(Node):
    def __init__(self):
        super().__init__("mpc_node")

        # Declare and get parameters
        self.declare_parameter("is_real_car", True)  # simulator or real car
        self.declare_parameter("is_work_with_safety_node", True)  # work with safety node together or not
        self.declare_parameter("config_file", "~/f1Tenth/src/mpc/mpc/config.yaml")
        self.declare_parameter('is_tracking_log', True)

        self.is_real_car = self.get_parameter("is_real_car").get_parameter_value().bool_value
        is_work_with_safety_node = self.get_parameter("is_work_with_safety_node").get_parameter_value().bool_value
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        config_file = os.path.expanduser(config_file)
        is_tracking_log = self.get_parameter('is_tracking_log').get_parameter_value().bool_value

        # Topics
        # sub
        traj_topic = "/planned_trajectory"
        odom_topic = "/pf/pose/odom" if self.is_real_car else "/ego_racecar/odom"
        # "/steering_angle" is now only suitable for the real car (see adaptions in vesc_ackermann),
        # can adapt f1tenth_gym and f1tenth_gym_ros in the future in order to
        # publish and subscribe the steering angle in the simulator directly
        steering_angle_topic = "/steering_angle"
        # pub
        drive_topic = "/controller_drive" if is_work_with_safety_node else "/drive"
        lap_count_topic = "/lap_count"
        cross_track_error_topic = "/cross_track_error"
        current_state_topic = "/current_state"
        ref_state_topic = "/ref_state"
        pred_state_topic = "/pred_state"
        pred_input_topic = "/pred_input"
        # ref_traj_marker_topic = "/ref_traj_viz"
        # pred_traj_marker_topic = "/pred_traj_viz"

        # Subscribers
        self.traj_sub = self.create_subscription(VehicleState, traj_topic, self.traj_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.steering_angle_sub = self.create_subscription(Float64, steering_angle_topic,
                                                           self.steering_angle_callback,10)

        # Publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.lap_count_pub = self.create_publisher(Int32, lap_count_topic, 10)
        self.cross_track_error_pub = self.create_publisher(Float64, cross_track_error_topic, 10)
        self.current_state_pub = self.create_publisher(VehicleState, current_state_topic, 10)
        self.ref_state_pub = self.create_publisher(VehicleState, ref_state_topic, 10)
        self.pred_state_pub = self.create_publisher(VehicleState, pred_state_topic, 10)
        self.pred_input_pub = self.create_publisher(VehicleInput, pred_input_topic, 10)
        # self.ref_traj_marker_pub = self.create_publisher(Marker, ref_traj_marker_topic, 10)
        # self.pred_traj_marker_pub = self.create_publisher(Marker, pred_traj_marker_topic, 10)

        # MPC solver setup
        with open(config_file) as f:
            config = yaml.safe_load(f)

        self.mpc_solver = MPCsolver(config=config)

        # MPC initial states
        self.xcurrent = None
        self.mpc_xinit = None
        self.steering_angle = 0.0

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
            log_name = "tracking_log_mpc_real" if self.is_real_car else "tracking_log_mpc_sim"
            self.tracking_logger = TrackingLogger(log_name)
        else:
            self.tracking_logger = None
        self.log_start_time = None

        # Markers initialization
        # self.ref_traj_marker = init_marker("map", color=[0.0, 0.0, 1.0, 1.0])
        # self.pred_traj_marker = init_marker("map", color=[1.0, 0.0, 0.0, 1.0])

    def traj_callback(self, msg):
        """
        Get waypoints from the trajectory planning node.
        preprocess waypoints to accelerate subsequent computations.

        Parameters
        ----------
        msg: vehicle_msgs.msg.VehicleState
            message from the trajectory planning node
        Returns
        -------

        """
        if self.waypoints is not None:
            return
        self.waypoints = np.array([getattr(msg, state) for state in self.mpc_solver.state_names]).T
        if not np.isclose(msg.v[0], self.mpc_solver.vref, atol=1e-2):
            raise ValueError(
                f"Mismatch. Expected each waypoint to have vref {self.mpc_solver.vref:.2f} in config.yaml,"
                f"but got {msg.v[0]:.2f} in the trajectory planning node.")
        self.waypoints_processor = WaypointsProcessor(self.waypoints)

    def steering_angle_callback(self, msg):
        """
        Get steering angle from the vesc_ackermann (see f1tenth_system)

        Parameters
        ----------
        msg: std_msgs.msg.Float64
            message from the vesc_ackermann
        Returns
        -------

        """
        self.steering_angle = msg.data

    def odom_callback(self, msg):
        """
        1. Get current states from the odometry or vecker_ackermann (if current steering angle needed).
        2. Based on the initial states (set as current states), reference states and inputs, get predicted states and
        inputs from the MPC solver.
        3. Publish drive control based on predicted steering angle and speed.
        4. Publish lap count and cross track error.
        5. Publish markers for visualization.
        6. Publish vehicle current/reference/predicted states.

        Parameters
        ----------
        msg: nav_msgs.msg.Odometry
            message from the odometry.
        Returns
        -------

        """
        if self.waypoints is None:
            self.get_logger().warn("Waiting for waypoints...")
            return

        # Get current position, theta and speed
        pose, twist = msg.pose.pose, msg.twist.twist
        x, y = pose.position.x, pose.position.y
        theta = quaternion_to_yaw(pose.orientation)
        v = twist.linear.x

        # Avoid large angle jump between previous theta and current theta
        if self.xcurrent is not None:
            prev_theta = self.xcurrent[2]
            theta = np.unwrap([prev_theta, theta])[-1]

        # MPC initial states
        if self.mpc_solver.model_type == "model_input_accel_steering":
            self.xcurrent = np.array([x, y, theta, v])
        elif self.mpc_solver.model_type == "model_input_accel_steeringrate":
            if self.is_real_car:
                delta = self.steering_angle
            else:
                omega = twist.angular.z
                delta = np.arctan2(self.mpc_solver.L * omega / v, 1.0) if abs(v) > 1e-3 else 0.0
            self.xcurrent = np.array([x, y, theta, v, delta])
        else:
            raise ValueError("Model type not supported")

        self.mpc_xinit = self.xcurrent

        # MPC reference values
        xref, uref = self.update_ref_x_u() # (N+1, num_states), (N+1, num_inputs)
        yref, yref_e = np.hstack((xref, uref))[:-1], xref[-1]  # (N, num_states + num_inputs), (1, num_states)

        # MPC predicted values
        xpred, upred = self.mpc_solver.solve(self.mpc_xinit, yref, yref_e)  # (N+1, num_states), (N, num_inputs)

        # Publish drive control
        speed, steering_angle = extract_drive_command(xpred, upred, self.mpc_solver)
        speed = max(0.2, speed)
        publish_drive(steering_angle, speed, self.drive_pub)

        # Publish lap count and cross track error
        self.lap_count_pub.publish(Int32(data=self.current_lap))
        self.cross_track_error_pub.publish(Float64(data=self.cross_track_error))

        # Publish markers
        # publish_marker(xref, self.ref_traj_marker, self.ref_traj_marker_pub)
        # publish_marker(xpred, self.pred_traj_marker, self.pred_traj_marker_pub)

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

        # Publish vehicle states
        current_state = fill_array_message_attributes(VehicleState(), self.xcurrent, self.mpc_solver)
        ref_state = fill_array_message_attributes(VehicleState(), xref, self.mpc_solver)
        pred_state = fill_array_message_attributes(VehicleState(), xpred, self.mpc_solver)
        pred_input = fill_array_message_attributes(VehicleInput(), upred, self.mpc_solver)

        self.current_state_pub.publish(current_state)
        self.ref_state_pub.publish(ref_state)
        self.pred_state_pub.publish(pred_state)
        self.pred_input_pub.publish(pred_input)

    def update_ref_x_u(self):
        """
        1. Based on the current position:
        - Calculate the index of the closest waypoint.
        - Compute the current progress and cross-track error.

        2. Determine the current lap count from the progress.

        3. Select N+1 waypoints starting from the closest waypoint index to index+N as reference state values
        at shooting nodes 0 to N. Unwrap heading angles to avoid numerical discontinuities.

        4. Set reference inputs to zero by default.

        Returns
        -------
        xref: ndarray
            An (N+1, num_states) array of reference states.
        uref: ndarray
            An (N+1, num_inputs) array of reference inputs.

        """
        current_position = self.mpc_xinit[:2]

        (
            start_idx,
            self.current_progress,
            self.cross_track_error,
            self.proj,
            _
        ) = self.waypoints_processor.find_closest_waypoint_idx(current_position)

        self.closest_waypoint = self.waypoints[start_idx]
        self.current_lap = self.lap_processor.update(self.current_progress)

        # N+1 reference trajectory points. For a closed-loop trajectory, ensure it
        # correctly loops back to the start when exceeding num_waypoints
        N = self.mpc_solver.N
        indices = np.mod(np.arange(start_idx, start_idx+N+1), len(self.waypoints))
        xref = self.waypoints[indices]

        # Avoid large angle jump between current_theta and xref:
        # xref[:, 2] = np.unwrap(xref[:, 2])
        current_theta  = self.mpc_xinit[2]
        xref = unwrap_traj(current_theta, xref) # (N+1, num_states)
        uref = np.zeros((N+1, self.mpc_solver.nu)) # (N+1, num_inputs)
        return xref, uref

def main(args=None):
    rclpy.init(args=args)
    print("MPC initialized")
    mpc_node = MPC()
    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
