import os
import rclpy
from rclpy.node import Node
from vehicle_msgs.msg import VehicleState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from trajectory_planning.utils import load_waypoints
from common_utils.utils import init_marker, publish_marker
from geometry_msgs.msg import Point

class TrajectoryPlanning(Node):
    """
    A ROS2 node for dummy trajectory planning using preprocessed waypoints.

    This node:
    - Loads preprocessed waypoints sampled at uniform arc length from a CSV file as
      an (n_timesteps, n_states) array, where each row [x, y, theta, v, delta].
    - Publishes a dummy trajectory, which contains lists for each state variable across time steps:
      [x_0, ..., x_N], [y_0, ..., y_N], [theta_0, ..., theta_N], [v_0, ..., v_N], [delta_0, ..., delta_N].
    - Publishes at a fixed rate (e.g., 250 Hz) for integration with the controllers。

    Declared Parameters
    -------------------
    waypoint_file : str
        Path to the CSV file containing arc-length-sampled waypoints.

    Attributes
    ----------
    traj_pub : rclpy.publisher.Publisher
        Publishes the planned trajectory to the /planned_trajectory topic.
    waypoints_marker_pub : rclpy.publisher.Publisher
        Publishes the waypoints' markers to the /waypoints_viz topic for visualization in RViz2.
    traj_timer : rclpy.timer.Timer
        Triggers trajectory publishing at a fixed rate.
    marker_timer : rclpy.timer.Timer
        Triggers waypoints marker publishing at a fixed rate.
    waypoints : ndarray
        Array of waypoints with row [x, y, theta, v, delta] used for dummy planning.
    waypoints_marker: visualization_msgs.msg.Marker
        The initialized marker ready to be published.
    state_names : list of str
        Names of the state variables loaded from the waypoint file.
    """
    def __init__(self):
        super().__init__('trajectory_planning')
        # arc_length=vref*Tf/N, Tf/N=1/50
        # vref=1, arc_length=0.02
        # vref=2, arc_length=0.04
        # vref=3, arc_length=0.06
        # vref=4, arc_length=0.08
        self.declare_parameter("waypoint_file",
                               "~/f1Tenth/waypoints/wp_scaled_uniform_arc_length_sampling_0.06.csv")
        waypoint_file = self.get_parameter("waypoint_file").get_parameter_value().string_value
        waypoint_file = os.path.expanduser(waypoint_file)

        self.waypoints, self.state_names = load_waypoints(filepath=waypoint_file)

        traj_topic = "/planned_trajectory"
        waypoints_marker_topic = '/waypoints_viz'

        self.traj_pub = self.create_publisher(VehicleState, traj_topic, 10)
        self.waypoints_marker_pub = self.create_publisher(Marker, waypoints_marker_topic, 10)

        self.traj_timer = self.create_timer(0.004, self.publish_trajectory)
        self.marker_timer = self.create_timer(1.0, self.publish_waypoints_marker)

        # Visualize waypoints (start point black, others red)
        self.start_marker = init_marker("map", color=[0.0, 0.0, 0.0, 1.0])
        self.start_marker.id = 102
        self.start_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        start_point = self.waypoints[0]
        self.start_marker.points = [Point(x=start_point[0], y=start_point[1], z=0.0)]
        self.start_marker.header.stamp = self.get_clock().now().to_msg()
        self.waypoints_marker_pub.publish(self.start_marker)

        self.waypoints_marker = init_marker("map", color=[0.0, 1.0, 0.0, 0.6])
        self.waypoints_marker.id = 101
        self.waypoints_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        self.waypoints_marker.points = [Point(x=p[0], y=p[1], z=0.0) for p in self.waypoints]
        self.waypoints_marker.header.stamp = self.get_clock().now().to_msg()
        self.waypoints_marker_pub.publish(self.waypoints_marker)

    def publish_trajectory(self):
        traj = VehicleState()
        traj.header = Header()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = "map"

        def fill_array_message_attributes(obj, data_source, attribute_names):
            if data_source.ndim == 1:
                data_source = data_source[None, :]  # shape (n_variables,) -> (1, n_variables)
            for i, name in enumerate(attribute_names):
                setattr(obj, name, data_source[:, i].tolist())
            return obj

        fill_array_message_attributes(traj, self.waypoints, self.state_names)

        self.traj_pub.publish(traj)

    def publish_waypoints_marker(self):
        self.start_marker.header.stamp = self.get_clock().now().to_msg()
        self.waypoints_marker_pub.publish(self.start_marker)

        self.waypoints_marker.header.stamp = self.get_clock().now().to_msg()
        self.waypoints_marker_pub.publish(self.waypoints_marker)

def main():
    rclpy.init()
    print("Trajectory planning initialized")
    node = TrajectoryPlanning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
