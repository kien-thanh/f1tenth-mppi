import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from interfaces.msg import DistanceMap

import plot 
from grid import Grid
from grid import resample_path

def distance_transform():
    pass

def remove_duplicate_consecutive_points(path: np.array) -> np.array:
    """
    Remove consecutive duplicate points from the path.
    Args:
        path (np.array): The input path as an array of shape (N, 2).
    Returns:
        np.array: The path with consecutive duplicate points removed.
    """
    if len(path) == 0:
        return path

    unique_path = [path[0]]
    for point in path[1:]:
        if not np.array_equal(point, unique_path[-1]):
            unique_path.append(point)
    
    return np.array(unique_path)

class DistanceTransform(Node):

    def __init__(self):
        super().__init__('distance_transform')

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_sub_callback, qos)
        self.plan_sub = self.create_subscription(Path, 'plan', self.plan_sub_callback, 1)

        self.distance_grid_pub = self.create_publisher(DistanceMap, 'distance_map', qos)
        self.angle_grid_pub = self.create_publisher(DistanceMap, 'angle_grid', qos)


    def map_sub_callback(self, msg: OccupancyGrid):
        """
        Callback to receive the map and extract its metadata.
        Args:
            msg (OccupancyGrid): The occupancy grid message.
        """
        self.map_size = (msg.info.width, msg.info.height)
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)


    def plan_sub_callback(self, msg: Path):
        if len(msg.poses) == 0:
            self.get_logger().info("Received empty path.")
            return
        if not hasattr(self, 'map_size'):
            self.get_logger().info("Map not received yet.")
            return
        
        x = [pose.pose.position.x for pose in msg.poses]
        y = [pose.pose.position.y for pose in msg.poses]

        # Creates path in the form [[x0, y0], [x1, y1], ...]
        path = np.column_stack((x, y))

        path = remove_duplicate_consecutive_points(path)

        path = resample_path(path, spacing=0.001)
        
        plot.path(path)

        resolution = 0.005 # 5mm per cell

        path_grid = Grid.from_nav2_path(
            path=path,
            padding=(1.0, 1.0),
            resolution=resolution
        )

        distance_grid = path_grid.distance_transform()

        self.publish_grid(distance_grid)

        # plot.heatmap(distance_grid.path, title="Distance Transform Heatmap")
        # plot.heatmap(distance_grid.angles, title="Angle Grid Heatmap")

    def publish_grid(self, grid: Grid):
        """
        Publish the distance grid as an DistanceMap message.

        Args:
            distance_grid (Grid): The distance grid to publish
        """
        msg = DistanceMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.data = grid.path.flatten().tolist()
        msg.resolution = grid.resolution
        msg.origin_x = grid.origin[0]
        msg.origin_y = grid.origin[1]
        msg.width = grid.width
        msg.height = grid.height
        self.distance_grid_pub.publish(msg)

        msg.data = grid.angles.flatten().tolist()
        self.angle_grid_pub.publish(msg)

        
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = DistanceTransform()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()