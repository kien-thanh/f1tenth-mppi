import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import plot 
from grid import Grid

def path_to_grid(path: Path, grid_width, grid_height, origin=(-8.192, -8.192), resolution=0.001):
    """
    Convert a Path message to a grid representation.
    Path is marked with 0, everywhere else with 1.

    Args:
        path (Path): The input path message.
        grid_width (int): Width of the output grid.
        grid_height (int): Height of the output grid.
        origin (tuple): Origin of the grid in world coordinates (x, y).
        resolution (float): Resolution of the grid in meters per cell.

    Returns:
        np.ndarray: 2D grid representation of the path.
    """
    grid = np.ones((grid_height, grid_width), dtype=np.uint8)
    for pose in path.poses:
        x = int((pose.pose.position.x - origin[0]) / resolution)
        # Invert y-axis because image coordinates start from top-left in OpenCV
        # y = grid_size[1] - 1 - int((pose.pose.position.y - origin[1]) / resolution)
        y = int((pose.pose.position.y - origin[1]) / resolution)
        if 0 <= x < grid_width and 0 <= y < grid_height:
            grid[y, x] = 0
    return grid

def distance_transform():
    pass

class DistanceTransform(Node):

    def __init__(self):
        super().__init__('distance_transform')

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_sub_callback, qos)
        self.plan_sub = self.create_subscription(Path, 'plan', self.plan_sub_callback, 1)

        self.distance_heatmap_pub = self.create_publisher(Image, 'distance_heatmap', qos)
        self.distance_map = self.create_publisher(Float32MultiArray, 'distance_map', qos)

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
        
        plot.path(msg)

        resolution = 0.01  # 1 cm per cell

        grid = Grid.from_nav2_path(
            msg, 
            padding=(1.0, 1.0),  # 1.0 meter padding
            resolution=resolution
        )

        dist = cv2.distanceTransform(grid.data, cv2.DIST_L2, 0)
        dist *= resolution  # convert to meters

        self.publish_distance_map(dist)

        plot.heatmap(dist, title="Distance Transform Heatmap")

    def publish_distance_map(self, distance_map: np.ndarray):
        """
        Publish the distance map as an Float32MultiArray message.

        Args:
            distance_map (np.ndarray): The distance map to publish
            size (tuple): The size of the distance map (width, height).
        """
        msg = Float32MultiArray()
        msg.data = distance_map.flatten().tolist()
        msg.layout.dim.append(MultiArrayDimension(label="rows", size=distance_map.shape[0]))
        msg.layout.dim.append(MultiArrayDimension(label="cols", size=distance_map.shape[1]))
        self.distance_map.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = DistanceTransform()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()