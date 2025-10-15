import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def path_to_grid(path: Path, grid_size=(2048, 2048), origin=(-51.224998, -51.224998), resolution=0.05):
    """
    Convert a Path message to a grid representation.
    Path is marked with 0, everywhere else with 1.

    Args:
        path (Path): The input path message.
        grid_size (tuple): Size of the output grid (width, height).
        origin (tuple): Origin of the grid in world coordinates (x, y).
        resolution (float): Resolution of the grid in meters per cell.

    Returns:
        np.ndarray: 2D grid representation of the path.
    """
    grid = np.ones(grid_size, dtype=np.uint8)
    for pose in path.poses:
        x = int((pose.pose.position.x - origin[0]) / resolution)
        # Invert y-axis because image coordinates start from top-left in OpenCV
        y = grid_size[1] - 1 - int((pose.pose.position.y - origin[1]) / resolution)
        if 0 <= x < grid_size[0] and 0 <= y < grid_size[1]:
            grid[y, x] = 0
    return grid

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
        
        grid = path_to_grid(msg, self.map_size, self.map_origin, self.map_resolution)

        dist = cv2.distanceTransform(grid, cv2.DIST_L2, 3)
        dist *= self.map_resolution  # convert to meters

        self.publish_distance_map(dist)

        # Normalize automatically for visualization (no clipping)
        dist_norm = cv2.normalize(dist, None, 0, 255, cv2.NORM_MINMAX)
        dist_norm = dist_norm.astype(np.uint8)

        # Apply color map for nice visualization
        dist_heatmap = cv2.applyColorMap(dist_norm, cv2.COLORMAP_JET)
        # Resize to reduce image size
        dist_heatmap = cv2.resize(dist_heatmap, (512, 512), interpolation=cv2.INTER_AREA)

        self.publish_distance_heatmap(dist_heatmap)

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

    def publish_distance_heatmap(self, heatmap: np.ndarray):
        """
        Publish the distance heatmap as an Image message.

        Args:
            heatmap (np.ndarray): The heatmap to publish.
        """
        # Publish as sensor_msgs/Image
        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(heatmap, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "map"
        self.distance_heatmap_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = DistanceTransform()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()