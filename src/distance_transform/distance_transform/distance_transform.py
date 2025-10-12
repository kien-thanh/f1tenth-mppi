import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DistanceTransform(Node):

    def __init__(self):
        super().__init__('distance_transform')
        self.plan_sub = self.create_subscription(Path, 'plan', self.plan_sub_callback, 10)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.dist_heatmap = self.create_publisher(Image, 'dist_heatmap', qos)

    def plan_sub_callback(self, msg: Path):
        origin_x = -51.224998
        origin_y = -51.224998
        resolution = 0.05  # meters per cell

        grid = np.ones((2048, 2048), dtype=np.uint8)
        for pose in msg.poses:
            x = int((pose.pose.position.x - origin_x) / resolution)
            y = 2048 - 1 -int((pose.pose.position.y - origin_y) / resolution)
            if 0 <= x < 2048 and 0 <= y < 2048:
                grid[y, x] = 0 
        
        dist = cv2.distanceTransform(grid, cv2.DIST_L2, 3)
        dist *= resolution  # convert to meters

        # Normalize automatically for visualization (no clipping)
        dist_norm = cv2.normalize(dist, None, 0, 255, cv2.NORM_MINMAX)
        dist_norm = dist_norm.astype(np.uint8)

        # Apply color map for nice visualization
        dist_color = cv2.applyColorMap(dist_norm, cv2.COLORMAP_JET)

        # Publish as sensor_msgs/Image
        bridge = CvBridge()
        vis = cv2.resize(dist_color, (512, 512), interpolation=cv2.INTER_AREA)
        img_msg = bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "map"
        self.dist_heatmap.publish(img_msg)

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