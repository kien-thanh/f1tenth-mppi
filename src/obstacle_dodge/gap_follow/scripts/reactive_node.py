import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

"""
Only tested in the simulator with levine_blocked.png as map
"""
class ReactiveFollowGap(Node):
    """
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """

    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.sub_lidar = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.pub_lidar_disparity = self.create_publisher(LaserScan, '/dbg_lidar_disparity', 10)

        # Visualize the gap, safety bubble and move direction
        vis_topic = 'vis_gap'
        self.pub_marker = self.create_publisher(Marker, vis_topic, 10)
        self.gap, self.bubble, self.move_direction = [], [], []

    def preprocess_lidar(self, ranges, window_width=5, max_dist=2):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # 1. Setting each value to the mean over some windows
        proc_ranges = np.zeros_like(ranges)
        for i in range(len(ranges)):
            start_i = max(0, i - window_width // 2)
            end_i = min(len(ranges), i + window_width // 2 + 1)
            window = ranges[start_i:end_i]
            # filter out nan, inf
            valid_values = window[np.isfinite(window)]
            proc_ranges[i] = np.mean(valid_values)
        # 2. Rejecting high values
        proc_ranges[proc_ranges > max_dist] = max_dist
        return proc_ranges

    def disparity_extension(self, ranges):
        disparity_condition = 0.1
        extension = 50# TODO: Make dependent on distance to the car

        i = 0
        while i < len(ranges) - 1:
            if (ranges[i] - ranges[i+1] > disparity_condition):
                for j in range(max(0, i-(extension-1)), i+1):
                    ranges[j] = ranges[i+1]
                i += 1
            if i == len(ranges) - 1:
                break
            if (ranges[i+1] - ranges[i] > disparity_condition):
                for j in range(i+1, min(len(ranges), i+extension+1)):
                    ranges[j] = ranges[i]
                i += extension+1
            else:
                i += 1
        
        lidar_disparity_msg = LaserScan()
        lidar_disparity_msg.ranges = ranges.tolist()
        self.pub_lidar_disparity.publish(lidar_disparity_msg)

        return ranges


    def find_max_gap(self, free_space_ranges):
        """  Find the longest gap (consecutive non-zero sequence) in free_space_ranges.
        Returns the start index and end index of the gap as [start_i, end_i)
        """
        max_start, max_len = -1, 0  # Track the longest gap's start index and length
        current_start, current_len = -1, 0  # Track the current gap's start index and length

        for i, value in enumerate(free_space_ranges):
            if abs(value) >= 1e-3:  # Check if the current value is "non-zero"
                if current_len == 0:  # Start a new gap
                    current_start = i
                current_len += 1
            else:  # End of a gap
                if current_len > max_len:  # Update the longest gap if current one is larger
                    max_len, max_start = current_len, current_start
                current_len = 0  # Reset the current gap

        # After the loop, we may need to check if the last gap was the longest,
        # since it may not be followed by a zero, else condition is not triggered
        if current_len > max_len:
            max_len, max_start = current_len, current_start

        # If no non-zero gap was found, return None
        if max_start == -1:
            return None
        return max_start, max_start + max_len

    def find_best_point(self, ranges):
        """
        Return index of best point in ranges
        """
        # Find the maximum value
        max_value = np.max(ranges)
        # Find all indices where the range is close to the maximum value
        max_indices = np.where(np.isclose(ranges, max_value))[0]
        # Choose the middle index of the maximum value indices as the best point index
        best_point_i = max_indices[len(max_indices) // 2]
        return best_point_i

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        lidar_timestamp = data.header.stamp

        ranges = np.array(data.ranges)
        angles = np.linspace(data.angle_min, data.angle_max, len(data.ranges))

        # Preprocess lidar ranges,
        ranges = self.preprocess_lidar(ranges)

        # Disparity extended lidar ranges
        ranges = self.disparity_extension(ranges)

        # Only consider a certain angle range in front of the vehicle when finding the gap
        valid_indices = (angles >= np.radians(-60)) & (angles <= np.radians(60))
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        # Find the closest point
        closest_point_idx = np.argmin(ranges)
        closest_point_range = ranges[closest_point_idx]
        closest_point_angle = angles[closest_point_idx]

        # # Eliminate all points inside 'bubble' (set them to zero)
        # radius = 0.7
        x0, y0 = closest_point_range * np.cos(closest_point_angle), closest_point_range * np.sin(closest_point_angle)
        # for i in range(len(ranges)):
        #     x = ranges[i] * np.cos(angles[i])
        #     y = ranges[i] * np.sin(angles[i])
        #     if (x - x0) ** 2 + (y - y0) ** 2 < radius ** 2:  # point inside the bubble
        #         ranges[i] = 0.0

        # Find the max gap
        start_i, end_i = self.find_max_gap(ranges)
        ranges = ranges[start_i:end_i]
        angles = angles[start_i:end_i]

        # Find the best point in the max gap
        best_point_idx = self.find_best_point(ranges)
        best_point_range = ranges[best_point_idx]
        best_point_angle = angles[best_point_idx]

        # Visualize the safety bubble, gap and move direction
        # bubble
        # angles_circle = np.linspace(0, 2 * np.pi, 10)
        # self.bubble = [[x0 + radius * np.cos(k),
        #                 y0 + radius * np.sin(k)] for k in angles_circle]
        # gap
        self.gap = [[r * np.cos(angle), r * np.sin(angle)]
                    for r, angle in zip(ranges, angles)
                    ]
        # move direction
        direction_x = np.linspace(0, best_point_range * np.cos(best_point_angle), 10)
        direction_y = np.linspace(0, best_point_range * np.sin(best_point_angle), 10)
        self.move_direction = [[x, y] for x, y in zip(direction_x, direction_y)]
        self.pub_marker.publish(self.create_marker(lidar_timestamp))

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = best_point_angle
        if np.radians(1e-5) < abs(best_point_angle) <= np.radians(10):
            drive_msg.drive.speed = 0.5
        elif np.radians(10) < abs(best_point_angle) <= np.radians(20):
            drive_msg.drive.speed = 0.5
        else:
            drive_msg.drive.speed = 0.5

        self.pub_drive.publish(drive_msg)

    def create_marker(self, lidar_timestamp):
        marker = Marker()
        marker.header.stamp = lidar_timestamp
        marker.header.frame_id = 'ego_racecar/laser'
        marker.ns = 'points'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0

        for x, y in self.bubble + self.move_direction + self.gap:
            point = Point()
            point.x, point.y, point.z = x, y, 0.0
            marker.points.append(point)
        return marker


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
