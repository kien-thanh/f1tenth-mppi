from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import atan2, sqrt
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import csv
import os
from datetime import datetime
import atexit

def preprocess_waypoints(waypoints):
    """
    Preprocess waypoints (assume closed-loop trajectory) to accelerate subsequent computations, e.g.,
    the nearst projection point of the car on the trajectory.

    Parameters
    ----------
    waypoints: ndarray
        Each row contains at least x,y coordinates

    Returns
    -------
    waypoints_positions: ndarray
        An (n,2) array of waypoints x,y positions
    segment_vectors: ndarray
        An (n, 2) array of segment vectors between consecutive waypoints,
        assuming a closed loop (last vector from n−1 to 0).
    segment_lengths_squared: ndarray
        (n,), squared lengths of segment vectors.
    segment_lengths:
        (n,), lengths of segment vectors.
    cumulative_lengths:
        (n+1,), cumulative arc lengths from the start (0) to points 0, 1, ..., n−1,
        and back to 0 in a closed-loop trajectory.

    """
    waypoints_positions = waypoints[:, :2]  # shape (n, 2)
    # s: 0, 1, ... , n-2, n-1; e:1, 2, ... , n-1, 0
    starts, ends = waypoints_positions, np.roll(waypoints_positions, -1, axis=0)  # shape (n,2)
    # vector from s to e (s,e): (0,1), (1,2), ..., (n-2,n-1), (n-1,0)
    segment_vectors = ends - starts  # shape (n,2)
    segment_lengths_squared = np.sum(segment_vectors ** 2, axis=1)
    segment_lengths = np.sqrt(segment_lengths_squared)  # shape (n,)
    # length from s to e: (0,0), (0,1), (0,2), ..., (0,n-1), (0, 0 (total loop length))
    cumulative_lengths = np.insert(np.cumsum(segment_lengths), 0, 0)
    return waypoints_positions, segment_vectors, segment_lengths_squared, segment_lengths, cumulative_lengths


class WaypointsProcessor:
    """
    A class for processing waypoints.

    Parameters
    ----------
    waypoints : ndarray
        A (n,2) array of 2D waypoints.

    Attributes
    ----------
    waypoints_positions: ndarray
        An (n,2) array of waypoints x,y positions.
    segment_vectors: ndarray
        An (n, 2) array of segment vectors between consecutive waypoints,
        assuming a closed loop (last vector from n−1 to 0).
    segment_lengths_squared: ndarray
        (n,), squared lengths of segment vectors.
    segment_lengths:
        (n,), lengths of segment vectors.
    cumulative_lengths:
        (n+1,), cumulative arc lengths from the start (0) to points 0, 1, ..., n−1,
        and back to 0 in a closed-loop trajectory.
    """
    def __init__(self, waypoints):
        """
        Initializes the WaypointsProcessor object and computes related information.

        Parameters
        ----------
        waypoints : ndarray
            An (n,2) array of waypoints x,y positions.
        """
        (
         self.waypoints_positions,
         self.segment_vectors,
         self.segment_lengths_squared,
         self.segment_lengths,
         self.cumulative_lengths
        ) = preprocess_waypoints(waypoints)

    def find_closest_waypoint_idx(self, current_position):
        """
        Finds the index of the waypoint closest to the current position
        based on accurate projection on the trajectory.

        Parameters
        ----------
        current_position : ndarray
            [x,y]

        Returns
        -------
        closest_waypoint_idx: int
            The index of the closest waypoint.
        current_progress: float
            The current position's projection progress on the trajectory,
            how far along the trajectory that projection point is, ranging from 0% to 100%.
        cross_track_error: float
            The distance (unit: m) from the current position to the nearest projection point on the trajectory.
        proj: ndarray
            The nearst projection of the current position on the trajectory.
        closest_t: float
            The project factor t on the segment closest_waypoint_idx -> closest_waypoint_idx+1

        Note
        ----
        projection factor t: the ratio of the projection of a on b to the length of b
        t = dot(point_vector, segment_vector) / segment_length**2
        t = 0/1 → projection is at the segment start/end point.
        t < 0 → projection is outside the start point,
                clamped to 0 (the nearest point on the segment to the current position is the start).
        t > 1 → projection is outside the end point,
                clamped to 1 (the nearest point on the segment to the current position is the end).
        """
        point_vectors = current_position - self.waypoints_positions  # shape (n,2)
        t = np.sum(point_vectors * self.segment_vectors, axis=1) / self.segment_lengths_squared # shape (n,)
        t = np.clip(t, 0.0, 1.0)
        projections = self.waypoints_positions + t[:, None] * self.segment_vectors  # shape (n,2)
        dists_squared = np.sum((projections - current_position) ** 2, axis=1)  # shape (n,)
        closest_waypoint_idx = np.argmin(dists_squared)
        # The project factor t on the segment closest_waypoint_idx -> closest_waypoint_idx+1
        proj = projections[closest_waypoint_idx]
        closest_t = t[closest_waypoint_idx]

        cross_track_error = np.sqrt(dists_squared[closest_waypoint_idx])

        # cumulative length from 0 to closest_waypoint_idx
        # + projection length on the segment closest_waypoint_idx -> closest_waypoint_idx+1
        current_length = (self.cumulative_lengths[closest_waypoint_idx]
                          + closest_t*self.segment_lengths[closest_waypoint_idx])
        current_progress = current_length/ self.cumulative_lengths[-1]

        return closest_waypoint_idx, current_progress, cross_track_error, proj, closest_t

    def find_closest_waypoint_idx_simple(self, current_position):
        """
        Finds the index of the waypoint closest to the current position
        based on the shortest distance to each waypoint.

        Parameters
        ----------
        current_position : ndarray
            [x,y]

        Returns
        -------
        closest_waypoint_idx: int
            The index of the closest waypoint.
        current_progress: float
            The current position's projection progress on the trajectory, ranging from 0% to 100%.
        cross_track_error: float
            The distance (unit: m) from the current position to the closest waypoint point.
        """
        dists_squared = np.sum((self.waypoints_positions - current_position) ** 2, axis=1, keepdims=True)
        closest_waypoint_idx = np.argmin(dists_squared)
        cross_track_error = np.sqrt(dists_squared[closest_waypoint_idx])
        current_length = self.cumulative_lengths[closest_waypoint_idx]
        current_progress = current_length / self.cumulative_lengths[-1]

        return closest_waypoint_idx, current_progress, cross_track_error


class LapProcessor:
    """
    A class for processing waypoints.

    Attributes
    ----------
    prev_progress: None or float
        How far along the trajectory the previous position's projection point is, ranging from 0% to 100%.
    total_progress: float
        Cumulative progress, e.g., if one and a half laps have been completed, 150%.
    lap_count: int
        Current lap count.

    """
    def __init__(self):
        """
        Initializes the LapProcessor object

        Parameters:
        ----------
        prev_progress: None or float
            How far along the trajectory the previous position's projection point is, ranging from 0% to 100%.
            Defaults to None.
        total_progress: float
            Cumulative progress, e.g., if one and a half laps have been completed, 150%.
            Defaults to 0.0.
        lap_count: int
            Current lap count.
            Defaults to 0.
        """
        self.prev_progress = None
        self.total_progress = 0.0
        self.lap_count = 0

    def update(self, progress):
        """
        Get the current lap count.

        Parameters
        ----------
        progress: float
            How far along the trajectory the current position's projection point is, ranging from 0% to 100%.

        Returns
        -------
        int
           Current lap count.
        """
        if self.prev_progress is None:
            self.prev_progress = progress
            return self.lap_count

        delta = progress - self.prev_progress

        # Handling the wraparound case (progress from 0.95 -> 0.05), the absolute value of the jump > 0.5
        if delta < -0.5:
            delta += 1

        self.total_progress += delta
        self.lap_count = int(self.total_progress)
        self.prev_progress = progress

        return self.lap_count


def init_marker(frame_id, orientation=None, scale=None, color=None):
    """
    Initializes ROS2 marker.

    Parameters
    ----------
    frame_id : str
        The reference coordinate frame for the marker (e.g., "map", "base_link").
    orientation : list of float, optional
        The orientation of the marker in quaternion format [x, y, z, w].
        Defaults to [0.0, 0.0, 0.0, 1.0] (no rotation).
    scale : list of float, optional
        The scale of the points in x and y directions. Default is [0.04, 0.04].
    color : list of float, optional
        The color of the marker in [r, g, b, a] format, each from 0.0 to 1.0.
        Default is black with full opacity: [0.0, 0.0, 0.0, 1.0].

    Returns
    -------
    marker : visualization_msgs.msg.Marker
        The initialized marker ready to be published.
    """
    orientation = orientation or [0.0, 0.0, 0.0, 1.0]
    scale = scale or [0.04, 0.04]
    color = color or [0.0, 0.0, 0.0, 1.0]  # black
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = orientation
    marker.scale.x, marker.scale.y = scale
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    return marker


def publish_marker(traj_points, marker, marker_publisher):
    """
    Publish ROS2 marker.

    Parameters
    ----------
    traj_points: ndarray
        An (n, 2) or (2,) array of trajectory points, where each row contains at least x and y coordinates.
    marker: visualization_msgs.msg.Marker
        The ROS2 marker to be published.
    marker_publisher: rclpy.publisher.Publisher[visualization_msgs.msg.Marker]
        The ROS2 maker publisher
    """
    marker.points.clear()
    marker.points = [Point(x=p[0], y=p[1], z=0.0) for p in np.atleast_2d(traj_points)]
    marker_publisher.publish(marker)


def quaternion_to_yaw(q):
    """
    Quaternion to yaw (around the z-axis)

    Parameters
    ----------
    q: geometry_msgs.msg.Quaternion
        ROS2 quaternion object
    Returns
    -------
    float
        yaw (unit: rad), range [-pi, pi].
    """
    sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
    cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(sin_yaw, cos_yaw)

def publish_drive(steering_angle, speed, drive_publisher):
    """
    Publishes a drive command with specified steering angle and speed.

    Parameters
    ----------
    steering_angle : float
        The desired steering angle (unit: rad).
    speed : float
        The desired speed of the vehicle (unit: m/s).
    drive_publisher : rclpy.publisher.Publisher[ackermann_msgs.msg.AckermannDriveStamped]
        A ROS2 AckermannDriveStamped message publisher
    """
    ackermann_drive_cmd = AckermannDriveStamped()
    ackermann_drive_cmd.drive.steering_angle = float(steering_angle)
    ackermann_drive_cmd.drive.speed = float(speed)
    drive_publisher.publish(ackermann_drive_cmd)


class TrackingLogger:
    """
    Log data in the controller for performance analysis.
    """
    FIELDNAMES = [
        "time",
        "current_x",
        "current_y",
        "current_theta",
        "current_v",
        "ref_x",
        "ref_y",
        "ref_theta",
        "ref_v",
        "proj_x",
        "proj_y",
        "progress",
        "lap",
        "cross_track_error"
    ]

    def __init__(self, save_dir="/tracking_log", filename="tracking_log.csv", timestamp_filename=True):
        os.makedirs(save_dir, exist_ok=True)

        if timestamp_filename:
            base, ext = os.path.splitext(filename)
            now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{base}_{now_str}{ext}"

        self.file_path = os.path.join(save_dir, filename)
        self.file = open(self.file_path, mode="w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()

        self.last_logged_data = None

        atexit.register(self.close)

    def log(self, time, current_x, current_y, current_theta, current_v,
            ref_x, ref_y, ref_theta, ref_v, proj_x, proj_y,
            progress, lap, cross_track_error):
        current_data = (
        current_x, current_y, current_theta, current_v, ref_x, ref_y, ref_theta, ref_v, proj_x, proj_y, progress,
        lap, cross_track_error)

        # avoid redundant data
        if self.last_logged_data and self.data_is_similar(current_data, self.last_logged_data):
            return

        row = {
            "time": time,
            "current_x": current_x,
            "current_y": current_y,
            "current_theta": current_theta,
            "current_v": current_v,
            "ref_x": ref_x,
            "ref_y": ref_y,
            "ref_theta": ref_theta,
            "ref_v": ref_v,
            "proj_x": proj_x,
            "proj_y": proj_y,
            "progress": progress,
            "lap": lap,
            "cross_track_error": cross_track_error
        }
        self.writer.writerow(row)
        self.file.flush()

        self.last_logged_data = current_data

    def data_is_similar(self, data1, data2, threshold=1e-4):
        return all(abs(d1 - d2) < threshold for d1, d2 in zip(data1, data2))

    def close(self):
        if not self.file.closed:
            self.file.close()
