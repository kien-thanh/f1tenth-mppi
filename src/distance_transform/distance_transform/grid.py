from __future__ import annotations

import numpy as np
import cv2
from scipy.interpolate import splprep, splev

from nav_msgs.msg import Path

def resample_path(path: np.array, spacing: float) -> np.array:
    """
    Resample the given path so that consecutive points are spaced at the given distance.
    Args:
        path (Path): The input path message.
        spacing (float): The desired spacing between consecutive points in meters.
    Returns:
        list: A list of (x, y) tuples representing the resampled path.
    """
    if not path.any():
        return []

    # Extract the x and y coordinates from the path
    x = path.T[0]
    y = path.T[1]

    # Create a B-spline representation of the path
    tck, _ = splprep([x, y], s=0)

    u_fine = np.linspace(0, 1, 100000)
    du = u_fine[1] - u_fine[0]
    dx, dy = splev(u_fine, tck, der=1)
    ds = np.sqrt(dx**2 + dy**2) * du
    cumulative_length = np.cumsum(ds)

    target_distances = np.arange(0, cumulative_length[-1], spacing)

    u_new = np.interp(target_distances, cumulative_length, u_fine)

    x_new, y_new = splev(u_new, tck)
    return np.column_stack((x_new, y_new))

class Grid:
    """
    A simple grid representation as a 2D numpy array in row-major order.
    0 indicates free space, 1 indicates occupied space.
    """
    def __init__(self, width: int, height: int, origin: tuple, resolution: float = 1.0):
        """
        Args:
            width (int): Width of the grid in number of cells.
            height (int): Height of the grid in number of cells.
            origin (tuple): Origin of the grid in world coordinates (x, y).
                            In ROS2 convention, the origin is at the bottom-left corner.
                            In OpenCV convention, the origin is at the top-left corner.
            resolution (float): Resolution of the grid in meters per cell.
        """

        self.width = width
        self.height = height
        self.origin = origin
        self.resolution = resolution
        self.data = None

    @classmethod
    def from_nav2_path(cls, path: np.array, padding: tuple, resolution: float):
        """
        Mark the grid cells corresponding to the given path as free (0).
        Args:
            path (Path): The input path message.
            padding (tuple): Padding to add around the path in meters.
            resolution (float): Resolution of the grid in meters per cell.
        Returns:
            Grid: The generated grid.
        """

        if not path.any():
            raise ValueError("Path contains no poses.")

        xs = path.T[0]
        ys = path.T[1]


        min_x = min(xs) - padding[0]
        max_x = max(xs) + padding[0]
        min_y = min(ys) - padding[1]
        max_y = max(ys) + padding[1]

        # Ceil to ensure that the grid fully contains the path with padding
        width = int(np.ceil(((max_x - min_x) / resolution) + 1e-6))
        height = int(np.ceil(((max_y - min_y) / resolution) + 1e-6))

        origin = (min_x, min_y)

        grid = cls(width, height, origin, resolution)

        # Initialize all cells as occupied
        grid.data = np.ones((height, width), dtype=np.uint8)
        for p in path:
            # Floor to ensure that points are in the cell containing them
            # Could happen that that two points are mapped to the same cell if resolution not low enough
            x = int(np.floor((p[0] - origin[0]) / resolution))
            y = int(np.floor((p[1] - origin[1]) / resolution))
            if 0 <= x < width and 0 <= y < height:
                grid.data[y, x] = 0
        return grid

    def distance_transform(self) -> Grid:
        """
        Compute the distance transform of the grid.
        Returns:
            Grid: A grid with the distance of each cell to the nearest free cell.
        """
        dist = cv2.distanceTransform(self.data, cv2.DIST_L2, 0)
        dist *= self.resolution  # convert to meters

        distance_grid = Grid(self.width, self.height, self.origin, self.resolution)
        distance_grid.data = dist

        return distance_grid
