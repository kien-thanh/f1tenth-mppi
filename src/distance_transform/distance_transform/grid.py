import numpy as np

from nav_msgs.msg import Path

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

    @classmethod
    def from_nav2_path(cls, path: Path, padding: tuple, resolution: float):
        """
        Mark the grid cells corresponding to the given path as free (0).
        Args:
            path (Path): The input path message.
            padding (tuple): Padding to add around the path in meters.
            resolution (float): Resolution of the grid in meters per cell.
        """

        if not path.poses:
            raise ValueError("Path contains no poses.")

        xs = [pose.pose.position.x for pose in path.poses]
        ys = [pose.pose.position.y for pose in path.poses]

        min_x = min(xs) - padding[0]
        max_x = max(xs) + padding[0]
        min_y = min(ys) - padding[1]
        max_y = max(ys) + padding[1]

        # Ceil to ensure that the grid fully contains the path with padding
        width = int(np.ceil(((max_x - min_x) / resolution) + 1e-6))
        height = int(np.ceil(((max_y - min_y) / resolution) + 1e-6))

        origin = (min_x, min_y)

        grid = cls(width, height, origin, resolution)
        grid.data = np.ones((height, width), dtype=np.uint8)
        for pose in path.poses:
            # Floor to ensure that points are in the cell containing them
            # Could happen that that two points are mapped to the same cell if resolution not low enough
            x = int(np.floor((pose.pose.position.x - origin[0]) / resolution))
            y = int(np.floor((pose.pose.position.y - origin[1]) / resolution))
            if 0 <= x < width and 0 <= y < height:
                grid.data[y, x] = 0
        return grid
