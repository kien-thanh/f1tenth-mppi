import unittest
from distance_transform import path_to_grid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TestDistanceTransform(unittest.TestCase):

    def test_path_to_grid(self):
        origin = (-10.0, -5.0)
        resolution = 1.0  # 1 meter per cell
        grid_width = 20
        grid_height = 10

        # Create a simple Path message with known poses
        path = Path()
        path.poses = []
        for i in range(10):
            pose = PoseStamped()
            x = float(i)
            y = float(i)
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)

        grid = path_to_grid(path, grid_width, grid_height, origin, resolution)

        # Should print a line of zeros from the center to the bottom right / (10,5) to (19,14) in the grid
        print(grid)

        # Check that the path points are marked with 0
        for i in range(5):
            x = int((i - (-10)) / 1)
            y = int((i - (-5)) / 1)
            self.assertEqual(grid[y, x], 0)

        # Check that other points are marked with 1
        self.assertEqual(grid[0, 0], 1)
        self.assertEqual(grid[3, 10], 1)

if __name__ == '__main__':
    unittest.main()