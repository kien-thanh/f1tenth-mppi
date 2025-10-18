import unittest
from grid import Grid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TestDistanceTransform(unittest.TestCase):

    def test_path_to_grid(self):
        # Create a simple Path message with known poses
        path = Path()
        path.poses = []
        for i in range(10):
            pose = PoseStamped()
            x = float(i) * 0.01
            y = float(i) * 0.01
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)

        grid = Grid.from_nav2_path(path, padding=(0.01, 0.01), resolution=0.01)

        # Should print a line of zeros from the center to the bottom right / (10,5) to (19,14) in the grid
        print(grid.data)

        # Check that the path points are marked with 0
        for i in range(10):
            x = int((i * 0.01 - grid.origin[0]) / grid.resolution)
            y = int((i * 0.01 - grid.origin[1]) / grid.resolution)
            self.assertEqual(grid.data[y, x], 0)

        # Check that other points are marked with 1
        self.assertEqual(grid.data[0, 0], 1)
        self.assertEqual(grid.data[3, 10], 1)

if __name__ == '__main__':
    unittest.main(exit=False)