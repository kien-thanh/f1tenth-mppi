import plotly.express as px
import numpy as np
import time

from nav_msgs.msg import Path

def path(path: Path):
    """
    Plot the given nav2 path on a 2D grid.
    """
    xs = [pose.pose.position.x for pose in path.poses]
    ys = [pose.pose.position.y for pose in path.poses]

    fig = px.scatter(
        x=xs, y=ys, 
        title="Path Visualization",
        labels={"x": "X", "y": "Y"}
    )
    fig.show()

def heatmap(data: np.ndarray, title="Heatmap"):
    """
    Plot a heatmap of the given 2D data array.
    """
    fig = px.imshow(
        data,
        color_continuous_scale='Viridis',
        title=title,
        labels={"x": "X", "y": "Y", "color": "Value"}
    )
    fig.show()