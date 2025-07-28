from datetime import datetime
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.widgets import Button, RectangleSelector
from scipy.interpolate import splprep, splev
import argparse

"""
1. Manually remove outliers in the recorded trajectory.
2. B-spline smoothing.

Subsequent processing can be found in uniform_sampling_based_on_arc_length.py.

"""

def load_waypoints(filepath, sampling_interval=None):
    df = pd.read_csv(filepath)
    # Apply sampling if specified
    if sampling_interval is not None:
        df = df.iloc[::sampling_interval]
    return df.to_numpy()


class PathEditor:
    def __init__(self, points, filepath: Path):
        self.btn_reset = None
        self.btn_smooth = None
        self.points = points
        self.filepath = filepath
        self.selected_indices = set()
        self.fig, self.ax = plt.subplots(dpi=300)  # Increased dpi for higher resolution
        self.rect_selector = None
        self.rect_patch = None  # To store the highlighted rectangle
        self.init_plot()

    def init_plot(self):
        self.ax.clear()
        self.ax.plot(self.points[:, 0], self.points[:, 1], 'o', label="Original Path", markersize=0.5)
        self.ax.set_title("Select Region to Remove | Press 'Smooth' to Apply")
        self.ax.legend()
        self.ax.axis("equal")

        # Rectangle selector for region selection
        self.rect_selector = RectangleSelector(
            self.ax, self.on_select,
            useblit=False,
            button=[1],  # Left click for selection
            minspanx=5, minspany=5,
            spancoords='pixels'
        )
        self.rect_selector.set_active(True)

        # Remove previous rectangle, if any
        if self.rect_patch:
            self.rect_patch.remove()
            self.rect_patch = None

        # Add buttons
        self.add_buttons()

        plt.show()

    def add_buttons(self):
        ax_smooth = plt.axes([0.7, 0.01, 0.1, 0.05])  # Button for smoothing
        ax_reset = plt.axes([0.81, 0.01, 0.1, 0.05])  # Button for reset

        self.btn_smooth = Button(ax_smooth, 'Smooth')
        self.btn_reset = Button(ax_reset, 'Reset')

        self.btn_smooth.on_clicked(self.handle_button(self.apply_smoothing))
        self.btn_reset.on_clicked(self.handle_button(self.reset_points))

    def handle_button(self, func):
        def wrapped(event):
            func(event)
        return wrapped

    def on_select(self, eclick, erelease):
        # Get the rectangular region from the event
        xmin, ymin = eclick.xdata, eclick.ydata
        xmax, ymax = erelease.xdata, erelease.ydata

        # Find points within the selected rectangle
        selected = []
        for i, (x, y) in enumerate(self.points):
            if xmin <= x <= xmax and ymin <= y <= ymax:
                selected.append(i)

        # Add selected points to the list of indices to be removed
        self.selected_indices.update(selected)

        # Update the plot to show selected region and points to be removed
        self.update_plot(xmin, ymin, xmax, ymax)

    def update_plot(self, xmin, ymin, xmax, ymax):
        self.ax.clear()

        # Mark points to be removed with 'x'
        for i, point in enumerate(self.points):
            if i in self.selected_indices:
                self.ax.plot(point[0], point[1], 'x', color='red', markersize=1)  # Mark with 'x'
            else:
                self.ax.plot(point[0], point[1], 'o', color='blue', markersize=1)  # Plot remaining points

        # Draw the rectangle selection overlay
        if self.rect_patch:
            self.rect_patch.remove()  # Remove the previous rectangle if it exists
        self.rect_patch = plt.Rectangle(
            (xmin, ymin), xmax - xmin, ymax - ymin,
            linewidth=2, edgecolor='orange', facecolor='none', linestyle='--'
        )
        self.ax.add_patch(self.rect_patch)

        self.ax.set_title("Select Region to Remove | Press 'Smooth' to Apply")
        self.ax.axis("equal")
        plt.draw()

    def apply_smoothing(self, event):
        # Remove selected points and apply B-spline smoothing
        remaining_points = self.points[[i for i in range(len(self.points)) if i not in self.selected_indices]]

        if len(remaining_points) < 3:
            print("Not enough points to smooth!")
            return

        # B-spline smoothing
        tck, u = splprep([remaining_points[:, 0], remaining_points[:, 1]], s=1, per=True)
        u_fine = np.linspace(0, 1, 1000)  # Increase resolution of the smooth curve
        x_smooth, y_smooth = splev(u_fine, tck)
        smoothed_path = np.column_stack((x_smooth, y_smooth))

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = self.filepath.with_name(f"{self.filepath.stem}_smoothed.csv")
        np.savetxt(filename, smoothed_path, delimiter=',', fmt='%f')

        # Update plot with smoothed path
        self.ax.clear()
        self.ax.plot(remaining_points[:, 0], remaining_points[:, 1], 'o-', label="Filtered Path", markersize=1)
        self.ax.plot(smoothed_path[:, 0], smoothed_path[:, 1], '-', label="B-Spline Smoothed Path", linewidth=2)
        self.ax.legend()
        self.ax.axis("equal")
        self.ax.set_title("B-Spline Smoothed Path")
        plt.draw()

    def reset_points(self, event):
        self.selected_indices.clear()

        # Clear any rectangle overlay
        if self.rect_patch:
            self.rect_patch.remove()
            self.rect_patch = None

        # Reset the plot
        self.init_plot()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Smooth a path using B-spline and remove outliers.")
    parser.add_argument("waypoint_file", type=str, help="Path to the waypoint CSV file")
    args = parser.parse_args()

    filepath = Path(args.waypoint_file).expanduser()
    points = load_waypoints(filepath)
    points = points[450:]  # drop the beginning points
    points = np.r_[points, [points[0]]]  # form a close loop path
    editor = PathEditor(points, filepath)
