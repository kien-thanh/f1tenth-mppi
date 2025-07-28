import os
import numpy as np
import pandas as pd
import yaml
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from trajectory_planning.utils import compute_steering_angle

"""
Perform equal-arc-length uniform sampling on the initially smoothed waypoints (see path_smoothing.py).
The adjustment of  the arc length ist based on vref, Tf and N.
And compute the heading (theta) and steering angle (delta) at each point to obtain [x, y, theta, v, delta] 
for dummy trajectory planning.
"""

mpc_config_file = '~/f1Tenth/src/mpc/mpc/config.yaml'
mpc_config_file = os.path.expanduser(mpc_config_file)
with open(mpc_config_file) as f:
    config = yaml.safe_load(f)

vref, N, Tf = 5, config["horizon"]["N"], config["horizon"]["Tf"]
L = config["L"]

# ----------------------
# Uniform Sampling Based on Arc Length
# ----------------------
# parameter
sampling_interval = vref*Tf/N  # Sampling interval in meters

# ----------------------
# 1. Read and preprocess data
# ----------------------
filepath = os.path.expanduser("~/f1Tenth/waypoints/wp-2025-07-25-17-08-42_smoothed.csv")
df = pd.read_csv(filepath, header=None)
raw_data = df.values

# Check if the trajectory is closed: if the last point is not equal to the first, add the first point to the end
# if not np.allclose(raw_data[0], raw_data[-1]):
#     raw_data = np.vstack([raw_data, raw_data[0]])

# scaling
x = 0.8*(raw_data[:, 0]) + 1.0  # [x0, x1, ..., xN-1, x0], dim N+1
y = 0.8*(raw_data[:, 1]) - 0.1  # [y0, y1, ..., yN-1, y0], dim N+1

# ----------------------
# 2. Parameterize the trajectory (using cumulative arc length)
# ----------------------
# Calculate the distance between consecutive points
dx = np.diff(x)
dy = np.diff(y)
ds = np.sqrt(dx ** 2 + dy ** 2)  # arc length between point [0->1, 1->2, ..., N-1->0], dim N

# Calculate the cumulative arc length parameter t (starting from 0)
# arc length between point [0->0, 0->1, 0->2, ..., 0->0 (total length of the closed loop)], dim N+1
t = np.insert(np.cumsum(ds), 0, 0.0)

# print("t[:10]:", t[:10])
# print("t[-10:]:", t[-10:])
# print("np.diff(t)[-10:]:", np.diff(t)[-10:])
# print("Any duplicates in t?", np.any(np.diff(t) <= 0))
# bad_indices = np.where(np.diff(t) <= 0)[0]
# print("Non-increasing t at indices:", bad_indices)
# for i in bad_indices:
#     print(f"t[{i}] = {t[i]:.6f}, t[{i+1}] = {t[i+1]:.6f}, Δ = {t[i+1] - t[i]:.6f}")
#     print(f"x[{i}] = {x[i]:.6f}, x[{i+1}] = {x[i+1]:.6f}")
#     print(f"y[{i}] = {y[i]:.6f}, y[{i+1}] = {y[i+1]:.6f}")
#     print("-" * 40)

# ----------------------
# 3. Create periodic spline interpolators
# ----------------------
# Use the cumulative arc length (t) and coordinates (x, y) to create periodic splines
cs_x = CubicSpline(t, x, bc_type='periodic')
cs_y = CubicSpline(t, y, bc_type='periodic')

# ----------------------
# 4. Uniform sampling
# ----------------------
total_length = t[-1]  # Total arc length of the trajectory

# Generate target sampling points (from 0 to total length, with fixed spacing)
num_samples = int(total_length // sampling_interval) + 1
# Exclude the endpoint (since the last point is equal to the first)
t_sampled = np.linspace(0, total_length, num_samples)

# Perform interpolation
x_sampled = cs_x(t_sampled)
y_sampled = cs_y(t_sampled)

# ----------------------
# 5. Calculate heading angles
# ----------------------
# Compute the differences between consecutive points (use periodic boundary for the last point)
dx_sampled = np.diff(x_sampled)  # Handle the last point
dy_sampled = np.diff(y_sampled)  # Handle the last point

# Compute the heading angles using arctan2, range [-pi, pi]
heading_angles = np.arctan2(dy_sampled, dx_sampled)

# ----------------------
# 6. Calculate steering angles, save the result [x, y, theta, v, delta]
# ----------------------
# remove the last point since it equals to the first one
sampled_positions = np.column_stack((x_sampled[:-1], y_sampled[:-1]))
steering_angles = compute_steering_angle(sampled_positions, L)

vs = vref*np.ones((heading_angles.shape[0], 1))
sampled_data = np.column_stack((x_sampled[:-1], y_sampled[:-1], heading_angles, vs, steering_angles))

input_dir = os.path.dirname(filepath)
input_filename = os.path.basename(filepath)
filename, file_extension = os.path.splitext(input_filename)
output_filename = f"wp_scaled_uniform_arc_length_sampling_{sampling_interval:.2f}{file_extension}"
output_path = os.path.join(input_dir, output_filename)
np.savetxt(output_path, sampled_data, delimiter=",", fmt="%.6f", header="x,y,theta,v,delta", comments="")
print(f"{output_filename} saved to {output_path}")

# ----------------------
# 7. Visualize the Results
# ----------------------
plt.figure(figsize=(10, 6), dpi=300)

# Plot the original trajectory
# plt.plot(x, y, "g-", alpha=0.4, label="Original Trajectory")

# Plot the uniformly sampled points
plt.plot(x_sampled, y_sampled, "ro", markersize=1, label="Uniform Sampling Points")

x_steering = x_sampled[:-1] + L * np.cos(heading_angles)
y_steering = y_sampled[:-1] + L * np.sin(heading_angles)

# Plot the heading angles as arrows
plt.quiver(
    x_sampled[:-1], y_sampled[:-1],  # Arrow locations
    np.cos(heading_angles), np.sin(heading_angles),  # Arrow directions
    angles="xy", scale_units="xy", scale=15,  # Arrow properties
    color="r", width=0.001, label="Heading Angles"
)

# Plot the steering angles based on the new position (after wheelbase l)
plt.quiver(
    x_steering, y_steering,  # New positions (after wheelbase shift)
    np.cos(steering_angles + heading_angles), np.sin(steering_angles + heading_angles),  # Arrow directions for steering angle
    angles="xy", scale_units="xy", scale=10,  # Adjust scale for steering arrows
    color="b", width=0.001, label="Steering Angles"
)

for i in range(len(x_sampled)-1):
    plt.plot([x_sampled[i], x_steering[i]], [y_sampled[i], y_steering[i]], 'k-', alpha=0.7, linewidth=0.2)  # Dashed line

# Plot settings
plt.axis("equal")
plt.legend()
plt.title("Uniform Sampling with Heading Angles")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.show()
