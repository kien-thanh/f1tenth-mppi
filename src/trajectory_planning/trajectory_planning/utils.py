import numpy as np
import pandas as pd


def load_waypoints(filepath, required_columns=None, sampling_interval=None):
    """
    Load waypoints from a CSV file and ensure consistent output column order.

    Parameters
    ----------
    filepath: filepath
        Path to the CSV file, which contains columns: x, y, theta, v, delta.
    required_columns: None or list of str
        A list of column names required to load waypoints,
        e.g. ["x", "y", "theta", "v"] or ["x", "y", "theta", "v", "delta"].
    sampling_interval: None or int
        Sampling interval for waypoints (default is no sampling).

    Returns
    -------
    ndarray
       waypoints with required columns order.
    """
    df = pd.read_csv(filepath)

    # Apply sampling if specified
    if sampling_interval is not None:
        df = df.iloc[::sampling_interval]

    # Define output column order
    if required_columns is None:
        return df.to_numpy(), df.columns.tolist()
    else:
        return df[required_columns].to_numpy(), required_columns


def compute_curvature(positions):
    """
    Compute curvature from positions.

    Parameters
    ----------
    positions: ndarray
        An (n, 2) array of positions, where each row contains (x, y) coordinates (unit: m).

    Returns
    -------
    ndarray
        An (n,2) array of curvature values.
    """
    ds = np.sqrt(np.sum(np.diff(positions, axis=0) ** 2, axis=1))
    s = np.concatenate([[0], np.cumsum(ds)])
    x, y = positions[:, 0], positions[:, 1]
    dx_ds = np.gradient(x, s, edge_order=2)
    dy_ds = np.gradient(y, s, edge_order=2)
    ddx_ds = np.gradient(dx_ds, s, edge_order=2)
    ddy_ds = np.gradient(dy_ds, s, edge_order=2)
    denominator = (dx_ds ** 2 + dy_ds ** 2) ** 1.5 + 1e-6
    curvature = (dx_ds * ddy_ds - dy_ds * ddx_ds) / denominator
    return curvature


def compute_steering_angle(positions, L):
    """
    Compute steering angle from positions and wheelbase L.

    Parameters
    ----------
    positions: ndarray
        An (n, 2) array of positions, where each row contains (x, y) coordinates.
    L: float
        Car wheelbase length (unit: m).

    Returns
    -------
    ndarray
        An (n,2) array of steering angle values (unit: rad) at each position，range (-pi/2, pi/2).
    """
    curvature = compute_curvature(positions)
    deltaref = np.arctan2(curvature * L, 1.0)
    return deltaref
