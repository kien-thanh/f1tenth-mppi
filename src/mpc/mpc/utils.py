import numpy as np
from vehicle_msgs.msg import VehicleState, VehicleInput

def unwrap_traj(current_yaw, traj):
    """
    Angles are periodic，e.g. -π and π radians represent the same direction. However, directly subtracting them
    gives a large error of 2π radians instead of the correct difference, which should be 0.

    In MPC, the cost function involves computing angle errors like: θ_error = θ_current - θ_ref. If an angle crosses
    the periodic boundary (e.g., from -0.99π to 0.99π), direct subtraction results in a false large error. This can
    lead to:

    - Optimization failure: The cost function becomes discontinuous, making it hard for the solver to converge.
    - Incorrect control commands: For example, a vehicle that should turn 0.02π radians clockwise might instead turn
      1.68π radians counterclockwise due to the discontinuity.

    To fix this, angle unwrapping is used. It adjusts angle sequences to be continuous over time, eliminating jumps
    and ensuring smooth optimization.

    Parameters
    ----------
    current_yaw: float
        Current yaw of the vehicle.
    traj: ndarray
        Trajectory.

    Returns
    -------
    ndarray
        Unwrapped trajectory.
    """
    # assume that the 2. column of the trajectory is yaws
    traj[:, 2] = np.unwrap(np.insert(traj[:, 2], 0, current_yaw))[1:]
    return traj


def extract_drive_command(xpred, upred, mpc_solver):
    """
    Extract speed and steering angle from predicted state and input arrays,
    considering that variable positions may differ across models., e.g.,
    model 1: x = [x, y, theta, v]^T, u = [a, delta]^T,
    model 2: x = [x, y, theta, v, delta]^T, u = [a, omega]^T.

    Parameters
    ----------
    xpred : ndarray
        Predicted state trajectory of shape (N+1, nx), where N is the horizon length.
    upred : ndarray
        Predicted input trajectory of shape (N, nu).
    mpc_solver : mpc.mpc_solver.MPCsolver
         The MPC solver containing MPC model information, including:
            - state_names: list of state variable names.
            - input_names: list of input variable names.
            - n_drive_control: integer index to extract control command.

    Returns
    -------
    speed : float
        The predicted vehicle speed at the control step, or None if not found.
    steering_angle : float
        The predicted steering angle at the control step, or None if not found.
    """
    state_dict = {name: i for i, name in enumerate(mpc_solver.state_names)}
    input_dict = {name: i for i, name in enumerate(mpc_solver.input_names)}
    n_drive_control = mpc_solver.n_drive_control

    speed, steering_angle = None, None

    for data, name_dict in [(xpred, state_dict), (upred, input_dict)]:
        if "v" in name_dict:
            speed = data[n_drive_control, name_dict["v"]]
        if "delta" in name_dict:
            steering_angle = data[n_drive_control, name_dict["delta"]]
        if speed is not None and steering_angle is not None:
            break

    return speed, steering_angle

def fill_array_message_attributes(obj, data_source, mpc_solver):
    """

    Parameters
    ----------
    obj: vehicle_msgs.msg.VehicleState or vehicle_msgs.msg.VehicleInput
        Custom ROS2 array message of vehicle state/input.
    data_source: ndarray
        An array of states or inputs in discrete time steps.
    mpc_solver : mpc.mpc_solver.MPCsolver
     The MPC solver containing MPC model information, including:
        - state_dict: dict of state variable names and their indices.
        - input_dict: dict of input variable names and their indices.

    Returns
    -------
    vehicle_msgs.msg.VehicleState or vehicle_msgs.msg.VehicleInput
        Custom ROS2 array message of vehicle state/input, with filled data fields.
    """
    if isinstance(obj, VehicleState):
        attribute_names = mpc_solver.state_names
    elif isinstance(obj, VehicleInput):
        attribute_names = mpc_solver.input_names
    else:
        raise TypeError("Object must be of type VehicleInput or VehicleState")
    if data_source.ndim == 1:
        data_source = data_source[None, :]  # shape (n_variables,) -> (1, n_variables)
    for i, name in enumerate(attribute_names):
        setattr(obj, name, data_source[:, i].tolist())
    return obj
