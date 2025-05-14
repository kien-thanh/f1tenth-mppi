import csv
import os.path
import pickle
import sys
import time
import numpy as np
import subprocess
import rclpy
from rclpy.node import Node
import yaml
from std_msgs.msg import Bool, Int32, Float64
import signal
from bayes_opt import BayesianOptimization

# Configuration
# Bayesian optimizer and checkpoints
CHECKPOINT_DIR = "checkpoints_mpc"  # directory to store checkpoint files
TOTAL_INIT_POINTS = 5           # number of initial random exploration points
TOTAL_BAYES_ITER = 300          # total Bayesian optimization iterations
IS_RESUME_LAST_CHECKPOINT = True # If resuming last optimization
# assume Q=Q_e, Q_x=Q_y, take log10 of Q and R diagonal elements for easier search,
# e.g., search Q_x in [1, 10^3] -> log_Q_x in [0,3]
PARAM_BOUNDS = {
    'log_Q_x': (0, 4),
    'log_Q_theta': (0, 4),
    'log_Q_v': (0, 4),
    'log_R_a': (-2, 2),
    'log_R_delta': (-2, 2)
}
CONFIG_PATH = os.path.expanduser("~/f1Tenth/src/mpc/mpc/config.yaml")
MAX_LAPS = 2 # How many laps the car should run on the closed-loop trajectory

# Results
RESULT_DIR = "sim_results_mpc"

# Ensure the directories exists
os.makedirs(CHECKPOINT_DIR, exist_ok=True)
os.makedirs(RESULT_DIR, exist_ok=True)

# Generate a unique filename with a timestamp
TIMESTAMP = time.strftime("%Y%m%d_%H%M%S", time.localtime())
CHECKPOINT_SAVE_PATH = os.path.join(CHECKPOINT_DIR, f"checkpoint_{TIMESTAMP}.pkl")
RESULT_SAVE_PATH = os.path.join(RESULT_DIR, f"sim_result_{TIMESTAMP}.csv")


class MPCEvaluation(Node):
    def __init__(self, processes):
        super().__init__("mpc_evaluation")

        # Topics sub
        self.emergency_braking_topic = "/emergency_braking"
        self.lap_count_topic = "/lap_count"
        self.cross_track_error = "/cross_track_error"

        # Subscribers
        self.emergency_braking_sub = self.create_subscription(Bool, self.emergency_braking_topic,
                                                              self.emergency_braking_callback, 10)
        self.lap_count_sub = self.create_subscription(Int32, self.lap_count_topic, self.lap_count_callback, 10)
        self.cross_track_error_sub = self.create_subscription(Float64, self.cross_track_error, self.cross_track_error_callback, 10)

        # all ros launch/run processes
        self.processes = processes

        self.collision_detected = False
        self.current_lap = 0
        self.cross_track_error_list = []

    def emergency_braking_callback(self, msg):
        if msg.data:
            self.get_logger().info("Emergency Braking. Kill all processes. The error will be set to inf.")
            self.collision_detected = True
            terminate_all_processes(self.processes)

    def lap_count_callback(self, msg):
        self.current_lap = msg.data

    def cross_track_error_callback(self, msg):
        self.cross_track_error_list.append(msg.data)

    def get_current_lap(self):
        return self.current_lap

    def compute_total_error(self):
        cross_track_error_rmse = np.sqrt(np.mean(np.array(self.cross_track_error_list) ** 2))
        return cross_track_error_rmse


def record_to_csv(Q, R, error, result_save_path=RESULT_SAVE_PATH):
    Q_values = Q.tolist()
    R_values = R.tolist()

    with open(result_save_path, mode='a', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(Q_values + R_values + [error])


def modify_mpc_params(Q, R, config_path=CONFIG_PATH):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    model_type = config["model_type"]
    config["models"][model_type]["weights"]["Q"] = Q.tolist()
    config["models"][model_type]["weights"]["Q_e"] = Q.tolist()
    config["models"][model_type]["weights"]["R"] = R.tolist()

    with open(config_path, "w") as f:
        yaml.dump(config, f)

    print("Updated config")


def start_process(command, sleep_time=2):
    process = subprocess.Popen(command, stdout=None, stderr=None, preexec_fn=os.setpgrp, text=True)
    time.sleep(sleep_time)
    if process.poll() is not None:
        print(f"process failed: {command}")
        return None
    return process


def terminate_all_processes(processes):
    for process in processes:
        if process.poll() is None:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)

    time.sleep(1)
    os.system("pkill -f ros2")


def check_processes(processes):
    for process in processes:
        if process.poll() is not None and process.returncode != 0:
            print(f"{process.args} crashed. Terminate current simulation.")
            terminate_all_processes(processes)
            return True
    return False


def run_ros_simulation(Q, R, max_laps=MAX_LAPS):
    modify_mpc_params(Q, R)

    sim_process = start_process(["ros2", "launch", "f1tenth_gym_ros", "gym_bridge_launch.py"])
    if sim_process is None:
        return 1e10

    trajectory_planning_process = start_process(["ros2", "run", "trajectory_planning", "trajectory_planning_node"])
    if trajectory_planning_process is None:
        terminate_all_processes([sim_process])
        return 1e10

    safety_node_process = start_process(["ros2", "run", "safety_node", "safety_node"])
    if safety_node_process is None:
        terminate_all_processes([sim_process, trajectory_planning_process])
        return 1e10

    mpc_process = start_process(["ros2", "run", "mpc", "mpc_node"], sleep_time=0)
    if mpc_process is None:
        terminate_all_processes([sim_process, trajectory_planning_process, safety_node_process])
        return 1e10

    processes = [sim_process, trajectory_planning_process, safety_node_process, mpc_process]

    try:
        rclpy.init()
        node = MPCEvaluation(processes)

        start_lap = node.get_current_lap()

        while node.get_current_lap() - start_lap < max_laps:
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.collision_detected:
                return 1e10

            if check_processes(processes):
                return 1e10

        total_error = node.compute_total_error()
        return total_error

    except Exception as e:
        print(f"Error in ROS2 simulation: {e}")
        return 1e10

    finally:
        terminate_all_processes(processes)
        rclpy.shutdown()


def evaluate(log_Q_x, log_Q_theta, log_Q_v, log_R_a, log_R_delta):
    log_Q_y = log_Q_x
    Q = 10**np.array([log_Q_x, log_Q_y, log_Q_theta, log_Q_v])
    R = 10**np.array([log_R_a, log_R_delta])
    error = run_ros_simulation(Q, R)
    record_to_csv(Q, R, error)
    return -error


def save_optimizer(optimizer):
    if optimizer is not None:
        with open(CHECKPOINT_SAVE_PATH, "wb") as f:
            pickle.dump(optimizer, f)
        print(f"Checkpoint saved: {CHECKPOINT_SAVE_PATH}, completed iterations: {len(optimizer.space) - TOTAL_INIT_POINTS}")
    else:
        print("No optimizer state to save.")


def main():
    """ Main function that loads previous optimization state if available,
        or starts a new optimization process.
    """
    optimizer = None

    try:
        if IS_RESUME_LAST_CHECKPOINT:
            checkpoint_files = sorted(
                [f for f in os.listdir(CHECKPOINT_DIR) if f.startswith("checkpoint_")],
                reverse=True
            )

            if checkpoint_files:
                # resume wanted, load the latest checkpoint
                latest_checkpoint = os.path.join(CHECKPOINT_DIR, checkpoint_files[0])
                with open(latest_checkpoint, "rb") as f:
                    optimizer = pickle.load(f)
                print(f"Found existing checkpoint ({latest_checkpoint}), resuming optimization...")

            else:
                # resume wanted, but no old checkpoints, new exploration
                optimizer = BayesianOptimization(f=evaluate, pbounds=PARAM_BOUNDS, random_state=42)
                print("Starting new optimization, initializing exploration...")
                optimizer.maximize(init_points=TOTAL_INIT_POINTS, n_iter=0)
                print("Initial exploration completed!")

        else:
            # new exploration
            optimizer = BayesianOptimization(f=evaluate, pbounds=PARAM_BOUNDS, random_state=42)
            print("Starting new optimization, initializing exploration...")
            optimizer.maximize(init_points=TOTAL_INIT_POINTS, n_iter=0)
            print("Initial exploration completed!")

        try:
            completed_iter = len(optimizer.space) - TOTAL_INIT_POINTS
            remaining_iter = TOTAL_BAYES_ITER - completed_iter
            if remaining_iter <= 0:
                print("Optimization complete! Final result:")
                print(optimizer.max)
                save_optimizer(optimizer)
                return

            print(f"Remaining Bayesian optimization iterations: {remaining_iter}/{TOTAL_BAYES_ITER}")
            optimizer.maximize(init_points=0, n_iter=remaining_iter)

            print("Optimization complete! Final result:")
            print(optimizer.max)
            save_optimizer(optimizer)

        except Exception as e:
            save_optimizer(optimizer)
            sys.exit(1)

    except KeyboardInterrupt:
        print("\nManual interruption detected! Saving progress...")
        save_optimizer(optimizer)
        sys.exit(0)


if __name__ == "__main__":
    main()
