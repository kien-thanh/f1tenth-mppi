import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float64
import subprocess
import os
import time
import signal
import yaml
import numpy as np
import datetime

SIM_OR_REAL = "real" # sim or real car
MAX_LAPS = 2 # wanted max lap count
SPEED = 2 # wanted speed
if SPEED == 2:
    # already saved result config
    SOURCE_CONFIG_PATH = "parameter_tuned/pure_pursuit_config_v_2.yaml"
    WAYPOINT_FILE = "~/f1Tenth/waypoints/wp_scaled_uniform_arc_length_sampling_0.04.csv"
elif SPEED == 3:
    SOURCE_CONFIG_PATH = "parameter_tuned/pure_pursuit_config_v_3.yaml"
    WAYPOINT_FILE = "~/f1Tenth/waypoints/wp_scaled_uniform_arc_length_sampling_0.06.csv"
else:
    raise ValueError("SPEED not supported now")
# config under the pure pursuit node dir
TARGET_CONFIG_PATH = "src/pure_pursuit/pure_pursuit/config.yaml"

class PurePursuitResult(Node):
    def __init__(self, processes):
        super().__init__("pure_pursuit_result")

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


def overwrite_config_file(source_config_path, target_config_path):
    with open(source_config_path, "r") as src_file:
        source_config = yaml.safe_load(src_file)

    with open(target_config_path, "w") as tgt_file:
        yaml.dump(source_config, tgt_file)

    print(f"Overwrote {target_config_path} with contents from {source_config_path}")


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

def start_rosbag_record(save_dir, bag_name=None):
    os.makedirs(save_dir, exist_ok=True)
    if bag_name is None:
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = f"bag_{timestamp}"
    full_bag_path = os.path.join(save_dir, bag_name)
    command = ["ros2", "bag", "record", "-a", "-o", full_bag_path]
    bag_process = subprocess.Popen(command, stdout=None, stderr=None, preexec_fn=os.setpgrp, text=True)
    time.sleep(1)
    return bag_process

def run_ros_experiment(max_laps=MAX_LAPS, sim_or_real=SIM_OR_REAL):
    overwrite_config_file(SOURCE_CONFIG_PATH, TARGET_CONFIG_PATH)
    if sim_or_real=="sim":
        bag_process = start_rosbag_record(save_dir="tracking_log_pure_pursuit_sim")

        sim_process = start_process(["ros2", "launch", "f1tenth_gym_ros", "gym_bridge_launch.py"])
        if sim_process is None:
            return 1e10

        trajectory_planning_process = start_process(["ros2", "run", "trajectory_planning", "trajectory_planning_node",
                                                     "--ros-args", "-p",
                                                     f"waypoint_file:={WAYPOINT_FILE}"
                                                     ])
        if trajectory_planning_process is None:
            terminate_all_processes([sim_process])
            return 1e10

        safety_node_process = start_process(["ros2", "run", "safety_node", "safety_node"])
        if safety_node_process is None:
            terminate_all_processes([sim_process, trajectory_planning_process])
            return 1e10

        mpc_process = start_process(["ros2", "run", "pure_pursuit", "pure_pursuit_node"], sleep_time=0)
        if mpc_process is None:
            terminate_all_processes([sim_process, trajectory_planning_process, safety_node_process])
            return 1e10

        processes = [sim_process, trajectory_planning_process, safety_node_process, mpc_process, bag_process]

    elif sim_or_real=="real":
        bag_process = start_rosbag_record(save_dir="tracking_log_pure_pursuit_real")

        real_process = start_process(["ros2", "launch", "f1tenth_stack", "bringup_launch.py"],
                                     sleep_time=4)
        if real_process is None:
            return 1e10

        localize_process = start_process(["ros2", "launch", "particle_filter", "localize_launch.py"],
                                         sleep_time=4)
        if localize_process is None:
            terminate_all_processes([real_process])
            return 1e10

        vis_process = start_process(["rviz2", "-d", "src/control.viz"])
        if vis_process is None:
            terminate_all_processes([real_process, localize_process])

        trajectory_planning_process = start_process(["ros2", "run", "trajectory_planning", "trajectory_planning_node",
                                                     "--ros-args", "-p",
                                                     f"waypoint_file:={WAYPOINT_FILE}"
                                                     ])
        if trajectory_planning_process is None:
            terminate_all_processes([real_process, localize_process, vis_process])
            return 1e10

        safety_node_process = start_process(["ros2", "run", "safety_node", "safety_node", "--ros-args", "-p",
                                             "is_real_car:=true"])
        if safety_node_process is None:
            terminate_all_processes([real_process, localize_process, vis_process, trajectory_planning_process])
            return 1e10

        mpc_process = start_process(["ros2", "run", "pure_pursuit", "pure_pursuit_node", "--ros-args", "-p",
                                     "is_real_car:=true"], sleep_time=0)
        if mpc_process is None:
            terminate_all_processes([real_process, localize_process, vis_process, trajectory_planning_process,
                                     safety_node_process])
            return 1e10

        processes = [real_process, localize_process, vis_process,
                     trajectory_planning_process, safety_node_process, mpc_process, bag_process]

    else:
        raise ValueError("Experiment type not defined")

    try:
        rclpy.init()
        node = PurePursuitResult(processes)

        start_lap = node.get_current_lap()

        while node.get_current_lap() - start_lap < max_laps:
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.collision_detected:
                return 1e10

            if check_processes(processes):
                return 1e10

        total_error = node.compute_total_error()
        print(f"Total RMSE cross-track error: {total_error}")
        return total_error

    except Exception as e:
        print(f"Error in ROS2 simulation: {e}")
        return 1e10

    finally:
        terminate_all_processes(processes)
        if bag_process.poll() is None:
            os.killpg(os.getpgid(bag_process.pid), signal.SIGTERM)
        rclpy.shutdown()


if __name__ == "__main__":
    run_ros_experiment()
