import subprocess
import time
import os

# --- Configuration Paths ---
# Here are some settings in the simulator, find suitable settings for the real car
MAP_PGM_PATH = "src/f1tenth_gym_ros/maps/map_1740576087s.pgm"
MAP_YAML_PATH = "src/f1tenth_gym_ros/maps/map_1740576087.yaml"
BAG_PATH = "bag_20250508_104518"  # Your rosbag file here
RVIZ_CONFIG_PATH = "src/f1tenth_gym_ros/launch/gym_bridge_new.rviz"

# Name of the map server node, default is /map_server
MAP_SERVER_NODE_NAME = "/map_server"

def execute_ros_command(command, description=""):
    """Helper function to execute ROS commands and print their status."""
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate() # Wait for command to complete
    if process.returncode != 0:
        print(f"Error executing {description}:")
        print(f"STDOUT: {stdout.decode()}")
        print(f"STDERR: {stderr.decode()}")
        return False
    return True

def start_map_server():
    print("Starting map_server...")
    command = [
        "ros2", "run", "nav2_map_server", "map_server",
        "--ros-args",
        "-p", f"yaml_filename:={os.path.abspath(MAP_YAML_PATH)}",
        "-p", "use_sim_time:=true"
    ]
    map_process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    print("Waiting for map_server node to be discoverable...")
    time.sleep(1)

    # Configure and activate map_server
    if not execute_ros_command([
        "ros2", "lifecycle", "set", MAP_SERVER_NODE_NAME, "configure"
    ], "Configure map_server"):
        map_process.terminate()
        map_process.wait()
        return None
    time.sleep(1)

    if not execute_ros_command([
        "ros2", "lifecycle", "set", MAP_SERVER_NODE_NAME, "activate"
    ], "Activate map_server"):
        map_process.terminate()
        map_process.wait()
        return None

    return map_process

def start_rosbag_play():
    command = [
        "ros2", "bag", "play", os.path.abspath(BAG_PATH),
        "--rate", "1.0", "--loop"
    ]
    return subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def start_rviz():
    command = [
        "rviz2", "-d", os.path.abspath(RVIZ_CONFIG_PATH),
        "--ros-args", "-p", "use_sim_time:=true"
    ]
    return subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def main():
    map_process = None
    bag_process = None
    rviz_process = None

    try:
        map_process = start_map_server()
        if not map_process:
            print("Map server failed to start. Exiting.")
            return

        time.sleep(1)

        bag_process = start_rosbag_play()
        time.sleep(1)

        rviz_process = start_rviz()

        print("Simulation running... Press Ctrl+C to stop.")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping all processes due to KeyboardInterrupt...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        processes_to_terminate = []
        if map_process and map_process.poll() is None:
            processes_to_terminate.append(("Map Server", map_process))
        if bag_process and bag_process.poll() is None:
            processes_to_terminate.append(("Rosbag Play", bag_process))
        if rviz_process and rviz_process.poll() is None:
            processes_to_terminate.append(("RViz2", rviz_process))

        for name, proc in processes_to_terminate:
            proc.terminate() # Send SIGTERM

        for name, proc in processes_to_terminate:
            try:
                proc.wait(timeout=5) # Wait up to 5 seconds
            except subprocess.TimeoutExpired:
                proc.kill() # Send SIGKILL
                proc.wait()

        print("All specified processes have been requested to stop.")

if __name__ == "__main__":
    main()
