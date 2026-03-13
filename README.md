# f1Tenth

## How to get started
It is highly recommended to follow this guide while being connected to the vehicle via SSH and use your local VS Code installation wih the remote development feature. It is possible to follow this guide purely on the vehicle but be prepared to disconnect and reconnect the vehicle from your display multiple times.

### Build
1. Make sure the PS4 controller is connected. Then clone the f1tenth_system repository and run in a container: 
    ```bash
    git clone https://github.tik.uni-stuttgart.de/ac133499/f1tenth_system
    cd f1tenth_system
    git switch kt-humble
    docker compose up
    ```
1. Use another terminal window to clone this repository if not already available on the machine:
    ```bash
    git clone https://github.tik.uni-stuttgart.de/ac133499/f1Tenth
    cd f1Tenth
    git switch kt
    git submodule update --init --recursive
    ```
1. Open the repository in VS Code. Press `CTRL + SHIFT + P` and select "Reopen in Container". This opens your repository in a Dev Container and you can develop and install tools wihout affecting the host. Subsequent commands are executed inside this container.

### Run

1. Launch Foxglove GUI via the application manager and connect to ws://ip-of-vehicle:8765 (localhost if running on vehicle). Then run foxglove bridge and you should be able to see LiDAR scan data
    ```
    foxglove_bridge foxglove_bridge_launch.xml
    ```

1. Start AMCL node
    ```bash
    ros2 launch f1tenth_bringup localization_launch.py map:=maps/map_1764078325.yaml
    ```
2. Initialize global localization by executing the following command:
    ```bash
    ros2 service call /reinitialize_global_localization std_srvs/srv/Empty {}
    ```
    Drive back and forth for the vehicle to find it's position on the map. If the vehicle locks into a wrong position, repeat this step.

3. To use a preconfigured global path, execute the following commands in seperate shells:
    ```bash
    ros2 launch f1tenth_bringup global_planner_launch.py

    /usr/bin/python3 /workspaces/f1Tenth/src/distance_transform/distance_transform/distance_transform.py

    ros2 topic pub --once /plan nav_msgs/msg/Path "$(cat global_path.yaml)"

    ```
    A browser window should pop up showing the global path. Sometimes the global path is not shown in Foxglove. In this case, execute the last command again or trust that it is available. If the browser pops up, the global path is processed and available.

4. Run MPPI
    ```
    ros2 launch f1tenth_bringup mppi_launch.py
    ```
    Wait until the you see "MPPI Initialized" in the output, then drive the vehicle forward to start autonomous operation.
## Create map
1. Start slam toolbox.
    ```
    ros2 launch slam_toolbox online_async_launch.py
3. Drive through the track until it is fully mapped.
2. Save the map.
    ```
    ros2 run nav2_map_server map_saver_cli

## Create global path
```
ros2 launch f1tenth_bringup global_planner_launch.py

/usr/bin/python3 /workspaces/f1Tenth/src/distance_transform/distance_transform/distance_transform.py

ros2 action send_goal /'compute_path_to_pose nav2_msgs/action/ComputePathToPose  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 8.5}}}}"
```
Replace x and y with a point directly behind the vehicle