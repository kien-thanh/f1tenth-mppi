# f1Tenth

## How to get started
### Requirements
* PS4 controller is connected
* SSH connection to the vehicle

### Build
1. Clone f1tenth_system repository
    ```
    git clone https://github.tik.uni-stuttgart.de/ac133499/f1Tenth
    cd f1tenth_system
    git switch kt-humble
1. Clone repository if not already available
    ```bash
    git clone https://github.tik.uni-stuttgart.de/ac133499/f1Tenth
    cd f1Tenth
    git switch kt
    ```
1. Clone MPPI-Generic submodule
    ```bash
    git submodule update --init --recursive
    ```
1. Build
    ```bash
    colcon build --symlink-install --cmake-args -GNinja -DCMAKE_CUDA_ARCHITECTURES=87
    ```

Launch foxglove
    ```
    foxglove_bridge foxglove_bridge_launch.xml
    ```

## Create map
1. Start slam toolbox.
    ```
    ros2 launch slam_toolbox online_async_launch.py
3. Drive through the track until it is fully mapped.
2. Save the map.
    ```
    ros2 run nav2_map_server map_saver_cli

## Localization
1. Start AMCL node
    ```
    ros2 launch f1tenth_bringup localization_launch.py map:=maps/map_1764078325.yaml
2. Initialize global localization by executing the following command and drive back and forth
    ```
    ros2 service call /reinitialize_global_localization std_srvs/srv/Empty {}

## Global path
ros2 launch f1tenth_bringup global_planner_launch.py

/usr/bin/python3 /workspaces/f1Tenth/src/distance_transform/distance_transform/distance_transform.py

ros2 topic pub --once /plan nav_msgs/msg/Path "$(cat global_path.yaml)"