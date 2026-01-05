# f1Tenth

## Dev Container

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
    ros2 launch f1tenth_bringup localization_launch.py map:=<path/to/map.yaml>
2. Initialize global localization by executing the following command and drive back and forth
    ```
    ros2 service call /reinitialize_global_localization std_srvs/srv/Empty {}

## Global path
