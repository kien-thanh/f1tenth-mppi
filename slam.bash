#!/bin/bash

gnome-terminal --tab --title="bringup" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && ros2 launch f1tenth_stack bringup_launch.py; exec bash"
sleep 5

gnome-terminal --tab --title="slam" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py params_file:=src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml; exec bash"
sleep 5

gnome-terminal --tab --title="viz" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && rviz2 -d src/slam.rviz; exec bash"
