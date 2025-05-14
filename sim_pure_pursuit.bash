#!/bin/bash

gnome-terminal --tab --title="bringup" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && ros2 launch f1tenth_gym_ros gym_bridge_launch.py; exec bash"
sleep 5

gnome-terminal --tab --title="trajectory" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && ros2 run trajectory_planning trajectory_planning_node; exec bash"
sleep 5

gnome-terminal --tab --title="safety" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && ros2 run safety_node safety_node; exec bash"
sleep 5

gnome-terminal --tab --title="pure_pursuit" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && ros2 run pure_pursuit pure_pursuit_node; exec bash"
