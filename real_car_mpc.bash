#!/bin/bash

gnome-terminal --tab --title="bringup" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && ros2 launch f1tenth_stack bringup_launch.py; exec bash"
sleep 5

gnome-terminal --tab --title="localize" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && ros2 launch particle_filter localize_launch.py; exec bash"
sleep 5

gnome-terminal --tab --title="rviz2" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && rviz2 -d src/control.rviz; exec bash"
sleep 5

gnome-terminal --tab --title="trajectory_planing" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && ros2 run trajectory_planning trajectory_planning_node; exec bash"
sleep 5

gnome-terminal --tab --title="safety" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && ros2 run safety_node safety_node --ros-args -p is_real_car:=true; exec bash"
sleep 5

gnome-terminal --tab --title="mpc" -- bash -c "cd $HOME/f1Tenth && source install/setup.bash && ros2 run mpc mpc_node --ros-args -p is_real_car:=true; exec bash"
