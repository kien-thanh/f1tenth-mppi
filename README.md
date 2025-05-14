# f1Tenth
Install ROS2 Foxy, add this line to .bashrc
``` 
source /opt/ros/foxy/setup.bash
```
## Make sure the odometry on the real car is tuned before running
## Mapping (need to be done on the real car)
### 1. Installing slam_toolbox
```
sudo apt install ros-foxy-slam-toolbox
```
### 2. Running slam_tool_box
see slam.bash
#### launch teleop
```
ros2 launch f1tenth_stack bringup_launch.py
```
#### launch slam_toolbox
```
cd $HOME/f1Tenth
ros2 launch slam_toolbox online_async_launch.py params_file:src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
```
#### visualization
- Launch 
```
rviz2 
``` 
- Add /map by topic
- Add /graph_visualization by topic
- On top left corner of rviz, panels – add new panel – add SlamToolBoxPlugin panel
- Use the joystick to move the car for mappping. Once you’re done mapping, save the map using the plugin. You can give it a name in the text box next to Save Map. Map will be saved in whichever directory you ran slam_toolbox.

## Localization
### Simulator
#### 1. Map file
Make sure the map (.pgm and .yaml) you get via mapping are placed under src/f1tenth_gym_ros/maps
### 2. Automatic localization by the simulator
### Real car
#### 1. Map fle
Make sure the map (.pgm and .yaml) you get via mapping are placed under src/particle_filter/maps 
#### 2. Installing range_libc
```
git clone https://github.com/f1tenth/range_libc.git
cd range_libc/pywrapper
sudo WITH_CUDA=ON python setup.py install
```
#### 3. Running particle filter
##### launch teleop
```
ros2 launch f1tenth_stack bringup_launch.py
```
##### launch particle filter
```
ros2 launch particle_filter localize_launch.py
```
##### visualization
- launch
```
rviz2
```
- Use add by topic, add map. In the settings for map, change durability policy under topic to transient local
- To show the current localization, add /pf/viz/inferred_pose
- Optionally, you can add /pf/viz/particles to see the particles

## Trajectory planning node
The waypoint file used by the trajectory planning node should match the corresponding vref (for Pure Pursuit)/vref, T,
N (for MPC).

| Parameter     | Value            |
|---------------|------------------|
| waypoint_file | <YOUR_FILE_HERE> |



## Safety Node
### 1. Parameters
| Parameter     | Value           |
|---------------|-----------------|
| is_real_car   | True/False      |
| ttc_threshold | <TUNING_NEEDED> |
| fov_half      | <TUNING_NEEDED> |
| range_margin  | <TUNING_NEEDED> |


### 2. Use joystick to release braking (now only for the real car):
#### (1) change in f1tenth_system/f1tenth_stack/config/mux.yaml:
ackerman_mux select between the topics /drive and /teleop based on their priority values. In the safety node, I want to use /drive to brake the car and ensure that the joystick /teleop doesn't cause the car to move when it is stopped. Therefore, /drive should have a higher priority.

#### (2) change in f1tenth_system/teleop_tools/joy_teleop/joy_teleop.py:
Use buttons 4 (L1) and 1 (Circle) to release the car from braking status.

### Simulator
```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 run safety_node safety_node
```
### Real car
```
ros2 launch f1tenth_stack bringup_launch.py
ros2 run safety_node safety_node
```

## Pure Pursuit
### 1. Parameters
#### in pure_pursuit_node.py
| Parameter                | Value            |
|--------------------------|------------------|
| is_real_car              | True/False       |
| is_work_with_safety_node | True/False       |
| config_file              | <YOUR_FILE_HERE> |
| is_tracking_log          | True/False       |

#### in config.yaml
The waypoint file used by the trajectory planning node should match the corresponding vref.

| Parameter      | Value            |
|----------------|------------------|
| l_wheelbase    |                  |
| vref           |                  |
| lookahead_dist | <TUNNING_NEEDED> |

### 2. Simulator
see sim_pure_pursuit.bash
```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 run trajectory_planning trajectory_planning_node
ros2 run safety_node safety_node
ros2 run pure_pursuit pure_pursuit_node
```
### 3. Real car
see real_car_pure_pursuit.bash
```
ros2 launch f1tenth_stack bringup_launch.py
ros2 launch particle_filter localize_launch.py
rviz2
ros2 run trajectory_planning trajectory_planning_node
ros2 run safety_node safety_node --ros-args -p is_real_car:=true
ros2 run pure_pursuit pure_pursuit_node --ros-args -p is_real_car:=true
```

## MPC
### 1. Parameter
#### in mpc_node.py

| Parameter                | Value            |
|--------------------------|------------------|
| is_real_car              | True/False       |
| is_work_with_safety_node | True/False       |
| config_file              | <YOUR_FILE_HERE> |
| is_tracking_log          | True/False       |

#### in config.yaml
The waypoint file used by the trajectory planning node should match the corresponding vref, T, N.

| Parameter  | Value                                                     |
|------------|-----------------------------------------------------------|
| L          |                                                           |
| vref       |                                                           |
| Tf         |                                                           |
| N          |                                                           |
| model_type | model_input_accel_steering/model_input_accel_steeringrate |
| Q          |                                                           |
| R          | <TUNING_NEEDED>                                           |
| Q_e        |                                                           |
| u_min      |                                                           |
| u_max      |                                                           |
| x_min      |                                                           |
| x_max      |                                                           |

### 2. Simulator
see sim_mpc.bash
```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 run trajectory_planning trajectory_planning_node
ros2 run safety_node safety_node
ros2 run mpc mpc_node
```
### 3. Real car
see real_car_mpc.bash
```
ros2 launch f1tenth_stack bringup_launch.py
ros2 launch particle_filter localize_launch.py
rviz2
ros2 run trajectory_planning trajectory_planning_node
ros2 run safety_node safety_node --ros-args -p is_real_car:=true
ros2 run mpc mpc_node --ros-args -p is_real_car:=true
```

## Parameter Tuning
See parameter_tuning_pure_pursuit.py, parameter_tuning_mpc.py
```
python3 parameter_tuning_pure_pursuit.py
python3 parameter_tuning_mpc.py
```
