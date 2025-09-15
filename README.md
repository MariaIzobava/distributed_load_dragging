## Tutorial

https://www.bitcraze.io/2024/09/crazyflies-adventures-with-ros-2-and-gazebo/ 


## Build
```
cd <PATH_TO_REPO>/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$PWD/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/"
```


## Run

### Simulation

#### No orientation, no robot height
```
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc:=True one_robot:=True
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_two_robots:=True two_robots:=True
```

#### With load orientation, no robot height
```
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_with_ori:=True one_robot:=True

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_two_robots_with_ori:=True two_robots:=True

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_ori:=True three_robots:=True robot_num:=3

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_ori:=True four_robots:=True robot_num:=4
```


#### With load orientation and robot height
```
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_height_and_ori:=True one_robot:=True robot_num:=1

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_height_and_ori:=True two_robots:=True robot_num:=2

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_height_and_ori:=True three_robots:=True robot_num:=3
```


#### No load orientation, with robot height
```
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_with_height:=True one_robot:=True

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_height:=True one_robot:=True robot_num:=1

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_height:=True two_robots:=True robot_num:=2

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_height:=True three_robots:=True robot_num:=3
```


#### Detach 4th robot

Need to run three commands: detach drone from cable, detach cable from the load and send a command to the MPC controller that there are now 3 drones.

First run the initial command (currently works only with 4 --> 3 drones scenario!):

```
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_ori:=True four_robots:=True robot_num:=4
```

then detach the drone:

```
gz topic -t "/drone/detach" -m gz.msgs.Empty -p "unused: true"
gz topic -t "/load/detach" -m gz.msgs.Empty -p "unused: true"
ros2 topic pub /last_robot_rm std_msgs/Bool 'data: True' -1
```


#### Landing the drones
```
ros2 topic pub /land std_msgs/Bool 'data: True' -1
```


### Real

Original Bitcraze code:
```
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_real.launch.py
```

One drone, MPC without ori and height:
```
ros2 launch crazyflie_ros2_multiranger_bringup crazyflie_real.launch.py
```


#### Manual control
```
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


## Code description

### src/

* `crazyflie_ros2_multiranger` - package containing the main control logic with relevant subpackages:
    - `crazyflie_ros2_controller_cpp` - various MPC implementations
    - `crazyflie_ros2_multiranger_bringup` - launch files
    - `crazyfile_ros2_trajectory_publisher` - load trajectory publishers
* `crazyswarm2` - package relevant for real experiments
* `factor_graph_lib` - library containing the implementations of all factor graph-related logic. Used my MPC controllers from `crazyflie_ros2_controller_cpp`.
* `phasespace_motion_capture` - package for working with phase space.
* `ros_gz_crazyflie` - package for bridging Gazebo and ROS2, contains:
    - `ros_gz_crazyflie_bringup` - launch files which launch Gazebo with relevant world files
    - `ros_gz_crazyflie_control` - low level Crazyflie controller
    - `ros_gz_crazyflie_gazebo` - Gazebo SDF models


### experiments/

* `crazyflie_latency` - tested the latency of the radio signal of the actual crazyflie
* `factor_graph_one_drone_one_step` - allows both running the tuning logic on many datapoints and to run a single datapoint with extensive logging and debugging info
* `metrics` - creates the metrics from the data collected during the simulation runs. All the simulation data is written here.

 