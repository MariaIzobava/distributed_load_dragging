## Tutorial

https://www.bitcraze.io/2024/09/crazyflies-adventures-with-ros-2-and-gazebo/ 

## Build

```
cd  ~/crazyflie_gazebo/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
```


## Environment vars

```
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="/home/$USER/distributed_load_dragging/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/"
```


## Run

### Simulation
```
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py
```

### Real
```
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_real.launch.py
```


## Manual control

```
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


## Run with automated flying new walls

ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_simulation.launch.py


## Usage

To land the drone:

```
ros2 topic pub /land std_msgs/Bool 'data: True' -1
```


## Meeting with Valerio

1. What are these motion capture things? 

the issue: radio connection is slow. can't run solver on board. MPC normally is not delay compensation. one thing: adding very slow mpc + something that is much faster old solution + 

send control action from mpc, then update 

what is the sending time of the drone. If it's too slow then add receiver on the drone. 

need to find a delay on the crazyflie channel then we can consider improving hardware 

mpc on crazyflie 

