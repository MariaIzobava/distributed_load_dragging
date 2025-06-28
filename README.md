## Tutorial

https://www.bitcraze.io/2024/09/crazyflies-adventures-with-ros-2-and-gazebo/ 

## Build

cd  ~/crazyflie_gazebo/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON

## Environment vars

source install/setup.bash

export GZ_SIM_RESOURCE_PATH="/home/$USER/distributed_load_dragging/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/"


## Run with manual control

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py

// Second terminal:

source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard


## Run with automated flying new walls

ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_simulation.launch.py
