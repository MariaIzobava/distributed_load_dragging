## Tutorial

https://www.bitcraze.io/2024/09/crazyflies-adventures-with-ros-2-and-gazebo/ 

## Build

```
cd <PATH_TO_REPO>/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
```


## Environment vars

```
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="$PWD/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/"
```


## Run

### Simulation
```
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc:=True one_robot:=True
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_with_height:=True one_robot:=True
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_with_ori:=True one_robot:=True

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_two_robots:=True two_robots:=True
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_two_robots_with_ori:=True two_robots:=True

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_ori:=True four_robots:=True robot_num:=4
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_ori:=True three_robots:=True robot_num:=3

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_height:=True one_robot:=True robot_num:=1
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_height:=True two_robots:=True robot_num:=2
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_height:=True three_robots:=True robot_num:=3

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py graph_mpc_multi_robots_with_height_and_ori:=True one_robot:=True robot_num:=1
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



## Meeting about introduction report
1. uncrued aerial vehicles
2. impact highlevel, the things I'm doing how it can change industry.
3. chapter not hardcoded
4. formulation: for vector - slamm letter, so X small but bold
5. Jacobians are good, keep it
6. if 2 robots centralised works, experiments should be done, even without orientation. 

7. this week work orientation, next week is demo!



# 01.08.2025

1. applications of my work, snesing something on a particular height


# 08.08.2025

1. Things I've tried so far:
    * substituting tether penalty factor to tether tension factor
    * running with both analytical and numerical derivatives for dynamics
    * having trajectory reference factors for each point and for the last point
    * tried to use Marginals object to print out and examine jacobians
        * on the approaches which was working the Marginals returned "indetermined system" error
    * tried adding priors on each control variable
    * tried penalyzing big differences between subsequent controls with BetweenFactor

2. things I want yet to try:
    * return the next trajectory point not based on time but based on the nearest trajectory point
    * limit the velocity of the drones to [-0.2, 0.2] in the factor graph
    * try exhastive search
    * try position only

3. notes

* move the beginning of problem statement up
* be more specific in problem statement, e.g. robots are coordinated, follow a trajectory for load while roobts don't collide and energy optimised.
* capped letters in the Headers
* tilda before citations 
* remove CADMM if I don't have time to apply or add itto fuutre work
* Important next steps:CADMM and plan real world experiment
* centralised in real world is bigger value than CADMM
* fix position -> report -> real experiment -> CADMM -> rotation
 