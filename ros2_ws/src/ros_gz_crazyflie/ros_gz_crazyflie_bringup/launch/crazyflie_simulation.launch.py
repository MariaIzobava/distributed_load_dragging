# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    two_drones_arg = DeclareLaunchArgument(
        'two_drones',
        default_value='false'
    )

    gz_ln_arg = DeclareLaunchArgument(
        'gazebo_launch',
        default_value='True'
    )
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim_two_drones = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        condition=IfCondition(LaunchConfiguration('two_drones')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'two_crazyflies_with_less_segments_world.sdf -r'
            #'two_crazyflies_with_load_world.sdf -r'
        ])}.items()
    )
    gz_sim_one_drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        condition=UnlessCondition(LaunchConfiguration('two_drones')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            #'crazyflie_with_load_many_segments_world.sdf -r'
            'crazyflie_with_load_in_the_middle_world.sdf -r'
        ])}.items()
    )

    bridge_two_drones = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_two_crazyflies_bridge.yaml'),
        }],

        output='screen',
        condition=IfCondition(LaunchConfiguration('two_drones'))
    )

    bridge_one_drone = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_crazyflie_bridge.yaml'),
        }],

        output='screen',
        condition=UnlessCondition(LaunchConfiguration('two_drones'))
    )

    # control = Node(
    #     package='ros_gz_crazyflie_control',
    #     executable='control_services',
    #     output='screen',
    #     parameters=[
    #         {'hover_height': 0.2},
    #         {'robot_prefix': '/crazyflie'},
    #         {'incoming_twist_topic': '/cmd_vel'},
    #         {'max_ang_z_rate': 0.4},
    #     ],
    #     condition=UnlessCondition(LaunchConfiguration('two_drones'))
    # )


    control_01 = Node(
        package='ros_gz_crazyflie_control',
        executable='control_services',
        output='screen',
        parameters=[
            {'hover_height': 0.2},
            {'robot_prefix': '/crazyflie_01'},
            {'incoming_twist_topic': '/cmd_vel_01'},
            {'max_ang_z_rate': 0.4},
        ] #,
        #condition=IfCondition(LaunchConfiguration('two_drones'))
    )

    control_02 = Node(
        package='ros_gz_crazyflie_control',
        executable='control_services',
        output='screen',
        parameters=[
            {'hover_height': 0.2},
            {'robot_prefix': '/crazyflie_02'},
            {'incoming_twist_topic': '/cmd_vel_02'},
            {'max_ang_z_rate': 0.4},
        ],
        condition=IfCondition(LaunchConfiguration('two_drones'))
    )

    return LaunchDescription([
        two_drones_arg,
        gz_ln_arg,
        gz_sim_two_drones,
        gz_sim_one_drone,
        bridge_two_drones,
        bridge_one_drone,
        #control,
        control_01,
        control_02
                ])