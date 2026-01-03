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

    one_drone_arg = DeclareLaunchArgument(
        'one_drone',
        default_value='false'
    )

    two_drones_arg = DeclareLaunchArgument(
        'two_drones',
        default_value='false'
    )

    three_drones_arg = DeclareLaunchArgument(
        'three_drones',
        default_value='false'
    )

    four_drones_arg = DeclareLaunchArgument(
        'four_drones',
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
    gz_sim_four_drones = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        condition=IfCondition(LaunchConfiguration('four_drones')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'four_crazyflies_2_segments_middle_world.sdf -r'
            #'four_crazyflies_2_segments_middle_det_world.sdf -r'
        ])}.items()
    )
    gz_sim_three_drones = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        condition=IfCondition(LaunchConfiguration('three_drones')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'three_crazyflies_2_segments_middle_world.sdf -r'
            #'three_crazyflies_4_segments_middle_world.sdf -r'
        ])}.items()
    )
    gz_sim_two_drones = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        condition=IfCondition(LaunchConfiguration('two_drones')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            #'two_crazyflies_10_segments_bottom_world.sdf -r'
            #'two_crazyflies_4_segments_middle_world.sdf -r'
            #'two_crazyflies_2_segments_bottom_world.sdf -r'
            #'two_crazyflies_2_segments_bottom_at_angle_world.sdf -r'
            #'two_crazyflies_2_segments_middle_world.sdf -r'
            #'two_crazyflies_2_segments_middle_same_atp_world.sdf -r'
            'two_crazyflies_4_segments_middle_same_atp_world.sdf -r'
        ])}.items()
    )
    gz_sim_one_drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        condition=IfCondition(LaunchConfiguration('one_drone')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            #'crazyflie_10_segments_bottom_world.sdf -r'
            #'crazyflie_4_segments_middle_world.sdf -r'
            #'crazyflie_2_segments_bottom_world.sdf -r'
            #'crazyflie_2_segments_middle_atp2_world.sdf -r'
            'crazyflie_2_segments_middle_world.sdf -r'
        ])}.items()
    )

    bridge_four_drones = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_four_crazyflies_bridge.yaml'),
        }],

        output='screen',
        condition=IfCondition(LaunchConfiguration('four_drones'))
    )

    bridge_three_drones = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_three_crazyflies_bridge.yaml'),
        }],

        output='screen',
        condition=IfCondition(LaunchConfiguration('three_drones'))
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
        condition=IfCondition(LaunchConfiguration('one_drone'))
    )

    control_01 = Node(
        package='ros_gz_crazyflie_control',
        executable='control_services',
        output='screen',
        parameters=[
            {'hover_height': 0.2},
            {'robot_prefix': '/crazyflie_01'},
            {'incoming_twist_topic': '/cmd_vel_01'},
            {'max_ang_z_rate': 0.4},
        ]
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
        condition=UnlessCondition(LaunchConfiguration('one_drone'))
    )

    control_033 = Node(
        package='ros_gz_crazyflie_control',
        executable='control_services',
        output='screen',
        parameters=[
            {'hover_height': 0.2},
            {'robot_prefix': '/crazyflie_03'},
            {'incoming_twist_topic': '/cmd_vel_03'},
            {'max_ang_z_rate': 0.4},
        ],
        condition=IfCondition(LaunchConfiguration('three_drones'))
    )

    control_043 = Node(
        package='ros_gz_crazyflie_control',
        executable='control_services',
        output='screen',
        parameters=[
            {'hover_height': 0.2},
            {'robot_prefix': '/crazyflie_03'},
            {'incoming_twist_topic': '/cmd_vel_03'},
            {'max_ang_z_rate': 0.4},
        ],
        condition=IfCondition(LaunchConfiguration('four_drones'))
    )
    control_044 = Node(
        package='ros_gz_crazyflie_control',
        executable='control_services',
        output='screen',
        parameters=[
            {'hover_height': 0.2},
            {'robot_prefix': '/crazyflie_04'},
            {'incoming_twist_topic': '/cmd_vel_04'},
            {'max_ang_z_rate': 0.4},
        ],
        condition=IfCondition(LaunchConfiguration('four_drones'))
    )

    return LaunchDescription([
        one_drone_arg,
        two_drones_arg,
        three_drones_arg,
        four_drones_arg,
        gz_ln_arg,
        gz_sim_four_drones,
        gz_sim_three_drones,
        gz_sim_two_drones,
        gz_sim_one_drone,
        bridge_four_drones,
        bridge_three_drones,
        bridge_two_drones,
        bridge_one_drone,
        control_01,
        control_02,
        control_033,
        control_043,
        control_044
    ])