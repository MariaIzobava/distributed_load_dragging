import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')

    # Setup to launch a crazyflie gazebo simulation from the ros_gz_crazyflie project
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_gazebo, 'launch', 'crazyflie_simulation.launch.py'))
    )

    # start a simple mapper node
    simple_traj_publisher = Node(
        package='crazyflie_ros2_trajectory_publisher',
        executable='trajectory_publisher',
        name='trajectory_publisher',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'},
            {'use_sim_time': True}
        ]
    )

    load_path_publisher = Node(
        package='crazyflie_ros2_trajectory_publisher',
        executable='load_path_publisher',
        name='load_path_publisher',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'},
            {'use_sim_time': True}
        ]
    )

    mpc_controller = Node(
        package='crazyflie_ros2_controller',
        executable='mpc',
        output='screen',
        parameters=[
            {'desired_height': 0.7},
            {'robot_prefix': 'crazyflie'},
        ]
    )

    mpcg_controller = Node(
        package='crazyflie_ros2_controller_cpp',
        executable='mpc',
        output='screen',
        parameters=[
            {'desired_height': 0.7},
            {'robot_prefix': 'crazyflie'},
        ]
    )

    # start a simple mapper node
    # simple_mapper = Node(
    #     package='crazyflie_ros2_multiranger_simple_mapper',
    #     executable='simple_mapper_multiranger',
    #     name='simple_mapper',
    #     output='screen',
    #     parameters=[
    #         {'robot_prefix': 'crazyflie'},
    #         {'use_sim_time': True}
    #     ]
    # )

    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'sim_mapping.rviz')

    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{
                "use_sim_time": True
            }]
            )

    return LaunchDescription([
        crazyflie_simulation,
        simple_traj_publisher,
        load_path_publisher,
        mpc_controller,
        mpcg_controller,
        #simple_mapper,
        rviz
        ])