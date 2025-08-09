import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')

    # Setup to launch a crazyflie gazebo simulation from the ros_gz_crazyflie project
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_gazebo, 'launch', 'crazyflie_simulation.launch.py')),
        launch_arguments={ 
            'two_drones': LaunchConfiguration('two_robots'), 
        }.items()
    )

    # start a simple mapper node
    simple_traj_publisher = Node(
        package='crazyflie_ros2_trajectory_publisher',
        executable='trajectory_publisher',
        name='trajectory_publisher',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie_01'},
            {'use_sim_time': True}
        ]
    )

    load_path_publisher = Node(
        package='crazyflie_ros2_trajectory_publisher',
        executable='load_path_publisher',
        name='load_path_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'custom_config.rviz')

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

    ###############
    # CONTROLLERS #
    ############### 

    mpc_controller = Node(
        package='crazyflie_ros2_controller',
        executable='mpc',
        output='screen',
        parameters=[
            {'desired_height': 0.7},
            {'robot_prefix': 'crazyflie_01'},
        ],
        condition=IfCondition(LaunchConfiguration('basic_mpc'))
    )

    mpcg_controller = Node(
        package='crazyflie_ros2_contoller_cpp',
        executable='mpc',
        output='screen',
        parameters=[
            {'desired_height': 0.7},
            {'robot_prefix': 'crazyflie_01'},
        ],
        condition=IfCondition(LaunchConfiguration('graph_mpc'))
    )

    mpcg_with_angle_controller = Node(
        package='crazyflie_ros2_contoller_cpp',
        executable='mpc_with_orientation',
        output='screen',
        parameters=[
            {'desired_height': 0.7},
            {'robot_prefix': 'crazyflie_01'},
        ],
        condition=IfCondition(LaunchConfiguration('graph_mpc_with_ori'))
    )

    mpcg_two_drones_controller = Node(
        package='crazyflie_ros2_contoller_cpp',
        executable='mpc_two_drones',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'}, # indexes will be added in the code
        ],
        condition=IfCondition(LaunchConfiguration('graph_mpc_two_robots'))
    )

    mpcg_two_drones_with_angle_controller = Node(
        package='crazyflie_ros2_contoller_cpp',
        executable='mpc_two_drones_with_orientation',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'}, # indexes will be added in the code
        ],
        condition=IfCondition(LaunchConfiguration('graph_mpc_two_robots_with_ori'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'basic_mpc',
            default_value='false',
            description='Basic python MPC controller, considers cable to be always non-slack'
        ),
        DeclareLaunchArgument(
            'graph_mpc',
            default_value='false',
            description='MPC on factor graph for a single robot.'
        ),
        DeclareLaunchArgument(
            'graph_mpc_with_ori',
            default_value='false',
            description='MPC on factor graph for a single robot with load orientation.'
        ),
        DeclareLaunchArgument(
            'graph_mpc_two_robots',
            default_value='false',
            description='MPC on factor graph for 2 robots.'
        ),
        DeclareLaunchArgument(
            'graph_mpc_two_robots_with_ori',
            default_value='false',
            description='MPC on factor graph for 2 robots with load orientation.'
        ),
        DeclareLaunchArgument(
            'two_robots',
            default_value='false',
            description='Whether simulator has 2 robots or not (or has only one).'
        ),

        crazyflie_simulation,
        simple_traj_publisher,
        load_path_publisher,
        mpc_controller,
        mpcg_controller,
        mpcg_with_angle_controller,
        mpcg_two_drones_controller,
        mpcg_two_drones_with_angle_controller,
        rviz
        ])