cmake_minimum_required(VERSION 3.0)
project(RobotLoadTrajectory)

find_package(GTSAM REQUIRED)
#find_package(factor_graph_lib REQUIRED)

include_directories(
${GTSAM_INCLUDE_DIR}
)

set(ROS2_INSTALL_DIR "/home/maryia/distributed_load_dragging/ros2_ws/install")

# Add the include directory for your library
include_directories(${ROS2_INSTALL_DIR}/factor_graph_lib/include)

# Link against your library
link_directories(${ROS2_INSTALL_DIR}/factor_graph_lib/lib)

# Add the executable
add_executable(robot_load_trajectory main.cpp)

# Link the executable to your library
target_link_libraries(robot_load_trajectory gtsam factor_graph_lib)

