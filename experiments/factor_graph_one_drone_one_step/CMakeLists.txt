cmake_minimum_required(VERSION 3.0)
project(RobotLoadTrajectory)

# This is necessary to enable FetchContent
include(FetchContent)

# 1. Declare the dependency
# This tells CMake where to find the library and gives it a name to use.
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.2 # Use a specific version for reproducibility
)

# 2. Make the dependency available to your project
# This downloads the source code and adds a target for the library.
FetchContent_MakeAvailable(nlohmann_json)

find_package(GTSAM REQUIRED)
#find_package(factor_graph_lib REQUIRED)

include_directories(
${GTSAM_INCLUDE_DIR}
)

set(ROS2_INSTALL_DIR "/home/maryia/legacy/ros2_ws/install")

# Add the include directory for your library
include_directories(${ROS2_INSTALL_DIR}/factor_graph_lib/include)

# Link against your library
link_directories(${ROS2_INSTALL_DIR}/factor_graph_lib/lib)

# Add the executable
add_executable(robot_load_trajectory main.cpp)

# Link the executable to your library
target_link_libraries(robot_load_trajectory gtsam factor_graph_lib nlohmann_json::nlohmann_json)

