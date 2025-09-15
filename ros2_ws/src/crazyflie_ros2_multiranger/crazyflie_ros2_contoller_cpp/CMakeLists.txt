cmake_minimum_required(VERSION 3.8)
project(crazyflie_ros2_contoller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(factor_graph_lib REQUIRED)
find_package(phasespace_msgs REQUIRED)
find_package(motion_capture_tracking_interfaces REQUIRED)

ament_export_include_directories(include)

include_directories(
  ${GTSAM_INCLUDE_DIRS}
)

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



add_executable(mpc src/mpc.cpp)
add_executable(mpc_two_drones src/mpc_two_drones.cpp)
add_executable(mpc_with_orientation src/mpc_with_orientation.cpp)
add_executable(mpc_two_drones_with_orientation src/mpc_two_drones_with_orientation.cpp)
add_executable(mpc_multi_drones_with_orientation src/mpc_multi_drone_with_orientation.cpp)
add_executable(mpc_with_height src/mpc_with_height.cpp)
add_executable(mpc_multi_drone_with_height src/mpc_multi_drone_with_height.cpp)
add_executable(mpc_multi_drone_with_height_and_ori src/mpc_multi_drone_with_height_and_ori.cpp)

ament_target_dependencies(
  mpc
  rclcpp
  GTSAM
  nav_msgs
  tf2
  factor_graph_lib
  phasespace_msgs
  motion_capture_tracking_interfaces
)
ament_target_dependencies(
  mpc_two_drones
  rclcpp
  GTSAM
  nav_msgs
  tf2
  factor_graph_lib
)
ament_target_dependencies(
  mpc_with_orientation
  rclcpp
  GTSAM
  nav_msgs
  tf2
  factor_graph_lib
)
ament_target_dependencies(
  mpc_two_drones_with_orientation
  rclcpp
  GTSAM
  nav_msgs
  tf2
  factor_graph_lib
)
ament_target_dependencies(
  mpc_with_height
  rclcpp
  GTSAM
  nav_msgs
  tf2
  factor_graph_lib
)
ament_target_dependencies(
  mpc_multi_drones_with_orientation
  rclcpp
  GTSAM
  nav_msgs
  tf2
  factor_graph_lib
)
ament_target_dependencies(
  mpc_multi_drone_with_height
  rclcpp
  GTSAM
  nav_msgs
  tf2
  factor_graph_lib
)
ament_target_dependencies(
  mpc_multi_drone_with_height_and_ori
  rclcpp
  GTSAM
  nav_msgs
  tf2
  factor_graph_lib
)

target_link_libraries(mpc
  gtsam
  gtsam_unstable
  nlohmann_json::nlohmann_json
)
target_link_libraries(mpc_two_drones
  gtsam
  gtsam_unstable
  nlohmann_json::nlohmann_json
)
target_link_libraries(mpc_with_orientation
  gtsam
  gtsam_unstable
  nlohmann_json::nlohmann_json
)
target_link_libraries(mpc_two_drones_with_orientation
  gtsam
  gtsam_unstable
  nlohmann_json::nlohmann_json
)
target_link_libraries(mpc_with_height
  gtsam
  gtsam_unstable
  nlohmann_json::nlohmann_json
)
target_link_libraries(mpc_multi_drones_with_orientation
  gtsam
  gtsam_unstable
  nlohmann_json::nlohmann_json
)
target_link_libraries(mpc_multi_drone_with_height
  gtsam
  gtsam_unstable
  nlohmann_json::nlohmann_json
)
target_link_libraries(mpc_multi_drone_with_height_and_ori
  gtsam
  gtsam_unstable
  nlohmann_json::nlohmann_json
)

target_include_directories(mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mpc PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(mpc_two_drones PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mpc_two_drones PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(mpc_with_orientation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mpc_with_orientation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(mpc_two_drones_with_orientation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mpc_two_drones_with_orientation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(mpc_with_height PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mpc_with_height PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(mpc_multi_drones_with_orientation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mpc_multi_drones_with_orientation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(mpc_multi_drone_with_height PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mpc_multi_drone_with_height PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(mpc_multi_drone_with_height_and_ori PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mpc_multi_drone_with_height_and_ori PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mpc
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS mpc_two_drones
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS mpc_with_orientation
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS mpc_two_drones_with_orientation
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS mpc_with_height
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS mpc_multi_drones_with_orientation
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS mpc_multi_drone_with_height
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS mpc_multi_drone_with_height_and_ori
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
