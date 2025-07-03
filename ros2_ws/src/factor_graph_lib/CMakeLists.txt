cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(factor_graph_lib)

find_package(ament_cmake REQUIRED)
find_package(GTSAM REQUIRED)

# Add your include directories
include_directories(
  include
  ${GTSAM_INCLUDE_DIR}
)

# Add the library target
add_library(${PROJECT_NAME}
  src/cable_factors.cpp
  src/control_factors.cpp
  src/dynamics_factors.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} gtsam)

# Install the headers so other packages can find them
install(DIRECTORY include/
  DESTINATION include
)

# Install the library so other packages can link against it
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin  # For shared libraries on Windows
  INCLUDES DESTINATION include
)

# For shared libraries, you often want to set properties like output directory
# For static libraries, this might not be as critical but good practice.
set_target_properties(${PROJECT_NAME} PROPERTIES
  PUBLIC_HEADER "include/factor_graph_lib/factors.hpp"
)

# Export the library for other packages to find
ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})

ament_package()