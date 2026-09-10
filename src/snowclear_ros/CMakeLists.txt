cmake_minimum_required(VERSION 3.16)
project(snowclear_ros LANGUAGES C CXX)

# =====================================================================
# snowclear_ros — the ROS 2 layer
# ---------------------------------------------------------------------
# Contains no algorithm. It bridges a PointCloud2 topic to
# snowclear::CloudOperations and turns the ROS parameter server into a
# snowclear::ParamSource. Everything numeric lives in snowclear_core, which
# is why the live node and the offline CLI produce identical detections.
# =====================================================================

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT MSVC)
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
# Must precede PCL: PCL's config pulls in VTK, whose link interface needs the
# MPI::MPI_C target to already exist.
find_package(MPI REQUIRED COMPONENTS C)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree search)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(snowclear_core REQUIRED)

# ---------------------------------------------------------- node library
# Built as a library so it can be loaded into an existing component
# container, and reused by the standalone executable below.
add_library(snowclear_node SHARED src/snowclear_node.cpp)
target_include_directories(snowclear_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_definitions(snowclear_node PRIVATE PCL_NO_PRECOMPILE)
# pcl_conversions is header-only and exports no CMake target, so the ROS
# dependencies go through ament_target_dependencies. That helper uses the plain
# target_link_libraries signature, so this call must use the plain signature too
# -- CMake refuses to mix the two on one target.
target_link_libraries(snowclear_node snowclear_core::snowclear_core)
ament_target_dependencies(snowclear_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  pcl_conversions
)
rclcpp_components_register_nodes(snowclear_node "snowclear_ros::SnowClearNode")

# ---------------------------------------------------- standalone executable
add_executable(snowclear_node_exe src/snowclear_node_main.cpp)
set_target_properties(snowclear_node_exe PROPERTIES OUTPUT_NAME snowclear_node)
target_link_libraries(snowclear_node_exe PRIVATE snowclear_node)

# ----------------------------------------------------------------- install
install(TARGETS snowclear_node
  EXPORT export_snowclear_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS snowclear_node_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/scan_to_cloud.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_snowclear_ros)
ament_export_dependencies(rclcpp sensor_msgs std_msgs pcl_conversions snowclear_core)

# ------------------------------------------------------------------ tests
if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
