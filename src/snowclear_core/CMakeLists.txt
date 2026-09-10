cmake_minimum_required(VERSION 3.16)
project(snowclear_core LANGUAGES C CXX)

# =====================================================================
# snowclear_core — the algorithm, with no ROS dependency
# ---------------------------------------------------------------------
# Links against PCL, OpenMP and TBB only. It never links rclcpp, so the
# library, the offline CLI and the unit tests all build and run without a
# ROS installation. snowclear_ros is the only package that pulls ROS in.
#
# Built by colcon/ament; the executable names below are what the launch
# files and the regression gate call.
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

# PCL's config drags in VTK, whose link interface asks for the MPI::MPI_C
# target. That target has to exist *before* find_package(PCL) runs, otherwise
# configure fails inside VTK-targets.cmake with "MPI::MPI_C not found".
find_package(MPI REQUIRED COMPONENTS C)

find_package(PCL REQUIRED COMPONENTS common filters io kdtree search)
find_package(OpenMP REQUIRED)
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------------------------------------------------------------- library
set(SNOWCLEAR_CORE_SOURCES
  src/cloud_operations.cpp
  src/dynamic_outlier_filters.cpp
  src/evaluator.cpp
  src/feature_extractor.cpp
  src/logging.cpp
  src/param_source.cpp
  src/parameter_optimizer.cpp
  src/preprocessor.cpp
  src/sensor_calibration.cpp
  src/snow_detector.cpp
  src/system_config.cpp
)

add_library(snowclear_core SHARED ${SNOWCLEAR_CORE_SOURCES})
add_library(snowclear::core ALIAS snowclear_core)

target_include_directories(snowclear_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_compile_definitions(snowclear_core PUBLIC PCL_NO_PRECOMPILE)
target_compile_definitions(snowclear_core PRIVATE ${PCL_DEFINITIONS})

target_link_libraries(snowclear_core
  PUBLIC
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    TBB::tbb
)

# ------------------------------------------------------------- executables
# Offline tooling. Kept in this package because it is ROS-free by nature;
# it is what makes the paper's numbers checkable without a live ROS graph.
add_executable(snowclear_cli apps/snowclear_cli.cpp apps/yaml_params.cpp)
target_link_libraries(snowclear_cli PRIVATE snowclear_core yaml-cpp)

add_executable(snowclear_runner apps/snowclear_runner.cpp apps/yaml_params.cpp)
target_link_libraries(snowclear_runner PRIVATE snowclear_core yaml-cpp)

# ----------------------------------------------------------------- install
install(TARGETS snowclear_core snowclear_cli snowclear_runner
  EXPORT export_snowclear_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# ---------------------------------------------------------------- exports
ament_export_include_directories(include)
ament_export_libraries(snowclear_core)
ament_export_targets(export_snowclear_core)
# The exported target carries PCL's full link interface (flann, VTK, Boost,
# TBB, OpenMP ...), so every consumer must be able to resolve those targets.
# MPI is listed first because PCL's config pulls in VTK, whose interface needs
# MPI::MPI_C to already exist.
ament_export_dependencies(MPI PCL OpenMP TBB)

# ------------------------------------------------------------------ tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
