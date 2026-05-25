cmake_minimum_required(VERSION 3.14)
project(ros2_perf VERSION 0.0.1 LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# ROS 2 environment guard
# Source a ROS 2 underlay first, e.g.:
#   source /opt/ros/jazzy/setup.bash
# ---------------------------------------------------------------------------
if(NOT DEFINED ENV{AMENT_PREFIX_PATH})
  message(
    FATAL_ERROR
    "AMENT_PREFIX_PATH is not set. Source a ROS 2 setup.bash before "
    "configuring, or CMake may fail to find ament/rclcpp/rosidl packages.")
endif()

# ---------------------------------------------------------------------------
# Global compiler settings
# ---------------------------------------------------------------------------
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile_commands.json")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Options
# Declare BUILD_TESTING *before* find_package(ament_cmake) so our default
# (OFF) takes precedence over ament's default (ON).
# ---------------------------------------------------------------------------
option(BUILD_TESTING        "Build unit tests (gtest)"              OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------------------------------------------------------------------------
# Message generation (headers: ros2_perf/msg/uks.hpp)
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Uks.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---------------------------------------------------------------------------
# Version header
# ---------------------------------------------------------------------------
configure_file(
  src/version.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/ros2_perf/version.h
  @ONLY
)

# ---------------------------------------------------------------------------
# Core library (STATIC — shared by the binary and unit tests)
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}_core STATIC
  src/args.cpp
  src/stats.cpp
  src/report.cpp
  src/mode_utils.cpp
  src/mode/pub_mode.cpp
  src/mode/sub_mode.cpp
  src/mode/ping_mode.cpp
  src/mode/pong_mode.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core rclcpp)

target_link_libraries(${PROJECT_NAME}_core
  "${cpp_typesupport_target}"
)

# ---------------------------------------------------------------------------
# Executable
# Target name must differ from ${PROJECT_NAME}: rosidl_generate_interfaces
# registers a custom target with that name.
# ---------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_bin src/main.cpp)

set_target_properties(${PROJECT_NAME}_bin PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}
  INSTALL_RPATH "\$ORIGIN/.."
  BUILD_RPATH_USE_ORIGIN TRUE
)

target_include_directories(${PROJECT_NAME}_bin PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_bin
  ${PROJECT_NAME}_core
  -Wl,--no-as-needed
  ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
  ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
  -Wl,--as-needed
)

add_dependencies(${PROJECT_NAME}_bin
  ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
  ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
)

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/include/ros2_perf/version.h
  DESTINATION include/ros2_perf
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  function(ros2_perf_add_gtest test_name)
    ament_add_gtest(${test_name} ${ARGN})
    if(TARGET ${test_name})
      target_link_libraries(${test_name} ${PROJECT_NAME}_core)
    endif()
  endfunction()

  ros2_perf_add_gtest(test_args   test/test_args.cpp)
  ros2_perf_add_gtest(test_stats  test/test_stats.cpp)
  ros2_perf_add_gtest(test_report test/test_report.cpp)
endif()

# ---------------------------------------------------------------------------
# Ament export
# ---------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
