cmake_minimum_required(VERSION 3.8)
project(oradar_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# SDK directory
set(ORADAR_SDK_DIR "sdk")
add_subdirectory(${ORADAR_SDK_DIR})

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ORADAR_SDK_DIR}
)

# Create executable
add_executable(oradar_scan src/oradar_scan_node.cpp)

# Add ROS2 compile definition
target_compile_definitions(oradar_scan PRIVATE ROS2_FOUND)

# Link against ROS2 dependencies
ament_target_dependencies(oradar_scan
  rclcpp
  sensor_msgs
  std_msgs
)

# Link against SDK
target_link_libraries(oradar_scan
  oradar_sdk
)

# Install executable
install(TARGETS oradar_scan
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and rviz files

  install(DIRECTORY launch rviz2 config
    DESTINATION share/${PROJECT_NAME}/
  )


# Export package
ament_package()
