# Copyright Aeva 2022

cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(ros2_aeva_publisher)

find_package(aeva_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(AEVA_API_ROS2_BUILD true)

set(AEVA_API_BUILD_TYPE "ros2")

find_package(AevaAPI CONFIG PATHS "/opt/aeva/aeva-api/cmake" "/opt/aeva/aeva-api-cuda/cmake")

include_directories("/opt/aeva/aeva-api/aeva/include")
link_directories("/opt/aeva/aeva-api/aeva/library")


set(DEPENDENCIES
  aeva_msgs
  rclcpp
  sensor_msgs
  std_msgs
)


add_executable(aeva_driver_node 
  src/aeva_driver_node.cpp
  src/PCDPublisher.cpp
)
ament_target_dependencies(aeva_driver_node ${DEPENDENCIES})

add_executable(odom_publisher_node
  src/OdomPublisher.cpp
)
ament_target_dependencies(odom_publisher_node
  rclcpp
  sensor_msgs
) 


target_link_libraries(aeva_driver_node
  AevaAPI
)
install(TARGETS
  # aeva_driver_node
  # odom_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
