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
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_msgs
  visualization_msgs
)

# add_library(AevaPublisherNode STATIC src/AevaPublisherNode.cc src/AevaPublisherNode.h)
# ament_target_dependencies(AevaPublisherNode ${DEPENDENCIES})

# add_library(AevaAPICallbacks STATIC src/AevaAPICallbacks.cc src/AevaAPICallbacks.h)
# ament_target_dependencies(AevaAPICallbacks ${DEPENDENCIES})

# add_executable(ros2_aeva_publisher src/ros2_aeva_publisher.cc)
# ament_target_dependencies(ros2_aeva_publisher ${DEPENDENCIES})

add_executable(aeva_driver_node 
  src/aeva_driver_node.cpp
  src/PublisherNode.cpp
)
ament_target_dependencies(aeva_driver_node ${DEPENDENCIES})

# target_link_libraries(ros2_aeva_publisher
#   ${rclcpp_LIBRARIES}
#   AevaAPI
#   AevaAPICallbacks
#   AevaPublisherNode
# )
target_link_libraries(aeva_driver_node
  AevaAPI
)
install(TARGETS
  # ros2_aeva_publisher
  aeva_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
