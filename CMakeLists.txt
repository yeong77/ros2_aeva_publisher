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
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


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
  nav_msgs
  tf2_ros
  tf2_eigen
  visualization_msgs
  pcl_conversions
  pcl_msgs
  tf2_geometry_msgs
)


add_executable(aeva_driver_node 
  src/aeva_driver_node.cpp
  src/PCDPublisher.cpp
)
ament_target_dependencies(aeva_driver_node ${DEPENDENCIES})

add_executable(doppler_icp_node
  src/doppler_icp_node.cpp
  src/registration_node.cpp
)
ament_target_dependencies(doppler_icp_node ${DEPENDENCIES}) 

add_executable(sync_odom_pcd
  src/sync_odom_pcd.cpp
)
ament_target_dependencies(sync_odom_pcd ${DEPENDENCIES}) 

add_executable(mapping_node
  src/mapping_node.cpp
)
ament_target_dependencies(mapping_node ${DEPENDENCIES})

add_executable(lidar_time_sync
  src/lidar_time_sync.cpp
)
ament_target_dependencies(lidar_time_sync ${DEPENDENCIES})

target_link_libraries(aeva_driver_node
  AevaAPI
)

target_link_libraries(doppler_icp_node
  ${Open3D_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(mapping_node ${PCL_LIBRARIES})

install(TARGETS
  aeva_driver_node
  doppler_icp_node
  sync_odom_pcd
  mapping_node
  lidar_time_sync
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
