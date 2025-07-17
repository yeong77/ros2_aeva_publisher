#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "aeva/api/AevaAPI.h"

#include <open3d/Open3D.h>
#include <unordered_map>

class PCDPublisher : public rclcpp::Node {
public:
  PCDPublisher();

private:
  void PublishPointCloud(const aeva::api::PointCloud& point_cloud);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  aeva::api::v12_2_0::AevaAPI aeva_api_;
};
