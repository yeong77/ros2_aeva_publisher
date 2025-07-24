#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "aeva/api/AevaAPI.h"
#include "PCDPublisher.h"

using namespace aeva::api::v12_2_0;

PCDPublisher::PCDPublisher()
: rclcpp::Node("pcd_publisher"), aeva_api_()
{
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1))
                    .best_effort()
                    .durability_volatile();
                    
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aeva/pointcloud", qos);

  aeva::api::Sensor sensor;
  sensor.id_ = "aeva_sensor";
  sensor.url_ = "192.168.0.10";
  sensor.platform_.type_ = aeva::api::Sensor::Platform::Type::AERIES2;
  sensor.platform_.configuration_ = aeva::api::Sensor::Platform::Configuration::STANDARD;
  sensor.protocol_ = aeva::api::Sensor::Protocol::TCP;

  if (!aeva_api_.Connect(sensor, aeva::api::ProcessingMode::DEFAULT)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to connect to sensor");
    rclcpp::shutdown();
    return;
  }

  aeva_api_.RegisterPointCloudMessageCallback(
    aeva::api::kPointCloudDataStreamId,
    // aeva::api::kPointCloudPointGroupDataStreamId,
    std::bind(&PCDPublisher::PublishPointCloud, this, std::placeholders::_1)
  );

  aeva_api_.Subscribe(sensor.id_, aeva::api::kPointCloudDataStreamId);
  // aeva_api_.Subscribe(sensor.id_, aeva::api::kPointCloudPointGroupDataStreamId);

  poll_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(5),
    [this]() { aeva_api_.PollEvents(); }
  );
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCDPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
