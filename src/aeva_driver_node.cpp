#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "aeva/api/AevaAPI.h"


using std::placeholders::_1;
using namespace aeva::api::v12_2_0;

class AevaDriver : public rclcpp::Node {
public:
  AevaDriver()
  : Node("pcd_publisher"), aeva_api_()
  {
  
    void PublishPointCloud(const aeva::api::PointCloud& point_cloud);
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aeva/pointcloud", 10);

    std::string sensor_ip = "192.168.0.10"; 
    std::string sensor_id = "aeva_sensor";

    // 센서 구성
    aeva::api::Sensor sensor;
    sensor.id_ = sensor_id;
    sensor.url_ = sensor_ip;
    sensor.platform_.type_ = aeva::api::Sensor::Platform::Type::AERIES2;
    sensor.platform_.configuration_ = aeva::api::Sensor::Platform::Configuration::STANDARD;
    sensor.protocol_ = aeva::api::Sensor::Protocol::TCP;

    if (!aeva_api_.Connect(sensor, aeva::api::ProcessingMode::DEFAULT)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to connect to sensor");
      rclcpp::shutdown();
      return;
    }

    // 콜백 등록
    aeva_api_.RegisterPointCloudMessageCallback(
      aeva::api::kPointCloudDataStreamId,
      std::bind(&AevaDriver::PublishPointCloud, this, std::placeholders::_1)
    );

    aeva_api_.Subscribe(sensor.id_, aeva::api::kPointCloudDataStreamId);

    // 타이머로 polling 수행
    poll_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      [this]() { aeva_api_.PollEvents(); }
    );
  }

private:
  void PublishPointCloud(const aeva::api::PointCloud& point_cloud){
    sensor_msgs::msg::PointCloud2 ros_msg;
    ros_msg.header.stamp = this->get_clock()->now();
    ros_msg.header.frame_id = "aeva_frame";

    ros_msg.height = 1;
    ros_msg.width = point_cloud.points.size();
    ros_msg.is_dense = true;
    ros_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(ros_msg);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(point_cloud.points.size());

    sensor_msgs::PointCloud2Iterator<float> x(ros_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> y(ros_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> z(ros_msg, "z");

    for (const auto& pt : point_cloud.points) {
      *x = pt.x;
      *y = pt.y;
      *z = pt.z;
      ++x;
      ++y;
      ++z;
    }
    pub_->publish(ros_msg);
  }
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;  
  rclcpp::TimerBase::SharedPtr poll_timer_;
  AevaAPI aeva_api_;
};


// === main ===

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AevaDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
