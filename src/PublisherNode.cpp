#include "PublisherNode.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>

void PCDPublisher::PublishPointCloud(const aeva::api::PointCloud& point_cloud) {
  sensor_msgs::msg::PointCloud2 ros_msg;
  ros_msg.header.stamp = this->get_clock()->now();
  ros_msg.header.frame_id = "aeva_frame";
  ros_msg.height = 1;
  ros_msg.width = point_cloud.points.size();
  ros_msg.is_dense = true;
  ros_msg.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier mod(ros_msg);

  mod.setPointCloud2Fields(5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
    "velocity", 1, sensor_msgs::msg::PointField::FLOAT32);

//   mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(point_cloud.points.size());

  sensor_msgs::PointCloud2Iterator<float> x(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<float> velocity(ros_msg, "velocity");

  for (const auto& pt : point_cloud.points) {
    *x = pt.x; 
    *y = pt.y; 
    *z = pt.z; 
    *intensity = pt.intensity;
    *velocity = pt.v;
    
    ++x; 
    ++y; 
    ++z; 
    ++intensity;
    ++velocity;
  }

  pub_->publish(ros_msg);
}
