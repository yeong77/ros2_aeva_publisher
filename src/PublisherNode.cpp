#include "PublisherNode.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>

void PCDPublisher::PublishPointCloud(const aeva::api::PointCloud& point_cloud) {
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
    *x = pt.x; *y = pt.y; *z = pt.z;
    ++x; ++y; ++z;
  }

  pub_->publish(ros_msg);
}
