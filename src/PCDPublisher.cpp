#include "PCDPublisher.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>

void PCDPublisher::PublishPointCloud(const aeva::api::PointCloud& point_cloud) {
  // std::cout << "!!!!!!!!!!!!!!!!!!" << std::endl;
  // std::cout << "sensor_id_ : " << point_cloud.header_.sensor_id_ << std::endl;
  // std::cout << "num_beams_ : " << (unsigned)point_cloud.num_beams_ << std::endl;
  // std::cout << "num_lines_ : " << (unsigned)point_cloud.num_lines_ << std::endl;
  // std::cout << "num_peaks_ : " << (unsigned)point_cloud.num_peaks_ << std::endl;
  // std::cout << "is_absolute_line_index_ : " << (unsigned)point_cloud.is_absolute_line_index_ << std::endl;
  // std::cout << "frame_index_ : " << (unsigned)point_cloud.frame_index_ << std::endl;
  // std::cout << "signal_mode_ : " << static_cast<int>(point_cloud.signal_mode_) << std::endl;
  // std::cout << "reference_frame_ : " << static_cast<int>(point_cloud.reference_frame_) << std::endl;

  sensor_msgs::msg::PointCloud2 ros_msg;
  ros_msg.header.stamp = this->get_clock()->now();
  ros_msg.header.frame_id = "aeva_frame"; //point_cloud.header_.sensor_id_;
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
    // "quality", 1, sensor_msgs::msg::PointField::FLOAT32,
    // "reflectivity", 1, sensor_msgs::msg::PointField::FLOAT32,
    // "time", 1, sensor_msgs::msg::PointField::FLOAT32,
    // "flag", 1, sensor_msgs::msg::PointField::INT32);
   

  mod.resize(point_cloud.points.size());

  sensor_msgs::PointCloud2Iterator<float> x(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> y(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> z(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<float> velocity(ros_msg, "velocity");
  // sensor_msgs::PointCloud2Iterator<float> quality(ros_msg, "quality");
  // sensor_msgs::PointCloud2Iterator<float> reflectivity(ros_msg, "reflectivity");
  // sensor_msgs::PointCloud2Iterator<float> time(ros_msg, "time");
  // sensor_msgs::PointCloud2Iterator<int> flag(ros_msg, "flag");


  for (const auto& pt : point_cloud.points) {

    // if ((unsigned)((pt.flags >> 16) & 0xFFFF) != 0) continue;
    *x = pt.x; 
    *y = pt.y; 
    *z = pt.z; 
    *intensity = pt.intensity;
    *velocity = pt.v;
    // *quality = pt.signal_quality;
    // *reflectivity = pt.reflectivity;
    // *time = static_cast<float>(pt.time_offset_ns / 1e9);
    // *flag = (unsigned)((pt.flags >> 16) & 0xFFFF);
    
    ++x; 
    ++y; 
    ++z; 
    ++intensity;
    ++velocity;
    // ++quality;
    // ++reflectivity;
    // ++time;
    // ++flag;
  }

  pub_->publish(ros_msg);
}
