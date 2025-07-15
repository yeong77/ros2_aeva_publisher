#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher()
  : Node("Odometry_Publisher")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/aeva/pointcloud",
      10,
      std::bind(&OdometryPublisher::pointcloud_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "PointCloud2 subscriber started.");
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    size_t count = 0;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> iter_velocity(*msg, "velocity");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_velocity) {
      if (count < 5) {
        RCLCPP_INFO(this->get_logger(),
          "Point %zu: x=%.2f, y=%.2f, z=%.2f, intensity=%.2f, velocity=%.2f",
          count, *iter_x, *iter_y, *iter_z, *iter_intensity, *iter_velocity);
      }
      count++;
    }

    RCLCPP_INFO(this->get_logger(), "Received %zu points.", count);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
