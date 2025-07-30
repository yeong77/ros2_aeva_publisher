#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class MappingNode : public rclcpp::Node
{
public:
    MappingNode() : Node("mapping_node")
    {

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/synced_odom", 10,
            std::bind(&MappingNode::odom_callback, this, std::placeholders::_1));

        point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/synced_points", 10,
            std::bind(&MappingNode::cloud_callback, this, std::placeholders::_1));


        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MappingNode::timer_callback, this));


        RCLCPP_INFO(this->get_logger(), "MappingNode initialized.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_odom_ = msg;
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {

        if (!last_odom_) {
            RCLCPP_WARN(this->get_logger(), "No odometry received yet.");
            return;
        }

        // 1. PointCloud2 → PCL 변환
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *cloud_in);

        // 2. Odometry → 변환 행렬
        geometry_msgs::msg::Pose pose = last_odom_->pose.pose;
        Eigen::Affine3d eigen_pose;
        tf2::fromMsg(pose, eigen_pose);
        Eigen::Matrix4f transform = eigen_pose.matrix().cast<float>();

        // 3. 변환 적용
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud_in, *cloud_transformed, transform);

        // 4. 누적
        *accumulated_map_ += *cloud_transformed;
    }

    void timer_callback()
    {
        if (!accumulated_map_->empty()) {
            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*accumulated_map_, map_msg);
            map_msg.header.stamp = this->get_clock()->now();
            map_msg.header.frame_id = "map"; 
            map_pub_->publish(map_msg);

            RCLCPP_DEBUG(this->get_logger(), "Map published (via timer)");
        }
    }
    

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry::SharedPtr last_odom_;


    // 누적 포인트맵
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_map_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
