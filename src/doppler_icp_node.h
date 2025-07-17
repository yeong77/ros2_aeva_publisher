#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <memory>
#include <vector>

class DopplerICPRealtime : public rclcpp::Node {
public:
    DopplerICPRealtime();

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publish_odometry(const Eigen::Matrix4d &pose, const std_msgs::msg::Header &header);

    open3d::pipelines::registration::RegistrationResult DopplerICP(
        const std::shared_ptr<open3d::geometry::PointCloud>& source,
        const std::shared_ptr<open3d::geometry::PointCloud>& target,
        const std::unordered_map<std::string, double>& params,
        const Eigen::Matrix4d& init_transform
    );

    std::shared_ptr<open3d::geometry::PointCloud> pointcloud2_to_o3d(const sensor_msgs::msg::PointCloud2& msg);

    std::vector<Eigen::Matrix4d> poses_;
    std::shared_ptr<open3d::geometry::PointCloud> prev_pcd_;
    bool has_prev_pcd_;
    nav_msgs::msg::Path path_msg_;
    std::unordered_map<std::string, double> params_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};
