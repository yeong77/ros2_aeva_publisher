#include "doppler_icp_node.h"

using namespace std::chrono_literals;

DopplerICPRealtime::DopplerICPRealtime()
: Node("realtime_doppler_icp_node"), has_prev_pcd_(false) {

    Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    poses_.push_back(identity);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/aeva/pointcloud", qos,
        std::bind(&DopplerICPRealtime::pointcloud_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    path_msg_.header.frame_id = "odom";

    // 파라미터 선언 및 읽기
    this->declare_parameter<double>("lambda_doppler", 0.015);
    this->declare_parameter<double>("geometric_k", 0.01);
    this->declare_parameter<double>("doppler_k", 0.01);
    this->declare_parameter<double>("outlier_thresh", 0.2);
    this->declare_parameter<int>("rejection_min_iters", 2);
    this->declare_parameter<int>("geometric_min_iters", 0);
    this->declare_parameter<int>("doppler_min_iters", 0);
    this->declare_parameter<double>("convergence_thresh", 1e-6);
    this->declare_parameter<int>("max_iters", 200);
    this->declare_parameter<double>("max_corr_distance", 0.1);
    this->declare_parameter<double>("normals_radius", 10.0);
    this->declare_parameter<int>("normals_max_nn", 30);
    this->declare_parameter<int>("downsample_factor", 1);
    this->declare_parameter<bool>("reject_outliers", true);

    params_["lambda_doppler"] = this->get_parameter("lambda_doppler").as_double();
    params_["geometric_k"] = this->get_parameter("geometric_k").as_double();
    params_["doppler_k"] = this->get_parameter("doppler_k").as_double();
    params_["outlier_thresh"] = this->get_parameter("outlier_thresh").as_double();
    params_["rejection_min_iters"] = this->get_parameter("rejection_min_iters").as_int();
    params_["geometric_min_iters"] = this->get_parameter("geometric_min_iters").as_int();
    params_["doppler_min_iters"] = this->get_parameter("doppler_min_iters").as_int();
    params_["convergence_thresh"] = this->get_parameter("convergence_thresh").as_double();
    params_["max_iters"] = this->get_parameter("max_iters").as_int();
    params_["max_corr_distance"] = this->get_parameter("max_corr_distance").as_double();
    params_["normals_radius"] = this->get_parameter("normals_radius").as_double();
    params_["normals_max_nn"] = this->get_parameter("normals_max_nn").as_int();
    params_["downsample_factor"] = this->get_parameter("downsample_factor").as_int();
    params_["reject_outliers"] = this->get_parameter("reject_outliers").as_bool();
    params_["period"] = 0.1;
}

void DopplerICPRealtime::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    if(msg->data.size() == 0) return;


    RCLCPP_INFO(this->get_logger(), "Received point cloud.");

    auto target_pcd = this->pointcloud2_to_o3d(*msg);

    if (!has_prev_pcd_) {
        prev_pcd_ = target_pcd;
        has_prev_pcd_ = true;
        return;
    }

    Eigen::Matrix4d init_transform = poses_.back();

    open3d::pipelines::registration::RegistrationResult result;


    try {
        result = this->DopplerICP(prev_pcd_, target_pcd, params_, init_transform);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "ICP Failed: %s", e.what());
        return;
    }

    Eigen::Matrix4d T_rel = result.transformation_.inverse();

    Eigen::Matrix4d new_pose = poses_.back() * T_rel; //Eigen::Matrix4d::Identity() * T_rel;
    poses_.push_back(new_pose);
    // poses_.push_back(result.transformation_);

    publish_odometry(new_pose, msg->header);

    prev_pcd_ = target_pcd;
}

std::shared_ptr<open3d::geometry::PointCloud> DopplerICPRealtime::pointcloud2_to_o3d(const sensor_msgs::msg::PointCloud2& msg) {
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();

    
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_velocity(msg, "velocity");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_velocity) {
        Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
        pcd->points_.push_back(point);
        pcd->dopplers_.push_back(*iter_velocity);
    }

    if (pcd->points_.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("doppler_icp_node"), "PointCloud is empty.");
        return nullptr;
    }

    
    return pcd;
}

void DopplerICPRealtime::publish_odometry(const Eigen::Matrix4d &pose, const std_msgs::msg::Header &header) {
    auto now = this->get_clock()->now();

    tf2::Matrix3x3 rot_matrix(
        pose(0,0), pose(0,1), pose(0,2),
        pose(1,0), pose(1,1), pose(1,2),
        pose(2,0), pose(2,1), pose(2,2)
    );

    tf2::Quaternion quat;
    rot_matrix.getRotation(quat);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "aeva_frame";

    odom.pose.pose.position.x = pose(0,3);
    odom.pose.pose.position.y = pose(1,3);
    odom.pose.pose.position.z = pose(2,3);
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "aeva_frame";
    t.transform.translation.x = pose(0,3);
    t.transform.translation.y = pose(1,3);
    t.transform.translation.z = pose(2,3);
    t.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odom.header;
    pose_stamped.pose = odom.pose.pose;

    path_msg_.poses.push_back(pose_stamped);
    path_msg_.header.stamp = now;
    path_pub_->publish(path_msg_);
}



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DopplerICPRealtime>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
