#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/qos.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class SyncOdomPcdNode : public rclcpp::Node
{
public:
    SyncOdomPcdNode() : Node("sync_odom_pcd")
    {
        RCLCPP_INFO(this->get_logger(), "IMU & LiDAR Synchronization Node Started.");

        this->declare_parameter("max_interval_sec", 0.01);  // default: 10ms
        double max_interval_sec = this->get_parameter("max_interval_sec").as_double();

        rclcpp::QoS odom_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        odom_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        odom_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // '/velodyne_points'의 퍼블리셔가 'BEST_EFFORT'이므로, Subscriber도 'BEST_EFFORT'로 설정
        rclcpp::QoS lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        lidar_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        lidar_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // SubscriptionOptions 사용하여 QoS 강제적용
        rclcpp::SubscriptionOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

        odom_sub_.subscribe(this, "/odom", rmw_qos_profile_sensor_data);
        lidar_sub_.subscribe(this, "/velodyne_points", rmw_qos_profile_sensor_data);

        // Approximate Time Synchronizer 동기화 정책 설정 
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), lidar_sub_, odom_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_interval_sec)); //best

        // sync_ = std::make_shared<message_filters::Synchronizer<ExactPolicy>>(ExactPolicy(10), lidar_sub_, odom_sub_);
        sync_->registerCallback(std::bind(&SyncOdomPcdNode::callback, this, _1, _2));

        // 동기화된 IMU 메시지 publish
        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/synced_points", lidar_qos_profile);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/synced_odom", odom_qos_profile);
    }

    private:
    // 동기화된 IMU & LiDAR 메시지가 도착했을 때 호출되는 콜백함수 
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg, 
                  const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Callback function triggered!");
        
        auto synced_velodyne_points = std::make_shared<sensor_msgs::msg::PointCloud2>(*lidar_msg);

        auto synced_odom = std::make_shared<nav_msgs::msg::Odometry>(*odom_msg);
     
        lidar_pub_->publish(*synced_velodyne_points);
        odom_pub_->publish(*synced_odom);
        RCLCPP_INFO(this->get_logger(), "Published synced pcd data at: %d", synced_velodyne_points->header.stamp.sec);
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;
    // using ExactPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    // std::shared_ptr<message_filters::Synchronizer<ExactPolicy>> sync_;


    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncOdomPcdNode>());
    rclcpp::shutdown();
    return 0;
}