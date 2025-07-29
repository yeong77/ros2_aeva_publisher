#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/qos.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using std::placeholders::_1;
using std::placeholders::_2;

class LidarTimeSync : public rclcpp::Node
{
public:
    LidarTimeSync() : Node("lidar_time_sync")
    {
        RCLCPP_INFO(this->get_logger(), "LiDAR Synchronization Node Started.");

        // '/velodyne_points'의 퍼블리셔가 'BEST_EFFORT'이므로, Subscriber도 'BEST_EFFORT'로 설정
        rclcpp::QoS lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        lidar_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        lidar_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


        // SubscriptionOptions 사용하여 QoS 강제적용
        rclcpp::SubscriptionOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

        aeva_sub.subscribe(this, "/aeva/pointcloud", rmw_qos_profile_sensor_data);
        velodyne_sub.subscribe(this, "/velodyne_points", rmw_qos_profile_sensor_data);

        // Approximate Time Synchronizer 동기화 정책 설 정 
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), velodyne_sub, aeva_sub);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(10.0)); //best
        

        // sync_ = std::make_shared<message_filters::Synchronizer<ExactPolicy>>(ExactPolicy(10), velodyne_sub, aeva_sub);
        sync_->registerCallback(std::bind(&LidarTimeSync::callback, this, _1, _2));

        // 동기화된 IMU 메시지 publish
        velodyne_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/synced_velodyne", lidar_qos_profile);
        aeva_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/synced_aeva", qos_profile);
        merged_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_points", lidar_qos_profile);
    }

    private:
    // 동기화된 IMU & LiDAR 메시지가 도착했을 때 호출되는 콜백함수 
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr velodyne_msg, 
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr aeva_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Callback function triggered!");

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_velodyne(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_aeva(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(*velodyne_msg, *pcl_velodyne);
        pcl::fromROSMsg(*aeva_msg, *pcl_aeva);

        pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>);
        *merged += *pcl_velodyne;
        *merged += *pcl_aeva;

        // PCL → ROS 변환
        sensor_msgs::msg::PointCloud2 merged_msg;
        pcl::toROSMsg(*merged, merged_msg);

        // 헤더 설정 (velodyne 기준)

        merged_msg.header = velodyne_msg->header;

        // 퍼블리시
        merged_pub->publish(merged_msg);

    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
    // using ExactPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    // std::shared_ptr<message_filters::Synchronizer<ExactPolicy>> sync_;


    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> aeva_sub;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> velodyne_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aeva_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarTimeSync>());
    rclcpp::shutdown();
    return 0;
}