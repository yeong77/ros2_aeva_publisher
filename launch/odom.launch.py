from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'ros2_aeva_publisher'
    pkg_share = get_package_share_directory(pkg_name)

    aeva_driver_node = Node(
        package=pkg_name,
        executable='aeva_driver_node',
        name='aeva_driver_node',
        output='screen'
    )

    doppler_icp_node = Node(
        package=pkg_name,
        executable='doppler_icp_node',
        name='doppler_icp_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'params.yaml')]
    )

    sync_odom_pcd_node = Node(
        package=pkg_name,
        executable='sync_odom_pcd',
        name='sync_odom_pcd',
        output='screen',
        parameters = [
            {'max_interval_sec' : 0.02}
        ]
    )

    lidar_time_sync = Node(
        package=pkg_name,
        executable='lidar_time_sync',
        name='lidar_time_sync',
        output='screen'
    )

    return LaunchDescription([
        aeva_driver_node,
        doppler_icp_node,
        sync_odom_pcd_node,
        lidar_time_sync
    ])
