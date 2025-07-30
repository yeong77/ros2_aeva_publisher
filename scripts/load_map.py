#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg
import open3d as o3d
import numpy as np

class PointCloudLoader(Node):
    def __init__(self):
        super().__init__('PointCloudLoader')
        self.publisher_ = self.create_publisher(PointCloud2, '/map', 10)

        pcd = o3d.io.read_point_cloud("accumulated_map.pcd")
        np_points = np.asarray(pcd.points)

        if np_points.shape[0] == 0:
            self.get_logger().warn("PCD has no points")
            return

        header = std_msgs.msg.Header()
        header.frame_id = 'map'
        self.points = np_points.tolist()
        self.header = header

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("PointCloudLoader started. Publishing repeatedly...")

    def timer_callback(self):
        self.header.stamp = self.get_clock().now().to_msg()
        cloud_msg = pc2.create_cloud_xyz32(self.header, self.points)
        self.publisher_.publish(cloud_msg)
        self.get_logger().debug("Published PointCloud2 to /map")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
