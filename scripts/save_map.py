#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import signal
import sys

class PointCloudAccumulator(Node):
    def __init__(self):
        super().__init__('pointcloud_accumulator')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/map',
            self.listener_callback,
            10
        )
        self.points_list = []
        self.get_logger().info("PointCloud accumulator started.")

    def listener_callback(self, msg):
        self.get_logger().info("PointCloud2 message received.")
        points = [
            [x, y, z] for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]
        if points:
            self.points_list.extend(points)
            self.get_logger().info(f"Accumulated: {len(self.points_list)} points.")
        else:
            self.get_logger().warn("No valid points in message.")

    def save_map(self, filename="accumulated_map.pcd"):
        if not self.points_list:
            self.get_logger().warn("No points to save.")
            return
        np_points = np.array(self.points_list)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points)
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f"Saved accumulated map to '{filename}'")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulator()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C pressed. Saving map before shutdown...")
        node.save_map("accumulated_map.pcd")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
