#!/usr/bin/env python3
"""
Simple test node that publishes a rotating point cloud cube.
Use this to verify your app receives and displays point clouds correctly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
import numpy as np
import struct
import math


class TestPointCloudPublisher(Node):
    def __init__(self):
        super().__init__("test_pointcloud_publisher")
        self.publisher = self.create_publisher(PointCloud2, "/camera/depth/points", 10)
        self.timer = self.create_timer(0.1, self.publish_cloud)
        self.angle = 0.0
        self.get_logger().info("Publishing test point cloud to /camera/depth/points")

    def publish_cloud(self):
        # simple cube of points
        points = []
        size = 50  # poitns per dimension
        scale = 0.02  # spacing between points

        for x in range(size):
            for y in range(size):
                for z in range(size):
                    if (
                        x == 0
                        or x == size - 1
                        or y == 0
                        or y == size - 1
                        or z == 0
                        or z == size - 1
                    ):
                        px = (x - size / 2) * scale
                        py = (y - size / 2) * scale
                        pz = (z - size / 2) * scale + 1.0  # offset in z

                        # rotate around Z axis
                        rx = px * math.cos(self.angle) - py * math.sin(self.angle)
                        ry = px * math.sin(self.angle) - py * math.cos(self.angle)

                        # color based on pisition(rgb
                        r = int((x / size)) * 255
                        g = int((y / size)) * 255
                        b = int((z / size)) * 255

                        rgb = struct.unpack(
                            "f", struct.pack("I", (r << 16) | (g << 8) | b)
                        )[0]
                        points.append([rx, ry, pz, rgb])

        self.angle += 0.05  # rotate slowly

        # create pointcloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_depth_optical_frame"

        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.point_step = 16  # 4floats * 4 bytes
        msg.row_step = msg.point_step * len(points)
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False

        # pack data
        msg.data = b"".join([struct.pack("ffff", *p) for p in points])

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = TestPointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
