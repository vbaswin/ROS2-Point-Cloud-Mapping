#!/usr/bin/env python3

"""
GPU-accelerated ICP using Open3D CUDA tensors.
This runs as a ROS2 service for the C++ app to call.
"""

import open3d as o3d
import open3d.core as o3c
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# class GpuIcpNode(Node):
