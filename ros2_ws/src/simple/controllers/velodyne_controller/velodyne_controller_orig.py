#!/usr/bin/env python3
"""
BMW X5 Webots controller (ROS2) — publishes 3D LiDAR, camera, IMU
NO TF BROADCASTING — all transforms handled by launch file.
"""

import struct
import math
from vehicle import Driver
from controller import Lidar
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from builtin_interfaces.msg import Time as RosTime


TIME_STEP = 10  # ms, match your Webots world

# Frame names (must match your launch file)
FRAME_LIDAR = "lidar_link"

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        self.driver = Driver()

        # Devices
        self.lidar = self.driver.getDevice('Velodyne HDL-32E')
        self.lidar.enable(TIME_STEP)
        try:
            self.lidar.enablePointCloud()
        except Exception:
            pass

        # Publishers
        self.pub_point_cloud = self.create_publisher(PointCloud2, '/point_cloud', 10)

        # Preallocated messages
        self.msg_pc = PointCloud2()
        self.msg_pc.header.frame_id = FRAME_LIDAR
        self.msg_pc.height = 1
        self.msg_pc.is_bigendian = False
        self.msg_pc.is_dense = False
        self.msg_pc.point_step = 12
        self.msg_pc.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # timing
        t0 = self.driver.getTime()
        self.last_lidar = t0
        self.last_camera = t0
        self.last_imu = t0

        self.lidar_interval = 0.1

        self.get_logger().info("Simple controller initialized (no TF broadcasting)")

    def make_ros_time(self, sim_time_seconds: float) -> RosTime:
        t = RosTime()
        t.sec = int(sim_time_seconds)
        t.nanosec = int((sim_time_seconds - int(sim_time_seconds)) * 1e9)
        return t

    def spin(self):
        while self.driver.step() != -1:
            sim_t = self.driver.getTime()

            # LIDAR
            if (sim_t - self.last_lidar) >= self.lidar_interval:
                self.last_lidar = sim_t
                points = self.lidar.getPointCloud()
                if points:
                    buf = bytearray()
                    for p in points:
                        buf.extend(struct.pack('<fff',
                                               float(p.x), float(p.y), float(p.z)))
                    self.msg_pc.data = bytes(buf)
                    self.msg_pc.width = len(points)
                    self.msg_pc.row_step = self.msg_pc.point_step * len(points)
                    self.msg_pc.header.stamp = self.make_ros_time(sim_t)
                    self.pub_point_cloud.publish(self.msg_pc)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

