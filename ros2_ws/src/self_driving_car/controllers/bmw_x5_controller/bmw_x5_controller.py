#!/usr/bin/env python3
"""
BMW X5 Webots controller (ROS2) — publishes 3D LiDAR, camera, IMU
NO TF BROADCASTING — all transforms handled by launch file.
"""

import struct
import math
from vehicle import Driver
from controller import Camera, Lidar, Accelerometer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image, Imu
from builtin_interfaces.msg import Time as RosTime


TIME_STEP = 10  # ms, match your Webots world

# Frame names (must match your launch file)
FRAME_LIDAR = "lidar_link"
FRAME_CAMERA = "camera_link"
FRAME_IMU = "imu_link"

# LiDAR yaw correction (right)
LIDAR_YAW_OFFSET = -math.radians(12.0)

# LiDAR pitch correction (flatten hill)
LIDAR_PITCH_OFFSET = -math.radians(30.0)  # 🔧 tune this

COS_YAW = math.cos(LIDAR_YAW_OFFSET)
SIN_YAW = math.sin(LIDAR_YAW_OFFSET)

COS_PITCH = math.cos(LIDAR_PITCH_OFFSET)
SIN_PITCH = math.sin(LIDAR_PITCH_OFFSET)


class BMWX5Controller(Node):
    def __init__(self):
        super().__init__('bmw_x5_controller')

        self.driver = Driver()

        # Devices
        self.camera = self.driver.getDevice('camera')
        self.camera.enable(TIME_STEP)

        self.lidar = self.driver.getDevice('Velodyne HDL-64E')
        self.lidar.enable(TIME_STEP)
        try:
            self.lidar.enablePointCloud()
        except Exception:
            pass

        self.accel = self.driver.getDevice('accelerometer')
        self.accel.enable(TIME_STEP)

        # Publishers
        self.pub_point_cloud = self.create_publisher(PointCloud2, '/point_cloud', 10)
        self.pub_camera = self.create_publisher(Image, '/camera/rgb/raw', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu', 10)

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

        self.msg_img = Image()
        self.msg_img.header.frame_id = FRAME_CAMERA
        self.msg_img.encoding = "bgra8"
        self.msg_img.width = self.camera.getWidth()
        self.msg_img.height = self.camera.getHeight()
        self.msg_img.step = self.msg_img.width * 4

        self.msg_imu = Imu()
        self.msg_imu.header.frame_id = FRAME_IMU

        # timing
        t0 = self.driver.getTime()
        self.last_lidar = t0
        self.last_camera = t0
        self.last_imu = t0

        self.lidar_interval = 0.1
        self.camera_interval = 0.1
        self.imu_interval = 0.02

        self.get_logger().info(
            "BMW X5 controller initialized (LiDAR yaw corrected by -15°)"
        )

    def make_ros_time(self, sim_time_seconds: float) -> RosTime:
        t = RosTime()
        t.sec = int(sim_time_seconds)
        t.nanosec = int((sim_time_seconds - int(sim_time_seconds)) * 1e9)
        return t

    def spin(self):
        while self.driver.step() != -1:
            sim_t = self.driver.getTime()

            # IMU
            if (sim_t - self.last_imu) >= self.imu_interval:
                self.last_imu = sim_t
                acc = self.accel.getValues()
                self.msg_imu.linear_acceleration.x = float(acc[0])
                self.msg_imu.linear_acceleration.y = float(acc[1])
                self.msg_imu.linear_acceleration.z = float(acc[2])
                self.msg_imu.header.stamp = self.make_ros_time(sim_t)
                self.pub_imu.publish(self.msg_imu)

            # LIDAR
            if (sim_t - self.last_lidar) >= self.lidar_interval:
                self.last_lidar = sim_t
                points = self.lidar.getPointCloud()
                if points:
                    buf = bytearray()
                    for p in points:
                        x = float(p.x)
                        y = float(p.y)
                        z = float(p.z)

                        # ---- YAW rotation (Z axis) ----
                        x_yaw = x * COS_YAW - y * SIN_YAW
                        y_yaw = x * SIN_YAW + y * COS_YAW
                        z_yaw = z

                        # ---- PITCH rotation (Y axis) ----
                        x_rot = x_yaw * COS_PITCH + z_yaw * SIN_PITCH
                        y_rot = y_yaw
                        z_rot = -x_yaw * SIN_PITCH + z_yaw * COS_PITCH

                        buf.extend(struct.pack('<fff', x_rot, y_rot, z))

                    self.msg_pc.data = bytes(buf)
                    self.msg_pc.width = len(points)
                    self.msg_pc.row_step = self.msg_pc.point_step * len(points)
                    self.msg_pc.header.stamp = self.make_ros_time(sim_t)
                    self.pub_point_cloud.publish(self.msg_pc)

            # CAMERA
            if (sim_t - self.last_camera) >= self.camera_interval:
                self.last_camera = sim_t
                img = self.camera.getImage()
                if img:
                    self.msg_img.data = img
                    self.msg_img.header.stamp = self.make_ros_time(sim_t)
                    self.pub_camera.publish(self.msg_img)


def main(args=None):
    rclpy.init(args=args)
    node = BMWX5Controller()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

