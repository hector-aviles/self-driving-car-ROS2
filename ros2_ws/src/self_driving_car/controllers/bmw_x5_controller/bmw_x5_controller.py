#!/usr/bin/env python3
"""
BMW X5 Webots controller (ROS2)

Publishes:
- /clock
- /BMW/point_cloud
- /BMW/frontal_camera/rgb/raw
- /BMW/accelerometer
- /BMW/frontal_radar

Subscribes:
- /BMW/speed (std_msgs/Float64)
- /BMW/steering (std_msgs/Float64)

NO TF BROADCASTING — all transforms handled by launch file.
"""

import struct
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2, PointField, Image, Imu
from radar_msgs.msg import RadarScan, RadarReturn
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as RosTime

from vehicle import Driver
from controller import Camera, Lidar, Accelerometer, Radar


# ---------------- Frames ----------------

FRAME_LIDAR = "lidar_link"
FRAME_CAMERA = "camera_link"
FRAME_ACCELEROMETER = "accelerometer_link"
FRAME_RADAR = "radar_link"


class BMWX5Controller(Node):

    def __init__(self):
        super().__init__('bmw_x5_controller')

        # -------- Webots driver --------
        self.driver = Driver()
        self.TIME_STEP = int(self.driver.getBasicTimeStep())  # e.g. 64 ms

        self.driver.setCruisingSpeed(0.0)
        self.driver.setSteeringAngle(0.0)

        # -------- Devices (enabled at TIME_STEP) --------
        self.camera = self.driver.getDevice('camera')
        self.camera.enable(self.TIME_STEP)

        self.lidar = self.driver.getDevice('Velodyne HDL-64E')
        self.lidar.enable(self.TIME_STEP)
        try:
            self.lidar.enablePointCloud()
        except Exception:
            pass

        self.accel = self.driver.getDevice('accelerometer')
        self.accel.enable(self.TIME_STEP)

        self.radar = self.driver.getDevice('Sms UMRR 0a31')
        self.radar.enable(self.TIME_STEP)

        # -------- Publishers --------
        self.pub_pc = self.create_publisher(PointCloud2, '/BMW/point_cloud', 5)
        self.pub_cam = self.create_publisher(Image, '/BMW/frontal_camera/rgb/raw', 5)
        self.pub_accel = self.create_publisher(Imu, '/BMW/accelerometer', 10)
        self.pub_radar = self.create_publisher(RadarScan, '/BMW/frontal_radar', 10)
        self.pub_clock = self.create_publisher(Clock, '/clock', 1)

        # -------- Subscribers --------
        self.create_subscription(Float64, '/BMW/speed', self.cb_speed, 10)
        self.create_subscription(Float64, '/BMW/steering', self.cb_steering, 10)

        # -------- Preallocated messages --------
        self.msg_pc = PointCloud2()
        self.msg_pc.header.frame_id = FRAME_LIDAR
        self.msg_pc.height = 1
        self.msg_pc.is_dense = False
        self.msg_pc.is_bigendian = False
        self.msg_pc.point_step = 12
        self.msg_pc.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
        ]

        self.msg_img = Image()
        self.msg_img.header.frame_id = FRAME_CAMERA
        self.msg_img.encoding = "bgra8"
        self.msg_img.width = self.camera.getWidth()
        self.msg_img.height = self.camera.getHeight()
        self.msg_img.step = self.msg_img.width * 4

        self.msg_accel = Imu()
        self.msg_accel.header.frame_id = FRAME_ACCELEROMETER

        # -------- Timing (Hz → seconds) --------
        t0 = self.driver.getTime()

        self.last_lidar = t0
        self.last_camera = t0
        self.last_accel = t0
        self.last_radar = t0

        # Conservative + reliable rates
        self.lidar_interval = 0.05   # 5 Hz
        self.camera_interval = 0.125 # 8 Hz
        self.accel_interval = 0.04   # 25 Hz
        self.radar_interval = 0.05   # 20 Hz

        self.get_logger().info(
            f"BMW X5 controller running | basicTimeStep={self.TIME_STEP} ms"
        )

    # ---------------- Time helper ----------------

    def make_ros_time(self, sim_t: float) -> RosTime:
        t = RosTime()
        t.sec = int(sim_t)
        t.nanosec = int((sim_t - t.sec) * 1e9)
        return t

    # ---------------- Callbacks ----------------

    def cb_speed(self, msg: Float64):
        self.driver.setCruisingSpeed(float(msg.data))

    def cb_steering(self, msg: Float64):
        self.driver.setSteeringAngle(-float(msg.data))  # ROS1 convention

    # ---------------- Main loop ----------------

    def run(self):

        while self.driver.step() != -1:

            # Allow ROS callbacks without blocking Webots
            rclpy.spin_once(self, timeout_sec=0.0)

            sim_t = self.driver.getTime()

            # ---- CLOCK ----
            clk = Clock()
            clk.clock = self.make_ros_time(sim_t)
            self.pub_clock.publish(clk)

            # ---- ACCELEROMETER (25 Hz) ----
            if sim_t - self.last_accel >= self.accel_interval:
                self.last_accel = sim_t
                ax, ay, az = self.accel.getValues()

                self.msg_accel.linear_acceleration.x = float(ax)
                self.msg_accel.linear_acceleration.y = float(ay)
                self.msg_accel.linear_acceleration.z = float(az)
                self.msg_accel.orientation.w = 1.0
                self.msg_accel.linear_acceleration_covariance[0] = -1.0
                self.msg_accel.header.stamp = self.make_ros_time(sim_t)

                self.pub_accel.publish(self.msg_accel)

            # ---- LIDAR (5 Hz) ----
            if sim_t - self.last_lidar >= self.lidar_interval:
                self.last_lidar = sim_t
                points = self.lidar.getPointCloud()

                if points:
                    buf = bytearray()
                    valid = 0
                    for p in points:
                        if not (math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z)):
                            buf.extend(struct.pack('<fff', p.x, p.y, p.z))
                            valid += 1

                    if valid:
                        self.msg_pc.data = bytes(buf)
                        self.msg_pc.width = valid
                        self.msg_pc.row_step = valid * self.msg_pc.point_step
                        self.msg_pc.header.stamp = self.make_ros_time(sim_t)
                        self.pub_pc.publish(self.msg_pc)

            # ---- CAMERA (8 Hz) ----
            if sim_t - self.last_camera >= self.camera_interval:
                self.last_camera = sim_t
                img = self.camera.getImage()
                if img:
                    self.msg_img.data = img
                    self.msg_img.header.stamp = self.make_ros_time(sim_t)
                    self.pub_cam.publish(self.msg_img)

            # ---- RADAR (20 Hz) ----
            if sim_t - self.last_radar >= self.radar_interval:
                self.last_radar = sim_t
                scan = RadarScan()
                scan.header.frame_id = FRAME_RADAR
                scan.header.stamp = self.make_ros_time(sim_t)

                for t in self.radar.getTargets():
                    r = RadarReturn()
                    r.range = float(t.distance)
                    r.doppler_velocity = float(t.speed)
                    r.azimuth = float(t.azimuth)
                    r.elevation = float('inf')
                    r.amplitude = float(t.receiver_power)
                    scan.returns.append(r)

                self.pub_radar.publish(scan)


# ---------------- Main ----------------

def main(args=None):
    rclpy.init(args=args)
    node = BMWX5Controller()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

