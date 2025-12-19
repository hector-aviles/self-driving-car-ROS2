#!/usr/bin/env python3
"""
BMW X5 Webots controller (ROS2)

Publishes:
- /clock
- /point_cloud
- /camera/rgb/raw
- /imu
- /frontal_radar_scan

Subscribes:
- /speed    (std_msgs/Float64)
- /steering (std_msgs/Float64)

NO TF BROADCASTING — all transforms handled by launch file.
"""

import struct
import math

from vehicle import Driver
from controller import Camera, Lidar, Accelerometer, Radar

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2, PointField, Image, Imu
from radar_msgs.msg import RadarScan, RadarReturn
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as RosTime


# ---------------- Constants ----------------

TIME_STEP = 10  # ms (match Webots world)

FRAME_LIDAR  = "lidar_link"
FRAME_CAMERA = "camera_link"
FRAME_IMU    = "imu_link"
FRAME_RADAR  = "radar_link"

# LiDAR corrections
LIDAR_YAW_OFFSET   = -math.radians(12.0)
LIDAR_PITCH_OFFSET = -math.radians(15.0)

COS_YAW = math.cos(LIDAR_YAW_OFFSET)
SIN_YAW = math.sin(LIDAR_YAW_OFFSET)


# ---------------- Controller ----------------

class BMWX5Controller(Node):

    def __init__(self):
        super().__init__('bmw_x5_controller')

        # -------- Webots driver --------
        self.driver = Driver()
        self.driver.setCruisingSpeed(0.0)
        self.driver.setSteeringAngle(0.0)

        # -------- Devices --------
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

        self.radar = self.driver.getDevice('Sms UMRR 0a31')
        self.radar.enable(TIME_STEP)

        # -------- Publishers --------
        self.pub_pc     = self.create_publisher(PointCloud2, '/point_cloud', 10)
        self.pub_cam    = self.create_publisher(Image, '/camera/rgb/raw', 10)
        self.pub_imu    = self.create_publisher(Imu, '/imu', 10)
        self.pub_radar  = self.create_publisher(RadarScan, '/frontal_radar_scan', 10)
        self.pub_clock  = self.create_publisher(Clock, '/clock', 1)

        # -------- Subscribers --------
        self.sub_speed = self.create_subscription(
            Float64,
            '/speed',
            self.cb_speed,
            10
        )

        self.sub_steering = self.create_subscription(
            Float64,
            '/steering',
            self.cb_steering,
            10
        )

        # -------- Preallocated messages --------
        self.msg_pc = PointCloud2()
        self.msg_pc.header.frame_id = FRAME_LIDAR
        self.msg_pc.height = 1
        self.msg_pc.is_dense = False
        self.msg_pc.is_bigendian = False
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

        # -------- Timing --------
        t0 = self.driver.getTime()
        self.last_lidar  = t0
        self.last_camera = t0
        self.last_imu    = t0
        self.last_radar  = t0

        self.lidar_interval  = 0.1
        self.camera_interval = 0.1
        self.imu_interval    = 0.02
        self.radar_interval  = 0.01

        self.get_logger().info(
            "BMW X5 ROS2 controller initialized "
            "(LiDAR + Camera + IMU + Radar + Speed/Steering)"
        )

    # -------- ROS time helper --------
    def make_ros_time(self, sim_t: float) -> RosTime:
        t = RosTime()
        t.sec = int(sim_t)
        t.nanosec = int((sim_t - t.sec) * 1e9)
        return t

    # -------- Callbacks (ROS1 parity) --------
    def cb_speed(self, msg: Float64):
        try:
            self.driver.setCruisingSpeed(float(msg.data))
        except Exception as e:
            self.get_logger().warn(f"Speed command failed: {e}")

    def cb_steering(self, msg: Float64):
        try:
            # Same sign convention as ROS1
            self.driver.setSteeringAngle(-float(msg.data))
        except Exception as e:
            self.get_logger().warn(f"Steering command failed: {e}")

    # -------- Main loop --------
    def spin(self):
        while self.driver.step() != -1:

            sim_t = self.driver.getTime()

            # ---- CLOCK ----
            clk = Clock()
            clk.clock = self.make_ros_time(sim_t)
            self.pub_clock.publish(clk)

            # ---- IMU ----
            if sim_t - self.last_imu >= self.imu_interval:
                self.last_imu = sim_t
                ax, ay, az = self.accel.getValues()
                self.msg_imu.linear_acceleration.x = float(ax)
                self.msg_imu.linear_acceleration.y = float(ay)
                self.msg_imu.linear_acceleration.z = float(az)
                self.msg_imu.header.stamp = self.make_ros_time(sim_t)
                self.pub_imu.publish(self.msg_imu)

            # ---- LIDAR ----
            if sim_t - self.last_lidar >= self.lidar_interval:
                self.last_lidar = sim_t
                points = self.lidar.getPointCloud()
                if points:
                    buf = bytearray()
                    for p in points:
                        # Check for NaN values from Webots
                        if math.isnan(p.x) or math.isnan(p.y) or math.isnan(p.z):
                            continue
                        buf.extend(struct.pack('<fff',
                                               float(p.x), float(p.y), float(p.z)))
                    
                    if len(buf) == 0:
                        continue
                        
                    self.msg_pc.data = bytes(buf)
                    self.msg_pc.width = len(points)
                    self.msg_pc.row_step = self.msg_pc.point_step * len(points)
                    self.msg_pc.header.stamp = self.make_ros_time(sim_t)
                    self.pub_pc.publish(self.msg_pc)

            # ---- CAMERA ----
            if sim_t - self.last_camera >= self.camera_interval:
                self.last_camera = sim_t
                img = self.camera.getImage()
                if img:
                    self.msg_img.data = img
                    self.msg_img.header.stamp = self.make_ros_time(sim_t)
                    self.pub_cam.publish(self.msg_img)

            # ---- RADAR ----
            if sim_t - self.last_radar >= self.radar_interval:
                self.last_radar = sim_t
                targets = self.radar.getTargets()

                scan = RadarScan()
                scan.header.frame_id = FRAME_RADAR
                scan.header.stamp = self.make_ros_time(sim_t)

                for t in targets:
                    r = RadarReturn()
                    r.range = float(t.distance)
                    r.doppler_velocity = float(t.speed)
                    r.azimuth = float(t.azimuth)
                    r.elevation = float('inf')
                    r.amplitude = float(t.receiver_power)
                    scan.returns.append(r)

                self.pub_radar.publish(scan)


# ---------------- main ----------------

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

