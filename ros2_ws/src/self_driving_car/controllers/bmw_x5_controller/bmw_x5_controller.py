#!/usr/bin/env python3
import math
import struct
from vehicle import Driver
from controller import Camera, Lidar, Accelerometer
import rclpy
from rclpy.node import Node
from rclpy.clock import ClockType
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, PointCloud2, PointField, Imu
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock

class BMWX5Controller(Node):
    def __init__(self):
        super().__init__('bmw_x5_controller')
        
        # INIT DRIVER
        self.driver = Driver()
        self.driver.setCruisingSpeed(0.0)
        self.driver.setSteeringAngle(0.0)
        
        # CONSTANTS
        self.TIME_STEP = int(self.driver.getBasicTimeStep())
        
        # SENSOR TIMING
        self.lidar_interval = 0.085  # ~11.76 Hz
        self.camera_interval = 0.028  # ~35.7 Hz
        self.imu_interval = 0.01  # 100 Hz
        
        # INIT SENSORS
        self.setup_sensors()
        
        # ROS 2 PUBLISHERS
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pub_camera_data = self.create_publisher(Image, '/camera/rgb/raw', 10)
        self.pub_point_cloud = self.create_publisher(PointCloud2, '/point_cloud', 10)
        self.pub_imu_accel = self.create_publisher(Imu, '/accelerometer', 10)
        
        # ROS 2 SUBSCRIBERS
        self.sub_speed = self.create_subscription(
            Float64, 
            '/speed', 
            self.callback_cruise_speed, 
            10
        )
        self.sub_steering = self.create_subscription(
            Float64, 
            '/steering', 
            self.callback_steering_angle, 
            10
        )
        
        # Create reusable messages
        self.create_messages()
        
        # Timing variables
        self.time_lidar_last = self.driver.getTime()
        self.time_camera_last = self.driver.getTime()
        self.time_imu_last = self.driver.getTime()
        
        # Timer for main control loop (matches Webots time step)
        self.timer = self.create_timer(self.TIME_STEP / 1000.0, self.control_loop)
        
        self.get_logger().info('BMW X5 Controller Node Started')
        self.get_logger().info('SIMULATION FOR THE AUTOMODELCAR LEAGUE OF THE MEXICAN ROBOTICS TOURNAMENT')
        self.get_logger().info('TO MOVE THE VEHICLE, USE THE CORRESPONDING TOPICS')
        self.get_logger().info('CHECK ONLINE DOCUMENTATION AT https://github.com/mnegretev/TMR-2022-AutoModelCar')
        self.get_logger().info(f"Using timestep={self.TIME_STEP} ms")
    
    def setup_sensors(self):
        """Initialize and enable all sensors"""
        self.camera = Camera('camera')
        self.camera.enable(self.TIME_STEP)
        
        # Note: name used here must match your Webots device name
        self.lidar = Lidar('Velodyne HDL-64E')
        self.lidar.enable(self.TIME_STEP)
        self.lidar.enablePointCloud()
        
        self.accel = Accelerometer('accelerometer')
        self.accel.enable(self.TIME_STEP)
    
    def create_messages(self):
        """Create reusable message objects"""
        # IMAGE MESSAGE
        self.msg_image = Image()
        self.msg_image.height = self.camera.getHeight()
        self.msg_image.width = self.camera.getWidth()
        self.msg_image.is_bigendian = False
        # Assuming camera uses 4 bytes per pixel (BGRA)
        self.msg_image.step = self.camera.getWidth() * 4
        self.msg_image.encoding = 'bgra8'
        
        # POINT CLOUD MESSAGE
        self.msg_point_cloud = PointCloud2()
        self.msg_point_cloud.header.frame_id = 'lidar_link'
        self.msg_point_cloud.height = 1
        # We'll set width dynamically when publishing because number of valid points can vary
        self.msg_point_cloud.width = 0
        # point_step: 3 floats (x,y,z) -> 3 * 4 = 12 bytes
        self.msg_point_cloud.point_step = 12
        self.msg_point_cloud.row_step = 0  # will update dynamically
        self.msg_point_cloud.is_dense = False
        self.msg_point_cloud.is_bigendian = False
        self.msg_point_cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        # ACCELEROMETER MESSAGE
        self.msg_accel = Imu()
        self.msg_accel.header.frame_id = 'accel_link'
    
    def callback_cruise_speed(self, msg):
        """Callback for speed control"""
        self.driver.setCruisingSpeed(msg.data)
        self.get_logger().debug(f'Setting speed to: {msg.data} km/h')
    
    def callback_steering_angle(self, msg):
        """Callback for steering control"""
        # sign inverted as in original code
        self.driver.setSteeringAngle(-msg.data)
        self.get_logger().debug(f'Setting steering angle to: {-msg.data} radians')
    
    def control_loop(self):
        """Main control loop called by timer"""
        if self.driver.step() == -1:
            self.get_logger().info('Simulation ended')
            return
        
        current_t = self.driver.getTime()
        
        # Publish clock
        self.publish_clock(current_t)
        
        # Publish sensor data at appropriate intervals
        self.publish_sensor_data(current_t)
    
    def publish_clock(self, current_t):
        """Publish simulation clock"""
        msg_clock = Clock()
        # Convert simulation time to ROS time
        clock_time = Time()
        clock_time.sec = int(current_t)
        clock_time.nanosec = int((current_t - clock_time.sec) * 1e9)
        msg_clock.clock = clock_time
        self.pub_clock.publish(msg_clock)
    
    def publish_sensor_data(self, current_t):
        """Publish all sensor data at appropriate intervals"""
        # Publish lidar data
        if (current_t - self.time_lidar_last) >= self.lidar_interval:
            self.time_lidar_last = current_t
            try:
                points = self.lidar.getPointCloud()  # no data_type arg
            except Exception as ex:
                self.get_logger().error(f'Error reading LiDAR point cloud: {ex}')
                points = None

            # Build PointCloud2.data robustly
            if points:
                # If Webots returned raw bytes already, try to use it directly (best-effort)
                if isinstance(points, (bytes, bytearray)):
                    self.msg_point_cloud.data = bytes(points)
                    # Try to infer width from lidar reported number of points
                    try:
                        num_points = int(self.lidar.getNumberOfPoints())
                    except Exception:
                        num_points = len(self.msg_point_cloud.data) // self.msg_point_cloud.point_step
                    self.msg_point_cloud.width = num_points
                    self.msg_point_cloud.row_step = self.msg_point_cloud.point_step * self.msg_point_cloud.width
                else:
                    # Expecting an iterable of (x,y,z) triples (or objects with [0],[1],[2])
                    buf = bytearray()
                    count = 0
                    for p in points:
                        try:
                            x = float(p[0])
                            y = float(p[1])
                            z = float(p[2])
                        except Exception:
                            # If element not indexable, try attributes
                            try:
                                x = float(p.x)
                                y = float(p.y)
                                z = float(p.z)
                            except Exception:
                                continue
                        buf.extend(struct.pack('<fff', x, y, z))
                        count += 1

                    self.msg_point_cloud.data = bytes(buf)
                    self.msg_point_cloud.width = count
                    self.msg_point_cloud.row_step = self.msg_point_cloud.point_step * self.msg_point_cloud.width

                # Stamp and publish if we have points
                self.msg_point_cloud.header.stamp = self.get_clock().now().to_msg()
                # Only publish when there is data (width > 0)
                if self.msg_point_cloud.width > 0:
                    self.pub_point_cloud.publish(self.msg_point_cloud)
                    self.get_logger().debug(f'Published LiDAR point cloud with {self.msg_point_cloud.width} points')
                else:
                    self.get_logger().debug('LiDAR returned no valid points to publish')
            else:
                self.get_logger().debug('No LiDAR data available this step')
        
        # Publish camera data
        if (current_t - self.time_camera_last) >= self.camera_interval:
            self.time_camera_last = current_t
            try:
                self.msg_image.data = self.camera.getImage()
                self.msg_image.header.stamp = self.get_clock().now().to_msg()
                self.pub_camera_data.publish(self.msg_image)
                self.get_logger().debug('Published camera image')
            except Exception as ex:
                self.get_logger().error(f'Error publishing camera image: {ex}')
        
        # Publish IMU data
        if (current_t - self.time_imu_last) >= self.imu_interval:
            self.time_imu_last = current_t
            try:
                self.msg_accel.header.stamp = self.get_clock().now().to_msg()
                accel_values = self.accel.getValues()
                # accel_values is typically a 3-tuple
                self.msg_accel.linear_acceleration.x = float(accel_values[0])
                self.msg_accel.linear_acceleration.y = float(accel_values[1])
                self.msg_accel.linear_acceleration.z = float(accel_values[2])
                self.pub_imu_accel.publish(self.msg_accel)
                self.get_logger().debug('Published accelerometer data')
            except Exception as ex:
                self.get_logger().error(f'Error publishing IMU data: {ex}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = BMWX5Controller()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in BMW X5 Controller: {e}")
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

