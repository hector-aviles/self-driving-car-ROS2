#!/usr/bin/env python3
"""
This node detects if the car succeeded its last action using Euclidean distance between points in two consecutive readings of the accelerometer sensor
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Imu

class SuccessNode(Node):
    def __init__(self):
        super().__init__('success')
        
        # Initialize variables
        self.success = True
        self.first_time = True
        self.x1 = 0.0
        self.y1 = 0.0
        self.z1 = 0.0
        
        print("INITIALIZING SUCCESS...", flush=True)
        
        # Create subscriber
        self.sub_accelerometer = self.create_subscription(
            Imu,
            '/accelerometer',
            self.callback_accelerometer,
            10
        )
        
        # Create publishers
        self.pub_success = self.create_publisher(Bool, "/success", 1)
        self.pub_accel_diff = self.create_publisher(Float64, "/accelerometer_diff", 1)
        
        self.get_logger().info("Success node initialized")

    def callback_accelerometer(self, msg):
        threshold = 200

        x2 = msg.linear_acceleration.x
        y2 = msg.linear_acceleration.y
        z2 = msg.linear_acceleration.z

        diff = 0.0
        if self.success:
            if self.first_time:
                self.first_time = False
            else:   
                diff = ((x2 - self.x1)**2 + (y2 - self.y1)**2 + (z2 - self.z1)**2)**0.5
                if diff > threshold:
                    self.success = False
                    # self.get_logger().warn("ERROR: Collision detected")

            self.x1 = msg.linear_acceleration.x
            self.y1 = msg.linear_acceleration.y
            self.z1 = msg.linear_acceleration.z

        # Publish success status
        success_msg = Bool()
        success_msg.data = self.success
        self.pub_success.publish(success_msg)
        
        # Publish acceleration difference
        diff_msg = Float64()
        diff_msg.data = diff
        self.pub_accel_diff.publish(diff_msg)

def main():
    rclpy.init()
    node = SuccessNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()