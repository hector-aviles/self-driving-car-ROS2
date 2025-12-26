#!/usr/bin/env python3
"""
This node detects if the car succeeded its last action using Euclidean
distance between points in two consecutive readings of the accelerometer.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Imu


class SuccessNode(Node):
    def __init__(self):
        super().__init__('success')
        
        # -------------------------------------------------
        # Internal state
        # -------------------------------------------------
        self.success = True
        self.first_time = True

        self.x1 = 0.0
        self.y1 = 0.0
        self.z1 = 0.0
        
        #self.get_logger().info("INITIALIZING SUCCESS NODE...")

        # -------------------------------------------------
        # Subscriber
        # -------------------------------------------------
        self.sub_accelerometer = self.create_subscription(
            Imu,
            '/BMW/accelerometer',
            self.callback_accelerometer,
            10
        )
        
        # -------------------------------------------------
        # Publishers
        # -------------------------------------------------
        self.pub_success = self.create_publisher(
            Bool, "/BMW/success", 1
        )

        self.pub_accel_diff = self.create_publisher(
            Float64, "/BMW/accelerometer_diff", 1
        )
        
        self.get_logger().info("Success node initialized")

    # =====================================================
    # Callback
    # =====================================================

    def callback_accelerometer(self, msg):
        threshold = 200.0

        x2 = msg.linear_acceleration.x
        y2 = msg.linear_acceleration.y
        z2 = msg.linear_acceleration.z

        diff = 0.0

        if self.success:
            if self.first_time:
                self.first_time = False
            else:
                diff = ((x2 - self.x1)**2 +
                        (y2 - self.y1)**2 +
                        (z2 - self.z1)**2) ** 0.5

                if diff > threshold:
                    self.success = False
                    self.get_logger().warn(
                        f"Collision detected! accel diff = {diff:.2f}"
                    )

            self.x1 = x2
            self.y1 = y2
            self.z1 = z2

        # -------------------------------------------------
        # Publish results
        # -------------------------------------------------
        self.pub_success.publish(Bool(data=self.success))
        self.pub_accel_diff.publish(Float64(data=diff))


# =========================================================
# MAIN (spin_once pattern, NO sleep)
# =========================================================

def main():
    rclpy.init()
    node = SuccessNode()

    try:
        while rclpy.ok():
            # Non-blocking ROS processing
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

