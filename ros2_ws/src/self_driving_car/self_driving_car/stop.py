#!/usr/bin/env python3
"""
This node stops the car by killing the nodes that control the car
and setting the car speed to 0.
"""
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool


class StopNode(Node):
    def __init__(self):
        super().__init__('stop')

        self.success = True
        self.goal_reached = False
        self.shutdown_started = False

        #self.get_logger().info("INITIALIZING STOP NODE...")

        # Subscribers
        self.create_subscription(
            Bool,
            "/BMW/success",
            self.callback_success,
            10
        )
        self.create_subscription(
            Bool,
            "/BMW/goal_reached",
            self.callback_goal_reached,
            10
        )

        # Publisher
        self.pub_speed = self.create_publisher(Float64, "/BMW/speed", 1)

    def callback_success(self, msg):
        self.success = msg.data

    def callback_goal_reached(self, msg):
        self.goal_reached = msg.data

    def stop_motion(self):
        msg = Float64()
        msg.data = 0.0
        self.pub_speed.publish(msg)

    def kill_ros2_node(self, node_name):
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=3.0
            )
            if node_name in result.stdout:
                subprocess.run(['ros2', 'node', 'kill', node_name], timeout=3.0)
                self.get_logger().info(f"Killed node: {node_name}")
        except Exception as e:
            self.get_logger().warn(f"Failed killing {node_name}: {e}")

    def update(self):
        """Main logic called every spin_once"""
        if self.shutdown_started:
            return

        if (not self.success) or self.goal_reached:
            self.shutdown_started = True
            self.get_logger().info("STOP: Starting shutdown sequence")

            # Always stop motion first
            self.stop_motion()

            nodes_to_kill = [
                '/behavior_selection',
                '/behaviors',
                '/lane_identification',
                '/lane_detector_canny_hough',
                '/obstacle_detector',
                '/latent_collision_detector',
                '/lane_tracking_control_P'
            ]

            for node in nodes_to_kill:
                self.kill_ros2_node(node)

            self.get_logger().info("STOP: Shutdown completed")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = StopNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

