#!/usr/bin/env python3
"""
This node stops the car by killing the processes that control the car
and setting the car speed to 0.
"""
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool


class StopNode(Node):
    def __init__(self):
        super().__init__('stop')

        # -------------------------------------------------
        # Internal state
        # -------------------------------------------------
        self.success = True
        self.goal_reached = False
        self.shutdown_started = False

        # -------------------------------------------------
        # Subscribers
        # -------------------------------------------------
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

        # -------------------------------------------------
        # Publisher
        # -------------------------------------------------
        self.pub_speed = self.create_publisher(Float64, "/BMW/speed", 1)

        self.get_logger().info("STOP node initialized")

    # =====================================================
    # Callbacks
    # =====================================================

    def callback_success(self, msg):
        self.success = msg.data

    def callback_goal_reached(self, msg):
        self.goal_reached = msg.data

    # =====================================================
    # Actions
    # =====================================================

    def stop_motion(self):
        """Immediately stop the vehicle"""
        msg = Float64()
        msg.data = 0.0
        self.pub_speed.publish(msg)
        self.get_logger().info("STOP: Speed set to 0")

    def kill_process_by_name(self, name):
        """
        Kill a ROS 2 node by killing its process.
        This is the only reliable way in ROS 2.
        """
        try:
            subprocess.run(
                ['pkill', '-f', name],
                timeout=2.0
            )
            self.get_logger().info(f"STOP: Killed process containing '{name}'")
        except Exception as e:
            self.get_logger().warn(f"STOP: Failed killing '{name}': {e}")

    # =====================================================
    # Main logic
    # =====================================================

    def update(self):
        """Main logic called every spin_once"""
        if self.shutdown_started:
            return

        if (not self.success) or self.goal_reached:
            self.shutdown_started = True
            self.get_logger().warn("STOP: Starting shutdown sequence")

            # 1) Always stop motion first
            self.stop_motion()

            # 2) Kill controller processes
            processes_to_kill = [
                'behavior_selection',
                'behaviors',
                'lane_identification',
                'lane_detector_canny_hough',
                'obstacle_detector',
                'latent_collision_detector',
                'lane_tracking_control_P',
                'action_policy'
            ]

            for proc in processes_to_kill:
                self.kill_process_by_name(proc)

            self.get_logger().warn("STOP: Shutdown sequence completed")


# =========================================================
# MAIN (single shutdown point)
# =========================================================

def main():
    rclpy.init()
    node = StopNode()

    try:
        while rclpy.ok() and not node.shutdown_started:
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

