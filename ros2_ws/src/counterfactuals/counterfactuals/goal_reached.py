#!/usr/bin/env python3
"""
This node determines if the BMW car has reached the goal distance
(ROS 2 Jazzy, spin_once-based loop)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D


class GoalReachedNode(Node):
    def __init__(self):
        super().__init__('goal_reached')

        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_reached = False

        # Parameters
        self.declare_parameter('goal_dist', 400.0)
        self.goal_dist = self.get_parameter('goal_dist').value

        self.get_logger().info("INITIALIZING GOAL_REACHED NODE...")
        self.get_logger().info(f"Goal distance set to: {self.goal_dist}")

        # Subscriber
        self.create_subscription(
            Pose2D,
            "/BMW/pose",
            self.callback_curr_pose,
            10
        )

        # Publisher
        self.pub_goal_reached = self.create_publisher(
            Bool,
            "/goal_reached",
            10
        )

    def callback_curr_pose(self, msg: Pose2D):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

        reached = self.x > self.goal_dist

        # Publish every update (or you could gate this if desired)
        goal_msg = Bool()
        goal_msg.data = reached
        self.pub_goal_reached.publish(goal_msg)

        # Log only on transition
        if reached and not self.goal_reached:
            self.get_logger().info("Goal reached!")

        self.goal_reached = reached


def main(args=None):
    rclpy.init(args=args)
    node = GoalReachedNode()

    try:
        # ---- Manual spin loop (non-blocking) ----
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

