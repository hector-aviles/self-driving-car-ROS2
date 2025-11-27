#!/usr/bin/env python3
"""
This node determines if the car has reached the goal distance 
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D

class GoalReachedNode(Node):
    def __init__(self):
        super().__init__('goal_reached')
        
        # Initialize variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_reached = False
        self.goal_dist = 400
        
        # Declare and get parameter
        self.declare_parameter('goal_dist', self.goal_dist)
        self.goal_dist = self.get_parameter('goal_dist').value
        
        print("INITIALIZING GOAL_REACHED NODE...", flush=True)
        
        # Create subscriber
        self.sub_curr_pose = self.create_subscription(
            Pose2D, 
            "/self_driving_pose", 
            self.callback_curr_pose, 
            10
        )
        
        # Create publisher
        self.pub_goal_reached = self.create_publisher(
            Bool, 
            "/goal_reached", 
            1
        )
        
        self.get_logger().info(f"Goal distance set to: {self.goal_dist}")

    def callback_curr_pose(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
 
        self.goal_reached = False
        if self.x > self.goal_dist:
            self.goal_reached = True

        # Create and publish message
        goal_msg = Bool()
        goal_msg.data = self.goal_reached
        self.pub_goal_reached.publish(goal_msg)
        
        # Optional: log when goal is reached
        if self.goal_reached:
            self.get_logger().info("Goal reached!")

def main():
    rclpy.init()
    node = GoalReachedNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
