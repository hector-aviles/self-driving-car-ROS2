#!/usr/bin/env python3
"""
This node stop the car by killing the nodes that control the car and sets the car speed to 0
  
"""
import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Empty, Bool

class StopNode(Node):
    def __init__(self):
        super().__init__('stop')
        
        # Initialize variables
        self.success = True
        self.goal_reached = False
        
        print('INITIALIZING STOP NODE...', flush=True)
        
        # Create subscribers
        self.sub_success = self.create_subscription(
            Bool,
            "/bmw_success",
            self.callback_success,
            10
        )
        self.sub_goal_reached = self.create_subscription(
            Bool,
            "/bmw_goal_reached",
            self.callback_goal_reached,
            10
        )
        
        # Create publisher
        self.pub_speed = self.create_publisher(Float64, '/bmw_speed', 1)
        
        # Create timer for main loop
        self.timer = self.create_timer(0.1, self.main_loop)  # 10Hz
        
        self.get_logger().info("Stop node initialized")

    def callback_success(self, msg):
        self.success = msg.data    
    
    def stop_motion(self):
        speed_msg = Float64()
        speed_msg.data = 0.0  
        self.pub_speed.publish(speed_msg)
        
    def callback_goal_reached(self, msg):
        self.goal_reached = msg.data

    def kill_ros2_node(self, node_name):
        """Helper function to kill ROS2 nodes"""
        try:
            # Use ros2 node list and ros2 node kill
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            if node_name in result.stdout:
                subprocess.run(['ros2', 'node', 'kill', node_name], timeout=5.0)
                self.get_logger().info(f"Killed node: {node_name}")
            else:
                self.get_logger().warn(f"Node {node_name} not found")
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"Timeout killing node: {node_name}")
        except Exception as e:
            self.get_logger().error(f"Error killing node {node_name}: {str(e)}")

    def main_loop(self):
        if not self.success or self.goal_reached:
            self.get_logger().info("STOP: Starting system shutdown...")
            
            # Kill nodes using ROS2 commands
            nodes_to_kill = [
                '/behavior_selection',
                '/behaviors', 
                '/lane_identification',
                '/lane_detector_canny_hough',
                '/obstacle_detector',
                '/success',
                '/stop',
                '/latent_collision_detector',
                '/lane_tracking_control_P'
            ]
            
            # First stop motion for safety
            self.stop_motion()
            self.get_logger().info("STOP: Motion stopped")
            
            # Then kill other nodes
            for node_name in nodes_to_kill:
                self.kill_ros2_node(node_name)
            
            self.get_logger().info("STOP: System shutdown completed")
            
            # Shutdown this node
            self.get_logger().info("STOP: Shutting down stop node")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = StopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
