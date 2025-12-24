#!/usr/bin/env python3
"""
This node implements a proportional control and is intended to be used
together with the lane_detector node. It is assumed both lane borders 
are given by two straight lines in rho-theta form. Given a desired rho-theta
for each lane border, an error is calculated.
Steering is calculated proportional to this error and linear speed is
set as a constant. 
"""
import cv2
import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool

class LaneTrackingNode(Node):
    def __init__(self):
        super().__init__('lane_tracking_control_P')
        
        # Initialize variables
        self.max_speed = 10
        self.k_rho = 0.001
        self.k_theta = 0.01
        
        self.lane_rho_l = 0
        self.lane_theta_l = 0
        self.lane_rho_r = 0
        self.lane_theta_r = 0
        
        self.goal_rho_l = 370.0
        self.goal_theta_l = 2.4
        self.goal_rho_r = 430.0
        self.goal_theta_r = 0.895
        
        self.requested_speed = 0.0
        self.enable_cruise = False
        self.enable_follow = False
        self.dist_to_obstacle = 9.0
        
        # Declare parameters
        self.declare_parameter('max_speed', self.max_speed)
        self.declare_parameter('k_rho', self.k_rho)
        self.declare_parameter('k_theta', self.k_theta)
        
        # Get parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.k_rho = self.get_parameter('k_rho').value
        self.k_theta = self.get_parameter('k_theta').value
        
        print('INITIALIZING LANE TRACKING NODE...', flush=True)
        
        # Create subscribers
        self.sub_left_lane = self.create_subscription(
            Float64MultiArray, 
            "/demo/left_lane", 
            self.callback_left_lane, 
            10
        )
        self.sub_right_lane = self.create_subscription(
            Float64MultiArray, 
            "/demo/right_lane", 
            self.callback_right_lane, 
            10
        )
        self.sub_enable_cruise = self.create_subscription(
            Bool, 
            "/cruise/enable", 
            self.callback_enable_cruise, 
            10
        )
        self.sub_enable_follow = self.create_subscription(
            Bool, 
            "/follow/enable", 
            self.callback_enable_follow, 
            10
        )
        self.sub_dist_to_obstacle = self.create_subscription(
            Float64, 
            "/obstacle/distance", 
            self.callback_dist_to_obstacle, 
            10
        )
        
        # Create publishers
        self.pub_speed = self.create_publisher(Float64, '/bmw_speed', 1)
        self.pub_steering = self.create_publisher(Float64, '/bmw_steering', 1)
        
        # Wait for initial lane detection messages
        print("Waiting for lane detection...")
        
        # Create timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        print("Using:")
        print("Max speed: " + str(self.max_speed))
        print("K_rho: " + str(self.k_rho))
        print("K_theta: " + str(self.k_theta))

    #
    # Steering is calculated proportional to two errors: distance error and angle error.
    # These errors correspond to differences between an observed line (in normal form)
    # and a desired line.
    # Speed is calculated as the max speed minus a speed proportional to the steering.
    # In this way, car goes with lower speed in curves and at max speed in straight roads. 
    #
    def calculate_control(self, rho_l, theta_l, rho_r, theta_r, goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r, dist=None):
        error_rho_l   = goal_rho_l   - rho_l
        error_theta_l = goal_theta_l - theta_l
        error_rho_r   = rho_r   - goal_rho_r
        error_theta_r = theta_r - goal_theta_r
        
        if rho_l != 0 and rho_r != 0:
            error_rho   = (error_rho_l + error_rho_r)/2
            error_theta = (error_theta_l + error_theta_r)/2
        elif rho_l != 0:
            error_rho   = error_rho_l
            error_theta = error_theta_l
        else:
            error_rho   = error_rho_r
            error_theta = error_theta_r
        
        steering = -self.k_rho*error_rho - self.k_theta*error_theta
        if dist is None:
            speed = self.max_speed*(1 - 1.5*abs(steering))
        else:
            speed = 36 + 1.5*(dist - 20)
        return speed, steering

    def callback_left_lane(self, msg):
        self.lane_rho_l, self.lane_theta_l = msg.data

    def callback_right_lane(self, msg):
        self.lane_rho_r, self.lane_theta_r = msg.data

    def callback_enable_cruise(self, msg):
        self.enable_cruise = msg.data

    def callback_enable_follow(self, msg):
        self.enable_follow = msg.data

    def callback_dist_to_obstacle(self, msg):
        self.dist_to_obstacle = msg.data
    
    def callback_requested_speed(self, msg):
        self.requested_speed = msg.data

    def control_loop(self):
        if self.enable_cruise:
            speed, steering = self.calculate_control(
                self.lane_rho_l, self.lane_theta_l, self.lane_rho_r, self.lane_theta_r,
                self.goal_rho_l, self.goal_theta_l, self.goal_rho_r, self.goal_theta_r)
        elif self.enable_follow:
            speed, steering = self.calculate_control(
                self.lane_rho_l, self.lane_theta_l, self.lane_rho_r, self.lane_theta_r,
                self.goal_rho_l, self.goal_theta_l, self.goal_rho_r, self.goal_theta_r, 
                self.dist_to_obstacle)
        else:
            return  # Skip publishing if neither mode is enabled

        # Create and publish messages
        speed_msg = Float64()
        speed_msg.data = speed
        self.pub_speed.publish(speed_msg)
        
        steering_msg = Float64()
        steering_msg.data = steering
        self.pub_steering.publish(steering_msg)

def main():
    rclpy.init()
    node = LaneTrackingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
