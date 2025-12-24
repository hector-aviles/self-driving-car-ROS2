#!/usr/bin/env python3
"""
This node detects potential front (or sideswipe) collisions using radar readings and knowledge about the current state-action pair.
"""

# https://sensible4.fi/technology/articles/obstacle-detection-and-tracking-system-odts-enables-a-smooth-safe-ride-for-everyone/
# https://cyberbotics.com/doc/reference/radar?tab-language=python
# https://cyberbotics.com/doc/guide/radar-sensors?version=R2022a
# https://cyberbotics.com/doc/reference/radar

# IMPORTANT: For ROS2, install radar_msgs from:
# sudo apt-get install ros-jazzy-radar-msgs

import math
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Bool, Float64MultiArray, Float64, String, Empty 
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose2D
from radar_msgs.msg import RadarReturn, RadarScan


class LatentCollisionDetector(Node):
    
    def __init__(self):
        super().__init__('latent_collision_detector')
        
        print("INITIALIZING LATENT_COLLISION_DETECTOR...", flush=True)
        
        # Declare parameter
        self.declare_parameter('transversal', False)
        transversal = self.get_parameter('transversal').get_parameter_value().bool_value
        print("Is transversal?: ", transversal, flush=True)
        
        # Initialize variables
        self.latent_collision = False
        self.type_latent_collision = 0.0
        self.sdc_vel_kh = 0.0
        
        self.free_N = True
        self.free_NW = True
        self.free_W = True
        self.free_NE = True
        self.free_E = True
        self.free_SE = True    
        self.free_SW = True
        self.curr_lane = True
        self.action = "NA"
        self.radar_scan = RadarScan()
        
        # Parameters for the radar
        min_dist_radar_ref = 1    # Minimum distance (in meters)
        max_dist_radar_ref = 45   # Maximum distance (in meters)
        lateral_vision = 3        # The radar sees 3m on each side

        # Field of view (FoV) at minimum and maximum distances
        self.radar_ref = {
            "min_dist": 1,  # Minimum distance (in meters)
            "max_dist": 10,  # Maximum distance (in meters)
            # The radar should sees 3m on each side
            "max_fov": math.atan2(3, min_dist_radar_ref),  # Max FoV at 1m
            "min_fov": math.atan2(3, max_dist_radar_ref)   # Min FoV at 45m
        }
        self.radar_ref["dist"] = self.radar_ref["max_dist"]
        self.radar_ref["fov"] = self.radar_ref["min_fov"]
        self.valid_dist = 25  # distance to a vehicle in front that poses a potential crash
        
        # Create subscribers
        self.curr_lane_sub = self.create_subscription(
            Bool, '/current_lane', self.callback_curr_lane, 10)
        self.speed_sub = self.create_subscription(
            Float64, '/bmw_speed', self.callback_sdc_vel, 10)
        self.free_north_sub = self.create_subscription(
            Bool, '/free/north', self.callback_free_N, 10)
        self.free_north_west_sub = self.create_subscription(
            Bool, '/free/north_west', self.callback_free_NW, 10)
        self.free_west_sub = self.create_subscription(
            Bool, '/free/west', self.callback_free_W, 10)
        self.free_north_east_sub = self.create_subscription(
            Bool, '/free/north_east', self.callback_free_NE, 10)
        self.free_east_sub = self.create_subscription(
            Bool, '/free/east', self.callback_free_E, 10)
        self.free_south_east_sub = self.create_subscription(
            Bool, '/free/south_east', self.callback_free_SE, 10)
        self.free_south_west_sub = self.create_subscription(
            Bool, '/free/south_west', self.callback_free_SW, 10)
        self.action_sub = self.create_subscription(
            String, '/action', self.callback_action, 10)
        self.radar_sub = self.create_subscription(
            RadarScan, '/bmw_frontal_radar', self.callback_radar, 10)
        
        # Create publishers
        self.pub_latent_collision = self.create_publisher(
            Bool, '/bmw_latent_collision', 10)
        self.pub_type_latent_collision = self.create_publisher(
            Float64, '/bmw_type_latent_collision', 10)
        
        # Wait for policy node to start
        self.policy_started = False
        self.policy_sub = self.create_subscription(
            Empty, '/policy_started', self.callback_policy_started, 10)
        
        # Create timer for main processing loop
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
    def callback_policy_started(self, msg):
        self.policy_started = True
        
    def callback_free_N(self, msg):
        self.free_N = msg.data

    def callback_free_NW(self, msg):
        self.free_NW = msg.data

    def callback_free_W(self, msg):
        self.free_W = msg.data
    
    def callback_free_NE(self, msg):
        self.free_NE = msg.data    

    def callback_free_E(self, msg):
        self.free_E = msg.data    
    
    def callback_free_SE(self, msg):
        self.free_SE = msg.data    
    
    def callback_free_SW(self, msg):
        self.free_SW = msg.data        
    
    def callback_curr_lane(self, msg):
        self.curr_lane = msg.data   
    
    def callback_action(self, msg):
        self.action = msg.data             
    
    def callback_sdc_vel(self, msg):
        self.sdc_vel_kh = msg.data 
        
    def callback_radar(self, msg):
        self.radar_scan = msg
    
    def timer_callback(self):
        # Wait for policy to start
        if not self.policy_started:
            return
        
        self.latent_collision = False
        self.type_latent_collision = 0.0
        obst_dist = 1000000  # meters
        
        # Patch   
        if self.curr_lane:
            self.free_NE = self.free_N
        else:
            self.free_NW = self.free_N    
        
        # Remove unnecessary data first 
        closest_obst_front = RadarReturn()
        closest_obst_front.range = -1.0
        
        # Filter out invalid returns (range != 1.0)
        valid_returns = [r for r in self.radar_scan.returns if r.range != 1.0]
        
        # If the car is swerving enlarge FoV 
        current_fov = self.radar_ref["min_fov"]
        if self.action == "swerve_left" or self.action == "swerve_right" or self.get_parameter('transversal').get_parameter_value().bool_value:
            current_fov = 0.8725  # 50 degrees each side
        else:
            current_fov = self.radar_ref["min_fov"]
       
        # Get the closest vehicle in front 
        for radar in valid_returns:
            if (-current_fov <= radar.azimuth <= current_fov 
                  and radar.range <= self.valid_dist
               ):
                if radar.range < obst_dist:
                    obst_dist = radar.range
                    closest_obst_front = radar 
       
        # Conditions to warn a potential collision when turning
        # running off the road    
        if self.action == 'change_to_right' and self.curr_lane:
            self.latent_collision = True
            self.type_latent_collision = 1.0
            print("Latent collision while changing to right on right lane", flush=True)
        # running off the road   
        elif self.action == 'change_to_left' and not self.curr_lane:
            self.latent_collision = True
            self.type_latent_collision = 2.0
            print("Latent collision while changing to left on left lane", flush=True)
        # sideSwipe collision   
        elif self.action == 'change_to_left' and self.curr_lane and (not self.free_NW or not self.free_W):
            self.latent_collision = True
            self.type_latent_collision = 3.0
            print("Latent collision while changing to left", self.free_NW, self.free_W, self.free_SW, sep=" ", flush=True)
        # sideSwipe collision
        elif self.action == 'change_to_right' and not self.curr_lane and (not self.free_NE or not self.free_E):   
            self.latent_collision = True
            self.type_latent_collision = 4.0
            print("Latent collision while changing to right", self.free_NE, self.free_E, self.free_SE, sep=" ", flush=True)
        # free to go    
        elif self.action == 'change_to_left' and self.curr_lane and self.free_NW and self.free_W and self.free_SW:
            self.latent_collision = False
            self.type_latent_collision = -1.0
        # free to go    
        elif self.action == 'change_to_right' and not self.curr_lane and self.free_NE and self.free_E and self.free_SE:
            self.latent_collision = False
            self.type_latent_collision = -1.0

        # Rear end collision    
        elif closest_obst_front.range != -1.0:
            self.latent_collision = True 

            # Interpolation of FoV based on the distance of the obstacle
            fov_range = self.radar_ref["max_fov"] - self.radar_ref["min_fov"]
            dist_ratio = ((self.radar_ref["max_dist"] -
                          closest_obst_front.range) /
                         (self.radar_ref["max_dist"] - self.radar_ref["min_dist"]))
            
            # self-driving car (sdc_vel_kh) from km/h to m/s         
            sdc_vel_ms = (self.sdc_vel_kh * 1000) / 3600       
            # V_AB = V_A - V_B ==> V_B = V_A + V_AB
            obst_vel = (sdc_vel_ms + closest_obst_front.doppler_velocity)
            
            if -1.0 < obst_vel <= 1.0:
                obst_vel = 0.0  # Assume the obstacle is stationary
                obst_stationary = True
                self.type_latent_collision = 5.0
            elif obst_vel < -1.0:
                obst_opposite_dir = True
                self.type_latent_collision = 6.0
            else:                   
                obst_same_dir = True
                self.type_latent_collision = 7.0
             
            safe_dist = 15              
            # Conditions of potential collisions 
            if obst_stationary:
                print("Obstacle stationary ahead", flush=True)
            elif obst_opposite_dir:
                print("Obstacle approaching in opposite direction", flush=True)
            elif self.action == "cruise" and (obst_dist < safe_dist and obst_vel < sdc_vel_ms):
                print("Obstacle moving in the same direction possibly at a slower velocity", flush=True)
     
        # Publish results
        latent_msg = Bool()
        latent_msg.data = self.latent_collision
        self.pub_latent_collision.publish(latent_msg)
        
        type_msg = Float64()
        type_msg.data = self.type_latent_collision
        self.pub_type_latent_collision.publish(type_msg)


def main(args=None):
    rclpy.init(args=args)
    latent_collision_detector = LatentCollisionDetector()
    rclpy.spin(latent_collision_detector)
    latent_collision_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
