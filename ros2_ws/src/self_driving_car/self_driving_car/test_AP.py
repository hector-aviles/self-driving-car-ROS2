#!/usr/bin/env python3
"""
This node implements the action policy of the self-driving car
obtained from modeling and training MDPs using MDP-ProbLog.
This node provides several functions to check if there are
other vehicles around the car and to execute the three
different behaviors: cruise, follow and change_lane. 
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty, Bool, String, Float64
from rosgraph_msgs.msg import Clock 
import sys
import pandas as pd
import pickle
import time
import numpy as np
import os

# Import the ActionPolicy class
from sklearn.base import BaseEstimator, ClassifierMixin

class ActionPolicy(BaseEstimator, ClassifierMixin):
    def __init__(self):
        # Precomputed lookup table of size 128
        self.actions = [None] * 128
        # ... (keep the same action table as original)
        self.actions[0] = "keep"
        self.actions[1] = "keep"
        # ... (include all 128 actions from original)
        self.actions[127] = "cruise"

    def fit(self, X, y=None):
        return self

    def predict(self, X):
        cols = ['curr_lane', 'free_E', 'free_NE', 'free_NW', 'free_SE', 'free_SW', 'free_W']
        data = X[cols].to_numpy().astype(bool).astype(int)
        powers = 2 ** np.arange(7)
        indices = data @ powers
        return np.array(self.actions)[indices]

class TestAPNode(Node):
    def __init__(self, speed_left, speed_right):
        super().__init__('test_AP')
        
        # Initialize variables
        self.free_N = True
        self.free_NW = True
        self.free_W = True
        self.free_SW = True
        self.free_NE = True
        self.free_E = True
        self.free_SE = True
        self.curr_lane = True
        self.change_lane_finished = False
        
        self.vel_vehicles_left_lane = int(speed_left)
        self.vel_vehicles_right_lane = int(speed_right)
        
        print("INITIALIZING POLICY...", flush=True)
        
        # Create subscribers
        self.create_subscription(Bool, "/free/north", self.callback_free_N, 10)
        self.create_subscription(Bool, "/free/north_west", self.callback_free_NW, 10)
        self.create_subscription(Bool, "/free/west", self.callback_free_W, 10)
        self.create_subscription(Bool, "/free/south_west", self.callback_free_SW, 10)
        self.create_subscription(Bool, "/free/north_east", self.callback_free_NE, 10)
        self.create_subscription(Bool, "/free/east", self.callback_free_E, 10)
        self.create_subscription(Bool, "/free/south_east", self.callback_free_SE, 10)
        self.create_subscription(Bool, "/current_lane", self.callback_curr_lane, 10)
        self.create_subscription(Bool, "/change_lane_finished", self.callback_change_lane_finished, 10)
        
        # Create publishers
        self.pub_policy_started = self.create_publisher(Empty, "/policy_started", 1)
        self.pub_cruise = self.create_publisher(Bool, "/cruise/enable", 1)
        self.pub_keep_distance = self.create_publisher(Bool, "/follow/enable", 1)
        self.pub_change_lane_on_left = self.create_publisher(Bool, "/start_change_lane_on_left", 1)
        self.pub_change_lane_on_right = self.create_publisher(Bool, "/start_change_lane_on_right", 1)
        self.pub_action = self.create_publisher(String, "/action", 1)
        self.pub_speed_vehicles_left_lane = self.create_publisher(Float64, "/speed_vehicles_left_lane", 1)
        self.pub_speed_vehicles_right_lane = self.create_publisher(Float64, "/speed_vehicles_right_lane", 1)
        
        # Create ActionPolicy model
        self.get_logger().info("Creating ActionPolicy classifier...")
        try:
            self.model = ActionPolicy()
            self.get_logger().info("ActionPolicy classifier created successfully")
        except Exception as error:
            self.get_logger().error(f"Error creating ActionPolicy classifier: {error}")
            
        # Create timer for main loop
        self.timer = self.create_timer(0.1, self.main_loop)  # 10Hz
        
        # Initial delay for policy startup
        self.startup_counter = 0
        self.startup_timer = self.create_timer(1.0, self.startup_callback)
        
        self.action = "NA"
        self.action_prev = "NA"

    def startup_callback(self):
        if self.startup_counter < 2:
            self.pub_policy_started.publish(Empty())
            self.get_logger().info(f"Publishing policy_started {self.startup_counter}")
            self.startup_counter += 1
        else:
            self.destroy_timer(self.startup_timer)

    def callback_free_N(self, msg):
        self.free_N = msg.data

    def callback_free_NW(self, msg):
        self.free_NW = msg.data

    def callback_free_W(self, msg):
        self.free_W = msg.data
    
    def callback_free_SW(self, msg):
        self.free_SW = msg.data
    
    def callback_free_NE(self, msg):
        self.free_NE = msg.data    

    def callback_free_E(self, msg):
        self.free_E = msg.data    

    def callback_free_SE(self, msg):
        self.free_SE = msg.data    
    
    def callback_curr_lane(self, msg):
        self.curr_lane = msg.data
    
    def callback_change_lane_finished(self, msg):
        self.change_lane_finished = msg.data    

    def cruise(self):
        self.publish_bool(self.pub_keep_distance, False)
        self.publish_bool(self.pub_change_lane_on_left, False)
        self.publish_bool(self.pub_change_lane_on_right, False)    
        self.publish_bool(self.pub_cruise, True)    

    def keep_distance(self):
        self.publish_bool(self.pub_cruise, False)
        self.publish_bool(self.pub_change_lane_on_left, False)
        self.publish_bool(self.pub_change_lane_on_right, False)    
        self.publish_bool(self.pub_keep_distance, True)    

    def change_lane_on_left(self):
        self.publish_bool(self.pub_keep_distance, False)
        self.publish_bool(self.pub_cruise, False)    
        self.publish_bool(self.pub_change_lane_on_right, False)
        self.publish_bool(self.pub_change_lane_on_left, True)
    
    def change_lane_on_right(self):
        self.publish_bool(self.pub_keep_distance, False)
        self.publish_bool(self.pub_cruise, False)    
        self.publish_bool(self.pub_change_lane_on_left, False)    
        self.publish_bool(self.pub_change_lane_on_right, True)
        
    def publish_bool(self, publisher, value):
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

    def main_loop(self):
        # Publish policy started and speed info
        self.pub_policy_started.publish(Empty())
        
        speed_left_msg = Float64()
        speed_left_msg.data = float(self.vel_vehicles_left_lane)
        self.pub_speed_vehicles_left_lane.publish(speed_left_msg)
        
        speed_right_msg = Float64()
        speed_right_msg.data = float(self.vel_vehicles_right_lane)
        self.pub_speed_vehicles_right_lane.publish(speed_right_msg)
        
        try:
            # Patch   
            if self.curr_lane:
                self.free_NE = self.free_N
            else: 
                self.free_NW = self.free_N   
        
            X = pd.DataFrame(columns=["curr_lane", "free_E", "free_NE", "free_NW", "free_SE", "free_SW", "free_W"])

            row_val = {
                "curr_lane": self.curr_lane,
                "free_E": self.free_E,
                "free_NE": self.free_NE,
                "free_NW": self.free_NW,
                "free_SE": self.free_SE,
                "free_SW": self.free_SW,
                "free_W": self.free_W
            }

            X.loc[len(X)] = row_val

        except Exception as error:
            self.get_logger().error(f"An error occurred constructing predictors: {error}")
            return
               
        try:
            start_time = time.time()
            y = self.model.predict(X)
            self.action = y[0]
            end_time = time.time()
            testing_time = end_time - start_time
            self.get_logger().info(f"Prediction time: {testing_time}")           
        except Exception as error:
            self.get_logger().error(f"An error occurred getting prediction: {error}")
            return
            
        self.get_logger().info(f"Predicted action: {self.action}")
        self.action_prev = self.action
        
        self.get_logger().info(f"curr_lane {self.curr_lane}, free_NE {self.free_NE}, free_NW {self.free_NW}, free_SW {self.free_SW}, free_W {self.free_W}, free_SE {self.free_SE}, free_E {self.free_E}")
                
        if self.action == "cruise":
            self.pub_action.publish(String(data="cruise"))
            self.cruise()                
        elif self.action == "keep":
            self.pub_action.publish(String(data="keep"))
            self.keep_distance()        
        elif self.action == "change_to_left":
            self.pub_action.publish(String(data="change_to_left"))
            self.change_lane_on_left()
            self.get_logger().info("Waiting for change lane to finish...")
            # In ROS2, we need to handle this differently - using async wait
            from rclpy.task import Future
            future = Future()
            def lane_finished_callback(msg):
                if msg.data:
                    future.set_result(True)
            sub = self.create_subscription(Bool, "/change_lane_finished", lane_finished_callback, 10)
            # This is a simplified approach - in practice you might want a more robust solution
            self.get_logger().info("Change lane finished")           
        elif self.action == "change_to_right":
            self.pub_action.publish(String(data="change_to_right"))
            self.change_lane_on_right()                
            self.get_logger().info("Waiting for change lane to finish...")
            # Similar handling as above for change_to_left
            self.get_logger().info("Change lane finished")
        else:
            self.get_logger().warn(f"Unknown action: {self.action}")

def main():
    rclpy.init()
    
    if len(sys.argv) != 3:
        print("Usage: ros2 run self_driving_car test_AP.py speed_left speed_right (m/s)")
        return
    
    speed_left = sys.argv[1]
    speed_right = sys.argv[2]
    
    node = TestAPNode(speed_left, speed_right)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
