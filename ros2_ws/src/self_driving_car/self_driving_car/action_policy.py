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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

qos_latched = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE
)


class ActionPolicy(BaseEstimator, ClassifierMixin):
    def __init__(self):
        # Precomputed lookup table of size 128
        self.actions = [None] * 128
        self.actions[0] = "keep"
        self.actions[1] = "keep"
        self.actions[2] = "keep"
        self.actions[3] = "keep"
        self.actions[4] = "keep"
        self.actions[5] = "cruise"
        self.actions[6] = "change_to_right"
        self.actions[7] = "cruise"
        self.actions[8] = "cruise"
        self.actions[9] = "keep"
        self.actions[10] = "cruise"
        self.actions[11] = "keep"
        self.actions[12] = "cruise"
        self.actions[13] = "cruise"
        self.actions[14] = "change_to_right"
        self.actions[15] = "cruise"
        self.actions[16] = "keep"
        self.actions[17] = "keep"
        self.actions[18] = "keep"
        self.actions[19] = "keep"
        self.actions[20] = "keep"
        self.actions[21] = "cruise"
        self.actions[22] = "change_to_right"
        self.actions[23] = "cruise"
        self.actions[24] = "cruise"
        self.actions[25] = "keep"
        self.actions[26] = "cruise"
        self.actions[27] = "keep"
        self.actions[28] = "cruise"
        self.actions[29] = "cruise"
        self.actions[30] = "change_to_right"
        self.actions[31] = "cruise"
        self.actions[32] = "keep"
        self.actions[33] = "keep"
        self.actions[34] = "keep"
        self.actions[35] = "keep"
        self.actions[36] = "keep"
        self.actions[37] = "cruise"
        self.actions[38] = "change_to_right"
        self.actions[39] = "cruise"
        self.actions[40] = "cruise"
        self.actions[41] = "keep"
        self.actions[42] = "cruise"
        self.actions[43] = "keep"
        self.actions[44] = "cruise"
        self.actions[45] = "cruise"
        self.actions[46] = "change_to_right"
        self.actions[47] = "cruise"
        self.actions[48] = "keep"
        self.actions[49] = "keep"
        self.actions[50] = "keep"
        self.actions[51] = "keep"
        self.actions[52] = "keep"
        self.actions[53] = "cruise"
        self.actions[54] = "change_to_right"
        self.actions[55] = "cruise"
        self.actions[56] = "cruise"
        self.actions[57] = "keep"
        self.actions[58] = "cruise"
        self.actions[59] = "keep"
        self.actions[60] = "cruise"
        self.actions[61] = "cruise"
        self.actions[62] = "change_to_right"
        self.actions[63] = "cruise"
        self.actions[64] = "keep"
        self.actions[65] = "keep"
        self.actions[66] = "keep"
        self.actions[67] = "keep"
        self.actions[68] = "keep"
        self.actions[69] = "cruise"
        self.actions[70] = "change_to_right"
        self.actions[71] = "cruise"
        self.actions[72] = "cruise"
        self.actions[73] = "change_to_left"
        self.actions[74] = "cruise"
        self.actions[75] = "change_to_left"
        self.actions[76] = "cruise"
        self.actions[77] = "cruise"
        self.actions[78] = "change_to_right"
        self.actions[79] = "cruise"
        self.actions[80] = "keep"
        self.actions[81] = "keep"
        self.actions[82] = "keep"
        self.actions[83] = "keep"
        self.actions[84] = "keep"
        self.actions[85] = "cruise"
        self.actions[86] = "change_to_right"
        self.actions[87] = "cruise"
        self.actions[88] = "cruise"
        self.actions[89] = "change_to_left"
        self.actions[90] = "cruise"
        self.actions[91] = "change_to_left"
        self.actions[92] = "cruise"
        self.actions[93] = "cruise"
        self.actions[94] = "change_to_right"
        self.actions[95] = "cruise"
        self.actions[96] = "keep"
        self.actions[97] = "keep"
        self.actions[98] = "keep"
        self.actions[99] = "keep"
        self.actions[100] = "keep"
        self.actions[101] = "cruise"
        self.actions[102] = "change_to_right"
        self.actions[103] = "cruise"
        self.actions[104] = "cruise"
        self.actions[105] = "change_to_left"
        self.actions[106] = "cruise"
        self.actions[107] = "change_to_left"
        self.actions[108] = "cruise"
        self.actions[109] = "cruise"
        self.actions[110] = "change_to_right"
        self.actions[111] = "cruise"
        self.actions[112] = "keep"
        self.actions[113] = "keep"
        self.actions[114] = "keep"
        self.actions[115] = "keep"
        self.actions[116] = "keep"
        self.actions[117] = "cruise"
        self.actions[118] = "change_to_right"
        self.actions[119] = "cruise"
        self.actions[120] = "cruise"
        self.actions[121] = "change_to_left"
        self.actions[122] = "cruise"
        self.actions[123] = "change_to_left"
        self.actions[124] = "cruise"
        self.actions[125] = "cruise"
        self.actions[126] = "change_to_right"
        self.actions[127] = "cruise"

    def fit(self, X, y=None):
        return self

    def predict(self, X):
        cols = ['curr_lane', 'free_E', 'free_NE', 'free_NW', 'free_SE', 'free_SW', 'free_W']
        data = X[cols].to_numpy().astype(bool).astype(int)
        powers = 2 ** np.arange(7)
        indices = data @ powers
        return np.array(self.actions)[indices]

class ActionPolicyNode(Node):
    def __init__(self, speed_left, speed_right, speed_citroen):
        super().__init__('action_policy')
        
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
        self.vel_citroen_czero = int(speed_citroen)
        
        print("INITIALIZING POLICY...", flush=True)
        
        # Create subscribers
        self.create_subscription(Bool, "/BMW/free_N", self.callback_free_N, 10)
        self.create_subscription(Bool, "/BMW/free_NW", self.callback_free_NW, 10)
        self.create_subscription(Bool, "/BMW/free_W", self.callback_free_W, 10)
        self.create_subscription(Bool, "/BMW/free_SW", self.callback_free_SW, 10)
        self.create_subscription(Bool, "/BMW/free_NE", self.callback_free_NE, 10)
        self.create_subscription(Bool, "/BMW/free_E", self.callback_free_E, 10)
        self.create_subscription(Bool, "/BMW/free_SE", self.callback_free_SE, 10)
        self.create_subscription(Bool, "/BMW/current_lane", self.callback_curr_lane, 10)
        self.create_subscription(Bool, "/BMW/change_lane/finished", self.callback_change_lane_finished, 10)
        
        # Create publishers
        self.pub_policy_started = self.create_publisher(Empty, "/BMW/policy/started", 1)
        self.pub_cruise = self.create_publisher(Bool, "/BMW/cruise/enable", qos_latched)
        self.pub_keep_distance = self.create_publisher(Bool, "/BMW/follow/enable", qos_latched)
        self.pub_change_lane_on_left = self.create_publisher(Bool, "/BMW/change_lane_on_left/started", qos_latched)
        self.pub_change_lane_on_right = self.create_publisher(Bool, "/BMW/change_lane_on_right/started", qos_latched)
        self.pub_action = self.create_publisher(String, "/bmw_action", qos_latched)
        self.pub_speed_vehicles_left_lane = self.create_publisher(Float64, "/vehicles_left_lane/speed", 1)
        self.pub_speed_vehicles_right_lane = self.create_publisher(Float64, "/vehicles_right_lane/speed", 1)
        self.pub_speed_citroen_czero = self.create_publisher(Float64, "/CitroenCZero/speed", 1)
        
        # Create ActionPolicy model
        self.get_logger().info("Creating the decision-maker module based on an action policy...")
        try:
            self.model = ActionPolicy()
            self.get_logger().info("Decision-maker module created successfully")
        except Exception as error:
            self.get_logger().error(f"Error creating the decision-maker module: {error}")
            
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

    def wait_for_change_lane_finished(self, timeout_sec=10.0):
        """
        ROS2 equivalent of rospy.wait_for_message with timeout.
        Returns True if finished signal received, False on timeout.
        """
        self.change_lane_finished = False
    
        def callback(msg):
            self.change_lane_finished = msg.data
    
        sub = self.create_subscription(
            Bool,
            "/BMW/change_lane/finished",
            callback,
            10
        )
    
        start_time = self.get_clock().now()
    
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
    
            if self.change_lane_finished:
                self.destroy_subscription(sub)
                return True
    
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            if elapsed > timeout_sec:
                self.get_logger().warn("Timeout waiting for change_lane_finished")
                self.destroy_subscription(sub)
                return False


    def main_loop(self):
        # Publish policy started
        self.pub_policy_started.publish(Empty())
        
        # Publish speed for vehicles in left lane
        speed_left_msg = Float64()
        speed_left_msg.data = float(self.vel_vehicles_left_lane)
        self.pub_speed_vehicles_left_lane.publish(speed_left_msg)
        
        # Publish speed for vehicles in right lane
        speed_right_msg = Float64()
        speed_right_msg.data = float(self.vel_vehicles_right_lane)
        self.pub_speed_vehicles_right_lane.publish(speed_right_msg)
        
        # Publish speed for Citroen CZero
        speed_citroen_msg = Float64()
        speed_citroen_msg.data = float(self.vel_citroen_czero)
        self.pub_speed_citroen_czero.publish(speed_citroen_msg)
        
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
        
            finished = self.wait_for_change_lane_finished(timeout_sec=500.0)
        
            if finished:
                self.get_logger().info("Change lane to LEFT finished")
            else:
                self.get_logger().warn("Change lane to LEFT timed out")
                           
        elif self.action == "change_to_right":
            self.pub_action.publish(String(data="change_to_right"))
            self.change_lane_on_right()
        
            self.get_logger().info("Waiting for change lane to finish...")
        
            finished = self.wait_for_change_lane_finished(timeout_sec=500.0)
        
            if finished:
                self.get_logger().info("Change lane to RIGHT finished")
            else:
                self.get_logger().warn("Change lane to RIGHT timed out")
        else:
            self.get_logger().warn(f"Unknown action: {self.action}")

def main():
    rclpy.init()
    
    if len(sys.argv) != 4:
        print("Usage: ros2 run self_driving_car action_policy speed_left_vehicles (m/s) speed_right_vehicles (m/s) speed_citroen (m/s)")
        print("Example: ros2 run self_driving_car action_policy 20 25 18")
        return
    
    speed_left = sys.argv[1]
    speed_right = sys.argv[2]
    speed_citroen = sys.argv[3]
    
    node = ActionPolicyNode(speed_left, speed_right, speed_citroen)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
