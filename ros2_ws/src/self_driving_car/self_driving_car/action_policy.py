#!/usr/bin/env python3
"""
Action Policy node (ROS2)

Implements the decision policy obtained from MDP-ProbLog.
Execution model:
- Explicit main loop
- spin_once() for callbacks
- Fixed-rate policy evaluation
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Empty, Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import sys
import time
import pandas as pd
import numpy as np

from sklearn.base import BaseEstimator, ClassifierMixin


# ---------------- QoS ----------------

qos_latched = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE
)


# ---------------- Action Policy ----------------

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
        cols = ['curr_lane', 'free_E', 'free_NE', 'free_NW',
                'free_SE', 'free_SW', 'free_W']
        data = X[cols].to_numpy().astype(bool).astype(int)
        powers = 2 ** np.arange(7)
        indices = data @ powers
        return np.array(self.actions)[indices]


# ---------------- Node ----------------

class ActionPolicyNode(Node):

    def __init__(self, speed_left, speed_right, speed_citroen):
        super().__init__('action_policy')

        # ---- State flags ----
        self.free_N = True
        self.free_NW = True
        self.free_W = True
        self.free_SW = True
        self.free_NE = True
        self.free_E = True
        self.free_SE = True
        self.curr_lane = True
        self.change_lane_finished = False

        self.executing_lane_change = False

        # ---- Speeds ----
        self.vel_vehicles_left_lane = float(speed_left)
        self.vel_vehicles_right_lane = float(speed_right)
        self.vel_citroen_czero = float(speed_citroen)

        # ---- Subscribers ----
        self.create_subscription(Bool, "/BMW/free_N", self.cb_free_N, 10)
        self.create_subscription(Bool, "/BMW/free_NW", self.cb_free_NW, 10)
        self.create_subscription(Bool, "/BMW/free_W", self.cb_free_W, 10)
        self.create_subscription(Bool, "/BMW/free_SW", self.cb_free_SW, 10)
        self.create_subscription(Bool, "/BMW/free_NE", self.cb_free_NE, 10)
        self.create_subscription(Bool, "/BMW/free_E", self.cb_free_E, 10)
        self.create_subscription(Bool, "/BMW/free_SE", self.cb_free_SE, 10)
        self.create_subscription(Bool, "/BMW/current_lane", self.cb_curr_lane, 10)
        self.create_subscription(Bool, "/BMW/change_lane/finished",
                                 self.cb_change_lane_finished, 10)

        # ---- Publishers ----
        self.pub_policy_started = self.create_publisher(
            Empty, "/BMW/policy/started", qos_latched)

        self.pub_cruise = self.create_publisher(
            Bool, "/BMW/cruise/enable", qos_latched)

        self.pub_keep_distance = self.create_publisher(
            Bool, "/BMW/follow/enable", qos_latched)

        self.pub_change_lane_left = self.create_publisher(
            Bool, "/BMW/change_lane_on_left/started", qos_latched)

        self.pub_change_lane_right = self.create_publisher(
            Bool, "/BMW/change_lane_on_right/started", qos_latched)

        self.pub_action = self.create_publisher(
            String, "/BMW/action", qos_latched)

        self.pub_speed_left = self.create_publisher(
            Float64, "/vehicles_left_lane/speed", 1)

        self.pub_speed_right = self.create_publisher(
            Float64, "/vehicles_right_lane/speed", 1)

        self.pub_speed_citroen = self.create_publisher(
            Float64, "/CitroenCZero/speed", 1)

        # ---- Policy model ----
        self.get_logger().info("Initializing ActionPolicy model...")
        self.model = ActionPolicy()
        self.get_logger().info("ActionPolicy ready")


        self.policy_period = 0.1  # 10 Hz

    # ---------------- Callbacks ----------------

    def cb_free_N(self, msg): self.free_N = msg.data
    def cb_free_NW(self, msg): self.free_NW = msg.data
    def cb_free_W(self, msg): self.free_W = msg.data
    def cb_free_SW(self, msg): self.free_SW = msg.data
    def cb_free_NE(self, msg): self.free_NE = msg.data
    def cb_free_E(self, msg): self.free_E = msg.data
    def cb_free_SE(self, msg): self.free_SE = msg.data
    def cb_curr_lane(self, msg): self.curr_lane = msg.data
    def cb_change_lane_finished(self, msg): self.change_lane_finished = msg.data

    # ---------------- Helpers ----------------

    def publish_bool(self, pub, value):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    # ---------------- Main loop ----------------

    def run(self):

        while rclpy.ok():


            # ---- Publish startup signal ONCE ----
            self.pub_policy_started.publish(Empty())

            start = time.time()            

            # ---- Process ROS callbacks ----
            rclpy.spin_once(self, timeout_sec=0.01)

            # ---- Publish speeds ----
            self.pub_speed_left.publish(Float64(data=self.vel_vehicles_left_lane))
            self.pub_speed_right.publish(Float64(data=self.vel_vehicles_right_lane))
            self.pub_speed_citroen.publish(Float64(data=self.vel_citroen_czero))

            # ---- Lane-change guard ----
            if self.executing_lane_change:
                if self.change_lane_finished:
                    self.get_logger().info("Lane change finished")
                    self.executing_lane_change = False
                    self.change_lane_finished = False
                self._sleep_remaining(start)
                continue

            # ---- Patch logic (UNCHANGED) ----
            if self.curr_lane:
                self.free_NE = self.free_N
            else:
                self.free_NW = self.free_N

            # ---- Build feature vector ----
            X = pd.DataFrame([{
                "curr_lane": self.curr_lane,
                "free_E": self.free_E,
                "free_NE": self.free_NE,
                "free_NW": self.free_NW,
                "free_SE": self.free_SE,
                "free_SW": self.free_SW,
                "free_W": self.free_W
            }])

            # ---- Predict ----
            action = self.model.predict(X)[0]
            self.get_logger().info(f"State: {X}")
            self.get_logger().info(f"Predicted action: {action}")

            # ---- Execute ----
            self.pub_action.publish(String(data=action))

            if action == "cruise":
                self.publish_bool(self.pub_cruise, True)

            elif action == "keep":
                self.publish_bool(self.pub_keep_distance, True)

            elif action == "change_to_left" and not self.executing_lane_change:
                self.get_logger().info("Starting lane change LEFT")
                self.executing_lane_change = True
                self.publish_bool(self.pub_change_lane_left, True)

            elif action == "change_to_right" and not self.executing_lane_change:
                self.get_logger().info("Starting lane change RIGHT")
                self.executing_lane_change = True
                self.publish_bool(self.pub_change_lane_right, True)

            self._sleep_remaining(start)

    def _sleep_remaining(self, start):
        elapsed = time.time() - start
        remaining = self.policy_period - elapsed
        if remaining > 0:
            time.sleep(remaining)


# ---------------- Main ----------------

def main():
    rclpy.init()

    if len(sys.argv) != 4:
        print("Usage: ros2 run self_driving_car action_policy v_left v_right v_citroen")
        return

    node = ActionPolicyNode(sys.argv[1], sys.argv[2], sys.argv[3])
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

