#!/usr/bin/env python3
"""
ROS2 Action Policy with Counterfactual Reasoning

- Deterministic hash-table policy (MDP-ProbLog)
- DFA-based arbitration
- Counterfactual CSV support
- ROS2 Jazzy compatible
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Empty, Float64
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import pandas as pd
import numpy as np
import sys
import time

# ---------------- QoS ----------------

qos_latched = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE
)

# ---------------- DFA states ----------------

DFA_INIT = 0
DFA_POLICY = 10
DFA_WHATIF = 20

# ---------------- Hash-table Policy ----------------

class ActionPolicy:

    def __init__(self):
        self.actions = [None] * 128
        self.actions[:] = [
            "keep","keep","keep","keep","keep","cruise","change_to_right","cruise",
            "cruise","keep","cruise","keep","cruise","cruise","change_to_right","cruise",
            "keep","keep","keep","keep","keep","cruise","change_to_right","cruise",
            "cruise","keep","cruise","keep","cruise","cruise","change_to_right","cruise",
            "keep","keep","keep","keep","keep","cruise","change_to_right","cruise",
            "cruise","keep","cruise","keep","cruise","cruise","change_to_right","cruise",
            "keep","keep","keep","keep","keep","cruise","change_to_right","cruise",
            "cruise","keep","cruise","keep","cruise","cruise","change_to_right","cruise",
            "keep","keep","keep","keep","keep","cruise","change_to_right","cruise",
            "cruise","change_to_left","cruise","change_to_left","cruise","cruise","change_to_right","cruise",
            "keep","keep","keep","keep","keep","cruise","change_to_right","cruise",
            "cruise","change_to_left","cruise","change_to_left","cruise","cruise","change_to_right","cruise",
            "keep","keep","keep","keep","keep","cruise","change_to_right","cruise",
            "cruise","change_to_left","cruise","change_to_left","cruise","cruise","change_to_right","cruise"
        ]

    def predict(self, X: pd.DataFrame):
        cols = [
            "curr_lane",
            "free_E",
            "free_NE",
            "free_NW",
            "free_SE",
            "free_SW",
            "free_W"
        ]
        data = X[cols].astype(bool).astype(int).to_numpy()
        powers = 2 ** np.arange(7)
        idx = data @ powers
        return np.array(self.actions)[idx]

# ---------------- Counterfactual logic ----------------

def get_WhatIf_action(obs_state, prev_action, df_whatif, df_choices):
    filtered = df_whatif[
        (df_whatif["action"] == prev_action) &
        (df_whatif["curr_lane"] == obs_state["curr_lane"]) &
        (df_whatif["free_E"] == obs_state["free_E"]) &
        (df_whatif["free_NE"] == obs_state["free_NE"]) &
        (df_whatif["free_NW"] == obs_state["free_NW"]) &
        (df_whatif["free_SE"] == obs_state["free_SE"]) &
        (df_whatif["free_SW"] == obs_state["free_SW"]) &
        (df_whatif["free_W"] == obs_state["free_W"])
    ]

    if filtered.empty:
        return "NA"

    alternatives = {
        "swerve_left": 0,
        "swerve_right": 0,
        "cruise": 0,
        "keep": 0,
        "change_to_left": 0,
        "change_to_right": 0
    }

    for a in filtered["iaction"]:
        if a in alternatives:
            alternatives[a] = 1

    row = df_choices[
        (df_choices[list(alternatives.keys())] ==
         pd.Series(alternatives)).all(axis=1)
    ]

    if row.empty:
        return "NA"

    return row["right_lane"].values[0] if obs_state["curr_lane"] else row["left_lane"].values[0]

# ---------------- ROS2 Node ----------------

class ActionPolicyNode(Node):

    def __init__(self, speed_left, speed_right, speed_citroen, cf_path, choices_path):
        super().__init__("action_policy")

        # ---- State ----
        self.curr_lane = True
        self.free_N = self.free_E = self.free_NE = self.free_NW = True
        self.free_SE = self.free_SW = self.free_W = True
        self.latent_collision = False

        self.prev_obs_state = {}
        self.prev_action = "cruise"
        self.prev_latent_collision = None

        self.dfa_state = DFA_INIT

        # ---- Policy ----
        self.model = ActionPolicy()
        self.df_whatif = pd.read_csv(cf_path)
        self.df_choices = pd.read_csv(choices_path)

        # ---- Speeds ----
        self.v_left = float(speed_left)
        self.v_right = float(speed_right)
        self.v_ego = float(speed_citroen)

        # ---- Publishers ----
        self.pub_action = self.create_publisher(String, "/BMW/action", qos_latched)
        self.pub_started = self.create_publisher(Empty, "/BMW/policy/started", qos_latched)

        self.pub_speed_left = self.create_publisher(Float64, "/vehicles_left_lane/speed", qos_latched)
        self.pub_speed_right = self.create_publisher(Float64, "/vehicles_right_lane/speed", qos_latched)
        self.pub_speed_ego = self.create_publisher(Float64, "/CitroenCZero/speed", qos_latched)

        # ---- Subscribers ----
        self.create_subscription(Bool, "/BMW/current_lane", lambda m: setattr(self, "curr_lane", m.data), 10)
        self.create_subscription(Bool, "/BMW/free_N", lambda m: setattr(self, "free_N", m.data), 10)
        self.create_subscription(Bool, "/BMW/free_E", lambda m: setattr(self, "free_E", m.data), 10)
        self.create_subscription(Bool, "/BMW/free_NE", lambda m: setattr(self, "free_NE", m.data), 10)
        self.create_subscription(Bool, "/BMW/free_NW", lambda m: setattr(self, "free_NW", m.data), 10)
        self.create_subscription(Bool, "/BMW/free_SE", lambda m: setattr(self, "free_SE", m.data), 10)
        self.create_subscription(Bool, "/BMW/free_SW", lambda m: setattr(self, "free_SW", m.data), 10)
        self.create_subscription(Bool, "/BMW/free_W", lambda m: setattr(self, "free_W", m.data), 10)
        self.create_subscription(Bool, "/latent_collision", lambda m: setattr(self, "latent_collision", m.data), 10)

        self.policy_period = 0.1  # 10 Hz

    # ---------------- Main loop ----------------

    def run(self):
        while rclpy.ok():
            start = time.time()
            rclpy.spin_once(self, timeout_sec=0.01)

            self.pub_started.publish(Empty())
            self.pub_speed_left.publish(Float64(data=self.v_left))
            self.pub_speed_right.publish(Float64(data=self.v_right))
            self.pub_speed_ego.publish(Float64(data=self.v_ego))

            if self.curr_lane:
                self.free_NE = self.free_N
            else:
                self.free_NW = self.free_N

            obs = {
                "curr_lane": self.curr_lane,
                "free_E": self.free_E,
                "free_NE": self.free_NE,
                "free_NW": self.free_NW,
                "free_SE": self.free_SE,
                "free_SW": self.free_SW,
                "free_W": self.free_W
            }

            if obs != self.prev_obs_state or self.latent_collision != self.prev_latent_collision:
                if self.latent_collision:
                    action = get_WhatIf_action(obs, self.prev_action, self.df_whatif, self.df_choices)
                    self.dfa_state = DFA_WHATIF
                else:
                    X = pd.DataFrame([obs])
                    action = self.model.predict(X)[0]
                    self.dfa_state = DFA_POLICY

                if action != self.prev_action:
                    self.pub_action.publish(String(data=action))
                    self.prev_action = action

                self.prev_obs_state = obs.copy()
                self.prev_latent_collision = self.latent_collision

            elapsed = time.time() - start
            if elapsed < self.policy_period:
                time.sleep(self.policy_period - elapsed)

# ---------------- Main ----------------

def main():
    rclpy.init()

    if len(sys.argv) != 6:
        print("Usage: ros2 run decision_making action_policy speed_left speed_right speed_ego counterfactuals.csv choices.csv")
        return

    node = ActionPolicyNode(*sys.argv[1:])
    node.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

