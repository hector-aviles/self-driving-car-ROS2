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
        #print(f"Powers: {powers} Index: {idx}", flush = True)
        return np.array(self.actions)[idx]

# ---------------- Counterfactual logic ----------------

def get_WhatIf_action(obs_state, prev_action, df_counterfactuals, df_choices):
    filtered = df_counterfactuals[
        (df_counterfactuals["action"] == prev_action) &
        (df_counterfactuals["curr_lane"] == obs_state["curr_lane"]) &
        (df_counterfactuals["free_E"] == obs_state["free_E"]) &
        (df_counterfactuals["free_NE"] == obs_state["free_NE"]) &
        (df_counterfactuals["free_NW"] == obs_state["free_NW"]) &
        (df_counterfactuals["free_SE"] == obs_state["free_SE"]) &
        (df_counterfactuals["free_SW"] == obs_state["free_SW"]) &
        (df_counterfactuals["free_W"] == obs_state["free_W"])
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
        self.df_counterfactuals = pd.read_csv(cf_path)
        self.df_choices = pd.read_csv(choices_path)

        # ---- Speeds ----
        self.v_left = float(speed_left)
        self.v_right = float(speed_right)
        self.v_citroen = float(speed_citroen)

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
        self.create_subscription(Bool, "/BMW/latent_collision", lambda m: setattr(self, "latent_collision", m.data), 10)

        self.policy_period = 0.1  # 10 Hz

    # ---------------- Main loop ----------------

    def run(self):
        while rclpy.ok():
            start = time.time()
            rclpy.spin_once(self, timeout_sec=0.01)

            self.pub_started.publish(Empty())
            self.pub_speed_left.publish(Float64(data=self.v_left))
            self.pub_speed_right.publish(Float64(data=self.v_right))
            self.pub_speed_ego.publish(Float64(data=self.v_citroen))

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
                    action = get_WhatIf_action(obs, self.prev_action, self.df_counterfactuals, self.df_choices)
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
        print("Usage: ros2 run decision_making action_policy speed_left speed_right speed_citroen counterfactuals.csv choices.csv")
        return

    node = ActionPolicyNode(*sys.argv[1:])
    node.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

