#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Empty, Float64
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

import os
import time
import sys
import pandas as pd
import numpy as np

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

# ---------------- Policy ----------------
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

    def predict(self, obs: dict) -> str:
        cols = [
            "curr_lane", "free_E", "free_NE", "free_NW",
            "free_SE", "free_SW", "free_W"
        ]
        data = np.array([int(obs[c]) for c in cols])
        idx = data @ (2 ** np.arange(7))
        return self.actions[idx]

# ---------------- Counterfactual logic ----------------
def get_WhatIf_action(obs, prev_action, df_counterfactuals, df_choices):
    filtered = df_counterfactuals[
        (df_counterfactuals["action"] == prev_action) &
        (df_counterfactuals["curr_lane"] == obs["curr_lane"]) &
        (df_counterfactuals["free_E"] == obs["free_E"]) &
        (df_counterfactuals["free_NE"] == obs["free_NE"]) &
        (df_counterfactuals["free_NW"] == obs["free_NW"]) &
        (df_counterfactuals["free_SE"] == obs["free_SE"]) &
        (df_counterfactuals["free_SW"] == obs["free_SW"]) &
        (df_counterfactuals["free_W"] == obs["free_W"])
    ]

    if filtered.empty:
        print("ERROR: no matching rows found in counterfactuals. Default NA", flush=True)
        return "NA"

    alternatives = {
        "swerve_left": 0,
        "swerve_right": 0,
        "cruise": 0,
        "keep": 0,
        "change_to_left": 0,
        "change_to_right": 0
    }

    for ia in filtered["iaction"]:
        if ia in alternatives:
            alternatives[ia] = 1

    row = df_choices[
        (df_choices["swerve_left"]     == alternatives["swerve_left"]) &
        (df_choices["swerve_right"]    == alternatives["swerve_right"]) &
        (df_choices["cruise"]          == alternatives["cruise"]) &
        (df_choices["keep"]            == alternatives["keep"]) &
        (df_choices["change_to_left"]  == alternatives["change_to_left"]) &
        (df_choices["change_to_right"] == alternatives["change_to_right"])
    ]

    if row.empty:
        print("ERROR: no matching rows found in choices. publishing NA", flush=True)
        return "NA"

    if obs["curr_lane"]:
        return row["right_lane"].values[0]
    else:
        return row["left_lane"].values[0]

# ---------------- ROS2 Node ----------------
class CounterfactualsNode(Node):
    def __init__(self, v_left, v_right, v_citroen,
                 counterfactuals_file, choices_file):
        super().__init__("counterfactuals_node")

        # ---- State ----
        self.curr_lane = True
        self.free_N = self.free_E = self.free_NE = self.free_NW = True
        self.free_SE = self.free_SW = self.free_W = True
        self.latent_collision = False

        self.change_lane_finished = False
        self.dfa_state = DFA_INIT

        # ---- Policy ----
        self.model = ActionPolicy()
        pkg_share = get_package_share_directory("decision_making")
        base_path = os.path.join(pkg_share, "counterfactuals_model")
        self.df_counterfactuals = pd.read_csv(os.path.join(base_path, counterfactuals_file))
        self.df_choices = pd.read_csv(os.path.join(base_path, choices_file))

        # ---- Speeds ----
        self.v_left = float(v_left)
        self.v_right = float(v_right)
        self.v_citroen = float(v_citroen)

        # ---- Timeout ----
        self.action_wait_timeout = 15.0

        # ---- Publishers ----
        self.pub_action = self.create_publisher(String, "/BMW/policy/action", qos_latched)
        self.pub_started = self.create_publisher(Empty, "/BMW/policy/started", qos_latched)
        self.pub_speed_left = self.create_publisher(Float64, "/vehicles_left_lane/speed", qos_latched)
        self.pub_speed_right = self.create_publisher(Float64, "/vehicles_right_lane/speed", qos_latched)
        self.pub_speed_citroen = self.create_publisher(Float64, "/CitroenCZero/speed", qos_latched)
        self.pub_counterfactuals = self.create_publisher(Bool, "/BMW/counterfactuals", qos_latched)

        # ---- Subscribers ----
        self.create_subscription(Bool, "/BMW/free_N", self.cb_free_N, 10)
        self.create_subscription(Bool, "/BMW/free_NW", self.cb_free_NW, 10)
        self.create_subscription(Bool, "/BMW/free_W", self.cb_free_W, 10)
        self.create_subscription(Bool, "/BMW/free_SW", self.cb_free_SW, 10)
        self.create_subscription(Bool, "/BMW/free_NE", self.cb_free_NE, 10)
        self.create_subscription(Bool, "/BMW/free_E", self.cb_free_E, 10)
        self.create_subscription(Bool, "/BMW/free_SE", self.cb_free_SE, 10)
        self.create_subscription(Bool, "/BMW/current_lane", self.cb_curr_lane, 10)
        self.create_subscription(Bool, "/BMW/change_lane/finished", self.cb_change_lane_finished, 10)
        self.create_subscription(Bool, "/BMW/latent_collision", self.cb_latent_collision, 10)

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
    def cb_latent_collision(self, msg): self.latent_collision = msg.data

    # ---------------- run ----------------
    def run(self):
        period = 0.1

        action = "cruise"
        prev_action = "NA"
        prev_obs = None
        prev_collision = None

        self.get_logger().info(f"Publishing initial action={action}")
        self.pub_action.publish(String(data=action))

        self.pub_counterfactuals.publish(Bool(data=True))

        while rclpy.ok():
            start = time.time()
            rclpy.spin_once(self, timeout_sec=0.01)

            self.pub_started.publish(Empty())
            self.pub_speed_left.publish(Float64(data=self.v_left))
            self.pub_speed_right.publish(Float64(data=self.v_right))
            self.pub_speed_citroen.publish(Float64(data=self.v_citroen))

            self._sleep(period, start)

    def _sleep(self, period, start):
        remaining = period - (time.time() - start)
        if remaining > 0:
            time.sleep(remaining)

# ---------------- main ----------------
def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 6:
        print(
            "Usage:\n"
            "ros2 run decision_making counterfactuals "
            "<speed_left> <speed_right> <speed_citroen> "
            "<counterfactuals.csv> <choices.csv>"
        )
        sys.exit(1)

    node = CounterfactualsNode(
        sys.argv[1],
        sys.argv[2],
        sys.argv[3],
        sys.argv[4],
        sys.argv[5]
    )

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

