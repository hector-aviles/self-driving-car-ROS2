#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, Bool, String
from geometry_msgs.msg import Pose2D
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# ---------------- QoS ----------------
qos_latched = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE
)

# ---------------- FSM States ----------------
SM_INIT = 0
SM_WAITING_FOR_NEW_TASK = 10
SM_CRUISE = 20
SM_KEEP = 30
SM_CHANGE_LEFT_1 = 40
SM_CHANGE_LEFT_2 = 45
SM_CHANGE_RIGHT_1 = 50
SM_CHANGE_RIGHT_2 = 55
SM_SWERVE_LEFT = 170
SM_SWERVE_RIGHT = 190
SM_UNDOING_TURN = 210

MAX_STEERING = 0.5


class BehaviorsNode(Node):

    def __init__(self):
        super().__init__('behaviors')

        # ---------------- Parameters ----------------
        self.max_speed = 30.0
        self.k_rho = 0.001
        self.k_theta = 0.01
        self.k_keeping = 10.0
        self.dist_to_car = 30.0

        # ---------------- Lane perception ----------------
        self.lane_rho_l = 0.0
        self.lane_theta_l = 0.0
        self.lane_rho_r = 0.0
        self.lane_theta_r = 0.0

        self.goal_rho_l   = 481.0
        self.goal_theta_l = 2.085
        self.goal_rho_r   = 466.0
        self.goal_theta_r = 0.99

        # Nominal lane parameters
        self.nominal_params = dict(
            k_rho=0.001,
            k_theta=0.01,
            max_speed=30.0,
            goal_rho_l=481.0,
            goal_theta_l=2.085,
            goal_rho_r=466.0,
            goal_theta_r=0.99
        )

        # ---------------- State ----------------
        self.state = SM_INIT
        self.action = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_a = 0.0
        self.latent_collision = False

        # Counterfactuals flag
        self.counterfactuals = False

        self.finished_published = False

        # ---------------- Subscribers ----------------
        self.create_subscription(String, "/BMW/policy/action",
                                 self.cb_action, qos_latched)
        self.create_subscription(Float64MultiArray, "/BMW/demo/left_lane",
                                 self.cb_left_lane, 10)
        self.create_subscription(Float64MultiArray, "/BMW/demo/right_lane",
                                 self.cb_right_lane, 10)
        self.create_subscription(Float64, "/BMW/obstacle_distance",
                                 self.cb_dist, 10)
        self.create_subscription(Pose2D, "/BMW/pose",
                                 self.cb_pose, 10)
        self.create_subscription(Bool, "/BMW/latent_collision",
                                 self.cb_latent_collision, 10)
        self.create_subscription(Bool, "/BMW/counterfactuals", 
                                 self.cb_counterfactuals, qos_latched)

        # ---------------- Publishers ----------------
        self.pub_speed = self.create_publisher(Float64, "/BMW/speed", 1)
        self.pub_steering = self.create_publisher(Float64, "/BMW/steering", 1)
        self.pub_finished = self.create_publisher(
            Bool, "/BMW/change_lane/finished", 1)

        # ---------------- Loop timing ----------------
        self.period = 0.1  # 10 Hz

        self.get_logger().info("Behaviors node initialized")

    # =====================================================
    # Parameter helpers
    # =====================================================

    def set_nominal_params(self):
        self.k_rho = self.nominal_params["k_rho"]
        self.k_theta = self.nominal_params["k_theta"]
        self.max_speed = self.nominal_params["max_speed"]
        self.goal_rho_l = self.nominal_params["goal_rho_l"]
        self.goal_theta_l = self.nominal_params["goal_theta_l"]
        self.goal_rho_r = self.nominal_params["goal_rho_r"]
        self.goal_theta_r = self.nominal_params["goal_theta_r"]

    # =====================================================
    # Callbacks
    # =====================================================

    def cb_action(self, msg):
        self.action = msg.data

    def cb_left_lane(self, msg):
        self.lane_rho_l, self.lane_theta_l = msg.data

    def cb_right_lane(self, msg):
        self.lane_rho_r, self.lane_theta_r = msg.data

    def cb_dist(self, msg):
        self.dist_to_car = msg.data

    def cb_pose(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_a = msg.theta

    def cb_latent_collision(self, msg):
        self.latent_collision = msg.data

    def cb_counterfactuals(self, msg):
        self.counterfactuals = msg.data
        self.get_logger().info("Topic /BMW/counterfactuals = True")

    # =====================================================
    # Control helpers
    # =====================================================

    def calculate_control(self, dist=None):
        er_l = self.goal_rho_l - self.lane_rho_l
        et_l = self.goal_theta_l - self.lane_theta_l
        er_r = self.lane_rho_r - self.goal_rho_r
        et_r = self.lane_theta_r - self.goal_theta_r

        er = (er_l + er_r) / 2.0
        et = (et_l + et_r) / 2.0

        steering = -self.k_rho * er - self.k_theta * et

        if dist is None:
            speed = self.max_speed * (1.0 - 1.5 * abs(steering))
        else:
            speed = self.max_speed + self.k_keeping * (dist - self.dist_to_car)
            speed = max(min(speed, self.max_speed), -10.0)

        return speed, steering

    def turning_steering(self, w, L, v):
        if v == 0:
            return 0.0
        k = max(min(L * w / v, 0.5), -0.5)
        return max(min(math.asin(k), MAX_STEERING), -MAX_STEERING)

    # =====================================================
    # FSM
    # =====================================================

    def main_loop(self):

        self.speed = 0.0
        self.steering = 0.0

        if self.state == SM_INIT:
            self.state = SM_WAITING_FOR_NEW_TASK

        elif self.state == SM_WAITING_FOR_NEW_TASK:
            self.finished_published = False

            if self.action == "cruise":
                self.state = SM_CRUISE
                self.action = None
            elif self.action == "keep":
                self.state = SM_KEEP
                self.action = None
            elif self.action == "change_to_left":
                self.state = SM_CHANGE_LEFT_1
                self.action = None
            elif self.action == "change_to_right":
                self.state = SM_CHANGE_RIGHT_1
                self.action = None

        elif self.state == SM_CRUISE:
            self.speed, self.steering = self.calculate_control()
            if self.action and self.action != "cruise":
                self.state = SM_WAITING_FOR_NEW_TASK

        elif self.state == SM_KEEP:
            self.speed, self.steering = self.calculate_control(self.dist_to_car)
            if self.action and self.action != "keep":
                self.state = SM_WAITING_FOR_NEW_TASK

        elif self.state == SM_CHANGE_LEFT_1:
            self.speed = self.max_speed
            self.steering = self.turning_steering(1.2, 2.9, self.speed)
            if self.current_y > -0.7:
                self.state = SM_CHANGE_LEFT_2

        elif self.state == SM_CHANGE_LEFT_2:
            self.speed = self.max_speed
            self.steering = self.turning_steering(-1.2, 2.9, self.speed)
            if self.current_y > 1.0 and abs(self.current_a) < 0.2:
                self.finish_lane_change()

        elif self.state == SM_CHANGE_RIGHT_1:
            self.speed = self.max_speed
            self.steering = self.turning_steering(-1.2, 2.9, self.speed)
            if self.current_y < 0.7:
                self.state = SM_CHANGE_RIGHT_2

        elif self.state == SM_CHANGE_RIGHT_2:
            self.speed = self.max_speed
            self.steering = self.turning_steering(1.2, 2.9, self.speed)
            if self.current_y < -1.0 and abs(self.current_a) < 0.2:
                self.finish_lane_change()

        undoable_states = [SM_CHANGE_LEFT_1, SM_CHANGE_RIGHT_1]
        if (
            self.counterfactuals
            and self.state in undoable_states
            and self.latent_collision
        ):
            self.state = SM_UNDOING_TURN
            self.get_logger().info(
                f"Aborting changing lane (counterfactuals enabled)"
            )
            self.set_nominal_params()

        self.pub_speed.publish(Float64(data=float(self.speed)))
        self.pub_steering.publish(Float64(data=float(self.steering)))

    def finish_lane_change(self):
        if not self.finished_published:
            self.pub_finished.publish(Bool(data=True))
            self.finished_published = True
        self.state = SM_WAITING_FOR_NEW_TASK

    # =====================================================
    # Explicit loop
    # =====================================================

    def run(self):
        while rclpy.ok():
            start = time.time()
            rclpy.spin_once(self, timeout_sec=0.01)
            self.main_loop()
            remaining = self.period - (time.time() - start)
            if remaining > 0:
                time.sleep(remaining)


# =====================================================
# Main
# =====================================================

def main():
    rclpy.init()
    node = BehaviorsNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

