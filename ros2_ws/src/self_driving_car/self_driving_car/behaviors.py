#!/usr/bin/env python3
"""
Behaviors node:
- Lane keeping
- Car following
- Lane change (FSM-based)

Execution model:
- Explicit main loop
- spin_once() for callbacks
- Fixed-rate FSM (10 Hz)
"""

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, Bool
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
SM_FOLLOW = 30
SM_TURN_LEFT_1 = 40
SM_TURN_LEFT_2 = 45
SM_TURN_RIGHT_1 = 50
SM_TURN_RIGHT_2 = 55

MAX_STEERING = 0.5


class BehaviorsNode(Node):

    def __init__(self):
        super().__init__('behaviors')

        # ---------------- Parameters ----------------
        self.max_speed = 30.0
        self.k_rho = 0.001
        self.k_theta = 0.01
        self.k_following = 10.0
        self.dist_to_car = 30.0

        # ---------------- Lane perception ----------------
        self.lane_rho_l = 0.0
        self.lane_theta_l = 0.0
        self.lane_rho_r = 0.0
        self.lane_theta_r = 0.0

        self.goal_rho_l = 370.0
        self.goal_theta_l = 2.4
        self.goal_rho_r = 430.0
        self.goal_theta_r = 0.895

        # ---------------- State ----------------
        self.state = SM_INIT
        self.enable_cruise = False
        self.enable_follow = False
        self.start_left = False
        self.start_right = False
        self.dist_to_obs = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_a = 0.0

        self.finished_published = False

        # ---------------- Subscribers ----------------
        self.create_subscription(Float64MultiArray, "/BMW/demo/left_lane",
                                 self.cb_left_lane, 10)
        self.create_subscription(Float64MultiArray, "/BMW/demo/right_lane",
                                 self.cb_right_lane, 10)
        self.create_subscription(Bool, "/BMW/cruise/enable",
                                 self.cb_cruise, 10)
        self.create_subscription(Bool, "/BMW/follow/enable",
                                 self.cb_follow, 10)
        self.create_subscription(Bool, "/BMW/change_lane_on_left/started",
                                 self.cb_change_left, qos_latched)
        self.create_subscription(Bool, "/BMW/change_lane_on_right/started",
                                 self.cb_change_right, qos_latched)
        self.create_subscription(Float64, "/BMW/obstacle_distance",
                                 self.cb_dist, 10)
        self.create_subscription(Pose2D, "/BMW/pose",
                                 self.cb_pose, 10)

        # ---------------- Publishers ----------------
        self.pub_speed = self.create_publisher(Float64, "/BMW/speed", 1)
        self.pub_steering = self.create_publisher(Float64, "/BMW/steering", 1)
        self.pub_finished = self.create_publisher(
            Bool, "/BMW/change_lane/finished", 1)

        # ---------------- Loop timing ----------------
        self.period = 0.1  # 10 Hz

        self.get_logger().info("Behaviors node initialized")

    # =====================================================
    # Callbacks
    # =====================================================

    def cb_left_lane(self, msg):
        self.lane_rho_l, self.lane_theta_l = msg.data

    def cb_right_lane(self, msg):
        self.lane_rho_r, self.lane_theta_r = msg.data

    def cb_cruise(self, msg):
        self.enable_cruise = msg.data
        if msg.data:
            self.enable_follow = False

    def cb_follow(self, msg):
        self.enable_follow = msg.data
        if msg.data:
            self.enable_cruise = False

    def cb_change_left(self, msg):
        if msg.data:
            self.start_left = True
            self.enable_cruise = False
            self.enable_follow = False

    def cb_change_right(self, msg):
        if msg.data:
            self.start_right = True
            self.enable_cruise = False
            self.enable_follow = False

    def cb_dist(self, msg):
        self.dist_to_obs = msg.data

    def cb_pose(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_a = msg.theta

    # =====================================================
    # Control helpers
    # =====================================================

    def lane_control(self, dist=None):
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
            speed = self.max_speed + self.k_following * (dist - self.dist_to_car)
            speed = max(min(speed, self.max_speed), -10.0)

        return speed, steering

    def turning_steering(self, w, L, v):
        if v == 0:
            return 0.0
        k = max(min(L * w / v, 0.5), -0.5)
        #self.get_logger().info(f"Steering calculation v:{v} k:{k}, L:{L} w:{w} math.asin(k): {math.asin(k)} MAX_STEERING:{MAX_STEERING} -MAX_STEERING:{-MAX_STEERING}")
        return max(min(math.asin(k), MAX_STEERING), -MAX_STEERING)

    # =====================================================
    # FSM
    # =====================================================

    def main_loop(self):

        speed = 0.0
        steering = 0.0

        if self.state == SM_INIT:
            self.state = SM_WAITING_FOR_NEW_TASK
            self.get_logger().info("Waiting for task")

        elif self.state == SM_WAITING_FOR_NEW_TASK:
            self.finished_published = False

            if self.enable_cruise:
                self.state = SM_CRUISE

            elif self.enable_follow:
                self.state = SM_FOLLOW

            elif self.start_left:
                self.state = SM_TURN_LEFT_1
                self.start_left = False

            elif self.start_right:
                self.state = SM_TURN_RIGHT_1
                self.start_right = False

        elif self.state == SM_CRUISE:
            speed, steering = self.lane_control()
            if not self.enable_cruise:
                self.state = SM_WAITING_FOR_NEW_TASK

        elif self.state == SM_FOLLOW:
            speed, steering = self.lane_control(self.dist_to_obs)
            if not self.enable_follow:
                self.state = SM_WAITING_FOR_NEW_TASK

        elif self.state == SM_TURN_LEFT_1:
            speed = self.max_speed
            steering = self.turning_steering(1.2, 2.9, speed)
            if self.current_y > -0.7:
                #self.get_logger().info(f"LEFT 1st condition Y: {self.current_y}")
                self.state = SM_TURN_LEFT_2

        elif self.state == SM_TURN_LEFT_2:
            speed = self.max_speed
            steering = self.turning_steering(-1.2, 2.9, speed)
            if self.current_y > 1.0 and abs(self.current_a) < 0.2:
                #self.get_logger().info(f"LEFT 2nd condition Y: {self.current_y} A: {self.current_a}") 
                self.finish_lane_change()

        elif self.state == SM_TURN_RIGHT_1:
            speed = self.max_speed
            steering = self.turning_steering(-1.2, 2.9, speed)
            if self.current_y < 0.7:
                #self.get_logger().info(f"RIGHT 1st condition Y: {self.current_y}")
                self.state = SM_TURN_RIGHT_2

        elif self.state == SM_TURN_RIGHT_2:
            speed = self.max_speed
            steering = self.turning_steering(1.2, 2.9, speed)
            if self.current_y < -1.0 and abs(self.current_a) < 0.2:
                #self.get_logger().info(f"RIGHT 2nd condition Y: {self.current_y} A: {self.current_a}")
                self.finish_lane_change()

        #self.get_logger().info(f"State: {self.state}")
        #if self.state >= SM_TURN_LEFT_1:
           #self.get_logger().info(f"State: {self.state} Speed: {speed} Steering {steering} Y: {self.current_y} A: {self.current_a}")
        self.pub_speed.publish(Float64(data=float(speed)))
        self.pub_steering.publish(Float64(data=float(steering)))

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

            elapsed = time.time() - start
            remaining = self.period - elapsed
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

