#!/usr/bin/env python3
"""
Behaviors node:
- Lane keeping
- Car keeping
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

MAX_STEERING = 0.5

#
# Steering is calculated proportional to two errors: distance error and angle error.
# These errors correspond to differences between an observed line (in normal form)
# and a desired line.
# Speed is calculated as the max speed minus a speed proportional to the steering.
# In this way, car goes with lower speed in curves and at max speed in straight roads. 
#
class BehaviorsNode(Node):

    def __init__(self):
        super().__init__('behaviors')

        # ---------------- Parameters ----------------
        self.max_speed = 30.0
        #self.k_rho = 0.001
        #self.k_theta = 0.01
        self.k_rho = 0.002
        self.k_theta = 0.02        
        self.k_keeping = 10.0
        self.dist_to_car = 30.0

        # ---------------- Lane perception ----------------
        self.lane_rho_l = 0.0
        self.lane_theta_l = 0.0
        self.lane_rho_r = 0.0
        self.lane_theta_r = 0.0

        #self.goal_rho_l = 370.0
        #self.goal_theta_l = 2.4
        #self.goal_rho_r = 430.0
        #self.goal_theta_r = 0.895
        
        self.goal_rho_l   = 481.0
        self.goal_theta_l = 2.085
        self.goal_rho_r   = 466.0
        self.goal_theta_r = 0.99

        # Nominal lane parameters
        self.nominal_params = dict(
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

        self.finished_published = False

        # ---------------- Subscribers ----------------
        
        self.create_subscription(String, "/BMW/action", 
                                 self.cb_action, 10)
        self.create_subscription(Float64MultiArray, "/BMW/demo/left_lane",
                                 self.cb_left_lane, 10)
        self.create_subscription(Float64MultiArray, "/BMW/demo/right_lane",
                                 self.cb_right_lane, 10)  
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
        
    def set_nominal_params(self):
       self.max_speed     = self.nominal_params["max_speed"]
       self.goal_rho_l    = self.nominal_params["goal_rho_l"]
       self.goal_theta_l = self.nominal_params["goal_theta_l"]
       self.goal_rho_r    = self.nominal_params["goal_rho_r"]
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
                
            elif self.action == "swerve_left":
                self.state = SM_SWERVE_LEFT
                self.max_speed = 30
                self.goal_rho_l   = 385.0
                self.goal_theta_l = 2.37
                self.goal_rho_r   = 508.0
                self.goal_theta_r = 1.16
                self.action = None
                
            elif self.action == "swerve_right":
                self.state = SM_SWERVE_RIGHT
                self.max_speed = 30
                self.goal_rho_l   = 517.0
                self.goal_theta_l = 1.96
                self.goal_rho_r   = 330.0
                self.goal_theta_r = 0.86
                self.action = None

        elif self.state == SM_CRUISE:
            speed, steering = self.calculate_control()
            if self.action != "cruise":
                self.state = SM_WAITING_FOR_NEW_TASK

        elif self.state == SM_KEEP:
            speed, steering = self.calculate_control(self.dist_to_car)
            if self.action != "keep":
                self.state = SM_WAITING_FOR_NEW_TASK

        elif self.state == SM_CHANGE_LEFT_1:
            speed = self.max_speed
            steering = self.turning_steering(1.2, 2.9, speed)
            if self.current_y > -0.7:
                #self.get_logger().info(f"LEFT 1st condition Y: {self.current_y}")
                self.state = SM_CHANGE_LEFT_2

        elif self.state == SM_CHANGE_LEFT_2:
            speed = self.max_speed
            steering = self.turning_steering(-1.2, 2.9, speed)
            if self.current_y > 1.0 and abs(self.current_a) < 0.2:
                #self.get_logger().info(f"LEFT 2nd condition Y: {self.current_y} A: {self.current_a}") 
                self.finish_lane_change()

        elif self.state == SM_CHANGE_RIGHT_1:
            speed = self.max_speed
            steering = self.turning_steering(-1.2, 2.9, speed)
            if self.current_y < 0.7:
                #self.get_logger().info(f"RIGHT 1st condition Y: {self.current_y}")
                self.state = SM_CHANGE_RIGHT_2

        elif self.state == SM_CHANGE_RIGHT_2:
            speed = self.max_speed
            steering = self.turning_steering(1.2, 2.9, speed)
            if self.current_y < -1.0 and abs(self.current_a) < 0.2:
                #self.get_logger().info(f"RIGHT 2nd condition Y: {self.current_y} A: {self.current_a}")
                self.finish_lane_change()
                
        #
        # STATE FOR SWERVING TO LEFT
        #
        elif self.state == SM_SWERVE_LEFT:
            self.get_logger().info("Swerving left")
            speed, steering = self.calculate_control()
            self.dist_to_car = None            
            if self.action != "swerve_left":
                self.set_nominal_params()
                self.state = SM_WAITING_FOR_NEW_TASK

        #
        # STATE FOR SWERVE RIGHT
        #
        elif self.state == SM_SWERVE_RIGHT:
            self.get_logger().info("Swerving right")
            speed, steering = self.calculate_control()
            self.dist_to_car = None
            if self.action != "swerve_right":
                self.set_nominal_params()
                self.state = SM_WAITING_FOR_NEW_TASK
                              
        else:
            self.get_logger().error(f"invalid STATE {self.state}")
            self.state = SM_WAITING_FOR_NEW_TASK

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

