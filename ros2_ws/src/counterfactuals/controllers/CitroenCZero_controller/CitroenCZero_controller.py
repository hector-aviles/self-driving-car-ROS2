#!/usr/bin/env python3

import math
import rclpy
import random
from rclpy.node import Node

from std_msgs.msg import Bool, Float64, Empty
from geometry_msgs.msg import Pose2D

from vehicle import Driver
from controller import Radar


# -------------------- FSM States --------------------

SM_INIT = 0
SM_TURN_RIGHT_1 = 10
SM_TURN_RIGHT_2 = 20
SM_TURN_LEFT_1 = 30
SM_TURN_LEFT_2 = 40
SM_CRUISE = 50

MAX_STEERING = 0.5


# -------------------- Helpers --------------------

def calculate_turning_steering(w, L, v):
    if v <= 0.1:
        return 0.0

    k = L * w / v
    k = max(min(k, 0.5), -0.5)

    steering = math.asin(k)
    return max(min(steering, MAX_STEERING), -MAX_STEERING)


# -------------------- Controller --------------------

class CitroenCZero_Controller(Node):

    def __init__(self):
        super().__init__('CitroenCZero')

        # --- Webots driver ---
        self.driver = Driver()
        self.TIME_STEP = int(self.driver.getBasicTimeStep())

        # --- Vehicle state ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_a = 0.0
        self.speed = 0.0
        self.max_speed = 30.0
        self.state = SM_INIT
        self.steering = 0.0
        self.free_N = True
        self.turn_direction = None

        random.seed()

        # --- Radar ---
        self.frontal_radar = Radar("Sms UMRR 0a29")
        self.frontal_radar.enable(self.TIME_STEP)

        self.valid_dist = 25.0
        self.fov = math.atan2(3.0, self.valid_dist)

        # --- ROS interfaces ---
        self.create_subscription(Float64, "/CitroenCZero/speed", self.callback_speed, 10)
        self.create_subscription(Pose2D, "/CitroenCZero/pose", self.callback_pose, 10)
        self.create_subscription(Empty, "/BMW/policy/started", self.callback_start_signal, 10)

        self.pub_radar = self.create_publisher(Bool, "/CitroenCZero/free_N", 10)

        self.start_signal_received = False

        # --- Timing (Hz → seconds) ---
        t0 = self.driver.getTime()

        self.last_radar = t0
        self.last_fsm = t0
        self.last_actuation = t0
        self.last_pub = t0

        self.radar_interval = 0.05     # 20 Hz
        self.fsm_interval = 0.05       # 20 Hz
        self.actuation_interval = 0.05 # 20 Hz
        self.pub_interval = 0.10       # 10 Hz

        self.get_logger().info(
            f"CitroenCZero controller initialized | basicTimeStep={self.TIME_STEP} ms"
        )

    # -------------------- ROS Callbacks --------------------

    def callback_speed(self, msg):
        self.speed = msg.data

    def callback_pose(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_a = msg.theta

    def callback_start_signal(self, _msg):
        self.start_signal_received = True
        #self.get_logger().info("Start signal received.")

    # -------------------- FSM Logic --------------------

    def process_fsm(self, radar_returns):

        for r in radar_returns:

            if self.state == SM_INIT:
                if -self.fov <= r.azimuth <= self.fov and r.distance <= self.valid_dist:
                    self.free_N = False
                    self.turn_direction = random.choice(['right', 'left'])

                    if self.turn_direction == 'right':
                        self.get_logger().info("START CitroenCZero swerving RIGHT")
                        self.state = SM_TURN_RIGHT_1
                    else:
                        self.get_logger().info("START CitroenCZero swerving LEFT")
                        self.state = SM_TURN_LEFT_1

            elif self.state == SM_TURN_RIGHT_1:
                self.speed = max(self.speed, self.max_speed)
                self.steering = calculate_turning_steering(-1.2, 2.9, self.speed)
                if self.current_y < 2.5:
                    self.state = SM_TURN_RIGHT_2

            elif self.state == SM_TURN_RIGHT_2:
                self.speed = max(self.speed, self.max_speed)
                self.steering = calculate_turning_steering(1.2, 2.9, self.speed)
                if self.current_y > 2.5 or abs(self.current_a) < 0.05:
                    self.get_logger().info("CitroenCZero RIGHT swerving FINISHED")
                    self.state = SM_CRUISE

            elif self.state == SM_TURN_LEFT_1:
                self.speed = max(self.speed, self.max_speed)
                self.steering = calculate_turning_steering(1.2, 2.9, self.speed)
                if self.current_y > -2.5:
                    self.state = SM_TURN_LEFT_2

            elif self.state == SM_TURN_LEFT_2:
                self.speed = max(self.speed, self.max_speed)
                self.steering = calculate_turning_steering(-1.2, 2.9, self.speed)
                if self.current_y < -2.5 or abs(self.current_a) < 0.05:
                    self.get_logger().info("CitroenCZero LEFT swerving FINISHED")
                    self.state = SM_CRUISE

            elif self.state == SM_CRUISE:
                self.steering = 0.0
                self.free_N = True

    # -------------------- Main Webots Loop --------------------

    def main_loop(self):

        radar_returns = []

        while self.driver.step() != -1:

            rclpy.spin_once(self, timeout_sec=0.0)

            if not self.start_signal_received:
                continue

            sim_t = self.driver.getTime()

            # --- Radar (20 Hz) ---
            if sim_t - self.last_radar >= self.radar_interval:
                self.last_radar = sim_t
                radar_returns = [
                    r for r in self.frontal_radar.getTargets()
                    if r.distance != 1.0
                ]

            # --- FSM (20 Hz) ---
            if sim_t - self.last_fsm >= self.fsm_interval:
                self.last_fsm = sim_t
                self.process_fsm(radar_returns)

            # --- Actuation (20 Hz) ---
            if sim_t - self.last_actuation >= self.actuation_interval:
                self.last_actuation = sim_t
                self.driver.setCruisingSpeed(self.speed)
                self.driver.setSteeringAngle(self.steering)

            # --- Publish free_N (10 Hz) ---
            if sim_t - self.last_pub >= self.pub_interval:
                self.last_pub = sim_t
                self.pub_radar.publish(Bool(data=self.free_N))


# -------------------- MAIN --------------------

def main(args=None):
    rclpy.init(args=args)
    node = CitroenCZero_Controller()
    node.main_loop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

