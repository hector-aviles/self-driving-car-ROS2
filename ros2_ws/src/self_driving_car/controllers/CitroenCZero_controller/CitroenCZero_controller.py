#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Bool, Float64, Empty
from geometry_msgs.msg import Pose2D

from vehicle import Driver
from controller import Radar

SM_INIT = 0
SM_TURN_RIGHT_1 = 10
SM_TURN_RIGHT_2 = 20
SM_CRUISE = 30
MAX_STEERING = 0.5


def calculate_turning_steering(w, L, v):
    if v == 0:
        return 0
    k = L * w / v
    k = max(min(k, 0.5), -0.5)

    steering = math.asin(k)
    steering = max(min(steering, MAX_STEERING), -MAX_STEERING)

    return steering


class CitroenCZero_Controller(Node):

    def __init__(self):
        super().__init__('CitroenCZero')

        # Webots driver
        self.driver = Driver()
        self.TIME_STEP = int(self.driver.getBasicTimeStep())

        # Internal state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_a = 0.0
        self.speed = 10.0
        self.max_speed = 30.0
        self.state = SM_INIT
        self.steering = 0.0
        self.free_N = True

        # Radar device
        self.frontal_radar = Radar("Sms UMRR 0a29")
        self.frontal_radar.enable(self.TIME_STEP)

        self.valid_dist = 25
        self.fov = math.atan2(3, self.valid_dist)

        # Subscribers
        self.create_subscription(
            Float64,
            "/CitroenCZero/speed",
            self.callback_speed,
            10
        )
        self.create_subscription(  # <--- ¿?
            Pose2D,
            "/CitroenCZero/pose",
            self.callback_pose,
            10
        )

        # Publisher
        self.pub_radar = self.create_publisher(
            Bool,
            "/CitroenCZero/free_N",
            10
        )

        self.get_logger().info("supervisor_CitroenCZero waiting for /policy_started...")

        # Wait for start signal through temporary subscription
        self.start_signal_received = False
        self.create_subscription(
            Empty,
            "/policy_started",
            self.callback_start_signal,
            10
        )

        # Spin a timer slowly until start signal
        self.wait_timer = self.create_timer(0.5, self.wait_for_start)

    # -------------------- ROS2 Callbacks --------------------

    def callback_speed(self, msg):
        self.speed = msg.data

    def callback_pose(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_a = msg.theta

    def callback_start_signal(self, msg):
        self.start_signal_received = True

    # -------------------- Start waiting --------------------

    def wait_for_start(self):
        if self.start_signal_received:
            self.get_logger().info("Start signal received. Beginning control loop.")
            self.wait_timer.cancel()
            # Main loop at TIME_STEP ms = Hz
            self.main_timer = self.create_timer(
                self.TIME_STEP / 1000.0, self.control_loop
            )

    # -------------------- Main control loop --------------------

    def control_loop(self):
        # Step Webots driver
        if self.driver.step() == -1:
            return

        self.driver.setCruisingSpeed(self.speed)
        self.driver.setSteeringAngle(self.steering)

        radar_returns = self.frontal_radar.getTargets()
        radar_returns = [r for r in radar_returns if r.distance != 1.0]

        # --- FSM ---
        for r in radar_returns:

            if self.state == SM_INIT:
                if -self.fov <= r.azimuth <= self.fov and r.distance <= self.valid_dist:
                    self.free_N = False
                    self.get_logger().info("START CitroenCZero swerving")
                    self.state = SM_TURN_RIGHT_1

            elif self.state == SM_TURN_RIGHT_1:
                if self.speed <= 10:
                    self.speed = self.max_speed
                self.steering = calculate_turning_steering(-1.2, 2.9, self.speed)
                if self.current_y < 2.5:
                    self.state = SM_TURN_RIGHT_2

            elif self.state == SM_TURN_RIGHT_2:
                if self.speed <= 10:
                    self.speed = self.max_speed
                self.steering = calculate_turning_steering(1.2, 2.9, self.speed)

                if self.current_y > 2.5 or abs(self.current_a) < 0.05:
                    self.get_logger().info("CitroenCZero swerving FINISHED")
                    self.state = SM_CRUISE

            elif self.state == SM_CRUISE:
                self.steering = 0.0

        # Publish radar free flag
        self.pub_radar.publish(Bool(data=self.free_N))


# ------------------------ MAIN ------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CitroenCZero_Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

