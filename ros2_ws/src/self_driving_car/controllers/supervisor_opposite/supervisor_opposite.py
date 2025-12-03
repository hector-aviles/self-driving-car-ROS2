#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import Pose2D

from controller import Supervisor

TIME_STEP = 10
robot = Supervisor()


class SupervisorNode(Node):

    def __init__(self):
        super().__init__('supervisor_opposite_node')

        self.get_logger().info("Starting Controller Supervisor...")

        # Shared variables
        self.speed_cars_left_lane = 0.0
        self.speed_cars_right_lane = 0.0
        self.start_received = False

        # Subscribers
        self.create_subscription(Float64, "/speed_cars_left_lane",
                                 self.callback_speed_cars_left_lane, 10)

        self.create_subscription(Float64, "/speed_cars_right_lane",
                                 self.callback_speed_cars_right_lane, 10)

        self.create_subscription(Empty, "/policy_started",
                                 self.callback_start, 10)

        # Publishers
        self.pub_bmw_pose = self.create_publisher(Pose2D, "/self_driving_pose", 10)
        self.pub_car_pose = [
            self.create_publisher(Pose2D, f"/car_{i+1}_pose", 10)
            for i in range(10)
        ]

        # ---- Initialize car references
        self.cars = [
            robot.getFromDef(f'vehicle_{i+1}')
            for i in range(10)
        ]

        # Apply initial random offset & reset physics
        self.tf = []
        for i, car in enumerate(self.cars):
            if car is not None:
                field = car.getField("translation")
                values = field.getSFVec3f()
                values[0] += np.random.uniform(-2, 2)
                field.setSFVec3f(values)
                car.resetPhysics()
                self.tf.append(field)

        self.bmw = robot.getFromDef('BMW_X5')

        self.get_logger().info("Supervisor -> Waiting for /policy_started signal...")

        # Timer for stepping Webots & publishing
        self.create_timer(TIME_STEP / 1000.0, self.update_step)

    # ---------------- SUBSCRIBERS ----------------

    def callback_speed_cars_left_lane(self, msg):
        self.speed_cars_left_lane = msg.data

    def callback_speed_cars_right_lane(self, msg):
        self.speed_cars_right_lane = msg.data

    def callback_start(self, msg):
        self.start_received = True
        self.get_logger().info("Supervisor -> Start signal received")

    # ---------------- TIMER LOOP ----------------

    def update_step(self):
        if not self.start_received:
            return

        if robot.step(TIME_STEP) == -1:
            return

        msg_car_pose = Pose2D()

        # Update each car
        for i, car in enumerate(self.cars):
            if car is None:
                continue

            values = self.tf[i].getSFVec3f()
            msg_car_pose.x = values[0]
            msg_car_pose.y = values[1]
            msg_car_pose.theta = values[2]

            # Speed depending on lane
            if msg_car_pose.y > 0:
                car.setVelocity([self.speed_cars_left_lane, 0, 0, 0, 0, 0])
            else:
                car.setVelocity([self.speed_cars_right_lane, 0, 0, 0, 0, 0])

            # Publish pose
            self.pub_car_pose[i].publish(msg_car_pose)

        # --- BMW Pose ---
        bmw_pose = self.bmw.getPosition()
        bmw_orient = self.bmw.getOrientation()

        msg_bmw = Pose2D()
        msg_bmw.x = bmw_pose[0]
        msg_bmw.y = bmw_pose[1]
        msg_bmw.theta = math.atan2(bmw_orient[3], bmw_orient[0])

        self.pub_bmw_pose.publish(msg_bmw)


# =====================================================================

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

