#!/usr/bin/env python3

import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import Pose2D
from controller import Supervisor

TIME_STEP = 10
MAX_VEHICLES = 5

robot = Supervisor()


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')

        # Speeds
        self.speed_vehicles_left_lane = 0.0
        self.speed_vehicles_right_lane = 0.0

        # Vehicles: same direction
        self.vehicles = [
            robot.getFromDef(f'vehicle_{i+1}')
            for i in range(MAX_VEHICLES)
        ]

        # Vehicles: opposite direction
        self.vehicles_opposite = [
            robot.getFromDef(f'vehicle_opposite_{i+1}')
            for i in range(MAX_VEHICLES)
        ]

        # Vehicles: transverse (kept, but unchanged)
        self.vehicles_transverse = [
            robot.getFromDef(f'vehicle_transverse_{i+1}')
            for i in range(MAX_VEHICLES)
        ]

        # Main vehicles
        self.bmw = robot.getFromDef('BMW_X5')
        self.citroenczero = robot.getFromDef('CitroenCZero_1')

        # Randomize initial positions
        self.initialize_vehicle_positions()

        # Publishers
        self.pub_bmw_pose = self.create_publisher(Pose2D, '/BMW/pose', 10)
        self.pub_citroenczero_pose = self.create_publisher(Pose2D, '/CitroenCZero/pose', 10)

        self.pub_vehicles_pose = [
            self.create_publisher(Pose2D, f'/vehicle_{i+1}/pose', 10)
            for i in range(MAX_VEHICLES)
        ]

        self.pub_vehicles_opposite_pose = [
            self.create_publisher(Pose2D, f'/vehicle_opposite_{i+1}/pose', 10)
            for i in range(MAX_VEHICLES)
        ]

        # Subscribers
        self.create_subscription(
            Float64,
            '/vehicles_left_lane/speed',
            self.callback_speed_vehicles_left_lane,
            10
        )

        self.create_subscription(
            Float64,
            '/vehicles_right_lane/speed',
            self.callback_speed_vehicles_right_lane,
            10
        )

        self.start_signal_received = False
        self.create_subscription(
            Empty,
            '/BMW/policy/started',
            self.callback_start_signal,
            10
        )

        self.get_logger().info('Supervisor Node Started')
        self.get_logger().info('Waiting for start signal...')

        # Main loop
        self.timer = self.create_timer(TIME_STEP / 1000.0, self.control_loop)

    # -----------------------------------------------------

    def initialize_vehicle_positions(self):
        self.get_logger().info('Initializing vehicle coordinates...')

        for i, vehicle in enumerate(self.vehicles):
            if vehicle:
                field = vehicle.getField("translation")
                pos = field.getSFVec3f()
                pos[0] += np.random.uniform(-2, 2)
                field.setSFVec3f(pos)
                vehicle.resetPhysics()

        for i, vehicle in enumerate(self.vehicles_opposite):
            if vehicle:
                field = vehicle.getField("translation")
                pos = field.getSFVec3f()
                pos[0] += np.random.uniform(-2, 2)
                field.setSFVec3f(pos)
                vehicle.resetPhysics()

        for i, vehicle in enumerate(self.vehicles_transverse):
            if vehicle:
                field = vehicle.getField("translation")
                pos = field.getSFVec3f()
                pos[1] += np.random.uniform(-2, 2)
                field.setSFVec3f(pos)
                vehicle.resetPhysics()

    # -----------------------------------------------------

    def callback_speed_vehicles_left_lane(self, msg):
        self.speed_vehicles_left_lane = msg.data
        #self.get_logger().info(f'Left lane speed = {msg.data:.2f}')

    def callback_speed_vehicles_right_lane(self, msg):
        self.speed_vehicles_right_lane = msg.data
        #self.get_logger().info(f'Right lane speed = {msg.data:.2f}')

    def callback_start_signal(self, msg):
        if not self.start_signal_received:
            self.start_signal_received = True
            self.get_logger().info(
                'Start signal received - Beginning supervision'
            )

    # -----------------------------------------------------

    def control_loop(self):
        if robot.step(TIME_STEP) == -1:
            self.get_logger().info('Simulation ended')
            return

        if not self.start_signal_received:
            return

        # Same-direction vehicles
        for i, vehicle in enumerate(self.vehicles):
            if vehicle is None:
                continue

            pos = vehicle.getPosition()
            y = pos[1]

            speed = (
                self.speed_vehicles_left_lane
                if y > 0.0
                else self.speed_vehicles_right_lane
            )

            vehicle.setVelocity([speed, 0, 0, 0, 0, 0])

            msg = Pose2D(x=pos[0], y=pos[1], theta=0.0)
            self.pub_vehicles_pose[i].publish(msg)

        # Opposite-direction vehicles
        for i, vehicle in enumerate(self.vehicles_opposite):
            if vehicle is None:
                continue

            pos = vehicle.getPosition()
            y = pos[1]

            speed = (
                self.speed_vehicles_left_lane
                if y > 0.0
                else self.speed_vehicles_right_lane
            )

            vehicle.setVelocity([-speed, 0, 0, 0, 0, 0])

            msg = Pose2D(x=pos[0], y=pos[1], theta=0.0)
            self.pub_vehicles_opposite_pose[i].publish(msg)

        self.publish_bmw_pose()
        self.publish_citroenczero_pose()

    # -----------------------------------------------------

    def publish_bmw_pose(self):
        if self.bmw:
            pos = self.bmw.getPosition()
            orient = self.bmw.getOrientation()
            theta = math.atan2(orient[1], orient[0])

            self.pub_bmw_pose.publish(
                Pose2D(x=pos[0], y=pos[1], theta=theta)
            )

    def publish_citroenczero_pose(self):
        if self.citroenczero:
            pos = self.citroenczero.getPosition()
            orient = self.citroenczero.getOrientation()
            theta = math.atan2(orient[1], orient[0])

            self.pub_citroenczero_pose.publish(
                Pose2D(x=pos[0], y=pos[1], theta=theta)
            )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SupervisorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

