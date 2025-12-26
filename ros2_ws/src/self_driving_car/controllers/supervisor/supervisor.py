#!/usr/bin/env python3

import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import Pose2D
from controller import Supervisor

robot = Supervisor()
TIME_STEP = int(robot.getBasicTimeStep())  # Webots step (ms) sync with world 
MAX_VEHICLES = 5


class SupervisorNode(Node):

    def __init__(self):
        super().__init__('supervisor')

        # ---------------- Speeds ----------------
        self.speed_vehicles_left_lane = 0.0
        self.speed_vehicles_right_lane = 0.0

        # ---------------- Vehicles ----------------
        self.vehicles = [robot.getFromDef(f'vehicle_{i+1}') for i in range(MAX_VEHICLES)]
        self.vehicles_opposite = [robot.getFromDef(f'vehicle_opposite_{i+1}') for i in range(MAX_VEHICLES)]
        self.vehicles_transverse = [robot.getFromDef(f'vehicle_transverse_{i+1}') for i in range(MAX_VEHICLES)]

        self.bmw = robot.getFromDef('BMW_X5')
        self.citroenczero = robot.getFromDef('CitroenCZero_1')

        self.initialize_vehicle_positions()

        # ---------------- Publishers ----------------
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

        # ---------------- Subscribers ----------------
        self.create_subscription(Float64, '/vehicles_left_lane/speed',
                                 self.callback_speed_vehicles_left_lane, 10)

        self.create_subscription(Float64, '/vehicles_right_lane/speed',
                                 self.callback_speed_vehicles_right_lane, 10)

        self.start_signal_received = False
        self.create_subscription(Empty, '/BMW/policy/started',
                                 self.callback_start_signal, 10)

        # ---------------- Timing ----------------
        t0 = robot.getTime()

        self.last_motion = t0
        self.last_vehicle_pub = t0
        self.last_bmw_pub = t0
        self.last_citroen_pub = t0

        self.motion_interval = 0.05        # 20 Hz
        self.vehicle_pub_interval = 0.10   # 10 Hz
        self.bmw_pub_interval = 0.05       # 20 Hz
        self.citroen_pub_interval = 0.05   # 20 Hz

        self.get_logger().info('Supervisor Node started')
        self.get_logger().info('Waiting for start signal...')

    # -----------------------------------------------------

    def initialize_vehicle_positions(self):
        self.get_logger().info('Initializing vehicle coordinates...')

        for group, axis in [
            (self.vehicles, 0),
            (self.vehicles_opposite, 0),
            (self.vehicles_transverse, 1)
        ]:
            for vehicle in group:
                if vehicle:
                    field = vehicle.getField("translation")
                    pos = field.getSFVec3f()
                    pos[axis] += np.random.uniform(-2, 2)
                    field.setSFVec3f(pos)
                    vehicle.resetPhysics()

    # -----------------------------------------------------

    def callback_speed_vehicles_left_lane(self, msg):
        self.speed_vehicles_left_lane = msg.data

    def callback_speed_vehicles_right_lane(self, msg):
        self.speed_vehicles_right_lane = msg.data

    def callback_start_signal(self, _msg):
        if not self.start_signal_received:
            self.start_signal_received = True
            self.get_logger().info('Start signal received – supervision enabled')

    # -----------------------------------------------------

    def step(self):

        if not self.start_signal_received:
            return

        sim_t = robot.getTime()

        # -------- Vehicle motion (20 Hz) --------
        if sim_t - self.last_motion >= self.motion_interval:
            self.last_motion = sim_t

            for vehicle in self.vehicles:
                if vehicle:
                    y = vehicle.getPosition()[1]
                    speed = self.speed_vehicles_left_lane if y > 0.0 else self.speed_vehicles_right_lane
                    vehicle.setVelocity([speed, 0, 0, 0, 0, 0])

            for vehicle in self.vehicles_opposite:
                if vehicle:
                    y = vehicle.getPosition()[1]
                    speed = self.speed_vehicles_left_lane if y > 0.0 else self.speed_vehicles_right_lane
                    vehicle.setVelocity([-speed, 0, 0, 0, 0, 0])

        # -------- Vehicle pose publishing (10 Hz) --------
        if sim_t - self.last_vehicle_pub >= self.vehicle_pub_interval:
            self.last_vehicle_pub = sim_t

            for i, vehicle in enumerate(self.vehicles):
                if vehicle:
                    pos = vehicle.getPosition()
                    self.pub_vehicles_pose[i].publish(
                        Pose2D(x=pos[0], y=pos[1], theta=0.0)
                    )

            for i, vehicle in enumerate(self.vehicles_opposite):
                if vehicle:
                    pos = vehicle.getPosition()
                    self.pub_vehicles_opposite_pose[i].publish(
                        Pose2D(x=pos[0], y=pos[1], theta=0.0)
                    )

        # -------- BMW pose (20 Hz) --------
        if sim_t - self.last_bmw_pub >= self.bmw_pub_interval:
            self.last_bmw_pub = sim_t
            self.publish_bmw_pose()

        # -------- Citroen pose (20 Hz) --------
        if sim_t - self.last_citroen_pub >= self.citroen_pub_interval:
            self.last_citroen_pub = sim_t
            self.publish_citroenczero_pose()

    # -----------------------------------------------------

    def publish_bmw_pose(self):
        if not self.bmw:
            return

        pos = self.bmw.getPosition()
        orient = self.bmw.getOrientation()
        theta = math.atan2(orient[3], orient[0])

        self.pub_bmw_pose.publish(Pose2D(x=pos[0], y=pos[1], theta=theta))

    def publish_citroenczero_pose(self):
        if not self.citroenczero:
            return

        pos = self.citroenczero.getPosition()
        orient = self.citroenczero.getOrientation()
        theta = math.atan2(orient[3], orient[0])

        self.pub_citroenczero_pose.publish(Pose2D(x=pos[0], y=pos[1], theta=theta))


# -----------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()

    try:
        while robot.step(TIME_STEP) != -1:
            node.step()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

