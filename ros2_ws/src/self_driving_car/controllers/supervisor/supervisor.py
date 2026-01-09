#!/usr/bin/env python3

import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import Pose2D
from controller import Supervisor

# -----------------------------------------------------
# Webots supervisor
# -----------------------------------------------------

robot = Supervisor()
TIME_STEP = int(robot.getBasicTimeStep())  # Webots step (ms)
MAX_VEHICLES = 5


class SupervisorNode(Node):

    def __init__(self):
        super().__init__('supervisor')

        # ---------------- Speeds ----------------
        self.speed_vehicles_left_lane = 0.0
        self.speed_vehicles_right_lane = 0.0

        # ---------------- Vehicles ----------------
        self.vehicles = [robot.getFromDef(f'vehicle_{i+1}') for i in range(MAX_VEHICLES)]

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

        # ---------------- Subscribers ----------------
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
        self.get_logger().info('Publishing poses immediately')
        self.get_logger().info('Vehicle motion will start after policy signal')

    # -----------------------------------------------------

    def initialize_vehicle_positions(self):
        self.get_logger().info('Initializing vehicle coordinates...')
    
        for vehicle in self.vehicles:
            if not vehicle:
                continue
    
            field = vehicle.getField("translation")
            pos = field.getSFVec3f()
    
            orient = vehicle.getOrientation()
            yaw = math.atan2(orient[3], orient[0])
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # normalize to (-pi, pi]
    
            # Decide longitudinal axis based on heading
            # North / South -> move along x
            if abs(yaw) < 0.2 or abs(abs(yaw) - math.pi) < 0.2:
                pos[0] += np.random.uniform(-2.0, 2.0)  # x-axis
    
            # East / West -> move along y
            elif abs(abs(yaw) - math.pi / 2) < 0.2:
                pos[1] += np.random.uniform(-2.0, 2.0)  # y-axis
    
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
            self.get_logger().info('Policy started – vehicle motion enabled')

    # -----------------------------------------------------

    def step(self):
        sim_t = robot.getTime()

        # -------- Vehicle motion (gated) --------
        if self.start_signal_received and sim_t - self.last_motion >= self.motion_interval:
            self.last_motion = sim_t

            for vehicle in self.vehicles:
                if vehicle:
                    x = vehicle.getPosition()[0]
                    y = vehicle.getPosition()[1]
                    orient = vehicle.getOrientation()
                    yaw = self.normalize_angle(math.atan2(orient[3], orient[0]))

                    heading = self.heading(yaw)
                    
                    # Vehicles heading northbound or southbound
                    if heading in ("north", "south"):
                        speed = (
                            self.speed_vehicles_left_lane
                            if y > 0.0 else
                            self.speed_vehicles_right_lane
                        )
                    # Vehicles heading eastbound or westbound                    
                    elif heading in ("east", "west"):
                        speed = (
                            self.speed_vehicles_left_lane
                            if x > 80.0 else
                            self.speed_vehicles_right_lane
                        )
                    else:
                        speed = 0.0  # safety default

                    vehicle.setVelocity([speed, 0, 0, 0, 0, 0])

        # -------- Vehicle pose publishing (always) --------
        if sim_t - self.last_vehicle_pub >= self.vehicle_pub_interval:
            self.last_vehicle_pub = sim_t

            for i, vehicle in enumerate(self.vehicles):
                if vehicle:
                    pos = vehicle.getPosition()
                    self.pub_vehicles_pose[i].publish(
                        Pose2D(x=pos[0], y=pos[1], theta=0.0)
                    )

        # -------- BMW pose (always) --------
        if sim_t - self.last_bmw_pub >= self.bmw_pub_interval:
            self.last_bmw_pub = sim_t
            self.publish_bmw_pose()

        # -------- Citroen pose (always) --------
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

        self.pub_bmw_pose.publish(
            Pose2D(x=pos[0], y=pos[1], theta=theta)
        )

    def publish_citroenczero_pose(self):
        if not self.citroenczero:
            return

        pos = self.citroenczero.getPosition()
        orient = self.citroenczero.getOrientation()
        theta = math.atan2(orient[3], orient[0])

        self.pub_citroenczero_pose.publish(
            Pose2D(x=pos[0], y=pos[1], theta=theta)
        )
    
    # −pi < yaw ≤ pi
    def normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))    
        
    def heading(self, yaw):
        if -0.1 <= yaw <= 0.1:
           return "north"
        elif abs(abs(yaw) - math.pi) < 0.1:
           return "south"
        elif -1.65 <= yaw <= -1.52:
           return "east"
        elif 1.52 <= yaw <= 1.65:
           return "west"
        return "unknown"

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

