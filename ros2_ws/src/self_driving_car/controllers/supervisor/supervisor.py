#!/usr/bin/env python3
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import Pose2D, Twist
from controller import Supervisor

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')
        
        self.robot = Supervisor()
        self.TIME_STEP = 10
        
        # Initialize vehicle speeds
        self.speed_cars_left_lane = 0.0
        self.speed_cars_right_lane = 0.0
        
        # Get all vehicle references
        self.cars = [
            self.robot.getFromDef('vehicle_1'),
            self.robot.getFromDef('vehicle_2'),
            self.robot.getFromDef('vehicle_3'),
            self.robot.getFromDef('vehicle_4'),
            self.robot.getFromDef('vehicle_5'),
            self.robot.getFromDef('vehicle_6'),
            self.robot.getFromDef('vehicle_7'),
            self.robot.getFromDef('vehicle_8'),
            self.robot.getFromDef('vehicle_9'),
            self.robot.getFromDef('vehicle_10')
        ]
        
        self.bmw = self.robot.getFromDef('BMW_X5')
        
        # Initialize vehicle positions with random offsets
        self.tf = []
        self.initialize_vehicle_positions()
        
        # ROS 2 Publishers
        self.pub_bmw_pose = self.create_publisher(Pose2D, '/self_driving_pose', 10)

        # Renamed to avoid conflict with read-only Node.publishers property
        self.car_publishers = {}

        for i in range(1, 11):
            self.car_publishers[f'car_{i}'] = self.create_publisher(
                Pose2D, 
                f'/car_{i}_pose', 
                10
            )
        
        # ROS 2 Subscribers
        self.sub_speed_left = self.create_subscription(
            Float64, 
            '/speed_cars_left_lane', 
            self.callback_speed_cars_left_lane, 
            10
        )
        self.sub_speed_right = self.create_subscription(
            Float64, 
            '/speed_cars_right_lane', 
            self.callback_speed_cars_right_lane, 
            10
        )
        
        # Wait for start signal
        self.start_signal_received = False
        self.sub_start = self.create_subscription(
            Empty,
            '/policy_started',
            self.callback_start_signal,
            10
        )
        
        self.get_logger().info('Supervisor Node Started')
        self.get_logger().info('Waiting for start signal...')
        
        # Timer for main control loop
        self.timer = self.create_timer(self.TIME_STEP / 1000.0, self.control_loop)
    
    def initialize_vehicle_positions(self):
        """Initialize vehicle positions with random offsets"""
        self.get_logger().info('Initializing vehicle positions...')
        
        self.tf = []
        for i, car in enumerate(self.cars):
            if car is not None:
                tf_field = car.getField("translation")
                self.tf.append(tf_field)
                
                values = tf_field.getSFVec3f()
                x, y = values[0], values[1]
                
                # Apply random offset to x position
                rand_val = np.random.uniform(-2, 2, 1)[0]
                values[0] = x + rand_val
                
                # Set new position and reset physics
                tf_field.setSFVec3f(values)
                car.resetPhysics()
            else:
                self.tf.append(None)
                self.get_logger().warning(f'Car {i+1} not found in simulation')
    
    def callback_speed_cars_left_lane(self, msg):
        self.speed_cars_left_lane = msg.data
    
    def callback_speed_cars_right_lane(self, msg):
        self.speed_cars_right_lane = msg.data
    
    def callback_start_signal(self, msg):
        if not self.start_signal_received:
            self.start_signal_received = True
            self.get_logger().info('Start signal received - Beginning supervision')
    
    def control_loop(self):
        if self.robot.step(self.TIME_STEP) == -1:
            self.get_logger().info('Simulation ended')
            return
        
        if not self.start_signal_received:
            return
        
        # Update and publish positions for all cars
        for i, car in enumerate(self.cars):
            if car is not None and self.tf[i] is not None:
                values = self.tf[i].getSFVec3f()
                
                msg_car_pose = Pose2D()
                msg_car_pose.x = values[0]
                msg_car_pose.y = values[1]
                msg_car_pose.theta = 0.0
                
                # Set velocity based on lane
                if msg_car_pose.y < 0:
                    car.setVelocity([self.speed_cars_left_lane, 0, 0, 0, 0, 0])
                else:
                    car.setVelocity([self.speed_cars_right_lane, 0, 0, 0, 0, 0])
                
                publisher_name = f'car_{i+1}'
                if publisher_name in self.car_publishers:
                    self.car_publishers[publisher_name].publish(msg_car_pose)
        
        # Publish BMW pose
        self.publish_bmw_pose()
    
    def publish_bmw_pose(self):
        if self.bmw is not None:
            bmw_pose = self.bmw.getPosition()
            bmw_orient = self.bmw.getOrientation()
            
            msg_bmw_pose = Pose2D()
            msg_bmw_pose.x = bmw_pose[0]
            msg_bmw_pose.y = bmw_pose[1]
            msg_bmw_pose.theta = math.atan2(bmw_orient[3], bmw_orient[0])
            
            self.pub_bmw_pose.publish(msg_bmw_pose)
    
def main(args=None):
    rclpy.init(args=args)
    
    try:
        supervisor = SupervisorNode()
        rclpy.spin(supervisor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in Supervisor Node: {e}")
    finally:
        if 'supervisor' in locals():
            supervisor.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

