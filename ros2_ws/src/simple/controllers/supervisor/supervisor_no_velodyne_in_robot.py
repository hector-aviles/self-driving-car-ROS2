#!/usr/bin/env python3

import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import Pose2D, Twist
from controller import Supervisor

TIME_STEP = 10
MAX_VEHICLES = 5
robot = Supervisor() 

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')
        
        # Initialize vehicle speeds
        self.speed_vehicle_left_lane = 0.0
        self.speed_vehicle_right_lane = 0.0
        
        # Vehicles travelling in the same direction as the self-driving BMW
        self.vehicles = [
            robot.getFromDef(f'vehicle_{i+1}')
            for i in range(MAX_VEHICLES)
        ]
        
        # Vehicles travelling in opposite direction to the self-driving BMW
        self.vehicles_opposite = [
            robot.getFromDef(f'vehicle_opposite_{i+1}')
            for i in range(MAX_VEHICLES)
        ]        
        
        # Vehicles travelling in transverse direction to the self-driving BMW
        self.vehicles_transverse = [
            robot.getFromDef(f'vehicle_transverse_{i+1}')
            for i in range(MAX_VEHICLES)
        ]                
        
        self.bmw = robot.getFromDef('BMW_X5')
        
        # Initialize vehicle positions with random offsets
        self.tf = []
        self.initialize_vehicle_positions()
        
        # ROS 2 Publishers
        self.pub_bmw_pose = self.create_publisher(Pose2D, '/self_driving_pose', 10)

        self.pub_vehicles_pose = [
            self.create_publisher(Pose2D, f"/vehicle_{i+1}_pose", 10)
            for i in range(MAX_VEHICLES)
        ]            
        
        self.pub_vehicles_opposite_pose = [
            self.create_publisher(Pose2D, f"/vehicle_opposite_{i+1}_pose", 10)
            for i in range(MAX_VEHICLES)
        ]                    
        
        self.pub_vehicles_transverse_pose = [
            self.create_publisher(Pose2D, f"/vehicle_transverse_{i+1}_pose", 10)
            for i in range(MAX_VEHICLES)
        ]        
        
        # ROS 2 Subscribers
        self.sub_speed_left = self.create_subscription(
            Float64, 
            '/speed_vehicles_left_lane', 
            self.callback_speed_vehicles_left_lane, 
            10
        )
        self.sub_speed_right = self.create_subscription(
            Float64, 
            '/speed_vehicles_right_lane', 
            self.callback_speed_vehicles_right_lane, 
            10
        )

        self.get_logger().info('Supervisor Node Started')
        self.get_logger().info('Waiting for start signal...')        
        
        # Wait for start signal
        self.start_signal_received = False
        self.sub_start = self.create_subscription(
            Empty,
            '/policy_started',
            self.callback_start_signal,
            10
        )
        
        
        # Timer for main control loop
        self.timer = self.create_timer(TIME_STEP / 1000.0, self.control_loop)
    
    def initialize_vehicle_positions(self):
        """Initialize vehicle coordinates with random offsets"""
        self.get_logger().info('Initializing vehicle coordinates...')
        
        self.tf = []
        for i, vehicle in enumerate(self.vehicles):
            if vehicle is not None:
                field = vehicle.getField("translation")
                values = field.getSFVec3f()
                values[0] += np.random.uniform(-2, 2)
                field.setSFVec3f(values)
                vehicle.resetPhysics()
                self.tf.append(field)
                self.get_logger().warning(f'Vehicle {i+1} found.')
                
            else:
                self.tf.append(None)
                #self.get_logger().warning(f'Vehicle {i+1} not found in simulation')

        self.tf = []
        for i, vehicle in enumerate(self.vehicles_opposite):
            if vehicle is not None:
                field = vehicle.getField("translation")
                values = field.getSFVec3f()
                values[0] += np.random.uniform(-2, 2)
                field.setSFVec3f(values)
                vehicle.resetPhysics()
                self.tf.append(field)
                self.get_logger().warning(f'Vehicle Opposite {i+1} found.')
                
            else:
                self.tf.append(None)
                #self.get_logger().warning(f'Vehicle Opposite {i+1} not found in simulation')                

        self.tf = []
        for i, vehicle in enumerate(self.vehicles_transverse):
            if vehicle is not None:
                field = vehicle.getField("translation")
                values = field.getSFVec3f()
                values[1] += np.random.uniform(-2, 2)
                field.setSFVec3f(values)
                vehicle.resetPhysics()
                self.tf.append(field)
                self.get_logger().warning(f'Vehicle Transverse {i+1} found.')
                
            else:
                self.tf.append(None)
                #self.get_logger().warning(f'Vehicle Transverse {i+1} not found in simulation')    
    
    def callback_speed_vehicles_left_lane(self, msg):
        self.speed_vehicles_left_lane = msg.data
    
    def callback_speed_vehicles_right_lane(self, msg):
        self.speed_vehicles_right_lane = msg.data
    
    def callback_start_signal(self, msg):
        if not self.start_signal_received:
            self.start_signal_received = True
            self.get_logger().info('Start signal received - Beginning supervision')
    
    def control_loop(self):
        if robot.step(TIME_STEP) == -1:
            self.get_logger().info('Simulation ended')
            return
        
        if not self.start_signal_received:
            return
        
        # Update and publish positions for all vehicles
        for i, vehicle in enumerate(self.vehicles):
            if vehicle is not None and self.tf[i] is not None:
                values = self.tf[i].getSFVec3f()
                
                msg_vehicles_pose = Pose2D()
                msg_vehicles_pose.x = values[0]
                msg_vehicles_pose.y = values[1]
                msg_vehicles_pose.theta = 0.0
                
                vehicle.setVelocity([self.speed_vehicles_left_lane, 0, 0, 0, 0, 0])
                
                publisher_name = f'vehicle_{i+1}'
                if publisher_name in self.pub_vehicles_pose:
                    self.pub_vehicles_pose[publisher_name].publish(msg_vehicles_pose)
                    
        # Update and publish positions for all vehicles in opposite direction
        for i, vehicle in enumerate(self.vehicles_opposite):
            if vehicle is not None and self.tf[i] is not None:
                values = self.tf[i].getSFVec3f()
                
                msg_vehicles_opposite_pose = Pose2D()
                msg_vehicles_opposite_pose.x = values[0]
                msg_vehicles_opposite_pose.y = values[1]
                msg_vehicles_opposite_pose.theta = 0.0
                
                vehicle.setVelocity([-self.speed_vehicles_left_lane, 0, 0, 0, 0, 0])
                
                publisher_name = f'vehicle_opposite_{i+1}'
                if publisher_name in self.pub_vehicles_pose:                   
                 self.pub_vehicles_pose[publisher_name].publish(msg_vehicles_opposite_pose)

        # Update and publish positions for all vehicles in transverse direction
        for i, vehicle in enumerate(self.vehicles_transverse):
            if vehicle is not None and self.tf[i] is not None:
                values = self.tf[i].getSFVec3f()
                
                msg_vehicles_transverse_pose = Pose2D()
                msg_vehicles_transverse_pose.x = values[0]
                msg_vehicles_transverse_pose.y = values[1]
                msg_vehicles_transverse_pose.theta = 0.0
                
                # Set velocity based on lane
                vehicle.setVelocity([0, -self.speed_vehicles_left_lane, 0, 0, 0, 0])
                
                publisher_name = f'vehicle_transverse_{i+1}'
                if publisher_name in self.pub_vehicles_pose:
                  self.pub_vehicles_pose[publisher_name].publish( 
                            msg_vehicles_transverse_pose
                  )
                 
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

