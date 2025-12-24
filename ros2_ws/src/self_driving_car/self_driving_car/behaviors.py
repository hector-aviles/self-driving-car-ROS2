#!/usr/bin/env python3
"""
This node implements several behaviors:
* Lane tracking using a proportional control (intended to be used together with the lane_detector node)
* Car following using also a proportional control (to be used together with the obstacle detector node)
* Change lane, using finite state machine and assuming we know the car position
* Pass, same conditions as the change lane behavior.  
"""
import cv2
import numpy
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, Empty, Bool
from geometry_msgs.msg import Pose2D

SM_INIT = 0
SM_WAITING_FOR_NEW_TASK = 10
SM_START_STEADY_MOTION = 20
SM_START_CAR_FOLLOWING = 30
SM_TURNING_LEFT_1 = 40
SM_TURNING_LEFT_2 = 45
SM_TURNING_RIGHT_1 = 50
SM_TURNING_RIGHT_2 = 55
SM_TURNING_FINISHED = 60
SM_PASS_ON_RIGHT_1 = 70
SM_PASS_ON_RIGHT_2 = 80
SM_PASS_ON_RIGHT_3 = 90
SM_PASS_ON_RIGHT_4 = 100
SM_PASS_ON_RIGHT_5 = 110
SM_PASS_ON_LEFT_1 = 120
SM_PASS_ON_LEFT_2 = 130
SM_PASS_ON_LEFT_3 = 140
SM_PASS_ON_LEFT_4 = 150
SM_PASS_ON_LEFT_5 = 160
MAX_STEERING = 0.5

class BehaviorsNode(Node):
    def __init__(self):
        super().__init__('behaviors')
        
        # Initialize global variables as instance variables
        self.max_speed = 30      # Maximum speed for following and steady motion behaviors
        self.k_rho = 0.001       # Gain for rho error in lane tracking
        self.k_theta = 0.01      # Gain for theta error in lane tracking
        self.k_following = 10.0
        self.dist_to_car = 30
        
        self.lane_rho_l = 0
        self.lane_theta_l = 0
        self.lane_rho_r = 0
        self.lane_theta_r = 0
        self.goal_rho_l = 370.0
        self.goal_theta_l = 2.4
        self.goal_rho_r = 430.0
        self.goal_theta_r = 0.895
        
        self.enable_cruise = False
        self.enable_follow = False
        self.dist_to_obs = None
        self.start_change_lane_on_left = False
        self.start_change_lane_on_right = False
        self.start_pass_on_left = False
        self.start_pass_on_right = False
        self.current_x, self.current_y, self.current_a = 0, 0, 0
        self.last_x = 0
        
        # Free space variables
        self.free_north = False
        self.free_north_west = False
        self.free_west = False
        self.free_south_west = False
        self.free_north_east = False
        self.free_east = False
        self.free_south_east = False
        
        # Declare parameters
        self.declare_parameter('max_speed', self.max_speed)
        self.declare_parameter('k_rho', self.k_rho)
        self.declare_parameter('k_theta', self.k_theta)
        self.declare_parameter('k_following', self.k_following)
        self.declare_parameter('dist_to_car', self.dist_to_car)
        
        # Get parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.k_rho = self.get_parameter('k_rho').value
        self.k_theta = self.get_parameter('k_theta').value
        self.k_following = self.get_parameter('k_following').value
        self.dist_to_car = self.get_parameter('dist_to_car').value
        
        print('INITIALIZING BEHAVIORS NODE...', flush=True)
        
        # Create subscribers
        self.sub_left_lane = self.create_subscription(
            Float64MultiArray, "/demo/left_lane", self.callback_left_lane, 10)
        
        self.sub_right_lane = self.create_subscription(
            Float64MultiArray, "/demo/right_lane", self.callback_right_lane, 10)
        self.sub_enable_cruise = self.create_subscription(
            Bool, "/cruise/enable", self.callback_enable_cruise, 10)
        self.sub_enable_follow = self.create_subscription(
            Bool, "/follow/enable", self.callback_enable_follow, 10)
        self.sub_start_change_left = self.create_subscription(
            Bool, "/start_change_lane_on_left", self.callback_start_change_lane_on_left, 10)
        self.sub_start_change_right = self.create_subscription(
            Bool, "/start_change_lane_on_right", self.callback_start_change_lane_on_right, 10)
        self.sub_start_pass_left = self.create_subscription(
            Bool, "/start_pass_on_left", self.callback_start_pass_on_left, 10)
        self.sub_start_pass_right = self.create_subscription(
            Bool, "/start_pass_on_right", self.callback_start_pass_on_right, 10)
        self.sub_dist_to_obstacle = self.create_subscription(
            Float64, "/obstacle/distance", self.callback_dist_to_obstacle, 10)
        self.sub_current_pose = self.create_subscription(
            Pose2D, "/bmw_pose", self.callback_current_pose, 10)
        self.sub_free_north = self.create_subscription(
            Bool, "/free/north", self.callback_free_north, 10)
        self.sub_free_north_west = self.create_subscription(
            Bool, "/free/north_west", self.callback_free_north_west, 10)
        self.sub_free_west = self.create_subscription(
            Bool, "/free/west", self.callback_free_west, 10)
        self.sub_free_south_west = self.create_subscription(
            Bool, "/free/south_west", self.callback_free_south_west, 10)
        self.sub_free_north_east = self.create_subscription(
            Bool, "/free/north_east", self.callback_free_north_east, 10)
        self.sub_free_east = self.create_subscription(
            Bool, "/free/east", self.callback_free_east, 10)
        self.sub_free_south_east = self.create_subscription(
            Bool, "/free/south_east", self.callback_free_south_east, 10)
        
        # Create publishers
        self.pub_speed = self.create_publisher(Float64, '/bmw_speed', 1)
        self.pub_angle = self.create_publisher(Float64, '/bmw_steering', 1)
        self.pub_change_lane_finished = self.create_publisher(Bool, '/change_lane_finished', 1)
        self.pub_pass_finished = self.create_publisher(Bool, '/pass_finished', 1)
        
        # Create timer for main loop
        self.timer = self.create_timer(0.1, self.main_loop)  # 10Hz
        
        self.state = SM_INIT
        
        print("Using:")
        print("Max speed: " + str(self.max_speed))
        print("K_rho: " + str(self.k_rho))
        print("K_theta: " + str(self.k_theta))
        print("K_following: " + str(self.k_following))
        print("Dist to car: " + str(self.dist_to_car))
        
        # Wait for initial lane detection messages
        print("Waiting for lane detection...")
    
    #
    # Steering is calculated proportional to two errors: distance error and angle error.
    # These errors correspond to differences between an observed line (in normal form)
    # and a desired line.
    # Speed is calculated as the max speed minus a speed proportional to the steering.
    # In this way, car goes with lower speed in curves and at max speed in straight roads. 
    #
    def calculate_control(self, rho_l, theta_l, rho_r, theta_r, goal_rho_l, goal_theta_l, goal_rho_r, goal_theta_r, dist=None):
        error_rho_l   = goal_rho_l   - rho_l
        error_theta_l = goal_theta_l - theta_l
        error_rho_r   = rho_r   - goal_rho_r
        error_theta_r = theta_r - goal_theta_r
        
        if rho_l != 0 and rho_r != 0:
            error_rho   = (error_rho_l + error_rho_r)/2
            error_theta = (error_theta_l + error_theta_r)/2
        elif rho_l != 0:
            error_rho   = error_rho_l
            error_theta = error_theta_l
        else:
            error_rho   = error_rho_r
            error_theta = error_theta_r
        
        steering = -self.k_rho*error_rho - self.k_theta*error_theta
        if dist is None:
            speed = self.max_speed*(1 - 1.5*abs(steering))
        else:
            speed = self.max_speed + self.k_following*(dist - self.dist_to_car)
            if speed > self.max_speed:
                speed = self.max_speed
            if speed < -10:
                speed = -10
        return speed, steering

    def callback_left_lane(self, msg):
        self.lane_rho_l, self.lane_theta_l = msg.data

    def callback_right_lane(self, msg):
        self.lane_rho_r, self.lane_theta_r = msg.data

    def callback_enable_cruise(self, msg):
        self.enable_cruise = msg.data
        if self.enable_cruise:
            self.enable_follow = False

    def callback_enable_follow(self, msg):
        self.enable_follow = msg.data
        if self.enable_follow:
            self.enable_cruise = False

    def callback_dist_to_obstacle(self, msg):
        self.dist_to_obs = msg.data

    def callback_start_change_lane_on_left(self, msg):
        self.start_change_lane_on_left = msg.data
        if self.start_change_lane_on_left:
            self.enable_follow, self.enable_cruise = False, False

    def callback_start_change_lane_on_right(self, msg):
        self.start_change_lane_on_right = msg.data
        if self.start_change_lane_on_right:
            self.enable_follow, self.enable_cruise = False, False

    def callback_start_pass_on_left(self, msg):
        self.start_pass_on_left = msg.data
        if self.start_pass_on_left:
            self.enable_follow, self.enable_cruise = False, False

    def callback_start_pass_on_right(self, msg):
        self.start_pass_on_right = msg.data
        if self.start_pass_on_right:
            self.enable_follow, self.enable_cruise = False, False

    def callback_current_pose(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_a = msg.theta

    def callback_free_north(self, msg):
        self.free_north = msg.data
    
    def callback_free_north_west(self, msg):
        self.free_north_west = msg.data
    
    def callback_free_west(self, msg):
        self.free_west = msg.data
    
    def callback_free_south_west(self, msg):
        self.free_south_west = msg.data
    
    def callback_free_north_east(self, msg):
        self.free_north_east = msg.data
    
    def callback_free_east(self, msg):
        self.free_east = msg.data
    
    def callback_free_south_east(self, msg):
        self.free_south_east = msg.data

    def calculate_turning_steering(self, w, L, v):
        # Steering is calculated from the kinematic model:  w = (v sin(d)) / L where:
        # v = current vehicle speed
        # w = desired angular speed
        # L = distance axis to axis
        if v == 0:
            return 0
        k = L * w / v

        if k > 0.5:
            k = 0.5
        if k < -0.5:
            k = -0.5

        steering = math.asin(k)
        if steering > MAX_STEERING: 
           self.get_logger().info(f"Setting steering to {MAX_STEERING}")
           steering = MAX_STEERING
        elif steering < -MAX_STEERING:
           self.get_logger().info(f"Setting steering to {MAX_STEERING}")
           steering = -MAX_STEERING       
 
        return steering
    
    def main_loop(self):
        speed, steering = 0, 0
        
        if self.state == SM_INIT:
            self.get_logger().info("Waiting for task...")
            self.state = SM_WAITING_FOR_NEW_TASK

        elif self.state == SM_WAITING_FOR_NEW_TASK:
            if self.enable_cruise:
                self.state = SM_START_STEADY_MOTION
                self.get_logger().info("Starting steady motion")
            elif self.enable_follow:
                self.state = SM_START_CAR_FOLLOWING
                self.get_logger().info("Starting follow car")
            elif self.start_change_lane_on_left:
                self.state = SM_TURNING_LEFT_1
                self.start_change_lane_on_left = False
                self.get_logger().info("Starting change lane on left")
            elif self.start_change_lane_on_right:
                self.state = SM_TURNING_RIGHT_1
                self.start_change_lane_on_right = False
                self.get_logger().info("Starting change lane on right")
            elif self.start_pass_on_left:
                self.state = SM_PASS_ON_LEFT_1
                self.start_pass_on_left = False
                self.get_logger().info("Starting passing on left")
            elif self.start_pass_on_right:
                self.state = SM_PASS_ON_RIGHT_1
                self.start_pass_on_right = False
                self.get_logger().info("Starting passing on right")
            else:
                speed, steering = 0, 0

        elif self.state == SM_START_STEADY_MOTION:
            speed, steering = self.calculate_control(
                self.lane_rho_l, self.lane_theta_l, self.lane_rho_r, self.lane_theta_r, 
                self.goal_rho_l, self.goal_theta_l, self.goal_rho_r, self.goal_theta_r)
            self.dist_to_obs = None
            
            if not self.enable_cruise:
                self.state = SM_WAITING_FOR_NEW_TASK

        elif self.state == SM_START_CAR_FOLLOWING:
            speed, steering = self.calculate_control(
                self.lane_rho_l, self.lane_theta_l, self.lane_rho_r, self.lane_theta_r,
                self.goal_rho_l, self.goal_theta_l, self.goal_rho_r, self.goal_theta_r, 
                self.dist_to_obs)
            if not self.enable_follow:
                self.state = SM_WAITING_FOR_NEW_TASK

        #
        # STATES FOR CHANGE TO LEFT LANE
        #
        elif self.state == SM_TURNING_LEFT_1:
            if speed <= 10:
                speed = self.max_speed
            steering = self.calculate_turning_steering(1.2, 2.9, speed)
            if self.current_y > -0.7:  # Vehicle has moved to the left. Right lane has y=-1.5 and center is around y=0
                self.get_logger().info("Moving to right to align with left lane")
                self.state = SM_TURNING_LEFT_2 

        elif self.state == SM_TURNING_LEFT_2:
            if speed <= 10:
                speed = self.max_speed
            steering = self.calculate_turning_steering(-1.2, 2.9, speed)
            if self.current_y > 1.0 or abs(self.current_a) < 0.2:  # Vehicle has moved to the left lane. Left lane has y=1.5
                self.get_logger().info("Change lane on left finished")
                self.pub_change_lane_finished.publish(Bool(data=True))
                self.state = SM_WAITING_FOR_NEW_TASK

        #
        # STATES FOR CHANGE TO RIGHT LANE
        #
        elif self.state == SM_TURNING_RIGHT_1:
            if speed <= 10:
                speed = self.max_speed
            steering = self.calculate_turning_steering(-1.2, 2.9, speed)
            if self.current_y < 0.7:  # Vehicle has moved to the right. Left lane has y=1.5 and center is around y=0
                self.get_logger().info("Moving to left to align with right lane")
                self.state = SM_TURNING_RIGHT_2

        elif self.state == SM_TURNING_RIGHT_2:
            if speed <= 10:
                speed = self.max_speed
            steering = self.calculate_turning_steering(1.2, 2.9, speed)
            if self.current_y < -1.0 or abs(self.current_a) < 0.2:  # Vehicle has moved to the right lane. Right lane has y=-1.5
                self.get_logger().info("Change lane on right finished")
                self.pub_change_lane_finished.publish(Bool(data=True))
                self.state = SM_WAITING_FOR_NEW_TASK

        #
        # STATES FOR PASSING ON THE LEFT
        #
        elif self.state == SM_PASS_ON_LEFT_1:
            if speed <= 10:
                speed = self.max_speed
            steering = self.calculate_turning_steering(1.2, 2.9, speed)
            if self.current_y > -0.7:  # Vehicle has moved to the left. Right lane has y=-1.5 and center is around y=0
                self.get_logger().info("Moving to right to align with left lane")
                self.state = SM_PASS_ON_LEFT_2

        elif self.state == SM_PASS_ON_LEFT_2:
            if speed <= 10:
                speed = self.max_speed
            steering = self.calculate_turning_steering(-1.2, 2.9, speed)
            if self.current_y > 1.0 or abs(self.current_a) < 0.2:  # Vehicle has moved to the left lane. Left lane has y=1.5
                self.get_logger().info("Moving to left lane finished. Starting to follow lane")
                self.state = SM_PASS_ON_LEFT_3
                self.last_x = self.current_x

        elif self.state == SM_PASS_ON_LEFT_3:
            speed, steering = self.calculate_control(
                self.lane_rho_l, self.lane_theta_l, self.lane_rho_r, self.lane_theta_r,
                self.goal_rho_l, self.goal_theta_l, self.goal_rho_r, self.goal_theta_r)
            if self.free_north_east and self.free_east:
                self.state = SM_PASS_ON_LEFT_4

        elif self.state == SM_PASS_ON_LEFT_4:
            if speed == 0:
                speed = self.max_speed
            steering = self.calculate_turning_steering(-1.2, 2.9, speed)
            if self.current_y < 0.7:  # Vehicle has moved to the right. Left lane has y=1.5 and center is around y=0
                self.get_logger().info("Moving to right to align with right lane after passing on left")
                self.state = SM_PASS_ON_LEFT_5

        elif self.state == SM_PASS_ON_LEFT_5:
            if speed <= 10:
                speed = self.max_speed
            steering = self.calculate_turning_steering(1.2, 2.9, speed)
            if self.current_y < -1.0 or abs(self.current_a) < 0.2:  # Vehicle has moved to the right lane. Right lane has y=-1.5
                self.pub_pass_finished.publish(Bool(data=True))
                self.get_logger().info("Passing on left finished")
                self.state = SM_WAITING_FOR_NEW_TASK

        #
        # STATES FOR PASSING ON THE RIGHT
        #
        elif self.state == SM_PASS_ON_RIGHT_1:
            if speed <= 10:
                speed = self.max_speed
            steering = self.calculate_turning_steering(-1.2, 2.9, speed)
            if self.current_y < 0.7:  # Vehicle has moved to the right. Right lane has y=-1.5 and center is around y=0
                self.get_logger().info("Moving to left to align with right lane")
                self.state = SM_PASS_ON_RIGHT_2

        elif self.state == SM_PASS_ON_RIGHT_2:
            if speed <= 10:
                speed = self.max_speed
            steering = self.calculate_turning_steering(1.2, 2.9, speed)
            if self.current_y < -1.0 or abs(self.current_a) < 0.2:  # Vehicle has moved to the right lane. Right lane has y=-1.5
                self.get_logger().info("Moving to right lane finished. Starting to follow lane.")
                self.state = SM_PASS_ON_RIGHT_3
                self.last_x = self.current_x

        elif self.state == SM_PASS_ON_RIGHT_3:
            speed, steering = self.calculate_control(
                self.lane_rho_l, self.lane_theta_l, self.lane_rho_r, self.lane_theta_r,
                self.goal_rho_l, self.goal_theta_l, self.goal_rho_r, self.goal_theta_r)
            if self.free_north_west and self.free_west:
                self.state = SM_PASS_ON_RIGHT_4

        elif self.state == SM_PASS_ON_RIGHT_4:
            if speed <= 10:
                speed = self.max_speed
            steering = self.calculate_turning_steering(1.2, 2.9, speed)
            if self.current_y > -0.7:  # Vehicle has moved to the right. Left lane has y=1.5 and center is around y=0
                self.get_logger().info("Moving to right to align with left lane after passing on right")
                self.state = SM_PASS_ON_RIGHT_5

        elif self.state == SM_PASS_ON_RIGHT_5:
            if speed <= 10:
                speed = self.max_speed
            steering = self.calculate_turning_steering(-1.2, 2.9, speed)
            if self.current_y > 1.0 or abs(self.current_a) < 0.2:  # Vehicle has moved to the left lane. Left lane has y=1.5
                self.get_logger().info("Passing on right finished")
                self.pub_pass_finished.publish(Bool(data=True))
                self.state = SM_WAITING_FOR_NEW_TASK
                
        else:
            self.get_logger().error("Invalid STATE")
            return
        
        #print("DEBUG speed type:", type(speed), "value:", speed, flush=True)

        #self.get_logger().debug(f"DEBUG speed raw: {speed!r} ({type(speed)}), steering raw: {steering!r} ({type(steering)})")
        # Ensure speed and steering are Python floats (avoid numpy types or None)
        try:
            speed_val = float(speed)
        except Exception as e:
            # debug help: print the bad value and use a safe fallback
            self.get_logger().warning(f"Invalid speed value {speed!r} ({type(speed)}), using 0.0. Error: {e}")
            speed_val = 0.0

        try:
            steering_val = float(steering)
        except Exception as e:
            self.get_logger().warning(f"Invalid steering value {steering!r} ({type(steering)}), using 0.0. Error: {e}")
            steering_val = 0.0

        # Publish control commands
        speed_msg = Float64()
        speed_msg.data = speed_val
        self.pub_speed.publish(speed_msg)

        steering_msg = Float64()
        steering_msg.data = steering_val
        self.pub_angle.publish(steering_msg)

def main():
    rclpy.init()
    node = BehaviorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
