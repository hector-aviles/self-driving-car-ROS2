#!/usr/bin/env python3
"""
This node detect obstacles around the car using the lidar sensor.
Obstacles are detected only by counting the number of points inside 
a bounding box. Each BB defines a region around de car:
----------------|---------|--------------|
North/North East| East    | South East   |
----------------|---------|--------------|
North/North-west| West    | South West   |
----------------|---------|--------------|
"""
import math
import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty, Bool, Float64
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock 
import ros2_numpy as ros_numpy

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('free_detector')
        
        # Initialize variables
        self.sim_secs = 0
        self.sim_nsecs = 0
        self.curr_time = 0.0
        
        print("INITIALIZING OBSTACLE DETECTOR...", flush=True)
        
        # Create subscribers
        self.sub_point_cloud = self.create_subscription(
            PointCloud2, 
            '/point_cloud', 
            self.callback_point_cloud, 
            10
        )
        self.sub_sim_time = self.create_subscription(
            Clock, 
            "/clock", 
            self.callback_sim_time, 
            10
        )
        
        # Create publishers
        self.pub_obs_N = self.create_publisher(Bool, "/free/north", 1)
        self.pub_obs_NW = self.create_publisher(Bool, "/free/north_west", 1)
        self.pub_obs_W = self.create_publisher(Bool, "/free/west", 1)
        self.pub_obs_SW = self.create_publisher(Bool, "/free/south_west", 1)
        self.pub_obs_NE = self.create_publisher(Bool, "/free/north_east", 1)
        self.pub_obs_E = self.create_publisher(Bool, "/free/east", 1)
        self.pub_obs_SE = self.create_publisher(Bool, "/free/south_east", 1)
        self.pub_obs_dist = self.create_publisher(Float64, "/obstacle/distance", 1)
        
        self.get_logger().info("Obstacle detector node started")

    def callback_sim_time(self, msg):
        self.sim_secs = msg.clock.sec 
        self.sim_nsecs = msg.clock.nanosec 
        self.curr_time = self.sim_secs + self.sim_nsecs / (10**9)        

    def callback_point_cloud(self, msg):
        try:
            # Convert PointCloud2 to numpy array
            # Note: ros2_numpy might have slightly different API than ros_numpy
            # You may need to adjust this conversion based on your ros2_numpy version
            xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

            # Define regions and count points
            N_points  = xyz[(xyz[:,0] >  2.5) & (xyz[:,0] <  25) & (xyz[:,1] < 1.5) & (xyz[:,1] > -1.5)]
            NW_points = xyz[(xyz[:,0] >  5) & (xyz[:,0] <   25) & (xyz[:,1] < 4.0) & (xyz[:,1] >  1.2)]
            W_points  = xyz[(xyz[:,0] > -5) & (xyz[:,0] <  5) & (xyz[:,1] < 4.0) & (xyz[:,1] >  1.2)]
            SW_points = xyz[(xyz[:,0] >  -25) & (xyz[:,0] < -5) & (xyz[:,1] < 4.0) & (xyz[:,1] >  1.2)]
            
            NE_points  = xyz[(xyz[:,0] >  5) & (xyz[:,0] <   25) & (xyz[:,1] > -4.0) & (xyz[:,1] <  -1.2)]        
            E_points  = xyz[(xyz[:,0] > -5) & (xyz[:,0] <  5) & (xyz[:,1] > -4.0) & (xyz[:,1] <  -1.2)] 
            SE_points  = xyz[(xyz[:,0] >  -25) & (xyz[:,0] < -5) & (xyz[:,1] > -4.0) & (xyz[:,1] <  -1.2)]               
            
            # Determine if regions are free based on point count thresholds
            free_N  = N_points.shape[0] < 1000
            #print('free_N=', N_points.shape[0], flush = True)
            free_NW = NW_points.shape[0] < 5000
            #print('free_NW=', NW_points.shape[0], flush = True)            
            free_W  = W_points.shape[0] < 5000
            #print('free_W=', W_points.shape[0], flush = True)
            free_SW = SW_points.shape[0] < 5000
            #print('free_SW=', SW_points.shape[0], flush = True)
            
            #print("NE_points.shape[0]: ", NE_points.shape[0], flush = True)  
            free_NE  = NE_points.shape[0] < 5000
            #print('free_NE=', NE_points.shape[0], flush = True)            
            free_E  = E_points.shape[0] < 5000
            #print('free_E=', E_points.shape[0], flush = True)            
            free_SE  = SE_points.shape[0] < 5000   
            #print('free_SE=', SE_points.shape[0], flush = True)
            
            # Publish free space information
            self.publish_bool(self.pub_obs_N, free_N)
            self.publish_bool(self.pub_obs_NW, free_NW)
            self.publish_bool(self.pub_obs_W, free_W)
            self.publish_bool(self.pub_obs_SW, free_SW)
            self.publish_bool(self.pub_obs_NE, free_NE)
            self.publish_bool(self.pub_obs_E, free_E)
            self.publish_bool(self.pub_obs_SE, free_SE)

            # Calculate obstacle distance in front of the car
            obs_points  = xyz[(xyz[:,0] >  2.5) & (xyz[:,0] < 100) & (xyz[:,1] < 1.0) & (xyz[:,1] > -1.0)]
            if obs_points.shape[0] > 100:
                distance = numpy.linalg.norm(numpy.mean(obs_points, axis=0))
                dist_msg = Float64()
                dist_msg.data = float(distance)
                self.pub_obs_dist.publish(dist_msg)
                
                # Optional logging
                # self.get_logger().info(f"Average distance north: {distance:.2f}")
            
            # Optional: log all region counts
            # self.get_logger().info(f"N: {N_points.shape[0]}, NW: {NW_points.shape[0]}, W: {W_points.shape[0]}, SW: {SW_points.shape[0]}, NE: {NE_points.shape[0]}, E: {E_points.shape[0]}, SE: {SE_points.shape[0]}, time: {self.curr_time}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {str(e)}")

    def publish_bool(self, publisher, value):
        """Helper function to publish Bool messages"""
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

def main():
    rclpy.init()
    node = ObstacleDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
