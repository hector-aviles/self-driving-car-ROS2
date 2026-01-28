#!/usr/bin/env python3
"""
This node detects obstacles around the car using the lidar sensor.
Obstacles are detected only by counting the number of points inside 
a bounding box. Each BB defines a region around the car.
"""

import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from sensor_msgs_py import point_cloud2

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # -------------------------------------------------
        # Simulation time
        # -------------------------------------------------
        self.sim_secs = 0
        self.sim_nsecs = 0
        self.curr_time = 0.0

        #self.get_logger().info("INITIALIZING OBSTACLE DETECTOR...")

        # -------------------------------------------------
        # Subscribers
        # -------------------------------------------------
        self.create_subscription(
            PointCloud2,
            '/BMW/point_cloud',
            self.callback_point_cloud,
            10
        )

        self.create_subscription(
            Clock,
            "/clock",
            self.callback_sim_time,
            10
        )

        # -------------------------------------------------
        # Publishers
        # -------------------------------------------------
        self.pub_obs_N  = self.create_publisher(Bool, "/BMW/free_N",  10)
        self.pub_obs_NW = self.create_publisher(Bool, "/BMW/free_NW", 10)
        self.pub_obs_W  = self.create_publisher(Bool, "/BMW/free_W",  10)
        self.pub_obs_SW = self.create_publisher(Bool, "/BMW/free_SW", 10)
        self.pub_obs_NE = self.create_publisher(Bool, "/BMW/free_NE", 10)
        self.pub_obs_E  = self.create_publisher(Bool, "/BMW/free_E",  10)
        self.pub_obs_SE = self.create_publisher(Bool, "/BMW/free_SE", 10)

        self.pub_obs_dist = self.create_publisher(
            Float64, "/BMW/obstacle_distance", 10
        )

        self.get_logger().info("Obstacle detector node started")

    # =====================================================
    # Callbacks
    # =====================================================

    def callback_sim_time(self, msg: Clock):
        self.sim_secs = msg.clock.sec
        self.sim_nsecs = msg.clock.nanosec
        self.curr_time = self.sim_secs + self.sim_nsecs * 1e-9

    def callback_point_cloud(self, msg: PointCloud2):
        try:
            # Convert PointCloud2 → numpy array
            # xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
            xyz = numpy.fromiter(
               point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True),
               dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')]
            )
            xyz = numpy.vstack((xyz['x'], xyz['y'], xyz['z'])).T

            # Filter points close to the ground
            xyz = xyz[(xyz[:, 2] > -1.0) & (xyz[:, 2] < 0.3)]

            # -------------------------------------------------
            # Define regions
            # -------------------------------------------------
            N_points  = xyz[(xyz[:,0] >  2.5) & (xyz[:,0] <  25) & (xyz[:,1] <  1.5) & (xyz[:,1] > -1.5)]
            NW_points = xyz[(xyz[:,0] >  5.0) & (xyz[:,0] <  25) & (xyz[:,1] <  7.0) & (xyz[:,1] >  1.2)]
            W_points  = xyz[(xyz[:,0] > -5.0) & (xyz[:,0] <   5) & (xyz[:,1] <  7.0) & (xyz[:,1] >  1.2)]
            SW_points = xyz[(xyz[:,0] > -25 ) & (xyz[:,0] <  -5) & (xyz[:,1] <  7.0) & (xyz[:,1] >  1.2)]

            NE_points = xyz[(xyz[:,0] >  5.0) & (xyz[:,0] <  25) & (xyz[:,1] > -7.0) & (xyz[:,1] < -1.2)]
            E_points  = xyz[(xyz[:,0] > -5.0) & (xyz[:,0] <   5) & (xyz[:,1] > -7.0) & (xyz[:,1] < -1.2)]
            SE_points = xyz[(xyz[:,0] > -25 ) & (xyz[:,0] <  -5) & (xyz[:,1] > -7.0) & (xyz[:,1] < -1.2)]

            # -------------------------------------------------
            # Free / occupied logic
            # -------------------------------------------------
            free_N  = N_points.shape[0]  < 10
            free_NW = NW_points.shape[0] < 20
            free_W  = W_points.shape[0]  < 20
            free_SW = SW_points.shape[0] < 20
            free_NE = NE_points.shape[0] < 20
            free_E  = E_points.shape[0]  < 20
            free_SE = SE_points.shape[0] < 20
            #self.get_logger().info(f"Free_N: {N_points.shape[0]} Free_NW: {NW_points.shape[0]} Free_W: {W_points.shape[0]} Free_SW: {SW_points.shape[0]} Free_NE: {NE_points.shape[0]} Free_E: {E_points.shape[0]} Free_SE: {SE_points.shape[0]}")

            self.publish_bool(self.pub_obs_N,  free_N)
            self.publish_bool(self.pub_obs_NW, free_NW)
            self.publish_bool(self.pub_obs_W,  free_W)
            self.publish_bool(self.pub_obs_SW, free_SW)
            self.publish_bool(self.pub_obs_NE, free_NE)
            self.publish_bool(self.pub_obs_E,  free_E)
            self.publish_bool(self.pub_obs_SE, free_SE)

            # -------------------------------------------------
            # Obstacle distance in front
            # -------------------------------------------------
            obs_points = xyz[
                (xyz[:,0] > 2.5) & (xyz[:,0] < 100.0) &
                (xyz[:,1] < 1.0) & (xyz[:,1] > -1.0)
            ]

            if obs_points.shape[0] > 10:
                distance = numpy.linalg.norm(numpy.mean(obs_points, axis=0))
                self.pub_obs_dist.publish(Float64(data=float(distance)))

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    # =====================================================

    def publish_bool(self, publisher, value: bool):
        msg = Bool()
        msg.data = value
        publisher.publish(msg)


# =========================================================

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()

    try:
        # ---- Non-blocking ROS loop ----
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

