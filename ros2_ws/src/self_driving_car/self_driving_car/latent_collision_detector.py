#!/usr/bin/env python3
"""
ROS 2 Jazzy node that detects potential front or sideswipe collisions
using radar readings and the current state–action pair.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Bool, Float64, String, Empty
from radar_msgs.msg import RadarScan


class LatentCollisionDetector(Node):

    def __init__(self):
        super().__init__('latent_collision_detector')

        # -------------------------------------------------
        # Use simulation time (Webots)
        # -------------------------------------------------
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------
        self.declare_parameter('transversal', False)
        self.transversal = self.get_parameter(
            'transversal').get_parameter_value().bool_value

        self.get_logger().info(f"Is transversal? {self.transversal}")

        # -------------------------------------------------
        # State variables
        # -------------------------------------------------
        self.free_N  = True
        self.free_NW = True
        self.free_W  = True
        self.free_NE = True
        self.free_E  = True
        self.free_SE = True
        self.free_SW = True
        self.curr_lane = True

        self.action = "NA"
        self.sdc_vel_kh = 0.0
        self.policy_started = False

        self.radar_scan = RadarScan()

        # -------------------------------------------------
        # Radar parameters
        # -------------------------------------------------
        self.min_dist_radar_ref = 1.0
        self.max_dist_radar_ref = 45.0
        self.valid_dist = 25.0

        self.radar_ref = {
            "min_dist": 1.0,
            "max_dist": 10.0,
            "max_fov": math.atan2(3, self.min_dist_radar_ref),
            "min_fov": math.atan2(3, self.max_dist_radar_ref),
            "fov": math.atan2(3, self.max_dist_radar_ref)
        }

        # -------------------------------------------------
        # Subscribers
        # -------------------------------------------------
        self.create_subscription(Bool, "/BMW/current_lane",
                                 self.cb("curr_lane"), 10)

        self.create_subscription(Float64, "/BMW/speed",
                                 self.cb("sdc_vel_kh"), 10)

        self.create_subscription(Bool, "/BMW/free_N",  self.cb("free_N"), 10)
        self.create_subscription(Bool, "/BMW/free_NW", self.cb("free_NW"), 10)
        self.create_subscription(Bool, "/BMW/free_W",  self.cb("free_W"), 10)
        self.create_subscription(Bool, "/BMW/free_NE", self.cb("free_NE"), 10)
        self.create_subscription(Bool, "/BMW/free_E",  self.cb("free_E"), 10)
        self.create_subscription(Bool, "/BMW/free_SE", self.cb("free_SE"), 10)
        self.create_subscription(Bool, "/BMW/free_SW", self.cb("free_SW"), 10)

        self.create_subscription(String, "/BMW/action",
                                 self.cb("action"), 10)

        self.create_subscription(RadarScan, "/BMW/frontal_radar",
                                 self.cb_radar, 10)

        self.create_subscription(
            Empty, "/BMW/policy/started",
            self.cb_policy_started, 10
        )

        # -------------------------------------------------
        # Publishers
        # -------------------------------------------------
        self.pub_latent_collision = self.create_publisher(
            Bool, "/BMW/latent_collision", 10)

        self.pub_type_latent_collision = self.create_publisher(
            Float64, "/BMW/latent_collision/type", 10)

        self.get_logger().info("Latent collision detector initialized")

    # =====================================================
    # Callbacks
    # =====================================================

    def cb(self, name):
        def _cb(msg):
            setattr(self, name, msg.data)
        return _cb

    def cb_radar(self, msg):
        self.radar_scan = msg

    def cb_policy_started(self, _):
        self.policy_started = True

    # =====================================================
    # Main logic (called manually)
    # =====================================================

    def main_loop(self):

        if not self.policy_started:
            return

        latent_collision = False
        type_latent_collision = 0.0

        obst_dist = float("inf")
        closest_obst = None

        # -------------------------------------------------
        # Patch (same as ROS1 logic)
        # -------------------------------------------------
        if self.curr_lane:
            self.free_NE = self.free_N
        else:
            self.free_NW = self.free_N

        # -------------------------------------------------
        # Adjust FoV if swerving
        # -------------------------------------------------
        if self.action in ("swerve_left", "swerve_right") or self.transversal:
            self.radar_ref["fov"] = 0.8725
        else:
            self.radar_ref["fov"] = self.radar_ref["min_fov"]

        # -------------------------------------------------
        # Find closest obstacle in front
        # -------------------------------------------------
        for r in self.radar_scan.returns:
            if (-self.radar_ref["fov"] <= r.azimuth <= self.radar_ref["fov"]
                    and r.range <= self.valid_dist):
                if r.range < obst_dist:
                    obst_dist = r.range
                    closest_obst = r

        # -------------------------------------------------
        # Lane-change collision logic
        # -------------------------------------------------
        if self.action == 'change_to_right' and self.curr_lane:
            latent_collision = True
            type_latent_collision = 1.0

        elif self.action == 'change_to_left' and not self.curr_lane:
            latent_collision = True
            type_latent_collision = 2.0

        elif self.action == 'change_to_left' and self.curr_lane and (
                not self.free_NW or not self.free_W):
            latent_collision = True
            type_latent_collision = 3.0

        elif self.action == 'change_to_right' and not self.curr_lane and (
                not self.free_NE or not self.free_E):
            latent_collision = True
            type_latent_collision = 4.0

        # -------------------------------------------------
        # Rear-end collision logic
        # -------------------------------------------------
        elif closest_obst is not None:

            latent_collision = True

            sdc_vel_ms = (self.sdc_vel_kh * 1000.0) / 3600.0
            obst_vel = sdc_vel_ms + closest_obst.doppler_velocity

            if -1.0 < obst_vel <= 1.0:
                type_latent_collision = 5.0
            elif obst_vel < -1.0:
                type_latent_collision = 6.0
            else:
                type_latent_collision = 7.0

        # -------------------------------------------------
        # Publish results
        # -------------------------------------------------
        self.pub_latent_collision.publish(
            Bool(data=latent_collision))

        self.pub_type_latent_collision.publish(
            Float64(data=type_latent_collision))


# =========================================================

def main(args=None):
    rclpy.init(args=args)
    node = LatentCollisionDetector()

    try:
        # ---- Non-blocking ROS loop ----
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.main_loop()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

