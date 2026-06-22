#!/usr/bin/env python3
"""
Behaviors node (ROS2)

Implements driving behaviors as a finite state machine:
  - Lane tracking with proportional control (cruise / keep)
  - Lane changes (change_to_left, change_to_right)
  - Evasive swerves (swerve_left, swerve_right) — handled by shifting the
    proportional-control goal parameters inside cb_action; no FSM states needed.
  - Abort/undo of an in-progress lane change when a collision is detected
    (SM_UNDOING_TURN), activated by the counterfactuals + latent_collision flags.

Actions are received on /BMW/policy/action (String).
Lane changes publish /BMW/change_lane/finished (Bool=True) on completion/abort.

Swerve strategy (ported from ROS1):
  Instead of steering the vehicle directly, swerve_left / swerve_right simply
  shift the goal (rho, theta) parameters of the proportional lane controller so
  the car naturally drifts within its lane. The nominal parameters are restored
  on the next cruise / keep / change action.
"""

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, Bool, String
from geometry_msgs.msg import Pose2D
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


# ------------------------------------------------------------------ QoS -----

qos_latched = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# ----------------------------------------------------------- FSM states -----

SM_INIT                 = 0
SM_WAITING_FOR_NEW_TASK = 10
SM_CRUISE               = 20
SM_KEEP                 = 30
SM_CHANGE_LEFT_1        = 40
SM_CHANGE_LEFT_2        = 45
SM_CHANGE_RIGHT_1       = 50
SM_CHANGE_RIGHT_2       = 55
SM_SWERVE_LEFT          = 60
SM_SWERVE_RIGHT         = 70
SM_UNDOING_TURN         = 210

MAX_STEERING = 0.5


# ================================================================ Node ======

class BehaviorsNode(Node):

    def __init__(self):
        super().__init__('behaviors')

        # ------------------------------------------------ Control params ----
        self.k_rho        = 0.001
        self.k_theta      = 0.01
        self.max_speed    = 30.0
        # HHAA
        '''
        self.k_keeping    = 10.0
        self.desired_dist = 30.0   # desired gap to the car ahead (m)
        '''
        self.k_keeping    = 3.0
        self.desired_dist = 15.0   # desired gap to the car ahead (m)
                 
        # Nominal (centre-of-lane) goal parameters — restored after swerves
        self.nominal_params = dict(
            k_rho        = 0.001,
            k_theta      = 0.01,
            max_speed    = 30.0,
            goal_rho_l   = 481.0,
            goal_theta_l = 2.085,
            goal_rho_r   = 466.0,
            goal_theta_r = 0.99,
        )
        self.set_nominal_params()

        # ------------------------------------------- Perception / state ----
        self.lane_rho_l    = 0.0
        self.lane_theta_l  = 0.0
        self.lane_rho_r    = 0.0
        self.lane_theta_r  = 0.0

        self.measured_dist     = None   # None → no vehicle detected ahead
        self.current_x         = 0.0
        self.current_y         = 0.0
        self.current_a         = 0.0
        self.latent_collision  = False
        self.counterfactuals   = False

        # ---------------------------------------------------- FSM state ----
        self.state              = SM_INIT
        self.finished_published = False

        # Actuator outputs (updated each cycle)
        self.speed    = 0.0
        self.steering = 0.0

        # ----------------------------------------------------- Subscribers --
        self.create_subscription(
            String, "/BMW/policy/action", self.cb_action, qos_latched)
        self.create_subscription(
            Float64MultiArray, "/BMW/demo/left_lane", self.cb_left_lane, 10)
        self.create_subscription(
            Float64MultiArray, "/BMW/demo/right_lane", self.cb_right_lane, 10)
        self.create_subscription(
            Float64, "/BMW/obstacle_distance", self.cb_dist, 10)
        self.create_subscription(
            Pose2D, "/BMW/pose", self.cb_pose, 10)
        self.create_subscription(
            Bool, "/BMW/latent_collision", self.cb_latent_collision, 10)
        self.create_subscription(
            Bool, "/BMW/counterfactuals", self.cb_counterfactuals, qos_latched)

        # ------------------------------------------------------ Publishers --
        self.pub_speed    = self.create_publisher(Float64, "/BMW/speed",    1)
        self.pub_steering = self.create_publisher(Float64, "/BMW/steering", 1)
        self.pub_finished = self.create_publisher(
            Bool, "/BMW/change_lane/finished", 1)

        self.period = 0.1   # 10 Hz

        self.get_logger().info("Behaviors node initialized")

    # ============================================= Parameter helpers =========

    def set_nominal_params(self):
        p = self.nominal_params
        self.k_rho        = p["k_rho"]
        self.k_theta      = p["k_theta"]
        self.max_speed    = p["max_speed"]
        self.goal_rho_l   = p["goal_rho_l"]
        self.goal_theta_l = p["goal_theta_l"]
        self.goal_rho_r   = p["goal_rho_r"]
        self.goal_theta_r = p["goal_theta_r"]

    # ================================================== Callbacks ===========

    def cb_action(self, msg):
        action = msg.data
        self.get_logger().info(f"Action received: {action}")

        if action == "cruise":
            # Restore nominal goals and switch to lane-tracking mode
            self.set_nominal_params()
            self.state = SM_CRUISE

        elif action == "keep":
            # Restore nominal goals and switch to car-following mode
            self.set_nominal_params()
            self.state = SM_KEEP

        elif action == "swerve_left":
            # Shift goal parameters so the proportional controller steers the
            # car gently to the left within its lane — no FSM state change.
            # The current cruise / keep state keeps running with the new goals.
            self.max_speed    = 30.0
            self.goal_rho_l   = 385.0
            self.goal_theta_l = 2.37
            self.goal_rho_r   = 508.0
            self.goal_theta_r = 1.16
            self.state = SM_SWERVE_LEFT
            #self.get_logger().info("Swerve left: goal parameters updated")

        elif action == "swerve_right":
            # Same idea — shift goals to the right
            self.max_speed    = 30.0
            self.goal_rho_l   = 514.0
            self.goal_theta_l = 1.93
            self.goal_rho_r   = 300.0
            self.goal_theta_r = 0.57
            self.state = SM_SWERVE_RIGHT            
            #self.get_logger().info("Swerve right: goal parameters updated")

        elif action == "change_to_left":
            self.set_nominal_params()
            self.state = SM_CHANGE_LEFT_1
            self.get_logger().info("Starting change lane to left")

        elif action == "change_to_right":
            self.set_nominal_params()
            self.state = SM_CHANGE_RIGHT_1
            self.get_logger().info("Starting change lane to right")

        else:
            self.get_logger().warn(f"Unknown action: '{action}'")

    def cb_left_lane(self, msg):
        self.lane_rho_l, self.lane_theta_l = msg.data

    def cb_right_lane(self, msg):
        self.lane_rho_r, self.lane_theta_r = msg.data

    def cb_dist(self, msg):
        # inf means no obstacle detected (sentinel from obstacle_detector)
        d = msg.data
        self.measured_dist = None if math.isinf(d) else d

    def cb_pose(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_a = msg.theta

    def cb_latent_collision(self, msg):
        self.latent_collision = msg.data

    def cb_counterfactuals(self, msg):
        self.counterfactuals = msg.data
        self.get_logger().info(f"Counterfactuals = {msg.data}")

    # ============================================= Control helpers ===========

    def calculate_control(self, dist=None):
        """
        Proportional lane-tracking control.

        Handles the case where only one lane line is detected:
          - Both lines detected → average the two errors
          - Only left detected  → use left error alone
          - Only right detected → use right error alone

        Speed is reduced proportionally to steering magnitude so the car
        slows in curves and runs at max speed on straights.
        When dist is provided (car-following), speed is adjusted to maintain
        the desired gap; otherwise lane-tracking speed formula is used.
        """
        rho_l, theta_l = self.lane_rho_l, self.lane_theta_l
        rho_r, theta_r = self.lane_rho_r, self.lane_theta_r

        er_l = self.goal_rho_l   - rho_l
        et_l = self.goal_theta_l - theta_l
        er_r = rho_r - self.goal_rho_r
        et_r = theta_r - self.goal_theta_r

        if rho_l != 0 and rho_r != 0:
            er = (er_l + er_r) / 2.0
            et = (et_l + et_r) / 2.0
        elif rho_l != 0:
            er, et = er_l, et_l
        else:
            er, et = er_r, et_r

        steering = -self.k_rho * er - self.k_theta * et

        if dist is None:
            speed = self.max_speed * (1.0 - 1.5 * abs(steering))
        else:
            speed = self.max_speed + self.k_keeping * (dist - self.desired_dist)
            # HHAA 
            #speed = max(min(speed, self.max_speed), -10.0)
            speed = max(min(speed, self.max_speed), 0.0)

        return speed, steering

    def turning_steering(self, w, L, v):
        """
        Compute steering angle from desired angular velocity w (rad/s),
        wheelbase L (m), and current speed v (m/s).
        Kinematic model: w = v * sin(delta) / L
        """
        if v == 0:
            return 0.0
        k = max(min(L * w / v, 0.5), -0.5)
        return max(min(math.asin(k), MAX_STEERING), -MAX_STEERING)

    # ======================================= Maneuver completion helper ======

    def finish_maneuver(self, label="Maneuver"):
        """Publish the finished signal once and return to idle."""
        if not self.finished_published:
            self.pub_finished.publish(Bool(data=True))
            self.finished_published = True
            self.get_logger().info(f"{label} finished")
        self.state = SM_WAITING_FOR_NEW_TASK

    # ====================================================== FSM =============

    def main_loop(self):

        self.speed    = 0.0
        self.steering = 0.0

        # --------------------------------------------------------------------
        if self.state == SM_INIT:
            self.state = SM_WAITING_FOR_NEW_TASK

        # --------------------------------------------------------------------
        elif self.state == SM_WAITING_FOR_NEW_TASK:
            self.finished_published = False
            # Sit idle (speed=0, steering=0) until cb_action fires

        # --------------------------------------------------------------------
        # CRUISE — continuous lane tracking at max speed
        # Swerve actions shift the goal params while remaining in this state.
        # Nominal params are restored when the next cruise/keep/change arrives.
        # --------------------------------------------------------------------
        elif self.state == SM_CRUISE:
            self.speed, self.steering = self.calculate_control()

        # --------------------------------------------------------------------
        # KEEP — car following with distance control.
        # When no vehicle is detected ahead, measured_dist is None and
        # calculate_control falls back to cruise speed formula automatically.
        # Swerve actions shift goal params while remaining in this state.
        # --------------------------------------------------------------------
        elif self.state == SM_KEEP:
            self.speed, self.steering = self.calculate_control(
                dist=self.measured_dist)

        # --------------------------------------------------------------------
        # CHANGE TO LEFT
        # Phase 1: steer left until crossing lane midpoint (y > −0.7)
        # --------------------------------------------------------------------
        elif self.state == SM_CHANGE_LEFT_1:
            
            if self.speed <= 10.0:
                self.speed = self.max_speed
            self.steering = self.turning_steering(1.2, 2.9, self.speed)
            # Right lane y ≈ −1.5; centre ≈ 0 → threshold at −0.7
            if self.current_y > -0.7:
                self.state = SM_CHANGE_LEFT_2

        # Phase 2: counter-steer to align with left lane (y ≈ +1.5)
        elif self.state == SM_CHANGE_LEFT_2:
            if self.speed <= 10.0:
                self.speed = self.max_speed
            self.steering = self.turning_steering(-1.2, 2.9, self.speed)
            if self.current_y > 1.0 and abs(self.current_a) < 0.2:
                self.finish_maneuver("Change lane to left")

        # --------------------------------------------------------------------
        # CHANGE TO RIGHT
        # Phase 1: steer right until crossing lane midpoint (y < +0.7)
        # --------------------------------------------------------------------
        elif self.state == SM_CHANGE_RIGHT_1:
            if self.speed <= 10.0:
                self.speed = self.max_speed
            self.steering = self.turning_steering(-1.2, 2.9, self.speed)
            # Left lane y ≈ +1.5; centre ≈ 0 → threshold at +0.7
            if self.current_y < 0.7:
                self.state = SM_CHANGE_RIGHT_2

        # Phase 2: counter-steer to align with right lane (y ≈ −1.5)
        elif self.state == SM_CHANGE_RIGHT_2:
            if self.speed <= 10.0:
                self.speed = self.max_speed
            self.steering = self.turning_steering(1.2, 2.9, self.speed)
            if self.current_y < -1.0 and abs(self.current_a) < 0.2:
                self.finish_maneuver("Change lane to right")
  
           
        # ------------------------------------------------
        # SWERVE RIGHT — gentle evasive veer to the right
        # ------------------------------------------------
        elif self.state == SM_SWERVE_RIGHT:
            self.speed, self.steering = self.calculate_control()
           
        # ------------------------------------------------
        # SWERVE LEFT — gentle evasive veer to the left
        # ------------------------------------------------           
        elif self.state == SM_SWERVE_LEFT:
            self.speed, self.steering = self.calculate_control()

        # --------------------------------------------------------------------
        # UNDOING TURN — abort an in-progress lane change by steering back
        # to align with the original heading (|current_a| → 0)
        # --------------------------------------------------------------------
        elif self.state == SM_UNDOING_TURN:
            if self.speed <= 10.0:
                self.speed = self.max_speed
            w = 1.2 if self.current_a < 0.0 else -1.2
            self.steering = self.turning_steering(w, 2.9, self.speed)
            if abs(self.current_a) < 0.1:
                self.set_nominal_params()
                self.finish_maneuver("Undo turn")

        else:
            self.get_logger().warn(f"Unknown FSM state: {self.state}")

        # --------------------------------------------------------------------
        # Counterfactual abort: if a lane change has just started (phase 1
        # only) and a collision is detected while counterfactual reasoning
        # is active, abort back to the original lane.
        # --------------------------------------------------------------------
        if (self.counterfactuals
                and self.state in (SM_CHANGE_LEFT_1, SM_CHANGE_RIGHT_1)
                and self.latent_collision):
            self.get_logger().info(
                f"Collision detected during lane change — aborting "
                f"(state={self.state})")
            self.state = SM_UNDOING_TURN

        # --------------------------------------------------------------------
        # Publish actuator commands every cycle
        # --------------------------------------------------------------------
        self.pub_speed.publish(Float64(data=float(self.speed)))
        self.pub_steering.publish(Float64(data=float(self.steering)))
        
        '''
        self.get_logger().debug(
            f"self.state — speed={self.speed:.1f}  steering={self.steering:.3f}"
            f"  dist={self.measured_dist}")
        '''    

    # ================================================= Explicit main loop ===

    def run(self):
        while rclpy.ok():
            start = time.time()
            rclpy.spin_once(self, timeout_sec=0.01)
            self.main_loop()
            remaining = self.period - (time.time() - start)
            if remaining > 0:
                time.sleep(remaining)


# ================================================================ Main ======

def main():
    rclpy.init()
    node = BehaviorsNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
