#!/usr/bin/env python3
"""
Keyboard Control node (ROS2)

Allows manual control of the self-driving car's driving actions using
the keyboard, publishing to the same topic used by the ActionPolicy node
(/BMW/policy/action), so it can be used as a drop-in substitute for
collecting human-control driving data.

Key bindings:
    e   -> cruise
    d   -> keep
    s   -> change_to_left
    f   -> change_to_right
    r   -> swerve_left
    t   -> swerve_right

    q / Ctrl-C  -> quit

Synchronization:
- Lane changes (change_to_left, change_to_right) block further input
  until /BMW/change_lane/finished is received, mirroring action_policy.py.
- All other actions (cruise, keep, swerve_*) are continuous: published
  once and re-sent every cycle until a new key is pressed, consistent
  with how action_policy.py fires-and-forgets non-lane-change actions.

Execution model:
- Explicit main loop with spin_once() for callbacks
- Raw terminal mode (setcbreak) for non-blocking single-char reads
- Fixed-rate loop at 10 Hz (same as ActionPolicy)
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import sys
import time
import select
import termios
import tty


# ---------------- QoS ----------------

qos_latched = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE
)


# ---------------- Key map ----------------
# Simple single-character keys — robust across all terminals,
# no escape-sequence parsing needed.

KEY_ACTION_MAP = {
    "e": "cruise",
    "d": "keep",
    "s": "change_to_left",
    "f": "change_to_right",
    "r": "swerve_left",
    "t": "swerve_right",
}

QUIT_KEYS = {"q", "Q", "\x03"}   # 'q', 'Q', or Ctrl-C


# ---------------- Node ----------------

class KeyboardCtrlNode(Node):

    def __init__(self):
        super().__init__('keyboard_ctrl')

        self.last_action = None
        self.executing_lane_change = False
        self.change_lane_finished = False

        # ---- Publishers (mirrors ActionPolicy node) ----
        self.pub_policy_started = self.create_publisher(
            Empty, "/BMW/policy/started", qos_latched)

        self.pub_action = self.create_publisher(
            String, "/BMW/policy/action", qos_latched)

        # ---- Subscriber: needed to clear the lane-change guard ----
        self.create_subscription(
            Bool, "/BMW/change_lane/finished",
            self.cb_change_lane_finished, 10)

        self.policy_period = 0.1   # 10 Hz — same as ActionPolicy

        self.get_logger().info("KeyboardCtrl ready")
        self._print_instructions()

    # ---------------- Callbacks ----------------

    def cb_change_lane_finished(self, msg):
        self.change_lane_finished = msg.data

    # ---------------- Helpers ----------------

    def _print_instructions(self):
        print(
            "\n"
            "==============================\n"
            " Keyboard control active\n"
            "==============================\n"
            "  e          ->  cruise\n"
            "  d          ->  keep\n"
            "  s          ->  change_to_left\n"
            "  f          ->  change_to_right\n"
            "  r          ->  swerve_left\n"
            "  t          ->  swerve_right\n"
            "  q / Ctrl-C ->  quit\n"
            "==============================\n",
            flush=True
        )

    def _publish_action(self, action):
        """Publish action. Always re-publish so the latched topic stays
        current; log only on change to avoid console spam."""
        if action != self.last_action:
            self.get_logger().info(f"Action: {action}")
            self.last_action = action

        self.pub_action.publish(String(data=action))

        if action in ("change_to_left", "change_to_right"):
            self.executing_lane_change = True

    # ---------------- Key reader ----------------

    @staticmethod
    def _read_key(fd):
        """Non-blocking read of a single character from stdin.
        Returns the character, or None if no key was pressed."""
        if select.select([fd], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    # ---------------- Main loop ----------------

    def run(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)   # single-char reads, no echo

            while rclpy.ok():

                # ---- Heartbeat ----
                self.pub_policy_started.publish(Empty())

                start = time.time()

                # ---- Process ROS callbacks ----
                rclpy.spin_once(self, timeout_sec=0.0)

                # ---- Lane-change guard ----
                # Block new commands until the maneuver is complete,
                # exactly as action_policy.py does.
                if self.executing_lane_change:
                    if self.change_lane_finished:
                        self.get_logger().info("Lane change finished")
                        self.executing_lane_change = False
                        self.change_lane_finished = False
                    self._sleep_remaining(start)
                    continue

                # ---- Read key ----
                key = self._read_key(fd)

                if key in QUIT_KEYS:
                    self.get_logger().info("Quit requested — shutting down")
                    break

                if key in KEY_ACTION_MAP:
                    self._publish_action(KEY_ACTION_MAP[key])
                elif key is not None:
                    # Unknown key: re-publish last action to keep the
                    # car doing something sensible, and hint to the user.
                    print(f"  (unknown key '{key}' — ignored)", flush=True)

                self._sleep_remaining(start)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _sleep_remaining(self, start):
        elapsed = time.time() - start
        remaining = self.policy_period - elapsed
        if remaining > 0:
            time.sleep(remaining)


# ---------------- Main ----------------

def main():
    rclpy.init()

    node = KeyboardCtrlNode()
    try:
        node.run()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass   # already shut down by ROS2 internals


if __name__ == "__main__":
    main()
