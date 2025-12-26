#!/usr/bin/env python3
"""
Lane Identification Node (ROS 2 Jazzy)
Determines whether the vehicle is in the right lane (True) or left lane (False)
based on the sign of the y-coordinate in the local frame.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D


class LaneIdentificationNode(Node):
    def __init__(self):
        super().__init__('lane_identification')

        # State
        self.right_lane = True  # Default assumption

        # Subscriber
        self.create_subscription(
            Pose2D,
            '/BMW/pose',
            self.pose_callback,
            10
        )

        # Publisher
        self.pub_right_lane = self.create_publisher(
            Bool,
            '/BMW/current_lane',
            10
        )

        self.get_logger().info('Lane Identification Node initialized.')

    def pose_callback(self, msg: Pose2D):
        y = msg.y

        # y <= 0 → right lane
        # y > 0  → left lane
        new_right_lane = y <= 0.0

        if new_right_lane != self.right_lane:
            self.right_lane = new_right_lane

            msg_out = Bool()
            msg_out.data = self.right_lane
            self.pub_right_lane.publish(msg_out)

            lane_str = "RIGHT lane" if self.right_lane else "LEFT lane"
            #self.get_logger().info(f'Vehicle is now in the {lane_str}')

    # No periodic logic needed, but kept for consistency
    def update(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = LaneIdentificationNode()

    try:
        while rclpy.ok():
            # Process incoming Pose2D messages
            rclpy.spin_once(node, timeout_sec=0.01)

            # No periodic computation required
            node.update()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

