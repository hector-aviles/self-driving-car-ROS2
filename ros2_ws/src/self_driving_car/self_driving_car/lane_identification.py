#!/usr/bin/env python3
"""
Lane Identification Node (ROS 2 Jazzy)
Determines whether the vehicle is in the right lane (True) or left lane (False)
based on the sign of the y-coordinate in the local frame.
Assumes: positive y = left lane, negative y = right lane (common in right-hand traffic)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D


class LaneIdentificationNode(Node):
    def __init__(self):
        super().__init__('lane_identification')

        # Parameters (could be made configurable later)
        self.right_lane = True  # Default assumption

        # Subscribers
        self.create_subscription(
            Pose2D,
            '/bmw_pose',
            self.pose_callback,
            10)

        # Publishers
        self.pub_right_lane = self.create_publisher(Bool, '/current_lane', 10)

        self.get_logger().info('Lane Identification Node has been started.')

    def pose_callback(self, msg: Pose2D):
        y = msg.y

        # Logic: in most right-hand drive conventions with +x forward:
        # y > 0 → left side of the road → left lane
        # y < 0 → right side of the road → right lane
        new_right_lane = y <= 0.0

        # Only publish if the lane changes (optional optimization)
        if new_right_lane != self.right_lane:
            self.right_lane = new_right_lane
            bool_msg = Bool()
            bool_msg.data = self.right_lane
            self.pub_right_lane.publish(bool_msg)
            lane_str = "RIGHT lane" if self.right_lane else "LEFT lane"
            self.get_logger().info(f'Vehicle is now in the {lane_str}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LaneIdentificationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Clean shutdown on Ctrl+C
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
