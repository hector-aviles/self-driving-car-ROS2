#!/usr/bin/env python3
"""
This node finds lanes using a Canny edge detector and Hough transform.
Detected lines are published in normal form (rho, theta).
"""

import math
import cv2
import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_canny_hough')

        self.bridge = CvBridge()

        print("INITIALIZING LANE DETECTION NODE...", flush=True)

        # Subscriber
        self.create_subscription(
            Image,
            '/BMW/frontal_camera/rgb/raw',
            self.callback_rgb_image,
            10
        )

        # Publishers
        self.pub_left_lane = self.create_publisher(
            Float64MultiArray,
            "/BMW/demo/left_lane",
            1
        )
        self.pub_right_lane = self.create_publisher(
            Float64MultiArray,
            "/BMW/demo/right_lane",
            1
        )

        self.get_logger().info("Lane detector initialized")

    # ---------------- Geometry helpers ----------------

    def to_normal_form(self, x1, y1, x2, y2):
        A = y2 - y1
        B = x1 - x2
        C = A * x1 + B * y1
        theta = math.atan2(B, A)
        rho = C / math.sqrt(A * A + B * B)
        if rho < 0:
            rho = -rho
            theta += math.pi
        return numpy.asarray([rho, theta])

    def translate_lines_to_bottom_center(self, lines, x_center, y_center):
        if lines is None:
            return None
        out = []
        for x1, y1, x2, y2 in lines:
            out.append([
                x1 - x_center,
                y_center - y1,
                x2 - x_center,
                y_center - y2
            ])
        return out

    # ---------------- Image processing ----------------

    def detect_edges(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        return cv2.Canny(blur, 50, 150)

    def filter_lines(self, lines):
        left, right = [], []
        for x1, y1, x2, y2 in lines:
            rho, theta = self.to_normal_form(x1, y1, x2, y2)

            if (-math.pi/2 + 0.3) < theta < -0.1 or (0.1 < theta < math.pi/2 - 0.3):
                right.append([x1, y1, x2, y2])

            if (math.pi/2 + 0.3) < abs(theta) < math.pi * 0.9:
                left.append([x1, y1, x2, y2])

        return left or None, right or None

    def weighted_average(self, lines):
        if not lines:
            return 0.0, 0.0

        weights = numpy.asarray([
            math.hypot(x2 - x1, y2 - y1) for x1, y1, x2, y2 in lines
        ])
        weights /= weights.sum()

        rho, theta = 0.0, 0.0
        for w, (x1, y1, x2, y2) in zip(weights, lines):
            r, t = self.to_normal_form(x1, y1, x2, y2)
            rho += w * r
            theta += w * t

        return rho, theta

    # ---------------- ROS callback ----------------

    def callback_rgb_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Crop lower part
            img = img[int(0.4 * img.shape[0]):int(0.97 * img.shape[0]), :, :]

            edges = self.detect_edges(img)
            lines = cv2.HoughLinesP(
                edges, 2, numpy.pi / 180, 80,
                minLineLength=80, maxLineGap=100
            )

            if lines is not None:
                lines = lines[:, 0]
                lines = self.translate_lines_to_bottom_center(
                    lines, img.shape[1] / 2, img.shape[0]
                )
                left, right = self.filter_lines(lines)
                rho_l, theta_l = self.weighted_average(left)
                rho_r, theta_r = self.weighted_average(right)
            else:
                rho_l = theta_l = rho_r = theta_r = 0.0

            msg_l = Float64MultiArray()
            msg_r = Float64MultiArray()
            msg_l.data = [float(rho_l), float(theta_l)]
            msg_r.data = [float(rho_r), float(theta_r)]

            self.pub_left_lane.publish(msg_l)
            self.pub_right_lane.publish(msg_r)

        except Exception as e:
            self.get_logger().error(f"Lane detection error: {e}")

    def update(self):
        """No periodic logic needed."""
        pass


# ---------------- MAIN ----------------

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()

    try:
        while rclpy.ok():
            # Non-blocking processing of image callbacks
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

