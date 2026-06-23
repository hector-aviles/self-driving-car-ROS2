#!/usr/bin/env python3
"""
This node finds lanes using a Canny edge detector and Hough transform.
Detected lines are published in normal form (rho, theta).

Extra features:
  - Displays the camera image with detected lane lines overlaid (OpenCV window)
  - Collects 500 samples of (rho_l, theta_l, rho_r, theta_r) and prints
    the averaged result in copy-paste format for behaviours.py
"""

import math
import cv2
import numpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Number of samples to collect before printing averaged parameters
N_SAMPLES = 50


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_canny_hough')

        self.bridge = CvBridge()

        # ---- Averaging accumulators ----
        self._samples      = []
        self._average_done = False

        # ---- Shared display frame (written by callback, read by main loop) ----
        self._display_frame = None

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

    # ================================================================
    # Geometry helpers
    # ================================================================

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

    # ================================================================
    # Image processing
    # ================================================================

    def detect_edges(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
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

    # ================================================================
    # Display helpers
    # ================================================================

    def draw_normal_form_line(self, img, rho, theta, color, thickness=2):
        """Draw a line given in normal form onto img (translated coords)."""
        if rho == 0.0 and theta == 0.0:
            return

        h, w = img.shape[:2]
        cx, cy = w / 2, h   # bottom-centre origin used during translation

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        pts = []
        for t in (-2000, 2000):
            x = rho * cos_t - t * sin_t
            y = rho * sin_t + t * cos_t
            img_x = int(x + cx)
            img_y = int(cy - y)
            pts.append((img_x, img_y))

        cv2.line(img, pts[0], pts[1], color, thickness)

    def overlay_lanes(self, img, rho_l, theta_l, rho_r, theta_r):
        """Return a copy of img with detected lane lines drawn on it."""
        out = img.copy()
        self.draw_normal_form_line(out, rho_l, theta_l, (0, 255, 0))   # left  — green
        self.draw_normal_form_line(out, rho_r, theta_r, (0, 0, 255))   # right — blue
        return out

    # ================================================================
    # Averaging and reporting
    # ================================================================

    def _accumulate(self, rho_l, theta_l, rho_r, theta_r):
        """Collect samples; print formatted averages when N_SAMPLES reached."""
        if self._average_done:
            return

        # Only accumulate when both lanes are detected
        if rho_l != 0.0 and rho_r != 0.0:
            self._samples.append((rho_l, theta_l, rho_r, theta_r))
            n = len(self._samples)

            if n % 50 == 0:
                self.get_logger().info(f"Collecting samples: {n}/{N_SAMPLES} ...")

            if n >= N_SAMPLES:
                arr = numpy.array(self._samples)
                avg_rho_l   = float(numpy.mean(arr[:, 0]))
                avg_theta_l = float(numpy.mean(arr[:, 1]))
                avg_rho_r   = float(numpy.mean(arr[:, 2]))
                avg_theta_r = float(numpy.mean(arr[:, 3]))

                print("\n" + "=" * 55, flush=True)
                print(f"  Averaged over {N_SAMPLES} samples — copy-paste ready:", flush=True)
                print("=" * 55, flush=True)
                print(f"            self.goal_rho_l   = {avg_rho_l:.3f}", flush=True)
                print(f"            self.goal_theta_l = {avg_theta_l:.3f}", flush=True)
                print(f"            self.goal_rho_r   = {avg_rho_r:.3f}", flush=True)
                print(f"            self.goal_theta_r = {avg_theta_r:.3f},", flush=True)
                print("=" * 55 + "\n", flush=True)

                self._average_done = True

    # ================================================================
    # ROS callback — process image, store frame for main loop display
    # ================================================================

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
                lines_t = self.translate_lines_to_bottom_center(
                    lines, img.shape[1] / 2, img.shape[0]
                )
                left, right = self.filter_lines(lines_t)
                rho_l, theta_l = self.weighted_average(left)
                rho_r, theta_r = self.weighted_average(right)
            else:
                rho_l = theta_l = rho_r = theta_r = 0.0

            # ---- Publish ----
            msg_l = Float64MultiArray()
            msg_r = Float64MultiArray()
            msg_l.data = [float(rho_l), float(theta_l)]
            msg_r.data = [float(rho_r), float(theta_r)]
            self.pub_left_lane.publish(msg_l)
            self.pub_right_lane.publish(msg_r)

            # ---- Accumulate for averaging ----
            self._accumulate(rho_l, theta_l, rho_r, theta_r)

            # ---- Build display frame (shown in main loop) ----
            display = self.overlay_lanes(img, rho_l, theta_l, rho_r, theta_r)

            if not self._average_done:
                n = len(self._samples)
                bar_w = int(display.shape[1] * n / N_SAMPLES)
                cv2.rectangle(display, (0, 0), (bar_w, 8), (0, 255, 255), -1)
                cv2.putText(
                    display,
                    f"Collecting: {n}/{N_SAMPLES}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
                )
            else:
                cv2.putText(
                    display,
                    "Done — see terminal for parameters",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )

            # Hand off to main loop — no imshow here
            self._display_frame = display

        except Exception as e:
            self.get_logger().error(f"Lane detection error: {e}")

    def update(self):
        """Called from the main loop — handles OpenCV window updates."""
        if self._display_frame is not None:
            cv2.imshow("Lane Detector", self._display_frame)
            cv2.waitKey(1)


# ================================================================
# MAIN
# ================================================================

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update()   # OpenCV window refresh happens here

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
