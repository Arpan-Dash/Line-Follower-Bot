#!/usr/bin/env python3
"""
Line Follower — TurtleBot3 Waffle Pi camera-based line tracking.

Subscribes to the camera image, detects a dark line on a light tiled floor
using brightness thresholding, computes the line centroid offset from the
image center, and publishes PD steering commands to cmd_vel.

Pipeline:
  1. Receive camera frame from /camera/image
  2. Crop to bottom portion (where the line is closest to robot)
  3. Convert to grayscale, threshold for dark pixels (the line)
  4. Compute centroid x-offset from image center
  5. PD controller: angular_z = -Kp * error - Kd * derivative
  6. Publish Twist to /cmd_vel
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

# ── Tuning parameters used by me──────────────────────────────────────────────
LINEAR_SPEED = 0.24        # m/s forward speed on straight sections
KP = 0.007                 # proportional gain
KD = 0.003                 # derivative gain for turn anticipation
MAX_ANGULAR = 1.8          # rad/s max turn rate
SEARCH_ANGULAR = 0.4       # rad/s spin when line lost
ERROR_FILTER_ALPHA = 0.35  # smooth camera noise without adding much delay

DARK_THRESHOLD = 80        # pixels darker than this = line
MIN_LINE_PIXELS = 200      # minimum dark pixels to count as "line found"
CROP_TOP_RATIO = 0.45      # keep the center and lower view where the line appears


class LineFollower(Node):
    """Follows a dark line on a light floor using camera and PD control."""

    def __init__(self):
        super().__init__('line_follower')

        self._img_sub = self.create_subscription(
            Image, '/camera/image', self._image_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._error_pub = self.create_publisher(Float32, '/line_follower/error', 10)

        self._prev_error = 0.0
        self._filtered_error = 0.0
        self._line_lost_count = 0
        self._error_sum = 0.0
        self._error_samples = 0
        self._max_deviation = 0.0

        self.get_logger().info('Line follower started — waiting for camera...')

    def _image_cb(self, msg):
        h, w = msg.height, msg.width

        # Convert ROS Image to numpy array
        if msg.encoding in ('rgb8', 'RGB8'):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
        elif msg.encoding in ('bgr8', 'BGR8'):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            img = img[:, :, ::-1]  # BGR -> RGB
        else:
            self.get_logger().warn(
                f'Unsupported encoding: {msg.encoding}',
                throttle_duration_sec=5.0)
            return

        # Crop bottom portion (where the line is nearest)
        crop_y = int(h * CROP_TOP_RATIO)
        roi = img[crop_y:, :]
        roi_h, roi_w = roi.shape[:2]

        # Convert to grayscale
        gray = (0.299 * roi[:, :, 0].astype(np.float32) +
                0.587 * roi[:, :, 1].astype(np.float32) +
                0.114 * roi[:, :, 2].astype(np.float32)).astype(np.uint8)

        # Threshold: dark pixels = line
        mask = (gray < DARK_THRESHOLD).astype(np.uint8)

        # Find dark pixel locations
        ys, xs = np.where(mask > 0)

        twist = Twist()

        if len(xs) >= MIN_LINE_PIXELS:
            # Line found — compute centroid offset
            centroid_x = np.mean(xs)
            center_x = roi_w / 2.0
            error = centroid_x - center_x
            self._filtered_error = (
                ERROR_FILTER_ALPHA * error
                + (1.0 - ERROR_FILTER_ALPHA) * self._filtered_error)
            self._error_pub.publish(Float32(data=float(error)))

            # PD controller
            derivative = self._filtered_error - self._prev_error
            angular = -KP * self._filtered_error - KD * derivative
            angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

            deviation = abs(error)
            deviation_percent = 100.0 * deviation / center_x
            self._error_sum += deviation
            self._error_samples += 1
            self._max_deviation = max(self._max_deviation, deviation)
            average_deviation = self._error_sum / self._error_samples

            # Use the camera error to slow before sharp turns, reducing
            # overshoot while preserving full speed on straight sections.
            error_ratio = min(1.0, abs(self._filtered_error) / center_x)
            speed_factor = max(0.30, 1.0 - 0.70 * error_ratio)
            twist.linear.x = LINEAR_SPEED * speed_factor
            twist.angular.z = angular

            self._prev_error = self._filtered_error
            self._line_lost_count = 0

            if abs(error) < 20:
                state = 'STRAIGHT'
            elif error > 0:
                state = 'TURN RIGHT'
            else:
                state = 'TURN LEFT'

            self.get_logger().info(
                f'{state}: offset={error:.0f}px  '
                f'deviation={deviation:.0f}px ({deviation_percent:.1f}%)  '
                f'avg={average_deviation:.0f}px  max={self._max_deviation:.0f}px  '
                f'v={twist.linear.x:.2f}  w={twist.angular.z:.2f}',
                throttle_duration_sec=0.5)
        else:
            # Line lost — search by spinning
            self._line_lost_count += 1
            self._error_pub.publish(Float32(data=float('nan')))
            twist.linear.x = 0.0

            if self._prev_error > 0:
                twist.angular.z = -SEARCH_ANGULAR
            else:
                twist.angular.z = SEARCH_ANGULAR

            self.get_logger().warn(
                f'LINE LOST (frame {self._line_lost_count}) — searching...',
                throttle_duration_sec=1.0)

        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node._cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
