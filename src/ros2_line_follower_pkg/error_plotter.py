#!/usr/bin/env python3
"""Displays the error as a live graph.(taken inspiration from someone's code on github)"""

import math
import threading
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ErrorPlotter(Node):
    """Receives the signed camera error and keeps a bounded plot history."""

    def __init__(self):
        super().__init__('line_error_plotter')
        self._lock = threading.Lock()
        self._start_time = self.get_clock().now()
        self._times = deque(maxlen=600)
        self._errors = deque(maxlen=600)
        self._subscription = self.create_subscription(
            Float32, '/line_follower/error', self._error_callback, 10)

    def _error_callback(self, message):
        with self._lock:
            elapsed = (
                self.get_clock().now() - self._start_time
            ).nanoseconds / 1e9
            self._times.append(elapsed)
            self._errors.append(float(message.data))

    def snapshot(self):
        with self._lock:
            return list(self._times), list(self._errors)


def main(args=None):
    rclpy.init(args=args)
    node = ErrorPlotter()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    figure, axis = plt.subplots(figsize=(10, 5))
    figure.canvas.manager.set_window_title('Line Follower Error')
    plt.ion()

    try:
        while rclpy.ok() and plt.fignum_exists(figure.number):
            times, errors = node.snapshot()
            finite = [
                (time, error) for time, error in zip(times, errors)
                if math.isfinite(error)
            ]
            axis.clear()
            axis.axhline(0.0, color='black', linewidth=1)
            axis.axhspan(-20, 20, color='green', alpha=0.10,
                         label='Straight / small error')

            if finite:
                plot_times, plot_errors = zip(*finite)
                axis.plot(plot_times, plot_errors, color='royalblue',
                          linewidth=1.8, label='Tracking error')
                turn_points = [
                    (time, error) for time, error in finite if abs(error) >= 20
                ]
                if turn_points:
                    turn_times, turn_errors = zip(*turn_points)
                    axis.scatter(turn_times, turn_errors, color='crimson',
                                 s=14, label='Turn (|error| >= 20 px)')
                latest = finite[-1][1]
                direction = 'LEFT' if latest < -20 else (
                    'RIGHT' if latest > 20 else 'STRAIGHT'
                )
                axis.set_title(
                    f'Current error: {latest:.0f} px   |   {direction}'
                )
                axis.set_xlim(max(0.0, finite[0][0]), max(10.0, finite[-1][0]))
                limit = max(30.0, max(abs(error) for _, error in finite) * 1.2)
                axis.set_ylim(-limit, limit)
            else:
                axis.set_title('Waiting for line follower error data...')

            axis.set_xlabel('Time (seconds)')
            axis.set_ylabel('Signed deviation from center (pixels)')
            axis.grid(True, alpha=0.3)
            axis.legend(loc='upper left')
            figure.tight_layout()
            figure.canvas.draw()
            figure.canvas.flush_events()
            plt.pause(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close(figure)


if __name__ == '__main__':
    main()
