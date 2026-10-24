#!/usr/bin/env python3
# Copyright 2026 Wuji Technology
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


NUM_JOINTS = 20
PUBLISH_RATE_HZ = 100.0
WAVE_FREQUENCY_HZ = 0.5
WAVE_AMPLITUDE_RAD = 0.5
FLEX_JOINT_INDICES = (4, 6, 7, 8, 10, 11, 12, 14, 15, 16, 18, 19)
STARTUP_ZERO_DURATION_SEC = 1.0


class Hand2WaveDemo(Node):

    def __init__(self):
        super().__init__('hand2_wave_demo')
        self.publisher = self.create_publisher(
            JointState, 'joint_commands', qos_profile_sensor_data
        )
        self.command = JointState()
        self.command.position = [0.0] * NUM_JOINTS
        self.started_at = time.monotonic()
        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ, self.publish_command
        )

    def publish_command(self):
        elapsed = time.monotonic() - self.started_at
        wave_elapsed = max(0.0, elapsed - STARTUP_ZERO_DURATION_SEC)
        phase = 2.0 * math.pi * WAVE_FREQUENCY_HZ * wave_elapsed
        position = (1.0 - math.cos(phase)) * WAVE_AMPLITUDE_RAD

        for index in FLEX_JOINT_INDICES:
            self.command.position[index] = position

        self.command.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.command)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = Hand2WaveDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
