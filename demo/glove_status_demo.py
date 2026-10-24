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

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args

from sensor_msgs.msg import Imu, PointCloud2

from wuji_msgs.msg import (
    HandJointAngles,
    TactileGrid,
    TactileZoneArray,
    TrackedPoseArray,
)


class TopicSpec:

    def __init__(self, topic, message_type):
        self.topic = topic
        self.message_type = message_type


TOPIC_SPECS = (
    TopicSpec('joint_angles', HandJointAngles),
    TopicSpec('emf_poses', TrackedPoseArray),
    TopicSpec('tip_poses', TrackedPoseArray),
    TopicSpec('hand_skeleton', TrackedPoseArray),
    TopicSpec('imu/palm', Imu),
    TopicSpec('tactile', TactileGrid),
    TopicSpec('tactile/residual', TactileGrid),
    TopicSpec('tactile/binary', TactileGrid),
    TopicSpec('tactile/zones', TactileZoneArray),
    TopicSpec('tactile/points', PointCloud2),
)


class StatusTracker:

    def __init__(
        self,
        output=None,
        clock=time.monotonic,
        print_joint_angles=True,
    ):
        self.output = output if output is not None else sys.stdout
        self.clock = clock
        self.print_joint_angles = print_joint_angles
        self.frame_counts = {spec.topic: 0 for spec in TOPIC_SPECS}
        self.first_frame_times = {spec.topic: None for spec in TOPIC_SPECS}
        self.last_frame_times = {spec.topic: None for spec in TOPIC_SPECS}

    def _print(self, text):
        print(text, file=self.output, flush=True)

    def record(self, topic, message):
        received_at = self.clock()
        if self.frame_counts[topic] == 0:
            self.first_frame_times[topic] = received_at
        self.last_frame_times[topic] = received_at
        self.frame_counts[topic] += 1
        if topic != 'joint_angles' or not self.print_joint_angles:
            return

        for finger in message.fingers:
            self._print(
                f'{finger.name} angles={list(finger.angles)} '
                f'confidence={finger.confidence}'
            )

    def report(self):
        for spec in TOPIC_SPECS:
            topic = spec.topic
            count = self.frame_counts[topic]
            rate = 0.0
            if count > 1:
                elapsed = max(
                    self.last_frame_times[topic]
                    - self.first_frame_times[topic],
                    sys.float_info.epsilon,
                )
                rate = (count - 1) / elapsed
            self._print(
                f'{topic}: frames={count} '
                f'receive_rate_hz={rate:.1f}'
            )

        if self.frame_counts['joint_angles'] == 0:
            self._print('error: no frames received on joint_angles')
            exit_code = 1
        else:
            exit_code = 0

        for spec in TOPIC_SPECS[1:]:
            if self.frame_counts[spec.topic] == 0:
                self._print(
                    f'warning: no frames received on {spec.topic}'
                )
        return exit_code


class GloveStatusDemo(Node):

    def __init__(self, print_joint_angles=True):
        super().__init__('glove_status_demo')
        self.tracker = StatusTracker(
            print_joint_angles=print_joint_angles,
        )
        self._subscriptions = [
            self.create_subscription(
                spec.message_type,
                spec.topic,
                self._callback(spec.topic),
                qos_profile_sensor_data,
            )
            for spec in TOPIC_SPECS
        ]

    def _callback(self, topic):
        def on_message(message):
            self.tracker.record(topic, message)

        return on_message

    def run(self, duration):
        start = time.monotonic()
        deadline = start + duration
        try:
            while rclpy.ok():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                rclpy.spin_once(self, timeout_sec=min(remaining, 0.1))
        except KeyboardInterrupt:
            pass
        return self.tracker.report()


def duration_value(value):
    try:
        duration = float(value)
    except ValueError as exception:
        raise argparse.ArgumentTypeError('must be a number') from exception
    if not math.isfinite(duration) or duration < 0.0:
        raise argparse.ArgumentTypeError(
            'must be finite and greater than or equal to zero'
        )
    return duration


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description='Print Wuji Glove joint angles and stream rates.'
    )
    parser.add_argument('--duration', type=duration_value, default=10.0)
    parser.add_argument(
        '--rate-only',
        action='store_true',
        help='Suppress per-frame joint angles while measuring receive rates.',
    )
    return parser.parse_args(args)


def main(args=None):
    ros_args = sys.argv if args is None else ['glove_status_demo', *args]
    options = parse_args(remove_ros_args(ros_args)[1:])
    rclpy.init(args=ros_args)
    demo = None
    try:
        demo = GloveStatusDemo(print_joint_angles=not options.rate_only)
        return demo.run(options.duration)
    finally:
        if demo is not None:
            demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
