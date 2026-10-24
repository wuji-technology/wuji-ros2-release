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
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu, JointState

from wuji_msgs.msg import JointDiagnosticsArray


class StatusDemo(Node):

    def __init__(self):
        super().__init__('hand2_status_demo')
        self.declare_parameter('duration', 10.0)
        self.duration = self.get_parameter('duration').value

        self.frame_counts = {
            'joint_states': 0,
            'joint_diagnostics': 0,
            'imu': 0,
        }

        self.complete_joint_states = False
        self.complete_joint_diagnostics = False

        self.create_subscription(
            JointState,
            'joint_states',
            self._on_joint_states,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointDiagnosticsArray,
            'joint_diagnostics',
            self._on_joint_diagnostics,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            'imu',
            self._on_imu,
            qos_profile_sensor_data,
        )

    def _on_joint_states(self, message):
        self.frame_counts['joint_states'] += 1
        if len(message.position) == 20:
            self.complete_joint_states = True

    def _on_joint_diagnostics(self, message):
        self.frame_counts['joint_diagnostics'] += 1
        if len(message.joints) == 20:
            self.complete_joint_diagnostics = True

    def _on_imu(self, _message):
        self.frame_counts['imu'] += 1

    def run(self):
        if (
            not isinstance(self.duration, (float, int))
            or not math.isfinite(self.duration)
            or self.duration <= 0
        ):
            self.get_logger().error(
                'duration must be finite and greater than zero'
            )
            return 1

        start = time.monotonic()
        deadline = start + float(self.duration)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(remaining, 0.1))

        return self._report(time.monotonic() - start)

    def _report(self, elapsed):
        print('Received data:')
        elapsed = max(elapsed, sys.float_info.epsilon)
        for topic, count in self.frame_counts.items():
            print(f'  {topic}: {count} frames ({count / elapsed:.1f} Hz)')

        required = {
            'joint_states': self.complete_joint_states,
            'joint_diagnostics': self.complete_joint_diagnostics,
            'imu': self.frame_counts['imu'] > 0,
        }
        for topic, passed in required.items():
            print(f'{"PASS" if passed else "FAIL"}: {topic}')

        return 0 if all(required.values()) else 1


def main(args=None):
    rclpy.init(args=args)
    demo = StatusDemo()
    try:
        return demo.run()
    except KeyboardInterrupt:
        return 0
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
