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

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import time
import unittest
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch_testing_ros.actions import EnableRmwIsolation
import pytest
import rclpy
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'hand2.launch.py'
NAMESPACE = 'hand_under_test'


def load_launch_module():
    spec = spec_from_file_location('wuji_bringup_hand2_launch_test', LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup = load_launch_module().build_launch(
        driver_executable='wuji_hand2_test_node'
    )
    return launch.LaunchDescription(
        [
            EnableRmwIsolation(),
            *bringup.entities,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestHand2Bringup(unittest.TestCase):

    def setUp(self):
        rclpy.init(args=[])
        self.node = rclpy.create_node('wuji_bringup_launch_test')
        self.joint_states = []
        self.dynamic_transforms = []
        self.static_transforms = []
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_qos = QoSProfile(depth=100)
        static_tf_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscriptions = [
            self.node.create_subscription(
                JointState,
                f'/{NAMESPACE}/joint_states',
                self.joint_states.append,
                sensor_qos,
            ),
            self.node.create_subscription(
                TFMessage,
                '/tf',
                lambda message: self.dynamic_transforms.extend(message.transforms),
                tf_qos,
            ),
            self.node.create_subscription(
                TFMessage,
                '/tf_static',
                lambda message: self.static_transforms.extend(message.transforms),
                static_tf_qos,
            ),
        ]

    def tearDown(self):
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.node.destroy_node()
        rclpy.shutdown()

    def wait_for(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return predicate()

    def read_parameter(self, remote_node, name):
        client = AsyncParameterClient(self.node, remote_node)
        self.assertTrue(client.wait_for_services(timeout_sec=5.0))
        future = client.get_parameters([name])
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        self.assertIsNotNone(future.result())
        return future.result().values[0]

    def test_namespaced_driver_description_joint_states_and_tf_contract(self):
        expected_nodes = {
            ('wuji_hand2_driver', f'/{NAMESPACE}'),
            ('robot_state_publisher', f'/{NAMESPACE}'),
        }
        self.assertTrue(
            self.wait_for(
                lambda: expected_nodes.issubset(
                    set(self.node.get_node_names_and_namespaces())
                ),
                10.0,
            )
        )

        driver_side = self.read_parameter(
            f'/{NAMESPACE}/wuji_hand2_driver', 'hand_side'
        )
        self.assertEqual(driver_side.string_value, 'left')

        description_value = self.read_parameter(
            f'/{NAMESPACE}/robot_state_publisher', 'robot_description'
        )
        expected_path = (
            Path(get_package_share_directory('wuji_hand2_1_description'))
            / 'urdf'
            / 'left-ros.urdf'
        )
        expected_description = expected_path.read_text(encoding='utf-8')
        self.assertEqual(description_value.string_value, expected_description)

        root = ElementTree.fromstring(description_value.string_value)
        self.assertIsNone(root.find(".//link[@name='l_thumb_tip_sensor_frame']"))
        self.assertIsNone(root.find(".//link[@name='l_index_finger_tip_sensor_frame']"))
        self.assertIsNone(root.find(".//link[@name='l_middle_finger_tip_sensor_frame']"))
        self.assertIsNone(root.find(".//link[@name='l_ring_finger_tip_sensor_frame']"))
        self.assertIsNone(root.find(".//link[@name='l_pinky_tip_sensor_frame']"))

        self.assertTrue(
            self.wait_for(
                lambda: bool(self.joint_states)
                and bool(self.dynamic_transforms)
                and bool(self.static_transforms),
                15.0,
            ),
            (
                len(self.joint_states),
                len(self.dynamic_transforms),
                len(self.static_transforms),
            ),
        )
        self.assertEqual(len(self.joint_states[-1].name), 20)

        transforms = self.dynamic_transforms + self.static_transforms
        self.assertFalse(
            any(
                transform.header.frame_id == 'world'
                and transform.child_frame_id == 'l_mount'
                for transform in transforms
            )
        )
        self.assertFalse(
            any(transform.child_frame_id == 'l_mount' for transform in transforms)
        )
        hand2_1_sensor_frames = {
            'l_thumb_tip_sensor_frame',
            'l_index_finger_tip_sensor_frame',
            'l_middle_finger_tip_sensor_frame',
            'l_ring_finger_tip_sensor_frame',
            'l_pinky_tip_sensor_frame',
        }
        self.assertTrue(
            hand2_1_sensor_frames.isdisjoint(
                {transform.child_frame_id for transform in transforms}
            )
        )


@launch_testing.post_shutdown_test()
class TestHand2BringupShutdown(unittest.TestCase):

    def test_all_processes_exit_successfully(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
