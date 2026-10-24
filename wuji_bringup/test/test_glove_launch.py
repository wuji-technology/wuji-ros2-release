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
import os
from pathlib import Path
import signal
import time
import unittest

import launch
from launch_ros.actions import Node
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from launch_testing_ros.actions import EnableRmwIsolation
import pytest
import rclpy


LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'glove.launch.py'
CASE = os.environ.get('WUJI_GLOVE_LAUNCH_CASE', 'namespace')
EXPECTED_SELECTOR_KIND = os.environ.get(
    'WUJI_GLOVE_EXPECTED_SELECTOR_KIND', 'unique_discovery'
)
EXPECTED_SELECTOR_VALUE = os.environ.get(
    'WUJI_GLOVE_EXPECTED_SELECTOR_VALUE', 'WGLJ000000000001'
)
CASE_NAMESPACES = {
    'serial_number': '',
    'endpoint': '',
    'hand_side': '',
    'unique_discovery': '',
    'namespace': 'glove_under_test',
    'publish_flags': 'flags_under_test',
}
REPRESENTATIVE_OPTIONAL_TOPICS = {
    'emf_poses',
    'imu/palm',
    'tactile',
    'tactile/zones',
    'tactile/points',
}


def load_launch_module():
    spec = spec_from_file_location('wuji_bringup_glove_launch_test', LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup = load_launch_module().build_launch(
        driver_executable='wuji_glove_test_node'
    )
    driver = next(
        entity for entity in bringup.entities if isinstance(entity, Node)
    )
    return (
        launch.LaunchDescription(
            [
                EnableRmwIsolation(),
                *bringup.entities,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {'driver': driver},
    )


class TestGloveBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if CASE not in CASE_NAMESPACES:
            raise RuntimeError(f'Unknown Glove launch test case: {CASE}')

    def setUp(self):
        rclpy.init(args=[])
        self.node = rclpy.create_node(f'wuji_glove_launch_test_{CASE}')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def wait_for(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return predicate()

    def qualified(self, topic):
        namespace = CASE_NAMESPACES[CASE]
        return f'/{namespace}/{topic}' if namespace else f'/{topic}'

    def test_01_graph_matches_the_selected_configuration(
        self, driver, proc_output
    ):
        proc_output.assertWaitFor(
            'WUJI_GLOVE_TEST_EVENT '
            f'selector_verified:{EXPECTED_SELECTOR_KIND}:'
            f'{EXPECTED_SELECTOR_VALUE}',
            process=driver,
            stream='stdout',
            timeout=10.0,
        )
        namespace = CASE_NAMESPACES[CASE]
        expected_node = ('wuji_glove_driver', f'/{namespace}' if namespace else '/')
        self.assertTrue(
            self.wait_for(
                lambda: expected_node
                in set(self.node.get_node_names_and_namespaces()),
                10.0,
            )
        )

        expected_topics = {self.qualified('joint_angles')}
        if CASE != 'publish_flags':
            expected_topics.update(
                self.qualified(topic) for topic in REPRESENTATIVE_OPTIONAL_TOPICS
            )
        self.assertTrue(
            self.wait_for(
                lambda: expected_topics.issubset(
                    {name for name, _ in self.node.get_topic_names_and_types()}
                ),
                10.0,
            ),
            self.node.get_topic_names_and_types(),
        )

        node_names = {
            name for name, _ in self.node.get_node_names_and_namespaces()
        }
        self.assertTrue(
            {
                'robot_state_publisher',
                'rviz2',
                'static_transform_publisher',
                'dynamic_transform_publisher',
            }.isdisjoint(node_names)
        )
        topic_names = {
            name for name, _ in self.node.get_topic_names_and_types()
        }
        self.assertNotIn('/tf', topic_names)
        self.assertNotIn('/tf_static', topic_names)

        if CASE == 'publish_flags':
            self.assertTrue(
                {
                    self.qualified(topic)
                    for topic in REPRESENTATIVE_OPTIONAL_TOPICS
                }.isdisjoint(topic_names)
            )

    def test_99_driver_exit_shuts_down_launch(self, driver, proc_info):
        os.kill(proc_info[driver].pid, signal.SIGINT)
        proc_info.assertWaitForShutdown(driver, timeout=10.0)


@launch_testing.post_shutdown_test()
class TestGloveBringupShutdown(unittest.TestCase):

    def test_driver_exits_successfully(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
