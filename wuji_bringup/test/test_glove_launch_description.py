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

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.events.process import ProcessExited
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest


LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'glove.launch.py'
ARGUMENT_DEFAULTS = {
    'namespace': '',
    'endpoint': '',
    'serial_number': '',
    'hand_side': '',
    'connect_timeout': '10.0',
    'publish_emf_poses': 'true',
    'publish_tip_poses': 'true',
    'publish_skeleton': 'true',
    'publish_imu': 'true',
    'publish_tactile': 'true',
    'publish_tactile_residual': 'true',
    'publish_tactile_binary': 'true',
    'publish_tactile_zones': 'true',
    'publish_tactile_points': 'true',
}
BOOLEAN_ARGUMENTS = {
    name for name in ARGUMENT_DEFAULTS if name.startswith('publish_')
}


def load_launch_module():
    spec = spec_from_file_location('wuji_bringup_glove_launch', LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def evaluate_argument(context, value):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(value)
    )


def expand_launch(overrides=None, executable='wuji_glove_node'):
    description = load_launch_module().build_launch(
        driver_executable=executable
    )
    declarations = [
        entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    for declaration in declarations:
        declaration.execute(context)
    return context, declarations, list(description.entities)


def node_parameters(context, node):
    evaluated = evaluate_parameters(context, node._Node__parameters)
    assert len(evaluated) == 1
    return evaluated[0]


def test_public_arguments_are_exact_unique_and_validated():
    _, declarations, _ = expand_launch()
    by_name = {item.name: item for item in declarations}

    assert len(declarations) == len(ARGUMENT_DEFAULTS)
    assert set(by_name) == set(ARGUMENT_DEFAULTS)
    assert 'driver_executable' not in by_name

    context = LaunchContext()
    for name, expected in ARGUMENT_DEFAULTS.items():
        declaration = by_name[name]
        assert declaration.default_value is not None
        assert evaluate_argument(context, declaration.default_value) == expected

    assert by_name['hand_side'].choices == ['', 'left', 'right']
    for name in BOOLEAN_ARGUMENTS:
        assert by_name[name].choices == ['false', 'true']
    for name in set(ARGUMENT_DEFAULTS) - BOOLEAN_ARGUMENTS - {'hand_side'}:
        assert by_name[name].choices is None


def test_arguments_forward_to_exact_node_targets_with_typed_values():
    values = {
        'namespace': 'lab/glove',
        'endpoint': 'usb://fixture',
        'serial_number': 'WGLJ000000000001',
        'hand_side': 'left',
        'connect_timeout': '2.5',
        **dict.fromkeys(BOOLEAN_ARGUMENTS, 'false'),
    }
    context, _, actions = expand_launch(values, 'wuji_glove_test_node')
    nodes = [action for action in actions if isinstance(action, Node)]

    assert len(nodes) == 1
    driver = nodes[0]
    driver._perform_substitutions(context)
    assert driver.expanded_node_namespace == '/lab/glove'
    assert node_parameters(context, driver) == {
        'endpoint': 'usb://fixture',
        'serial_number': 'WGLJ000000000001',
        'hand_side': 'left',
        'connect_timeout': 2.5,
        **dict.fromkeys(BOOLEAN_ARGUMENTS, False),
    }


@pytest.mark.parametrize(
    ('executable',), [('wuji_glove_node',), ('wuji_glove_test_node',)]
)
def test_launch_has_only_the_fixed_driver_node(executable):
    context, _, actions = expand_launch(executable=executable)
    nodes = [action for action in actions if isinstance(action, Node)]

    assert len(nodes) == 1
    driver = nodes[0]
    driver._perform_substitutions(context)
    assert driver.node_package == 'wuji_glove_driver'
    assert driver.node_name == '/wuji_glove_driver'
    assert driver.node_executable == executable
    assert driver._ExecuteLocal__respawn is False
    forbidden = {
        'robot_state_publisher',
        'rviz2',
        'static_transform_publisher',
        'dynamic_transform_publisher',
    }
    assert forbidden.isdisjoint(
        {node.node_package for node in nodes}
        | {node.node_executable for node in nodes}
    )


def test_only_driver_exit_shuts_down_launch():
    _, _, actions = expand_launch()
    driver = next(action for action in actions if isinstance(action, Node))
    registrations = [
        action for action in actions if isinstance(action, RegisterEventHandler)
    ]
    assert len(registrations) == 1
    handler = registrations[0].event_handler
    driver_exit = ProcessExited(
        action=driver,
        name='driver',
        cmd=[],
        cwd=None,
        env=None,
        pid=1,
        returncode=0,
    )
    unrelated = Node(package='example', executable='other')
    unrelated_exit = ProcessExited(
        action=unrelated,
        name='other',
        cmd=[],
        cwd=None,
        env=None,
        pid=2,
        returncode=0,
    )

    assert handler.matches(driver_exit)
    assert not handler.matches(unrelated_exit)
    shutdown_actions = list(handler.handle(driver_exit, LaunchContext()))
    assert len(shutdown_actions) == 1
    assert isinstance(shutdown_actions[0], Shutdown)
