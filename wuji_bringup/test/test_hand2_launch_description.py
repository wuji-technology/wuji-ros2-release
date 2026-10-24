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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.events.process import ProcessExited
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest


LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'hand2.launch.py'
VIEW_LAUNCH_FILE = (
    Path(__file__).resolve().parents[1] / 'launch' / 'hand2_view.launch.py'
)
RVIZ_CONFIG = Path(__file__).resolve().parents[1] / 'rviz' / 'hand2.rviz'
MODEL_CASES = [
    (
        side,
        revision,
        f'wuji_hand2_{revision}_description',
        f'{side}-ros.urdf',
    )
    for side in ('left', 'right')
    for revision in ('1', '2')
]


def load_launch_module():
    spec = spec_from_file_location('wuji_bringup_hand2_launch', LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_view_launch_module():
    spec = spec_from_file_location('wuji_bringup_hand2_view_launch', VIEW_LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def evaluate_argument(context, argument):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(argument)
    )


def expand_launch(module, side, revision, namespace, executable):
    description = module.build_launch(driver_executable=executable)
    context = LaunchContext()
    context.launch_configurations.update(
        {
            'hand_side': side,
            'model_revision': revision,
            'namespace': namespace,
            'endpoint': 'usb://fixture',
            'serial_number': 'WH2J000000000001',
            'connect_timeout': '4.0',
            'auto_enable': 'false',
            'publish_joint_diagnostics': 'true',
            'publish_imu': 'true',
            'publish_fingertip_tactile': 'true',
        }
    )
    declarations = [
        entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    for declaration in declarations:
        declaration.execute(context)
    opaque = next(
        entity for entity in description.entities if isinstance(entity, OpaqueFunction)
    )
    nodes = opaque.execute(context)
    assert nodes is not None
    return context, declarations, list(nodes)


def node_parameters(context, node):
    evaluated = evaluate_parameters(context, node._Node__parameters)
    assert len(evaluated) == 1
    return evaluated[0]


def test_public_arguments_are_exact_and_validated():
    declarations = [
        entity
        for entity in load_launch_module().generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    assert [item.name for item in declarations] == [
        'hand_side',
        'model_revision',
        'namespace',
        'endpoint',
        'serial_number',
        'connect_timeout',
        'auto_enable',
        'publish_joint_diagnostics',
        'publish_imu',
        'publish_fingertip_tactile',
    ]
    assert declarations[0].default_value is None
    assert declarations[0].choices == ['left', 'right']
    assert (
        evaluate_argument(LaunchContext(), declarations[-1].default_value) == 'false'
    )
    assert declarations[1].choices == ['1', '2']
    assert evaluate_argument(LaunchContext(), declarations[1].default_value) == '1'


@pytest.mark.parametrize(
    ('side', 'revision', 'package', 'filename'), MODEL_CASES
)
def test_launch_matrix_selects_exact_urdf_driver_parameters_and_node_graph(
    side, revision, package, filename
):
    namespace = f'integration/hand2_{revision}/{side}'
    context, declarations, nodes = expand_launch(
        load_launch_module(),
        side,
        revision,
        namespace,
        'wuji_hand2_test_node',
    )
    nodes = [action for action in nodes if isinstance(action, Node)]

    assert len(nodes) == 2
    assert [(node.node_package, node.node_executable) for node in nodes] == [
        ('wuji_hand2_driver', 'wuji_hand2_test_node'),
        ('robot_state_publisher', 'robot_state_publisher'),
    ]
    assert 'driver_executable' not in [item.name for item in declarations]
    for node in nodes:
        node._perform_substitutions(context)
        assert node.expanded_node_namespace == f'/{namespace}'

    assert node_parameters(context, nodes[0]) == {
        'hand_side': side,
        'endpoint': 'usb://fixture',
        'serial_number': 'WH2J000000000001',
        'connect_timeout': 4.0,
        'auto_enable': False,
        'publish_joint_diagnostics': True,
        'publish_imu': True,
        'publish_fingertip_tactile': True,
    }
    expected_urdf = (
        Path(get_package_share_directory(package)) / 'urdf' / filename
    ).read_text(encoding='utf-8')
    assert node_parameters(context, nodes[1]) == {
        'robot_description': expected_urdf
    }


def test_production_launch_uses_the_production_driver():
    _, _, nodes = expand_launch(
        load_launch_module(), 'left', '1', '', 'wuji_hand2_node'
    )
    assert nodes[0].node_executable == 'wuji_hand2_node'


def test_driver_exit_shuts_down_the_launch_without_respawn():
    _, _, actions = expand_launch(
        load_launch_module(), 'left', '1', '', 'wuji_hand2_node'
    )
    driver = next(
        action
        for action in actions
        if isinstance(action, Node)
        and action.node_package == 'wuji_hand2_driver'
    )
    robot_state_publisher = next(
        action
        for action in actions
        if isinstance(action, Node)
        and action.node_package == 'robot_state_publisher'
    )
    registration = next(
        action for action in actions if isinstance(action, RegisterEventHandler)
    )
    handler = registration.event_handler
    driver_exit = ProcessExited(
        action=driver,
        name='driver',
        cmd=[],
        cwd=None,
        env=None,
        pid=1,
        returncode=1,
    )
    assert handler.matches(driver_exit)
    assert not handler.matches(
        ProcessExited(
            action=robot_state_publisher,
            name='robot_state_publisher',
            cmd=[],
            cwd=None,
            env=None,
            pid=1,
            returncode=1,
        )
    )
    shutdown_actions = list(handler.handle(driver_exit, LaunchContext()))
    assert len(shutdown_actions) == 1
    assert isinstance(shutdown_actions[0], Shutdown)


def test_view_launch_requires_serial_number():
    description = load_view_launch_module().generate_launch_description()
    declaration = next(
        entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument) and entity.name == 'serial_number'
    )

    with pytest.raises(RuntimeError, match='Required launch argument "serial_number"'):
        declaration.execute(LaunchContext())


def test_view_launch_leaves_fingertip_tactile_off_by_default():
    description = load_view_launch_module().generate_launch_description()
    declaration = next(
        entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == 'publish_fingertip_tactile'
    )

    assert evaluate_argument(LaunchContext(), declaration.default_value) == 'false'


def test_view_launch_includes_base_launch_and_rviz_config():
    description = load_view_launch_module().generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(
        {
            'hand_side': 'right',
            'serial_number': 'WH2K000000000001',
            'model_revision': '1',
            'namespace': 'view_fixture',
            'endpoint': '',
            'connect_timeout': '10.0',
            'publish_joint_diagnostics': 'true',
            'publish_imu': 'true',
            'publish_fingertip_tactile': 'false',
        }
    )
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    opaque = next(
        entity for entity in description.entities if isinstance(entity, OpaqueFunction)
    )
    actions = list(opaque.execute(context))
    include = next(
        action for action in actions if isinstance(action, IncludeLaunchDescription)
    )
    rviz = next(action for action in actions if isinstance(action, Node))

    included_description = include.launch_description_source.get_launch_description(context)
    included_arguments = {
        entity.name
        for entity in included_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert {'serial_number', 'auto_enable'} <= included_arguments
    include_arguments = {
        evaluate_argument(context, name): evaluate_argument(context, value)
        for name, value in include.launch_arguments
    }
    assert include_arguments['serial_number'] == 'WH2K000000000001'
    assert include_arguments['auto_enable'] == 'false'
    assert rviz.node_package == 'rviz2'
    assert rviz.node_executable == 'rviz2'
    assert [
        evaluate_argument(context, argument) for argument in rviz._Node__arguments
    ] == ['-d', str(RVIZ_CONFIG), '-f', 'r_mount']
