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

from contextlib import redirect_stderr
from importlib.util import module_from_spec, spec_from_file_location
from io import StringIO
from pathlib import Path

import pytest

from wuji_msgs.msg import FingerJointAngles, HandJointAngles


DEMO_FILE = Path(__file__).resolve().parents[1] / 'demo' / 'glove_status_demo.py'


def load_demo_module():
    spec = spec_from_file_location('wuji_glove_status_demo', DEMO_FILE)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def demo_module():
    return load_demo_module()


def make_joint_angles():
    message = HandJointAngles()
    message.fingers = [
        FingerJointAngles(
            name=name,
            angles=[float(index), float(index) + 0.5],
            confidence=0.9 - index * 0.1,
        )
        for index, name in enumerate(
            ('thumb', 'index', 'middle', 'ring', 'pinky')
        )
    ]
    return message


def test_topic_contract_has_exactly_ten_relative_sensor_topics(demo_module):
    expected = (
        'joint_angles',
        'emf_poses',
        'tip_poses',
        'hand_skeleton',
        'imu/palm',
        'tactile',
        'tactile/residual',
        'tactile/binary',
        'tactile/zones',
        'tactile/points',
    )

    assert tuple(spec.topic for spec in demo_module.TOPIC_SPECS) == expected
    assert len(demo_module.TOPIC_SPECS) == 10
    assert all(not spec.topic.startswith('/') for spec in demo_module.TOPIC_SPECS)
    assert tuple(
        spec.message_type.__name__ for spec in demo_module.TOPIC_SPECS
    ) == (
        'HandJointAngles',
        'TrackedPoseArray',
        'TrackedPoseArray',
        'TrackedPoseArray',
        'Imu',
        'TactileGrid',
        'TactileGrid',
        'TactileGrid',
        'TactileZoneArray',
        'PointCloud2',
    )


def test_node_creates_all_subscriptions_with_sensor_data_qos(demo_module):
    demo_module.rclpy.init(args=[])
    node = None
    try:
        node = demo_module.GloveStatusDemo()
        subscriptions = list(node.subscriptions)
        assert len(subscriptions) == 10
        assert {subscription.topic_name for subscription in subscriptions} == {
            f'/{spec.topic}' for spec in demo_module.TOPIC_SPECS
        }
        expected = demo_module.qos_profile_sensor_data
        for subscription in subscriptions:
            actual = subscription.qos_profile
            assert actual.history == expected.history
            assert actual.depth == expected.depth
            assert actual.reliability == expected.reliability
            assert actual.durability == expected.durability
    finally:
        if node is not None:
            node.destroy_node()
        demo_module.rclpy.shutdown()


def test_every_joint_frame_prints_all_fingers_and_unchanged_data_passes(
    demo_module,
):
    output = StringIO()
    tracker = demo_module.StatusTracker(output=output)
    message = make_joint_angles()

    tracker.record('joint_angles', message)
    tracker.record('joint_angles', message)
    result = tracker.report()

    assert result == 0
    rendered = output.getvalue()
    for name in ('thumb', 'index', 'middle', 'ring', 'pinky'):
        assert rendered.count(f'{name} angles=') == 2
    assert rendered.count('confidence=') == 10
    assert 'joint_angles: frames=2 receive_rate_hz=' in rendered


def test_rate_only_uses_each_topics_first_and_last_callback_times(demo_module):
    output = StringIO()
    timestamps = iter((10.0, 10.01, 20.0, 20.02))
    tracker = demo_module.StatusTracker(
        output=output,
        clock=lambda: next(timestamps),
        print_joint_angles=False,
    )
    message = make_joint_angles()

    tracker.record('joint_angles', message)
    tracker.record('joint_angles', message)
    tracker.record('tactile', object())
    tracker.record('tactile', object())

    assert tracker.report() == 0
    rendered = output.getvalue()
    assert 'thumb angles=' not in rendered
    assert 'joint_angles: frames=2 receive_rate_hz=100.0' in rendered
    assert 'tactile: frames=2 receive_rate_hz=50.0' in rendered


def test_single_received_frame_reports_zero_receive_rate(demo_module):
    output = StringIO()
    tracker = demo_module.StatusTracker(output=output, clock=lambda: 10.0)

    tracker.record('joint_angles', make_joint_angles())

    assert tracker.report() == 0
    assert 'joint_angles: frames=1 receive_rate_hz=0.0' in output.getvalue()


def test_report_prints_all_topics_and_warns_for_missing_optional_streams(
    demo_module,
):
    output = StringIO()
    tracker = demo_module.StatusTracker(output=output)
    tracker.record('joint_angles', make_joint_angles())

    result = tracker.report()

    assert result == 0
    rendered = output.getvalue()
    assert rendered.count('receive_rate_hz=') == 10
    assert 'joint_angles: frames=1 receive_rate_hz=0.0' in rendered
    assert 'warning: no frames received on tactile/points' in rendered
    assert 'warning: no frames received on joint_angles' not in rendered


def test_every_configured_stream_frame_is_counted(demo_module):
    output = StringIO()
    tracker = demo_module.StatusTracker(output=output)

    for spec in demo_module.TOPIC_SPECS:
        message = make_joint_angles() if spec.topic == 'joint_angles' else object()
        tracker.record(spec.topic, message)

    assert tracker.report() == 0
    assert set(tracker.frame_counts.values()) == {1}
    assert 'warning:' not in output.getvalue()


def test_missing_joint_angles_is_failure(demo_module):
    output = StringIO()
    tracker = demo_module.StatusTracker(output=output)

    result = tracker.report()

    assert result == 1
    assert 'error: no frames received on joint_angles' in output.getvalue()


@pytest.mark.parametrize('value', ['-0.1', 'nan', 'inf', '-inf'])
def test_duration_rejects_negative_and_nonfinite_values(demo_module, value):
    errors = StringIO()

    with redirect_stderr(errors), pytest.raises(SystemExit) as exception:
        demo_module.parse_args(['--duration', value])

    assert exception.value.code == 2
    assert '--duration' in errors.getvalue()


def test_duration_defaults_to_ten_and_accepts_zero(demo_module):
    assert demo_module.parse_args([]).duration == 10.0
    assert demo_module.parse_args(['--duration', '0']).duration == 0.0


def test_rate_only_defaults_off_and_can_be_enabled(demo_module):
    assert demo_module.parse_args([]).rate_only is False
    assert demo_module.parse_args(['--rate-only']).rate_only is True


def test_unknown_demo_argument_is_an_argparse_error(demo_module):
    errors = StringIO()

    with redirect_stderr(errors), pytest.raises(SystemExit) as exception:
        demo_module.parse_args(['--unknown'])

    assert exception.value.code == 2
    assert 'unrecognized arguments: --unknown' in errors.getvalue()
