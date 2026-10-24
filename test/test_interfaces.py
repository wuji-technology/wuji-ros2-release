# Copyright 2026 Wuji Technology
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Verify the generated ROS interfaces."""

from pathlib import Path

import pytest

from rosidl_runtime_py import get_interfaces
from wuji_msgs.msg import FingerJointAngles
from wuji_msgs.msg import FingertipTactile
from wuji_msgs.msg import HandJointAngles
from wuji_msgs.msg import JointDiagnostics
from wuji_msgs.msg import JointDiagnosticsArray
from wuji_msgs.msg import TactileGrid
from wuji_msgs.msg import TactileZone
from wuji_msgs.msg import TactileZoneArray
from wuji_msgs.msg import TrackedPose
from wuji_msgs.msg import TrackedPoseArray
from wuji_msgs.srv import ClearFaults
from wuji_msgs.srv import SetEffortLimit
from wuji_msgs.srv import SetEffortLimits
from wuji_msgs.srv import SetEnabled
from wuji_msgs.srv import SetMitParams


EXPECTED_MESSAGE_NAMES = {
    'FingerJointAngles',
    'FingertipTactile',
    'HandJointAngles',
    'JointDiagnostics',
    'JointDiagnosticsArray',
    'TactileGrid',
    'TactileZone',
    'TactileZoneArray',
    'TrackedPose',
    'TrackedPoseArray',
}
GLOVE_MESSAGES = {
    'TrackedPose.msg': ['string name', 'geometry_msgs/Pose pose', 'float32 confidence'],
    'TrackedPoseArray.msg': ['std_msgs/Header header', 'TrackedPose[] poses'],
    'FingerJointAngles.msg': ['string name', 'float64[] angles', 'float64 confidence'],
    'HandJointAngles.msg': ['std_msgs/Header header', 'FingerJointAngles[5] fingers'],
    'TactileGrid.msg': [
        'std_msgs/Header header',
        'uint32 rows',
        'uint32 columns',
        'float32[] data',
    ],
    'TactileZone.msg': ['string name', 'float32[] values'],
    'TactileZoneArray.msg': ['std_msgs/Header header', 'TactileZone[] zones'],
}

EXPECTED_SERVICE_NAMES = {
    'ClearFaults',
    'SetEffortLimit',
    'SetEffortLimits',
    'SetEnabled',
    'SetMitParams',
}

MESSAGE_SCHEMAS = {
    FingerJointAngles: {
        'name': 'string',
        'angles': 'sequence<double>',
        'confidence': 'double',
    },
    FingertipTactile: {
        'header': 'std_msgs/Header',
        'info_digest': 'uint32',
        'point_forces': 'sequence<geometry_msgs/Vector3>',
        'total_force': 'geometry_msgs/Vector3',
        'temperature': 'float',
    },
    JointDiagnostics: {
        'nid': 'uint8',
        'name': 'string',
        'status_word': 'uint32',
        'current': 'float',
        'bus_voltage': 'float',
        'mcu_temperature': 'float',
        'error_code': 'uint16',
        'comm_response_rate_pct': 'uint8',
        'comm_timeout_total': 'uint32',
    },
    HandJointAngles: {
        'header': 'std_msgs/Header',
        'fingers': 'wuji_msgs/FingerJointAngles[5]',
    },
    JointDiagnosticsArray: {
        'header': 'std_msgs/Header',
        'joints': 'sequence<wuji_msgs/JointDiagnostics>',
    },
    TactileGrid: {
        'header': 'std_msgs/Header',
        'rows': 'uint32',
        'columns': 'uint32',
        'data': 'sequence<float>',
    },
    TactileZone: {
        'name': 'string',
        'values': 'sequence<float>',
    },
    TactileZoneArray: {
        'header': 'std_msgs/Header',
        'zones': 'sequence<wuji_msgs/TactileZone>',
    },
    TrackedPose: {
        'name': 'string',
        'pose': 'geometry_msgs/Pose',
        'confidence': 'float',
    },
    TrackedPoseArray: {
        'header': 'std_msgs/Header',
        'poses': 'sequence<wuji_msgs/TrackedPose>',
    },
}

SERVICE_SCHEMAS = {
    SetEnabled: (
        {'enabled': 'boolean', 'mask': 'boolean[20]'},
        {'success': 'boolean', 'message': 'string'},
    ),
    ClearFaults: (
        {'mask': 'boolean[20]'},
        {'success': 'boolean', 'message': 'string'},
    ),
    SetEffortLimit: (
        {'effort_limit': 'float'},
        {'success': 'boolean', 'message': 'string'},
    ),
    SetEffortLimits: (
        {'effort_limits': 'float[20]'},
        {'success': 'boolean', 'message': 'string'},
    ),
    SetMitParams: (
        {'kp': 'float[20]', 'kd': 'float[20]'},
        {'success': 'boolean', 'message': 'string'},
    ),
}


def _message_declaration_lines(source_lines):
    """Return normalized field declarations without comments or blank lines."""
    return [
        line.strip()
        for line in source_lines
        if line.strip() and not line.lstrip().startswith('#')
    ]


def test_message_declaration_lines_ignore_formatting_only_content():
    """Normalize fields while ignoring blank lines and indented comments."""
    source_lines = [
        '  std_msgs/Header header  ',
        '   ',
        '  # field documentation',
        'float32[] data',
    ]

    assert _message_declaration_lines(source_lines) == [
        'std_msgs/Header header',
        'float32[] data',
    ]


def test_glove_message_sources():
    """Require the seven Glove message field contracts in declaration order."""
    message_dir = Path(__file__).parents[1] / 'msg'
    for filename, expected_lines in GLOVE_MESSAGES.items():
        source_lines = (message_dir / filename).read_text(
            encoding='utf-8'
        ).splitlines()
        actual_lines = _message_declaration_lines(source_lines)
        assert actual_lines == expected_lines


def test_glove_message_sources_document_units():
    """Require the SDK position and joint-angle units in message comments."""
    message_dir = Path(__file__).parents[1] / 'msg'

    tracked_pose = (message_dir / 'TrackedPose.msg').read_text(encoding='utf-8')
    joint_angles = (message_dir / 'FingerJointAngles.msg').read_text(encoding='utf-8')

    assert 'Position values are in meters.' in tracked_pose
    assert 'joint angle values for this finger, in radians.' in joint_angles


def field_names(interface_type):
    """Return generated field names in declaration order."""
    return list(interface_type.get_fields_and_field_types())


def assert_generated_interface_inventory(interfaces_by_package):
    """Require exactly the approved generated interfaces."""
    interface_names = set(interfaces_by_package['wuji_msgs'])
    message_names = {
        name.removeprefix('msg/')
        for name in interface_names
        if name.startswith('msg/')
    }
    service_names = {
        name.removeprefix('srv/')
        for name in interface_names
        if name.startswith('srv/')
    }

    assert message_names == EXPECTED_MESSAGE_NAMES
    assert service_names == EXPECTED_SERVICE_NAMES
    assert interface_names == (
        {f'msg/{name}' for name in EXPECTED_MESSAGE_NAMES}
        | {f'srv/{name}' for name in EXPECTED_SERVICE_NAMES}
    )


@pytest.mark.parametrize('interface_type, expected_types', MESSAGE_SCHEMAS.items())
def test_message_schema(interface_type, expected_types):
    """Require each approved message schema."""
    assert field_names(interface_type) == list(expected_types)
    assert interface_type.get_fields_and_field_types() == expected_types


@pytest.mark.parametrize('service_type, schemas', SERVICE_SCHEMAS.items())
def test_service_schema(service_type, schemas):
    """Require each approved service request and response schema."""
    request_types, response_types = schemas
    assert field_names(service_type.Request) == list(request_types)
    assert service_type.Request.get_fields_and_field_types() == request_types
    assert field_names(service_type.Response) == list(response_types)
    assert service_type.Response.get_fields_and_field_types() == response_types


def test_generated_interface_inventory():
    """Require exactly the approved installed message and service inventories."""
    assert_generated_interface_inventory(get_interfaces(['wuji_msgs']))


def test_generated_interface_inventory_rejects_an_unexpected_service():
    """Reject an unexpected generated service in a controlled inventory fixture."""
    controlled_interfaces = {
        'wuji_msgs': [
            *{f'msg/{name}' for name in EXPECTED_MESSAGE_NAMES},
            *{f'srv/{name}' for name in EXPECTED_SERVICE_NAMES},
            'srv/Unexpected',
        ]
    }

    with pytest.raises(AssertionError):
        assert_generated_interface_inventory(controlled_interfaces)
