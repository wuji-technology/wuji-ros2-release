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

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_view(context):
    hand_side = LaunchConfiguration('hand_side').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    launch_directory = Path(__file__).resolve().parent
    prefix = 'l' if hand_side == 'left' else 'r'
    fixed_frame = f'{prefix}_mount'
    rviz_config = launch_directory.parent / 'rviz' / 'hand2.rviz'

    base_launch_arguments = {
        'hand_side': LaunchConfiguration('hand_side'),
        'serial_number': LaunchConfiguration('serial_number'),
        'model_revision': LaunchConfiguration('model_revision'),
        'namespace': LaunchConfiguration('namespace'),
        'endpoint': LaunchConfiguration('endpoint'),
        'connect_timeout': LaunchConfiguration('connect_timeout'),
        'auto_enable': 'false',
        'publish_joint_diagnostics': LaunchConfiguration(
            'publish_joint_diagnostics'
        ),
        'publish_imu': LaunchConfiguration('publish_imu'),
        'publish_fingertip_tactile': LaunchConfiguration(
            'publish_fingertip_tactile'
        ),
    }
    rviz_arguments = ['-d', str(rviz_config), '-f', fixed_frame]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_directory / 'hand2.launch.py')
            ),
            launch_arguments=base_launch_arguments.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace=namespace,
            arguments=rviz_arguments,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'hand_side',
                description='Hand side to launch.',
                choices=['left', 'right'],
            ),
            DeclareLaunchArgument(
                'serial_number',
                description='Serial number for the selected Hand 2.',
            ),
            DeclareLaunchArgument(
                'model_revision',
                default_value='1',
                description='Wuji Hand 2 model revision.',
                choices=['1', '2'],
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Namespace shared by the launched nodes.',
            ),
            DeclareLaunchArgument(
                'endpoint',
                default_value='',
                description='SDK endpoint for the selected Hand 2.',
            ),
            DeclareLaunchArgument(
                'connect_timeout',
                default_value='10.0',
                description='Seconds to wait for a Hand 2 connection.',
            ),
            DeclareLaunchArgument(
                'publish_joint_diagnostics',
                default_value='true',
                choices=['false', 'true'],
                description='Publish Hand 2 joint diagnostics.',
            ),
            DeclareLaunchArgument(
                'publish_imu',
                default_value='true',
                choices=['false', 'true'],
                description='Publish Hand 2 IMU data.',
            ),
            DeclareLaunchArgument(
                'publish_fingertip_tactile',
                default_value='false',
                choices=['false', 'true'],
                description='Publish Hand 2 fingertip tactile data.',
            ),
            OpaqueFunction(function=launch_view),
        ]
    )
