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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PUBLISH_ARGUMENTS = (
    'publish_emf_poses',
    'publish_tip_poses',
    'publish_skeleton',
    'publish_imu',
    'publish_tactile',
    'publish_tactile_residual',
    'publish_tactile_binary',
    'publish_tactile_zones',
    'publish_tactile_points',
)


def build_launch(driver_executable='wuji_glove_node'):
    driver = Node(
        package='wuji_glove_driver',
        executable=driver_executable,
        name='wuji_glove_driver',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {
                'endpoint': LaunchConfiguration('endpoint'),
                'serial_number': LaunchConfiguration('serial_number'),
                'hand_side': LaunchConfiguration('hand_side'),
                'connect_timeout': ParameterValue(
                    LaunchConfiguration('connect_timeout'), value_type=float
                ),
                **{
                    name: ParameterValue(
                        LaunchConfiguration(name), value_type=bool
                    )
                    for name in PUBLISH_ARGUMENTS
                },
            }
        ],
        respawn=False,
    )

    declarations = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the Glove driver.',
        ),
        DeclareLaunchArgument(
            'endpoint',
            default_value='',
            description='SDK endpoint for the selected Wuji Glove.',
        ),
        DeclareLaunchArgument(
            'serial_number',
            default_value='',
            description='Serial number for the selected Wuji Glove.',
        ),
        DeclareLaunchArgument(
            'hand_side',
            default_value='',
            choices=['', 'left', 'right'],
            description='Hand side for the selected Wuji Glove.',
        ),
        DeclareLaunchArgument(
            'connect_timeout',
            default_value='10.0',
            description='Seconds to wait for a Wuji Glove connection.',
        ),
    ]
    declarations.extend(
        DeclareLaunchArgument(
            name,
            default_value='true',
            choices=['false', 'true'],
            description=f'Enable the {name.removeprefix("publish_")} stream.',
        )
        for name in PUBLISH_ARGUMENTS
    )

    return LaunchDescription(
        [
            *declarations,
            driver,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=driver,
                    on_exit=[
                        Shutdown(reason='Wuji Glove driver exited'),
                    ],
                )
            ),
        ]
    )


def generate_launch_description():
    return build_launch()
