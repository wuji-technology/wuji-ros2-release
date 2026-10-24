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
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_nodes(context, driver_executable):
    hand_side = LaunchConfiguration('hand_side').perform(context)
    model_revision = LaunchConfiguration('model_revision').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    driver_parameters = {
        'hand_side': hand_side,
        'endpoint': LaunchConfiguration('endpoint'),
        'serial_number': LaunchConfiguration('serial_number'),
        'connect_timeout': ParameterValue(
            LaunchConfiguration('connect_timeout'), value_type=float
        ),
        'auto_enable': ParameterValue(
            LaunchConfiguration('auto_enable'), value_type=bool
        ),
        'publish_joint_diagnostics': ParameterValue(
            LaunchConfiguration('publish_joint_diagnostics'), value_type=bool
        ),
        'publish_imu': ParameterValue(
            LaunchConfiguration('publish_imu'), value_type=bool
        ),
        'publish_fingertip_tactile': ParameterValue(
            LaunchConfiguration('publish_fingertip_tactile'), value_type=bool
        ),
    }

    description_package = f'wuji_hand2_{model_revision}_description'
    description_share = Path(
        FindPackageShare(description_package).perform(context)
    )
    urdf_path = description_share / 'urdf' / f'{hand_side}-ros.urdf'
    try:
        robot_description = urdf_path.read_text(encoding='utf-8')
    except OSError as error:
        raise RuntimeError(f'Unable to read Hand 2 URDF: {urdf_path}') from error

    driver_node = Node(
        package='wuji_hand2_driver',
        executable=driver_executable,
        name='wuji_hand2_driver',
        namespace=namespace,
        parameters=[driver_parameters],
        respawn=False,
    )

    return [
        driver_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=driver_node,
                on_exit=[
                    Shutdown(reason='Wuji Hand 2 driver exited'),
                ],
            )
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            parameters=[
                {
                    'robot_description': ParameterValue(
                        robot_description, value_type=str
                    )
                }
            ],
        ),
    ]


def build_launch(driver_executable='wuji_hand2_node'):
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'hand_side',
                description='Hand side to launch.',
                choices=['left', 'right'],
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
                description='Namespace shared by the driver and description nodes.',
            ),
            DeclareLaunchArgument(
                'endpoint',
                default_value='',
                description='SDK endpoint for the selected Hand 2.',
            ),
            DeclareLaunchArgument(
                'serial_number',
                default_value='',
                description='Serial number for the selected Hand 2.',
            ),
            DeclareLaunchArgument(
                'connect_timeout',
                default_value='10.0',
                description='Seconds to wait for a Hand 2 connection.',
            ),
            DeclareLaunchArgument(
                'auto_enable',
                default_value='false',
                choices=['false', 'true'],
                description='Enable the Hand 2 after startup.',
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
            OpaqueFunction(
                function=launch_nodes,
                kwargs={'driver_executable': driver_executable},
            ),
        ]
    )


def generate_launch_description():
    return build_launch()
