# Copyright 2024 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pose_initializer_node = Node(
        package='cartographer_utils', executable='pose_initializer_node_exe',
        name='pose_initializer_node',
        parameters=[
            {
                'configuration_directory': LaunchConfiguration('configuration_directory'),
                'configuration_basename': LaunchConfiguration('configuration_basename')
            }
        ],
        remappings=[
            ('get_trajectory_states',
                LaunchConfiguration('input_get_trajectory_states_service_name')),
            ('finish_trajectory',
                LaunchConfiguration('input_finish_trajectory_service_name')),
            ('start_trajectory',
                LaunchConfiguration('input_start_trajectory_service_name'))
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs']
    )

    return [
        pose_initializer_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('configuration_directory')
    add_launch_arg('configuration_basename')
    add_launch_arg('input_get_trajectory_states_service_name',
                   '/localization/get_trajectory_states')
    add_launch_arg('input_finish_trajectory_service_name', '/localization/finish_trajectory')
    add_launch_arg('input_start_trajectory_service_name', '/localization/start_trajectory')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
