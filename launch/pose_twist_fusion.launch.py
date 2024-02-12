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
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    param_path = LaunchConfiguration('pose_twist_fusion_param_file').perform(context)
    if not param_path:
        param_path = PathJoinSubstitution(
            [FindPackageShare('cartographer_utils'), 'config', 'pose_twist_fusion.param.yaml']
        ).perform(context)

    pose_twist_fusion_node = Node(
        package='cartographer_utils',
        executable='pose_twist_fusion_node_exe',
        name='pose_twist_fusion_node',
        parameters=[
            param_path
        ],
        remappings=[
            ('pose', LaunchConfiguration('input_pose')),
            ('twist', LaunchConfiguration('input_twist')),
            ('pose_with_covariance', LaunchConfiguration('input_pose_with_covariance')),
            ('twist_with_covariance', LaunchConfiguration('input_twist_with_covariance')),
            ('odometry', LaunchConfiguration('output_odometry'))
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
    )

    return [
        pose_twist_fusion_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('pose_twist_fusion_param_file', '')
    add_launch_arg('input_pose', 'pose')
    add_launch_arg('input_twist', 'twist')
    add_launch_arg('input_pose_with_covariance', 'pose_with_covariance')
    add_launch_arg('input_twist_with_covariance', 'twist_with_covariance')
    add_launch_arg('output_odometry', 'odometry')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
