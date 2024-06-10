# Copyright (c) 2024 Takumi Asada

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import os

def generate_launch_description():
    # Constants for paths to different files and folders
    description_pkg_name = 'quad_robot_description'
    gazebo_pkg_name = 'quad_robot_gazebo'
    robot_name_in_model = 'quad_robot'
    urdf_file_path = 'urdf/quad_robot.xacro'
    world_file_path = 'worlds/basic.world'
    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.6'
    spawn_yaw_val = '0.0'

    description_pkg_share = FindPackageShare(package=description_pkg_name).find(description_pkg_name)
    gazebo_pkg_share = FindPackageShare(package=gazebo_pkg_name).find(gazebo_pkg_name)
    world_path = os.path.join(gazebo_pkg_share, world_file_path)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': world_path}.items(),
    )

    urdf_model_path = os.path.join(description_pkg_share, urdf_file_path)

    robot_description_config = xacro.process_file(
        urdf_model_path
    )

    # Robot state publisher
    params = {'use_sim_time': True, 'robot_description': robot_description_config.toxml()}
    gazebo_robot_state_pub_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])

    # https://answers.gazebosim.org//question/27584/ros2-setting-initial-joint-position-at-spawn/
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
            arguments=[
                '-topic', '/robot_description',
                '-entity', robot_name_in_model,
                '-x', spawn_x_val,
                '-y', spawn_y_val,
                '-z', spawn_z_val,
                '-R', '0',
                '-P', '0',
                '-Y', spawn_yaw_val,
            ],
            output='screen')

    nodes = [

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        gazebo,
        gazebo_robot_state_pub_node,
        spawn_entity,
    ]

    return LaunchDescription(nodes)
