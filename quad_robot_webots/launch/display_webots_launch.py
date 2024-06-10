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

import os
import shutil
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('quad_robot_webots')
    quad_robot_description_path = os.path.join(package_dir, 'resource', 'quad_robot.urdf')

    proto_file = 'basic_world.proto'
    wbt_file = 'basic_world.wbt'
    current_dir = os.getcwd()
    # proto_path = os.path.join(package_dir, 'worlds', proto_file)
    # wbt_path = os.path.join(package_dir, 'worlds', wbt_file)
    proto_current_path = os.path.join(current_dir, 'src/quad_robot/quad_robot_webots/worlds/', proto_file)
    wbt_current_path = os.path.join(current_dir, 'src/quad_robot/quad_robot_webots/worlds/', wbt_file)

    # Create a new .wbt file by copying the .proto file
    # shutil.copy(proto_path, wbt_path)
    shutil.copy(proto_current_path, wbt_current_path)

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', wbt_current_path)
    )

    quad_robot_driver = WebotsController(
        robot_name='quad_robot',
        parameters=[
            {'robot_description': quad_robot_description_path},
        ]
    )

    return LaunchDescription([
        webots,
        quad_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])