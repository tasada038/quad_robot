// Copyright (c) 2024 Takumi Asada

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "quad_robot_webots/QuadRobotDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

namespace quad_robot_driver {
void QuadRobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  left_wing = wb_robot_get_device("rf_upper_leg_link");
  right_wing = wb_robot_get_device("lf_upper_leg_link");
  wb_motor_set_position(left_wing, 0.0);
  wb_motor_set_position(right_wing, 0.0);
  // wb_motor_set_velocity(left_wing, 0.0);
  // wb_motor_set_velocity(right_wing, 0.0);

  joint_state_msg.position.resize(2);

  joint_state_subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS().reliable(),
      std::bind(&QuadRobotDriver::jointCallback, this, std::placeholders::_1));
}

void QuadRobotDriver::jointCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg)
{
  joint_state_msg.position[0] = joint_msg->position[0];
  joint_state_msg.position[1] = joint_msg->position[1];
}

void QuadRobotDriver::step() {
  auto target_right_angle = joint_state_msg.position[0];
  auto target_left_angle = joint_state_msg.position[1];

  wb_motor_set_position(right_wing, target_right_angle);
  wb_motor_set_position(left_wing, target_left_angle);
}
} // namespace quad_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quad_robot_driver::QuadRobotDriver,
                        webots_ros2_driver::PluginInterface)