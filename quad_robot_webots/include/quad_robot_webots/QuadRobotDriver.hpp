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

#ifndef quad_robot_DRIVER_HPP
#define quad_robot_DRIVER_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"

namespace quad_robot_driver {
class QuadRobotDriver : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr joint_msg);
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  sensor_msgs::msg::JointState joint_state_msg;
  WbDeviceTag right_wing;
  WbDeviceTag left_wing;
};
} // namespace quad_robot_driver
#endif  // quad_robot_DRIVER_HPP