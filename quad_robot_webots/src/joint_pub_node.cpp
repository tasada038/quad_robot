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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStatePublisher : public rclcpp::Node
{
public:
  JointStatePublisher()
  : Node("joint_state_publisher")
  {
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&JointStatePublisher::publishJointStates, this));
  }

private:
  void publishJointStates()
  {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name = {"rev_right_fin", "rev_left_fin"};
    
    // 正弦波でジョイント位置を設定
    double t = static_cast<double>(count_) / static_cast<double>(count_limit_);
    double angle = 2.0 * M_PI * t;
    joint_state.position = {std::sin(angle) * 0.5, -std::sin(angle) * 0.5};
    
    RCLCPP_INFO(get_logger(), "Publishing Joint States: %f, %f", joint_state.position[0], joint_state.position[1]);

    joint_state_pub_->publish(joint_state);

    count_++;
    if (count_ > count_limit_) {
      count_ = 0;
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_ = 0;
  int count_limit_ = 100;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
