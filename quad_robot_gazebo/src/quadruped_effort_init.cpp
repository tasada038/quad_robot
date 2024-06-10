// Copyright 2020 Open Source Robotics Foundation, Inc
// and maintained continuously by Takumi Asada.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

std::shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

// Implementation of common_goal_response
void common_goal_response(
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle)
{
    // Implementation code...
}

// Implementation of common_result_response
void common_result_response(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult &result)
{
    // Implementation code...
}

// Implementation of common_feedback
void common_feedback(
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
    const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
    // Implementation code...
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("trajectory_test_node");

    std::cout << "Node created" << std::endl;

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
    action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        node->get_node_base_interface(),
        node->get_node_graph_interface(),
        node->get_node_logging_interface(),
        node->get_node_waitables_interface(),
        "/joint_trajectory_controller/follow_joint_trajectory");

    bool response = action_client->wait_for_action_server(std::chrono::seconds(1));
    if (!response) {
        throw std::runtime_error("Could not get action server");
    }
    std::cout << "Action server created" << std::endl;

    std::vector<std::string> joint_names = {
            "rf_hip_joint", "rh_hip_joint", "lf_hip_joint", "lh_hip_joint",
            "rf_upper_leg_joint", "rh_upper_leg_joint", "lf_upper_leg_joint", "lh_upper_leg_joint",
            "rf_lower_leg_joint", "rh_lower_leg_joint", "lf_lower_leg_joint", "lh_lower_leg_joint",
    };

    // Create a goal with joint names and positions
    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names;

    // Set desired positions for each joint
    goal_msg.trajectory.points.emplace_back();
    goal_msg.trajectory.points.back().positions = {
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
    };
    goal_msg.trajectory.points.back().time_from_start = rclcpp::Duration(1, 0);  // Specify the time duration for the trajectory

    // Ensure that the size of positions matches the number of joints
    if (goal_msg.trajectory.joint_names.size() != goal_msg.trajectory.points.back().positions.size()) {
        throw std::runtime_error("Mismatch between the number of joints and the size of positions");
    }

    // Send the goal to the action server
    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&common_goal_response, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&common_result_response, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&common_feedback, std::placeholders::_1, std::placeholders::_2);

    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

    // Wait for the result
    rclcpp::spin_until_future_complete(node, goal_handle_future);

    // Declare and initialize joint_control_topic
    std::string joint_control_topic = "joint_trajectory_controller/command";

    // Publish joint commands
    auto joint_command_publisher =
        node->create_publisher<std_msgs::msg::Float64MultiArray>(joint_control_topic, 10);

    std_msgs::msg::Float64MultiArray joint_command_msg;
    // Replace with your desired joint commands
    joint_command_msg.data = { /* specify joint commands here */ };

    joint_command_publisher->publish(joint_command_msg);

    rclcpp::shutdown();
    return 0;
}
