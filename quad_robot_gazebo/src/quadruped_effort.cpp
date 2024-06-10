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

class TrajectoryController : public rclcpp::Node {
public:
    TrajectoryController(const std::string & node_name)
        : Node(node_name),
          action_client_(rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
              get_node_base_interface(),
              get_node_graph_interface(),
              get_node_logging_interface(),
              get_node_waitables_interface(),
              "/joint_trajectory_controller/follow_joint_trajectory")) {

        if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            throw std::runtime_error("Could not get action server");
        }

        RCLCPP_INFO(get_logger(), "Action server created");

        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TrajectoryController::sendTrajectory, this));
    }

private:
    void sendTrajectory() {
        // TODO add angle
        std::vector<double> l_r1 = {
          30.0, 27.98, 25.96, 23.95, 22.0, 20.12, 18.35, 16.71, 15.21, 13.88,
          12.72, 11.76, 11.0, 10.45, 10.12, 10.01, 10.12, 10.45, 11.0, 11.76,
          12.72, 13.87, 15.21, 16.7, 18.34, 20.11, 21.98, 23.92, 25.92, 27.94,
          29.94, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0,
          30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0,
          30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0,
          30.0
        };

        std::vector<double> r_r1 = {
          -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
          -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
          -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -30.0,
          -30.0, -27.98, -25.96, -23.95, -22.0, -20.12, -18.35, -16.71, -15.21, -13.88,
          -12.72, -11.76, -11.0, -10.45, -10.12, -10.01, -10.12, -10.45, -11.0, -11.76,
          -12.72, -13.87, -15.21, -16.7, -18.34, -20.11, -21.98, -23.92, -25.92, -27.94,
          -29.94
        };

        std::vector<double> ltop_p = {
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, -3.9, -7.6, -11.1, -14.4, -17.5, -20.4, -23.1, -25.6, -27.9,
          -30.0, -31.9, -33.6, -35.1, -36.4, -37.5, -38.4, -39.1, -39.6, -39.9,
          -40.0, -39.9, -39.6, -39.1, -38.4, -37.5, -36.4, -35.1, -33.6, -31.9,
          -30.0, -27.9, -25.6, -23.1, -20.4, -17.5, -14.4, -11.1, -7.6, -3.9,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0
        };

        std::vector<double> lmid_p = {
          -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0,
          -5.0, -4.03, -3.1, -2.23, -1.4, -0.62, 0.1, 0.78, 1.4, 1.97,
          2.5, 2.97, 3.4, 3.77, 4.1, 4.38, 4.6, 4.78, 4.9, 4.97,
          5.0, 4.97, 4.9, 4.78, 4.6, 4.38, 4.1, 3.77, 3.4, 2.98,
          2.5, 1.97, 1.4, 0.78, 0.1, -0.62, -1.4, -2.23, -3.1, -4.02,
          -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0, -5.0,
          -5.0
        };

        std::vector<double> lbtm_p = {
          40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
          40.0, 36.1, 32.4, 28.9, 25.6, 22.5, 19.6, 16.9, 14.4, 12.1,
          10.0, 8.1, 6.4, 4.9, 3.6, 2.5, 1.6, 0.9, 0.4, 0.1,
          0.0, 0.1, 0.4, 0.9, 1.6, 2.5, 3.6, 4.9, 6.4, 8.1,
          10.0, 12.1, 14.4, 16.9, 19.6, 22.5, 25.6, 28.9, 32.4, 36.1,
          40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
          40.0
        };

        std::vector<double> rtop_p = {
          40.0, 39.9, 39.6, 39.1, 38.4, 37.5, 36.4, 35.1, 33.6, 31.9,
          30.0, 27.9, 25.6, 23.1, 20.4, 17.5, 14.4, 11.1, 7.6, 3.9,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 3.9, 7.6, 11.1, 14.4, 17.5, 20.4, 23.1, 25.6,
          27.9, 30.0, 31.9, 33.6, 35.1, 36.4, 37.5, 38.4, 39.1, 39.6,
          39.9
        };

        std::vector<double> rmid_p = {
          -5.0, -4.97, -4.9, -4.78, -4.6, -4.38, -4.1, -3.77, -3.4, -2.98,
          -2.5, -1.97, -1.4, -0.78, -0.1, 0.62, 1.4, 2.23, 3.1, 4.02,
          5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
          5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0,
          5.0, 5.0, 4.03, 3.1, 2.23, 1.4, 0.62, -0.1, -0.78, -1.4,
          -1.97, -2.5, -2.97, -3.4, -3.77, -4.1, -4.38, -4.6, -4.78, -4.9,
          -4.97
        };

        std::vector<double> rbtm_p = {
          0.0, -0.35, -0.91, -1.67, -2.62, -3.77, -5.09, -6.59, -8.26, -10.09,
          -12.07, -14.2, -16.47, -18.87, -21.41, -24.06, -26.84, -29.72, -32.72, -35.83,
          -39.04, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0,
          -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0,
          -40.0, -40.0, -36.07, -32.29, -28.67, -25.22, -21.97, -18.92, -16.08, -13.46,
          -11.06, -8.89, -6.95, -5.25, -3.78, -2.55, -1.55, -0.79, -0.25, 0.05,
          0.14
        };

        static double r2 = 45;

        // Servo Angle Power
        static double power_th1 = 1.0;
        static double power_phi = 0.8;
        static double power_th2 = 0.8;

        auto angle_len =  r_r1.size();

        for (std::vector<double>::size_type i=0; i != angle_len; i++){
          l_r1.at(i) = l_r1.at(i) * power_th1;
          r_r1.at(i) = r_r1.at(i) * power_th1;

          ltop_p.at(i) = ltop_p.at(i) * power_phi;
          lmid_p.at(i) = lmid_p.at(i) * power_phi;
          lbtm_p.at(i) = lbtm_p.at(i) * power_phi;

          rtop_p.at(i) = rtop_p.at(i) * power_phi;
          rmid_p.at(i) = rmid_p.at(i) * power_phi;
          rbtm_p.at(i) = rbtm_p.at(i) * power_phi;

        }
        // Convert URDF Position parameter
        for (size_t i=0; i != angle_len; i++){
          l_r1.at(i) = l_r1.at(i)/50 * -1;
          r_r1.at(i) = r_r1.at(i)/50 * -1;

          ltop_p.at(i) = ltop_p.at(i)/60;
          lmid_p.at(i) = lmid_p.at(i)/60;
          lbtm_p.at(i) = lbtm_p.at(i)/60;

          rtop_p.at(i) = rtop_p.at(i)/60;
          rmid_p.at(i) = rmid_p.at(i)/60;
          rbtm_p.at(i) = rbtm_p.at(i)/60;
        }

        r2 = r2 * power_th2;
        r2 = r2/60;

        std::vector<std::string> joint_names = {
            "rf_hip_joint", "rh_hip_joint", "lf_hip_joint", "lh_hip_joint",
            "rf_upper_leg_joint", "rh_upper_leg_joint", "lf_upper_leg_joint", "lh_upper_leg_joint",
            "rf_lower_leg_joint", "rh_lower_leg_joint", "lf_lower_leg_joint", "lh_lower_leg_joint",
          };


        auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = joint_names;

        // Set desired positions for each joint
        auto points = std::vector<trajectory_msgs::msg::JointTrajectoryPoint>();
        // size_t angle_len = /* specify the length */;
        double duration_time = 0.02; /* specify the time duration between consecutive points */;

        double error = 0.3;
        for (size_t i = 0; i != angle_len; i++) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.time_from_start = rclcpp::Duration::from_seconds(duration_time * i);  // start asap
            point.positions = {
                -l_r1.at(i), r_r1.at(i), -r_r1.at(i), l_r1.at(i),
                -rtop_p.at(i)+error, ltop_p.at(i)+error, ltop_p.at(i)+error, -rtop_p.at(i)+error,
                -rtop_p.at(i), ltop_p.at(i), ltop_p.at(i), -rtop_p.at(i)
            };
            points.push_back(point);
        }

        goal_msg.trajectory.points = points;

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&TrajectoryController::commonGoalResponse, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&TrajectoryController::commonResultResponse, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&TrajectoryController::commonFeedback, this, std::placeholders::_1, std::placeholders::_2);

        auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

        // Wait for the result
        // rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future);
    }

    void commonGoalResponse(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle) {
        // Implementation of goal response...
    }

    void commonResultResponse(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {
        // Implementation of result response...
    }

    void commonFeedback(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr, const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback) {
        // Implementation of feedback...
    }

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryController>("trajectory_controller_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
