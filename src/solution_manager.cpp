// Copyright 2024 Zhihao Liu (zhihaoliu@ieee.org)
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

// File: solution_manager.cpp

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "solution_manager/solution_manager.hpp"

using namespace std::chrono_literals;

namespace solution_manager
{
SolutionManagerNode::SolutionManagerNode(): Node("multi_robot_solution_manager")
{
  // The namespaces for industrial robots in this solution. Each robot and its managers are in one specific namespace.
  robot_namespaces_ = {"ips_abb", "ips_kuka", "ips_ur"};

  // Create service clients to change and get the node state
  for (const auto & robot_namespace : robot_namespaces_) {

    // Clients to change node state
    change_state_clients_[robot_namespace] = this->create_client<lifecycle_msgs::srv::ChangeState>(
      "/" + robot_namespace + "/robot_manager/change_state");

    // Clients to get node state
    get_state_clients_[robot_namespace] = this->create_client<lifecycle_msgs::srv::GetState>(
      "/" + robot_namespace + "/robot_manager/get_state");
  }

  solution_establishment();
}


void SolutionManagerNode::solution_establishment()
{

  // Define the specific order of robot namespaces
  std::vector<std::string> ordered_robot_namespaces = {"ips_abb", "ips_kuka", "ips_ur"};

  for (const auto & robot_namespace : ordered_robot_namespaces) {
    RCLCPP_INFO(this->get_logger(), "Starting configuration for %s", robot_namespace.c_str());

    // Wait for user input before configuring the robot
    if (!wait_for_user_input("Press Enter to configure " + robot_namespace + " or type 'q' to abort...")) {
      RCLCPP_ERROR(this->get_logger(), "User aborted during configuration of %s", robot_namespace.c_str());
      continue;
    }

    // Configure the robot
    if (!change_robot_state(robot_namespace, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure namespace %s", robot_namespace.c_str());
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "%s is now configured", robot_namespace.c_str());

    // Wait for user input before activating the robot
    if (!wait_for_user_input("Press Enter to activate " + robot_namespace + " or type 'q' to abort...")) {
      RCLCPP_ERROR(this->get_logger(), "User aborted during activation of %s", robot_namespace.c_str());
      continue;
    }

    // Activate the robot
    if (!change_robot_state(robot_namespace, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to activate namespace %s", robot_namespace.c_str());
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "%s is now active", robot_namespace.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Solution establishment completed.");
}


bool SolutionManagerNode::change_robot_state(const std::string & robot_namespace, uint8_t transition)
{
  // Check if the service is available
  if (!change_state_clients_[robot_namespace]->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Service not available for namespace %s", robot_namespace.c_str());
    return false;
  }

  // Create a service request
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;

  // Call the service synchronously
  auto result = change_state_clients_[robot_namespace]->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service for namespace %s", robot_namespace.c_str());
    return false;
  }

  auto response = result.get();

  if (response->success) {
    RCLCPP_INFO(this->get_logger(), "State transition succeeded for namespace %s", robot_namespace.c_str());
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "State transition failed for namespace %s", robot_namespace.c_str());
    return false;
  }
}


  
bool SolutionManagerNode::wait_for_user_input(const std::string & prompt)
{
  // Print the prompt message
  RCLCPP_INFO(this->get_logger(), "%s", prompt.c_str());

  // Flush the output to ensure the prompt is displayed immediately
  std::cout << std::flush;

  // Wait for user input
  std::string input;
  std::getline(std::cin, input);

  // Return true unless the user inputs 'q' or 'quit' to abort
  return !(input == "q" || input == "quit");
}



} // namespace solution_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<solution_manager::SolutionManagerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


