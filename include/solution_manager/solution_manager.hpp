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

// File: solution_manager.hpp

#ifndef SOLUTION_MANAGER_HPP
#define SOLUTION_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <chrono>
#include <future>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <map>

/**
 * @class SolutionManagerNode
 * @brief Manages the lifecycle states of multiple robot_manager nodes in a coordinated manner.
 */

namespace solution_manager
{
class SolutionManagerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the SolutionManagerNode.
   */
  SolutionManagerNode();
  
private:
  /**
   * @brief Establishes the solution by configuring and activating robots in order.
   */
  void solution_establishment();
  
  /**
   * @brief Changes the lifecycle state of a robot_manager node.
   * @param robot_namespace The namespace of the robot.
   * @param transition The lifecycle transition to perform.
   * @return True if the state change was successful, false otherwise.
   */
  bool change_robot_state(const std::string & robot_namespace, uint8_t transition);

  /**
   * @brief Prompts the user and waits for input to proceed.
   * @param prompt The message to display to the user.
   * @return True if the user chooses to proceed, false if the user aborts.
   */
  bool wait_for_user_input(const std::string & prompt);

  /// List of robot namespaces.
  std::vector<std::string> robot_namespaces_;

  /// Map of service clients for changing robot states.
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> change_state_clients_;
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> get_state_clients_;

  /// Timer to schedule the initial call to solution_establishment().
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace solution_manager

#endif  // SOLUTION_MANAGER_HPP
