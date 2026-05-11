// Copyright (c) 2021 Samsung Research
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

#include "robotics_scenario/scenario/delivery_scenario.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace robotics_scenario {

bool DeliveryScenario::configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) {
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  if (!node->has_parameter("start_blackboard_id")) {
    node->declare_parameter("start_blackboard_id", std::string("start_goal"));
  }

  start_goal_blackboard_id_ = node->get_parameter("start_blackboard_id").as_string();

  if (!node->has_parameter("end_blackboard_id")) {
    node->declare_parameter("end_blackboard_id", std::string("end_goal"));
  }

  end_goal_blackboard_id_ = node->get_parameter("end_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }

  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  if (!node->has_parameter("return_blackboard_id")) {
    node->declare_parameter("return_blackboard_id", std::string("return_goal"));
  }

  return_goal_blackboard_id_ = node->get_parameter("return_blackboard_id").as_string();

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());
  loadSemanticGoals();
  loadTaskBehaviorTrees(parent_node);
  return true;
}

std::string DeliveryScenario::getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) {
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter(default_bt_parameter_name_)) {
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("robotics_scenario");
    std::string default_bt_file = pkg_share_dir + "/behavior_trees/" + scene_id_ + "_delivery.xml";
    node->declare_parameter<std::string>(default_bt_parameter_name_, default_bt_file);
  }

  node->get_parameter(default_bt_parameter_name_, default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool DeliveryScenario::cleanup() {
  goal_sub_.reset();
  self_client_.reset();
  return true;
}

bool DeliveryScenario::goalReceived(ActionT::Goal::ConstSharedPtr goal) {
  if (!goal->scene_id.empty() && goal->scene_id != scene_id_) {
    RCLCPP_ERROR(logger_, "Goal scene_id '%s' does not match scenario '%s'", goal->scene_id.c_str(), scene_id_.c_str());
    return false;
  }

  auto bt_xml_filename = goal->behavior_tree;
  if (bt_xml_filename.empty() || bt_xml_filename == "None") {
    bt_xml_filename = getDefaultBTForTask(goal->task_type);
  }
  RCLCPP_INFO(logger_, "bt_name: %s", bt_xml_filename.c_str());

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(logger_, "BT file not found");
    return false;
  }

  initializeGoalPose(goal);

  return true;
}

void DeliveryScenario::goalCompleted(typename ActionT::Result::SharedPtr result,
                                     const nav2_behavior_tree::BtStatus final_bt_status) {
  result->result = final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED;
  switch (final_bt_status) {
  case nav2_behavior_tree::BtStatus::SUCCEEDED:
    result->final_status = "SUCCEEDED";
    result->task_status = "COMPLETED";
    break;
  case nav2_behavior_tree::BtStatus::FAILED:
    result->final_status = "FAILED";
    result->task_status = "FAILED";
    break;
  case nav2_behavior_tree::BtStatus::CANCELED:
    result->final_status = "CANCELED";
    result->task_status = "CANCELED";
    break;
  }
}

void DeliveryScenario::onLoop() {
  auto feedback = std::make_shared<ActionT::Feedback>();
  feedback->feedback = 0;
  feedback->current_phase = "EXECUTING";
  bt_action_server_->publishFeedback(feedback);
}

void DeliveryScenario::onPreempt(ActionT::Goal::ConstSharedPtr goal) {
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
      (goal->behavior_tree.empty() &&
       bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename())) {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    initializeGoalPose(bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(logger_, "Preemption request was rejected since the requested BT XML file is not the same "
                         "as the one that the current goal is executing. Preemption with a new BT is invalid "
                         "since it would require cancellation of the previous goal instead of true preemption."
                         "\nCancel the current goal and send a new action request if you want to use a "
                         "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

void DeliveryScenario::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal) {
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0); // NOLINT

  // // Update the goal pose on the blackboard
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  blackboard->set<geometry_msgs::msg::PoseStamped>("initial_goal", pose);
  blackboard->set<geometry_msgs::msg::PoseStamped>(start_goal_blackboard_id_, goal->start_pose);
  blackboard->set<geometry_msgs::msg::PoseStamped>(end_goal_blackboard_id_, goal->end_pose);
  blackboard->set<geometry_msgs::msg::PoseStamped>(return_goal_blackboard_id_, goal->return_pose);

  std::vector<geometry_msgs::msg::PoseStamped> route;
  if (!goal->semantic_route.empty()) {
    for (const auto &semantic_id : goal->semantic_route) {
      geometry_msgs::msg::PoseStamped semantic_pose;
      if (!lookupSemanticGoal(semantic_id, semantic_pose)) {
        RCLCPP_WARN(logger_, "Semantic goal '%s' was not found, skipping it", semantic_id.c_str());
        continue;
      }
      route.push_back(semantic_pose);
    }
  }

  if (route.empty()) {
    route.push_back(goal->start_pose);
    route.push_back(goal->end_pose);
    route.push_back(goal->return_pose);
  }

  if (route.empty()) {
    route.push_back(pose);
  }

  for (size_t i = 0; i < 8; ++i) {
    const auto &route_pose = route[std::min(i, route.size() - 1)];
    blackboard->set<geometry_msgs::msg::PoseStamped>("route_goal_" + std::to_string(i), route_pose);
  }

  if (!goal->semantic_route.empty()) {
    std::ostringstream route_stream;
    for (size_t i = 0; i < goal->semantic_route.size(); ++i) {
      if (i != 0) {
        route_stream << " -> ";
      }
      route_stream << goal->semantic_route[i];
    }
    RCLCPP_INFO(logger_, "[%s] semantic route: %s", scene_id_.c_str(), route_stream.str().c_str());
  }
}

geometry_msgs::msg::PoseStamped DeliveryScenario::makePose(double x, double y, double yaw, const std::string &floor) const {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  (void)floor;
  return pose;
}

void DeliveryScenario::loadSemanticGoals() {
  semantic_goals_.clear();
  const auto pkg_share_dir = ament_index_cpp::get_package_share_directory("robotics_scenario");
  const auto semantic_path = pkg_share_dir + "/param/" + scene_id_ + "_semantic_goals.yaml";
  std::ifstream file(semantic_path);
  if (!file.is_open()) {
    RCLCPP_WARN(logger_, "Could not open semantic goals file: %s", semantic_path.c_str());
    return;
  }

  std::string line;
  std::string current_goal;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string floor;
  bool has_x = false;
  bool has_y = false;
  bool has_yaw = false;

  auto trim = [](std::string value) {
    const auto first = value.find_first_not_of(" \t\r\"");
    const auto last = value.find_last_not_of(" \t\r\"");
    if (first == std::string::npos || last == std::string::npos) {
      return std::string();
    }
    return value.substr(first, last - first + 1);
  };

  auto flush_goal = [&]() {
    if (!current_goal.empty() && has_x && has_y && has_yaw) {
      semantic_goals_[current_goal] = makePose(x, y, yaw, floor);
    }
  };

  while (std::getline(file, line)) {
    const auto first = line.find_first_not_of(" \t");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    const auto trimmed = trim(line.substr(first));
    if (trimmed.empty()) {
      continue;
    }

    if (line.rfind("  ", 0) == 0 && line.rfind("    ", 0) != 0 && trimmed.back() == ':') {
      flush_goal();
      current_goal = trimmed.substr(0, trimmed.size() - 1);
      x = 0.0;
      y = 0.0;
      yaw = 0.0;
      floor.clear();
      has_x = false;
      has_y = false;
      has_yaw = false;
      continue;
    }

    const auto separator = trimmed.find(':');
    if (separator == std::string::npos || current_goal.empty()) {
      continue;
    }
    const auto key = trimmed.substr(0, separator);
    const auto value = trim(trimmed.substr(separator + 1));

    if (key == "x") {
      x = std::stod(value);
      has_x = true;
    } else if (key == "y") {
      y = std::stod(value);
      has_y = true;
    } else if (key == "yaw") {
      yaw = std::stod(value);
      has_yaw = true;
    } else if (key == "floor") {
      floor = value;
    }
  }

  flush_goal();
  RCLCPP_INFO(logger_, "Loaded %zu semantic goals from %s", semantic_goals_.size(), semantic_path.c_str());
}

bool DeliveryScenario::lookupSemanticGoal(const std::string &goal_id, geometry_msgs::msg::PoseStamped &pose) const {
  const auto it = semantic_goals_.find(goal_id);
  if (it == semantic_goals_.end()) {
    return false;
  }
  pose = it->second;
  return true;
}

void DeliveryScenario::loadTaskBehaviorTrees(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) {
  task_behavior_trees_.clear();

  auto node = parent_node.lock();
  const auto pkg_share_dir = ament_index_cpp::get_package_share_directory("robotics_scenario");

  auto ensure_parameter = [&](const std::string &parameter_name, const std::string &default_path) {
    if (!node->has_parameter(parameter_name)) {
      node->declare_parameter<std::string>(parameter_name, default_path);
    }

    std::string configured_path;
    node->get_parameter(parameter_name, configured_path);
    return configured_path;
  };

  task_behavior_trees_["default"] =
      ensure_parameter(default_bt_parameter_name_, pkg_share_dir + "/behavior_trees/" + scene_id_ + "_delivery.xml");

  if (scene_id_ == "warehouse") {
    task_behavior_trees_["patrol_loop"] = ensure_parameter(
        "default_warehouse_patrol_bt_xml", pkg_share_dir + "/behavior_trees/warehouse_patrol.xml");
    task_behavior_trees_["stage2_demo"] = ensure_parameter(
        "default_warehouse_stage2_demo_bt_xml", pkg_share_dir + "/behavior_trees/warehouse_demo.xml");
  } else if (scene_id_ == "office") {
    task_behavior_trees_["patrol_loop"] = ensure_parameter(
        "default_office_patrol_bt_xml", pkg_share_dir + "/behavior_trees/office_patrol.xml");
    task_behavior_trees_["stage2_demo"] = ensure_parameter(
        "default_office_stage2_demo_bt_xml", pkg_share_dir + "/behavior_trees/office_demo.xml");
  }
}

std::string DeliveryScenario::getDefaultBTForTask(const std::string &task_type) const {
  const auto it = task_behavior_trees_.find(task_type);
  if (it != task_behavior_trees_.end()) {
    return it->second;
  }

  const auto fallback = task_behavior_trees_.find("default");
  if (fallback != task_behavior_trees_.end()) {
    return fallback->second;
  }

  return bt_action_server_->getDefaultBTFilename();
}

} // namespace robotics_scenario
