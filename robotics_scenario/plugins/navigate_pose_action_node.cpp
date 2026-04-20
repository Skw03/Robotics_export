#include "robotics_scenario/plugins/navigate_pose_action_node.hpp"

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace robotics_scenario {

NavigatePoseActionNode::NavigatePoseActionNode(const std::string &xml_tag_name, const std::string &action_name,
                                               const BT::NodeConfiguration &conf)
    : nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf) {}

void NavigatePoseActionNode::on_tick() {
  if (!getInput("goal", goal_.pose)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigatePoseActionNode: goal not provided");
    return;
  }

  getInput("behavior_tree", goal_.behavior_tree);
}

} // namespace robotics_scenario

BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config) {
    return std::make_unique<robotics_scenario::NavigatePoseActionNode>(name, "navigate_to_pose", config);
  };

  factory.registerBuilder<robotics_scenario::NavigatePoseActionNode>("ComputeGoalToTopology", builder);
}
