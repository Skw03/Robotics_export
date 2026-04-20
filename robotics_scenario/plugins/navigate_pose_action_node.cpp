#include "robotics_scenario/plugins/navigate_pose_action_node.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config) {
    return std::make_unique<robotics_scenario::NavigatePoseActionNode>(name, config);
  };

  factory.registerBuilder<robotics_scenario::NavigatePoseActionNode>("ComputeGoalToTopology", builder);
}
