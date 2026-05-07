#ifndef __ROBOTICS_DELIVERY_SCENARIO_H__
#define __ROBOTICS_DELIVERY_SCENARIO_H__

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotics_interfaces/action/delivery.hpp"
#include "robotics_scenario/scenario/scenario.hpp"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotics_scenario {
/**
 * @class DeliveryScenario
 * @brief A navigator for navigating to a specified pose
 */
class DeliveryScenario : public robotics_scenario::Scenario<robotics_interfaces::action::Delivery> {
public:
  using ActionT = robotics_interfaces::action::Delivery;

  /**
   * @brief A constructor for DeliveryScenario
   */
  DeliveryScenario(const std::string &action_name, const std::string &scene_id,
                   const std::string &default_bt_parameter_name)
      : Scenario(), action_name_(action_name), scene_id_(scene_id),
        default_bt_parameter_name_(default_bt_parameter_name) {}

  /**
   * @brief A configure state transition to configure navigator's state
   * @param node Weakptr to the lifecycle node
   * @param odom_smoother Object to get current smoothed robot's speed
   */
  bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  /**
   * @brief A cleanup state transition to remove memory allocated
   */
  bool cleanup() override;

  /**
   * @brief Get action name for this navigator
   * @return string Name of action server
   */
  std::string getName() override { return action_name_; }

  /**
   * @brief Get navigator's default BT
   * @param node WeakPtr to the lifecycle node
   * @return string Filepath to default XML
   */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   * @param goal Action template's goal message
   * @return bool if goal was received successfully to be processed
   */
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  void onLoop() override;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that is called when a the action is completed, can fill in
   * action result message or indicate that this action is done.
   * @param result Action template result message to populate
   * @param final_bt_status Resulting status of the behavior tree execution that may be
   * referenced while populating the result.
   */
  void goalCompleted(typename ActionT::Result::SharedPtr result,
                     const nav2_behavior_tree::BtStatus final_bt_status) override;

  /**
   * @brief Goal pose initialization on the blackboard
   * @param goal Action template's goal message to process
   */
  void initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);
  geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw, const std::string &floor) const;
  void loadSemanticGoals();
  bool lookupSemanticGoal(const std::string &goal_id, geometry_msgs::msg::PoseStamped &pose) const;
  void loadTaskBehaviorTrees(rclcpp_lifecycle::LifecycleNode::WeakPtr node);
  std::string getDefaultBTForTask(const std::string &task_type) const;

  rclcpp::Time start_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string start_goal_blackboard_id_;
  std::string end_goal_blackboard_id_;
  std::string return_goal_blackboard_id_;
  std::string path_blackboard_id_;
  std::string action_name_;
  std::string scene_id_;
  std::string default_bt_parameter_name_;
  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> semantic_goals_;
  std::unordered_map<std::string, std::string> task_behavior_trees_;
};

} // namespace robotics_scenario
#endif // __DELIVERY_SCENARIO_H__
