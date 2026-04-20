#include "robotics_scenario/robotics_scenario.hpp"

namespace robotics_scenario {
ScenarioManager::ScenarioManager() : nav2_util::LifecycleNode("robotics_scenario", "") {
  m_hotel_delivery_scenario =
      std::make_unique<DeliveryScenario>("hotel_delivery_scenario", "hotel", "default_hotel_delivery_bt_xml");
  m_warehouse_delivery_scenario = std::make_unique<DeliveryScenario>("warehouse_delivery_scenario", "warehouse",
                                                                     "default_warehouse_delivery_bt_xml");
}

nav2_util::CallbackReturn ScenarioManager::on_configure(const rclcpp_lifecycle::State & /*state*/) {
  std::vector<std::string> plugin_lib_names = {
      "nav2_recovery_node_bt_node",           "nav2_pipeline_sequence_bt_node",
      "nav2_round_robin_node_bt_node",        "nav2_navigate_through_poses_action_bt_node",
      "nav2_navigate_to_pose_action_bt_node", "nav2_is_battery_charging_condition_bt_node",
      "nav2_wait_action_bt_node"};
  if (!m_hotel_delivery_scenario->on_configure(shared_from_this(), plugin_lib_names, &m_muxer)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (!m_warehouse_delivery_scenario->on_configure(shared_from_this(), plugin_lib_names, &m_muxer)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ScenarioManager::on_activate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Activating");

  if (!m_hotel_delivery_scenario->on_activate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (!m_warehouse_delivery_scenario->on_activate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ScenarioManager::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (!m_hotel_delivery_scenario->on_deactivate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (!m_warehouse_delivery_scenario->on_deactivate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ScenarioManager::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  if (!m_hotel_delivery_scenario->on_cleanup()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (!m_warehouse_delivery_scenario->on_cleanup()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  m_hotel_delivery_scenario.reset();
  m_warehouse_delivery_scenario.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ScenarioManager::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

} // namespace robotics_scenario
