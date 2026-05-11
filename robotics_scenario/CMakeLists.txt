cmake_minimum_required(VERSION 3.8)
project(robotics_scenario)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(robotics_interfaces REQUIRED)
find_package(behaviortree_cpp_v3 3.8.4 REQUIRED)

message(STATUS "Current source directory: ${CMAKE_CURRENT_SOURCE_DIR}")

if(behaviortree_cpp_v3_FOUND)
    if(behaviortree_cpp_v3_VERSION)
        message(WARNING "Found behaviortree_cpp_v3 version: ${behaviortree_cpp_v3_VERSION}")
    else()
        message(WARNING "behaviortree_cpp_v3 found but version information is not available.")
    endif()
else()
    message(FATAL_ERROR "behaviortree_cpp_v3 not found!")
endif()

nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  nav2_util
  nav2_msgs
  robotics_interfaces
  behaviortree_cpp_v3
  nav2_behavior_tree
)

add_library(${library_name}
  src/robotics_scenario.cpp
  src/scenario/delivery_scenario.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(nav2_compute_goal_to_topology_bt_node SHARED
  plugins/navigate_pose_action_node.cpp
)
ament_target_dependencies(nav2_compute_goal_to_topology_bt_node
  rclcpp
  nav2_util
  nav2_msgs
  behaviortree_cpp_v3
  nav2_behavior_tree
)
target_compile_definitions(nav2_compute_goal_to_topology_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_executable(${library_name}_node
  apps/robotics_scenario_node.cpp
)
target_link_libraries(${library_name}_node
  ${library_name}
)

install(TARGETS ${library_name}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/course_task_dispatcher.py
  scripts/course_nl_command.py
  scripts/course_llm_command.py
  scripts/course_experiment_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/_course_task_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS nav2_compute_goal_to_topology_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch behavior_trees param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_tutorial_test test/tutorial_test.cpp)
  target_include_directories(${PROJECT_NAME}_tutorial_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_tutorial_test
    std_msgs
  )
  target_link_libraries(${PROJECT_NAME}_tutorial_test)
endif()

ament_package()
