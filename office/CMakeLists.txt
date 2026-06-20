cmake_minimum_required(VERSION 3.8)
project(office)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dispatch_delivery
  scripts/dispatch_patrol
  scripts/dispatch_clean
  scripts/gui_delivery_test
  scripts/gui_patrol_test
  scripts/gui_clean_test
  scripts/office_status
  scripts/office_llm_command.py
  scripts/office_synthetic_lidar.py
  scripts/office_scan_mapper.py
  scripts/office_dynamic_obstacle_avoidance.py
  scripts/office_task_scheduler.py
  scripts/office_experiment.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
