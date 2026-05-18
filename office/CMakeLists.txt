cmake_minimum_required(VERSION 3.8)
project(office)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dispatch_delivery
  scripts/gui_delivery_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
