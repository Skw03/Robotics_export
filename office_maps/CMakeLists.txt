cmake_minimum_required(VERSION 3.5)

project(office_maps)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY generated_maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()
