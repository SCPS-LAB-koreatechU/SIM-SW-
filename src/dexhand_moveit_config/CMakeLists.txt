cmake_minimum_required(VERSION 3.8)
project(dexhand_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
