cmake_minimum_required(VERSION 3.5)
project(kortex_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
