cmake_minimum_required(VERSION 3.5)
project(gen3_robotiq_2f_85_move_it_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()