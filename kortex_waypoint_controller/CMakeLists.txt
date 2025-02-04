cmake_minimum_required(VERSION 3.8)
project(kortex_waypoint_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(kortex_driver REQUIRED) # Ensure this exists if using messages from kortex_driver

# Executable
add_executable(execute_waypoint_server src/execute_joint_waypoints_server.cpp)
ament_target_dependencies(execute_waypoint_server rclcpp rclcpp_action kortex_driver)

install(TARGETS
  execute_waypoint_server
  DESTINATION lib/${PROJECT_NAME}
)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteJointWaypoints.action"
  DEPENDENCIES kortex_driver
)

# Needed code block since the custom interface used in the executable source code (ExecuteJointWaypoints.action 
# in this case) is defined in the same package that is using it. Otherwise ament_target_dependencies is enough 
#(this was done for JointAngle.msg) from kortex_driver 
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(execute_waypoint_server "${cpp_typesupport_target}")

ament_package()

