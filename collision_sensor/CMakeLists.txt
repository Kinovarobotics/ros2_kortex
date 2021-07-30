cmake_minimum_required(VERSION 3.5)
project(collision_sensor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

## A low-level library for contact detection
add_library(
  contact_detection
  SHARED
  src/contact_detection/butterworth_low_pass_filter.cpp
  src/contact_detection/contact_detection.cpp
  src/contact_detection/filter_base_class.cpp
)
target_include_directories(
  contact_detection
  PRIVATE
  include
)
ament_target_dependencies(
  contact_detection
  rclcpp
)

## ros2_control sensor library
add_library(
  collision_sensor
  SHARED
  src/collision_sensor.cpp
)
target_include_directories(
  collision_sensor
  PUBLIC
  include
)
ament_target_dependencies(
  collision_sensor
  sensor_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)
target_link_libraries(collision_sensor contact_detection)

## Simple binary for testing
add_executable(
  test_from_bag
  src/test_from_bag.cpp
)
target_include_directories(
  test_from_bag
  PUBLIC
  include
)
ament_target_dependencies(
  test_from_bag
  sensor_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
target_link_libraries(test_from_bag contact_detection)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(collision_sensor PRIVATE "COLLISION_SENSOR_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(collision_sensor PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  controller_interface collision_sensor.xml)

# Install Libraries
install(
  TARGETS
    collision_sensor
    contact_detection
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install binaries
install(
  TARGETS
    test_from_bag
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  collision_sensor
  contact_detection
)
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  hardware_interface
)

ament_package()
