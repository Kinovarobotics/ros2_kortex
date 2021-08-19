cmake_minimum_required(VERSION 3.5)
project(kortex2_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# this is needed by the kortex_api to tell it we are compiling for linux
add_definitions(-D_OS_UNIX=1)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(forward_command_controller REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  forward_command_controller
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

## COMPILE
add_library(twist_controller SHARED
  src/twist_controller.cpp
)
target_include_directories(twist_controller PRIVATE include)
ament_target_dependencies(twist_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(twist_controller PRIVATE "TWIST_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(twist_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface twist_controller_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)

# INSTALL
install(
  TARGETS twist_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  diff_drive_controller
)
ament_package()
