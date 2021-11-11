cmake_minimum_required(VERSION 3.5)
project(kortex2_hacks)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Subscribe to twist, convert to Float64MultiArray so we can jog the arm with the Kortex api
# Create the main functionality in a way allowing it to be composable
add_library(twist_to_array_republisher_lib SHARED
            src/twist_to_array_republisher.cpp)
target_compile_definitions(twist_to_array_republisher_lib
  PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(twist_to_array_republisher_lib rclcpp rclcpp_components geometry_msgs)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

# Executable for subscribing to twist, converting to Float64MultiArray so we can jog the arm with the Kortex api
add_executable(twist_to_array_republisher_main src/twist_to_array_republisher_main.cpp)
target_link_libraries(twist_to_array_republisher_main twist_to_array_republisher_lib)
ament_target_dependencies(twist_to_array_republisher_main
  rclcpp)

install(TARGETS
  twist_to_array_republisher_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  twist_to_array_republisher_main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
endif()

ament_package()
