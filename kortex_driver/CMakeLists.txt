cmake_minimum_required(VERSION 3.14)
project(kortex_driver)

include(FetchContent)

# Select correct Kinova Binary API zip based on platform
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(API_FILENAME "linux_x86-64_gcc_5.4.zip")
  set(API_URL "https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.7.0/${API_FILENAME}")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(API_FILENAME "linux_aarch64_gcc_7.4.zip")
  set(API_URL "https://artifactory.kinovaapps.com:443/artifactory/generic-local-public/kortex/API/2.7.0/${API_FILENAME}")
else()
  message(FATAL_ERROR "Unsupported System: Detected ${CMAKE_SYSTEM_NAME} and ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Use the local zip if present (required for offline builds such as the ROS build farm
# where dh_auto_configure sandboxes network access). Falls back to downloading from
# Kinova's Artifactory when the local file is absent.
set(_LOCAL_ZIP "${CMAKE_CURRENT_SOURCE_DIR}/kortex_api_zip/${API_FILENAME}")
if(EXISTS "${_LOCAL_ZIP}")
  set(FETCH_URL "file://${_LOCAL_ZIP}")
else()
  set(FETCH_URL "${API_URL}")
endif()

FetchContent_Declare(
  kinova_binary_api
  URL ${FETCH_URL}
)
FetchContent_MakeAvailable(kinova_binary_api)

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
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kortex_api REQUIRED)

# CMake does not allow IMPORTED libraries to be installed
# The package kortex_api will download and setup the include directories
add_library(KortexApiCpp STATIC IMPORTED)
set_target_properties(KortexApiCpp PROPERTIES
  IMPORTED_LOCATION ${kinova_binary_api_SOURCE_DIR}/lib/release/libKortexApiCpp.a
  INTERFACE_LINK_LIBRARIES KortexApiCpp
)
target_link_libraries(KortexApiCpp INTERFACE pthread)
add_dependencies(KortexApiCpp kortex_api)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/hardware_interface.cpp
  src/kortex_math_util.cpp
)
target_link_libraries(${PROJECT_NAME} KortexApiCpp)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
# kortex_api is the headers for the Kortex API
ament_target_dependencies(
  ${PROJECT_NAME}
  SYSTEM kortex_api
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  kortex_api
  pluginlib
  rclcpp
)
ament_package()
