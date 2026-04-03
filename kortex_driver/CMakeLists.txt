cmake_minimum_required(VERSION 3.14)
include(FetchContent)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  FetchContent_Declare(
    kinova_binary_api
    URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.7.0/linux_x86-64_gcc_5.4.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64"))
  FetchContent_Declare(
    kinova_binary_api
    URL https://artifactory.kinovaapps.com:443/artifactory/generic-local-public/kortex/API/2.7.0/linux_aarch64_gcc_7.4.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
else()
  message(FATAL_ERROR "Unsupported System: currently support is for Linux x86_64 and Linux aarch64. Detected ${CMAKE_SYSTEM_NAME} and ${CMAKE_SYSTEM_PROCESSOR}")
endif()
FetchContent_MakeAvailable(kinova_binary_api)

project(kortex_driver)

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
