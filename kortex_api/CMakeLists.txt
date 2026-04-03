cmake_minimum_required(VERSION 3.14)
include(FetchContent)
#include(ExternalProject)

project(kortex_api)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  FetchContent_Declare(
    kinova_binary_api
    URL ${CMAKE_CURRENT_SOURCE_DIR}/linux_x86-64_x86_gcc.zip
    URL_HASH MD5=64bd86e7ab8bda90ef1fc7d6a356e080
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(KORTEX_API_SUPPORTED TRUE)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64"))
  FetchContent_Declare(
    kinova_binary_api
    URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.7.0/linux_aarch64_gcc_7.4.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(KORTEX_API_SUPPORTED TRUE)
else()
  set(KORTEX_API_SUPPORTED FALSE)
  message(WARNING
    "kortex_api: Unsupported platform (${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}). "
    "The Kortex hardware interface will not be built. "
    "Supported platforms: Linux x86_64, Linux aarch64."
  )
endif()

find_package(ament_cmake REQUIRED)

if(KORTEX_API_SUPPORTED)
  FetchContent_MakeAvailable(kinova_binary_api)

  ament_export_include_directories(include/kortex_api)

  install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/client/ DESTINATION include/kortex_api)
  install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/common/ DESTINATION include/kortex_api)
  install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/messages/ DESTINATION include/kortex_api)
  install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/client_stubs/ DESTINATION include/kortex_api)
  install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/google/protobuf DESTINATION include/kortex_api/google)
endif()

ament_package()
