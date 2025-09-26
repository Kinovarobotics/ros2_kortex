cmake_minimum_required(VERSION 3.14)
include(FetchContent)

project(kortex_api)

#Future:
# convert API_URL components from CMAKE_SYSTEM variables.
# The kinova artifactory server artifacts have slightly different expectations and they need some transformations
# Could build something like: (URL_OS)_(URL_PROCESSOR)_(?)_(URL_COMPILER).zip
# string(REPLACE "_" "-" URL_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR}) # x86_64 -> x86-64
# string(TOLOWER ${CMAKE_SYSTEM_NAME} URL_OS) # Linux -> linux

#Current: only support Linux x86_64 and aarch64
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  FetchContent_Declare(
    kinova_binary_api
    URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.6.0/linux_aarch64_gcc_7.4.zip
    URL_HASH MD5=d7771a7c106888717ace359164d40235
  )
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  FetchContent_Declare(
    kinova_binary_api
    URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.6.0/linux_x86-64_x86_gcc.zip
    URL_HASH MD5=9d787a0f5405f02031db1e01dffa408f
  )
else()
  set(API_URL "")
  message(FATAL_ERROR "Unsupported System: currently support is for Linux x68_64 or aarch64. Detected ${CMAKE_SYSTEM_NAME} and ${CMAKE_SYSTEM_PROCESSOR}")
endif()

FetchContent_MakeAvailable(kinova_binary_api)

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include/kortex_api)

install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/client/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/common/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/messages/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/client_stubs/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/google/protobuf DESTINATION include/kortex_api/google)

ament_package()
