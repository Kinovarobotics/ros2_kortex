cmake_minimum_required(VERSION 3.14)
project(kortex_api)

include(FetchContent)

# Select correct Kinova Binary API zip based on platform

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(API_URL "https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.7.0/linux_x86-64_gcc_5.4.zip")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(API_URL "https://artifactory.kinovaapps.com:443/artifactory/generic-local-public/kortex/API/2.7.0/linux_aarch64_gcc_7.4.zip")
else()
  set(API_URL "")
  message(FATAL_ERROR "Unsupported System: Currently support is for Linux x68_64 or aarch64. Detected ${CMAKE_SYSTEM_NAME} and ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Download/unpack the Kinova binary API headers
FetchContent_Declare(
  kinova_binary_api
  URL ${API_URL}
)

FetchContent_MakeAvailable(kinova_binary_api)

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include/kortex_api)

install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/client/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/common/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/messages/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/client_stubs/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/google/protobuf DESTINATION include/kortex_api/google)

ament_package()
