cmake_minimum_required(VERSION 3.18)
project(kortex_api)

find_package(ament_cmake REQUIRED)

if(NOT (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64"))
  message(FATAL_ERROR "The bundled Kortex API supports Linux x86_64. Detected ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}.")
endif()

# Extract the bundled SDK even when Debian disables FetchContent.
set(sdk_archive "${CMAKE_CURRENT_SOURCE_DIR}/linux_x86-64_gcc_5.4.zip")
set(sdk_dir "${CMAKE_CURRENT_BINARY_DIR}/sdk")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${sdk_archive}")
file(ARCHIVE_EXTRACT INPUT "${sdk_archive}" DESTINATION "${sdk_dir}")

ament_export_include_directories(include/kortex_api)

install(DIRECTORY ${sdk_dir}/include/client/ DESTINATION include/kortex_api)
install(DIRECTORY ${sdk_dir}/include/common/ DESTINATION include/kortex_api)
install(DIRECTORY ${sdk_dir}/include/messages/ DESTINATION include/kortex_api)
install(DIRECTORY ${sdk_dir}/include/client_stubs/ DESTINATION include/kortex_api)
install(DIRECTORY ${sdk_dir}/include/google/protobuf DESTINATION include/kortex_api/google)

install(FILES "${sdk_dir}/lib/release/libKortexApiCpp.a" DESTINATION lib/kortex_api)

ament_package(CONFIG_EXTRAS cmake/kortex_api_library.cmake)
