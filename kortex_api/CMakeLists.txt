cmake_minimum_required(VERSION 3.10.2)
project(kortex_api)

find_package(ament_cmake REQUIRED)

# TODO(marqrazz) This is needed by TransportClientTcp.h line14 but it doesn't see it??
add_definitions(-D_OS_UNIX)

ament_export_libraries(${CMAKE_CURRENT_SOURCE_DIR}/lib/release/libKortexApiCpp.a)
link_libraries(pthread)

ament_export_include_directories(include)

install(DIRECTORY include/client/ DESTINATION include)
install(DIRECTORY include/common/ DESTINATION include)
install(DIRECTORY include/messages/ DESTINATION include)
install(DIRECTORY include/client_stubs/ DESTINATION include)
install(DIRECTORY include/google/protobuf DESTINATION include/google)

ament_package()
