cmake_minimum_required(VERSION 3.22)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

message(STATUS "cmake standard: ${CMAKE_CXX_STANDARD}")

project(udp_server)

find_package(Boost REQUIRED COMPONENTS system)

add_executable(udp_test main.cpp)

target_link_libraries(udp_test ${Boost_SYSTEM_LIBRARY})
