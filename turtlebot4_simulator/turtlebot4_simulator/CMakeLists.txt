cmake_minimum_required(VERSION 3.8)
project(turtlebot4_simulator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs -ftest-coverage")

find_package(ament_cmake REQUIRED)
ament_package()
