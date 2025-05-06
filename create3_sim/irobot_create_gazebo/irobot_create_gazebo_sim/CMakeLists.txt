cmake_minimum_required(VERSION 3.5)
project(irobot_create_gazebo_sim)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs -ftest-coverage")

find_package(ament_cmake REQUIRED)
ament_package()
