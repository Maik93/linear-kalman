cmake_minimum_required(VERSION 2.8.4)
project(linear-kalman)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(example kalman.cpp example.cpp)
