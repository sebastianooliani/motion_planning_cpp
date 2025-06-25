cmake_minimum_required(VERSION 3.10)
project(motion_planning_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(discrete_planning discrete_planning.cpp utils.cpp)