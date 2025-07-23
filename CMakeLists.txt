cmake_minimum_required(VERSION 3.10)

# Project name
project(DiscretePlanning)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(discrete_planning discrete_planning.cpp)
