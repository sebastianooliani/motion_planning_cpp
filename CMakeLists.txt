cmake_minimum_required(VERSION 3.10)

# Project name
project(motion_planning)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(include)

file(GLOB SOURCES "src/*.cpp")

# Add the executable
add_executable(main ${SOURCES})