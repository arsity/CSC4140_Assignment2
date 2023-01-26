cmake_minimum_required(VERSION 3.24)
project(CSC4140_Assignment2)

set(CMAKE_CXX_STANDARD 20)

# to avoid missing headers
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3/")

add_executable(CSC4140_Assignment2 src/main.cpp src/rasterizer.cpp src/Triangle.cpp)

# to avoid missing headers
target_link_libraries(CSC4140_Assignment2 ${OpenCV_LIBS})