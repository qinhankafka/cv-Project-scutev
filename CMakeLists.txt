cmake_minimum_required(VERSION 3.16)
project(calibration)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV 4.3.0 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(calibration src/main.cpp src/calibrationtools.hpp src/topview.hpp )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})