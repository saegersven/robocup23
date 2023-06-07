cmake_minimum_required(VERSION 3.16)
project(capture_from_camera VERSION 23)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(capture_from_camera capture_from_camera.cpp)

target_link_libraries(capture_from_camera ${OpenCV_LIBS})