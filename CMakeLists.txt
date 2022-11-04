cmake_minimum_required(VERSION 3.16)
project(robocup VERSION 23)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(WiringPi REQUIRED)
include_directories(${WIRINGPI_INCLUDE_DIRS})

find_package(Threads REQUIRED)

#add_subdirectory("libs/tensorflow_src/tensorflow/lite"
#  "libs/tflite_build/tensorflow_lite" EXCLUDE_FROM_ALL)

#include_directories("libs/tensorflow_src")
#include_directories("libs/tflite_build/flatbuffers/include")

#find_package(Eigen3 3.3 REQUIRED NO_MODULE)

#include_directories("${EIGEN3_INCLUDE_DIR}")

add_executable(robocup src/main.cpp src/robot.cpp src/utils.cpp src/line.cpp)

target_link_libraries(robocup atomic)
target_link_libraries(robocup ${OpenCV_LIBS})
#target_link_libraries(robocup tensorflow-lite)
target_link_libraries(robocup ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(robocup ${WIRINGPI_LIBRARIES})
target_link_libraries(robocup i2c)
#target_link_libraries(robocup Eigen3::Eigen)
