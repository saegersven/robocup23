cmake_minimum_required(VERSION 3.16)
project(robocup VERSION 23)

set(TFLITE_ENABLE_XNNPACK OFF)
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -latomic")

#add_subdirectory(
#  "libs/tensorflow_src/tensorflow/lite"
#  "libs/tflite_build/tensorflow-lite"
#  EXCLUDE_FROM_ALL)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(WiringPi REQUIRED)
include_directories(${WIRINGPI_INCLUDE_DIRS})

find_package(Threads REQUIRED)

#include_directories("libs/tensorflow_src")
#include_directories("libs/tflite_build/flatbuffers/include")
include_directories("libs/tensorflow-2.11.0")

#find_package(Eigen3 3.3 REQUIRED NO_MODULE)

#include_directories("${EIGEN3_INCLUDE_DIR}")

include_directories("libs")

add_library(bno055-driver STATIC libs/BNO055_driver/bno055.c libs/BNO055_driver/bno055.h)

#add_library(vl53l0x-linux STATIC libs/vl53l0x-linux/VL53L0X.cpp libs/vl53l0x-linux/VL53L0X.hpp libs/vl53l0x-linux/VL53L0X_defines.hpp)

add_executable(robocup src/main.cpp src/robot.cpp src/utils.cpp src/line.cpp
    src/silver_ml.cpp src/victim_ml.cpp src/corner_ml.cpp libs/BNO055_driver/bno055.c)

target_link_libraries(robocup bno055-driver)
#target_link_libraries(robocup vl53l0x-linux)
target_link_libraries(robocup atomic)
target_link_libraries(robocup ${OpenCV_LIBS})
target_link_libraries(robocup tensorflow-lite)
target_link_libraries(robocup ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(robocup ${WIRINGPI_LIBRARIES})
target_link_libraries(robocup i2c)
target_link_libraries(robocup tensorflow-lite)
#target_link_libraries(robocup Eigen3::Eigen)
