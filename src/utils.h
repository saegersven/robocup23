#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>
#include <functional>
#include <thread>

// CONSTANTS
#define PI (3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679)
#define PI05 (PI / 2.0f)
#define PI2 (PI * 2.0f)

#define R180 PI
#define R90 PI05

#define DTOR(x) (x / 180.0f * PI)
#define RTOR(x) (x * 180.0f / PI)

#define delay(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms));

cv::Mat in_range(cv::Mat in, std::function<bool (uint8_t, uint8_t, uint8_t)> f, uint32_t* num_pixels = nullptr);

float clamp(float n, float min, float max);