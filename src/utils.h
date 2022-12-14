#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>
#include <functional>
#include <thread>

// CONSTANTS
// TODO: Too inaccurate
#define PI (3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679)
#define PI05 (PI / 2.0f)
#define PI2 (PI * 2.0f)

#define R180 PI
#define R90 PI05

/**
 * Degrees to Radians and
 * Radians to Degrees macros
 */
#define DTOR(x) (x / 180.0f * PI)
#define RTOD(x) (x * 180.0f / PI)

/**
 * Millisecond delay
 */
#define delay(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

/**
 * Milliseconds since January 1st, 1970 at 00:00:00
 */
#define millis() std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()

/**
 * Performs a binary operation on an image. Applies the specified function on every pixel
 * and colors pixels white (0xFF) or black (0x00) depending on function return value.
 */
cv::Mat in_range(cv::Mat in, std::function<bool (uint8_t, uint8_t, uint8_t)> f, uint32_t* num_pixels = nullptr);

/**
 * Force a number into the specified range.
 */
float clamp(float n, float min, float max);

void save_img(cv::Mat& img, const std::string& subfolder);