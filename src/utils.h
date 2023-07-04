#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>
#include <functional>
#include <thread>

// CONSTANTS
// TODO: Too inaccurate
#define PI (3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679)
#define PI025 (PI / 4.0f)
#define PI05 (PI / 2.0f)
#define PI2 (PI * 2.0f)

#define R180 PI
#define R90 PI05
#define R45 PI025

#define BOOL_DIR_LEFT false
#define BOOL_DIR_RIGHT true

/**
 * Degrees to Radians and
 * Radians to Degrees macros
 */
#define DTOR(x) ((x) / 180.0f * PI)
#define RTOD(x) ((x) * 180.0f / PI)

/**
 * Millisecond delay
 */
#define delay(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

/**
 * Microsecond delay
 */
#define delayMicros(ms) std::this_thread::sleep_for(std::chrono::microseconds(ms))

/**
 * Milliseconds since January 1st, 1970 at 00:00:00
 */
#define millis_() std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()

/**
 * Microseconds since January 1st, 1970 at 00:00:00
 */
#define micros_() std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()

/**
 * Performs a binary operation on an image. Applies the specified function on every pixel
 * and colors pixels white (0xFF) or black (0x00) accordingly.
 */
cv::Mat in_range(cv::Mat in, std::function<bool (uint8_t, uint8_t, uint8_t)> f, uint32_t* num_pixels = nullptr);

/**
 * Force a number into the specified range.
 */
float clamp(float n, float min, float max);

/**
 * Save iamge to Desktop/iamges/subfolder.
 */
void save_img(cv::Mat& img, const std::string& subfolder);

/**
 * Convert two channel image returned by victim neural net to three channel image for debugging.
 */
cv::Mat two_channel_to_three_channel(cv::Mat in);

/**
 * returns average difference between two given frames
 */
float average_difference(cv::Mat a, cv::Mat b);

float map(float s, float a1, float a2, float b1, float b2);
