#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>

#define delay(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms));

cv::Mat in_range(cv::Mat in, std::function<bool (uint8_t, uint8_t, uint8_t)> f, uint32_t* num_pixels = nullptr);