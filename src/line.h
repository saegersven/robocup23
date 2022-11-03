#pragma once

#include <cstdlib>
#include <vector>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>

class Line {
private:
	// Maps
	cv::Mat distance_weight_map;
	cv::Mat pixel_angles_map;

	float last_line_angle;

	void create_maps();

	float difference_weight(float x);
	float distance_weight(float x);

	float line_angle(cv::Mat in);

public:
	void start();
	void stop();

	void line();
}