#pragma once

#include <cstdlib>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "robot.h"
#include "utils.h"

/**
 * Single-pixel thresholding operation
 */
bool is_black(uint8_t b, uint8_t g, uint8_t r);

class Line {
private:
	cv::VideoCapture cap;
	std::shared_ptr<Robot> robot;

	// Maps
	cv::Mat distance_weight_map;
	cv::Mat pixel_angles_map;

	// Current frame from camera
	cv::Mat frame;
	// Clone of frame for drawing debug data
	cv::Mat debug_frame;
	// Thresholded frame
	cv::Mat black;

	float last_line_angle;

	std::chrono::time_point<std::chrono::high_resolution_clock> last_frame_t;

	void grab_frame();

	/**
	 * Function for determining difference weights.
	 * Input range -0.5 to 0.5
	 * Output range 0 to 1
	 */
	float difference_weight(float x);

	/**
	 * Function for determining the distance weights.
	 * Input range 0 to 1
	 * Output range 0 to 1
	 */
	float distance_weight(float x);

	/**
	 * Creates maps to speed up line following.
	 * The distance_weight_map contains the distance weights from 0 to 255.
	 * The pixel_angles_map contains the angle of every pixel from 0 (-90°) to 255 (90°)
	 */
	void create_maps();

	/**
	 * Calculates line angle by using a weighted sum of the angles
	 * of all non-zero pixels. The angles are weighed based on the difference
	 * to the last line angle and the distance from the bottom center.
	 */
	float get_line_angle(cv::Mat in);
	float circular_line(cv::Mat& in);

	/**
	 * Calls get_line_angle and draws result to debug frame.
	 * Adjusts motor speeds to follow line.
	 */
	void follow();

public:
	Line(std::shared_ptr<Robot> robot);

	void start();
	void stop();

	/**
	 * Main line method. Handles cameras and calls subroutines.
	 */
	void line();
};