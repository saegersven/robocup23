#pragma once

#include <cstdlib>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "robot.h"
#include "utils.h"
#include "defines.h"

#define BLACK_MAX_SUM 200

#define GREEN_RATIO_THRESHOLD 0.75f
#define GREEN_MIN_VALUE 25

#define GREEN_RESULT_LEFT 1
#define GREEN_RESULT_RIGHT 2
#define GREEN_RESULT_DEAD_END 3

// LINE PARAMETERS
#define LINE_FRAME_WIDTH 80
#define LINE_FRAME_HEIGHT 48

#define LINE_FOLLOW_BASE_SPEED 55
#define LINE_FOLLOW_SENSITIVITY 70.0f

/**
 * Single-pixel thresholding operation for line.
 */
bool is_black(uint8_t b, uint8_t g, uint8_t r);

/**
 * Thresholding operation for green dots.
 */
bool is_green(uint8_t b, uint8_t g, uint8_t r);

/**
 * A group of pixels with a center and the number of pixels (size).
 * Used to describe green dots.
 */
struct Group {
	float x, y;
	uint32_t num_pixels;
}

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

	// Camera control
	bool camera_opened;
	void open_camera();
	void close_camera();
	void grab_frame();

	/*########################
	METHODS FOR LINE-FOLLOWING
	########################*/

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

	/**
	 * Calls get_line_angle and draws result to debug frame.
	 * Adjusts motor speeds to follow line.
	 */
	void follow();

	/*#######################
	METHODS FOR INTERSECTIONS
	#######################*/

	/**
	 * Methods to find groups/contours of a single color recursively.
	 * Used to find green dots.
	 */
	void add_to_group_center(int x_pos, int y_pos, cv::Mat ir, uint32_t& num_pixels, float& center_x, float& center_y);
	std::vector<Group> find_groups(cv::Mat frame, cv::Mat ir, std::function<bool (uint8_t, uint8_t, uint8_t)> f);

	/**
	 * Determine direction of intersection.
	 * Return values:
	 * 0 -> No intersection
	 * 1 -> Go left
	 * 2 -> Go right
	 * 3 -> Turn around (both left and right)
	 */
	uint8_t green_direction(float& global_average_x, float& global_average_y);

	/**
	 * Main green method. Calls green_direction, approaches intersection,
	 * calls green_direction again, then makes final decision and turns.
	 */
	void green();

public:
	Line(std::shared_ptr<Robot> robot);

	void start();
	void stop();

	/**
	 * Main line method. Handles cameras and calls methods.
	 */
	void line();
};