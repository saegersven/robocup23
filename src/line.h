#pragma once

#include <cstdlib>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <functional>

#include <opencv2/opencv.hpp>

#include "robot.h"
#include "utils.h"
#include "defines.h"
#include "silver_ml.h"
#include "victim_ml.h"

#define BLACK_MAX_SUM 200

#define GREEN_RATIO_THRESHOLD 0.7f
#define GREEN_MIN_VALUE 14

#define BLUE_RATIO_THRESHOLD 1.7f
#define BLUE_MIN_VALUE 50

#define RED_RATIO_THRESHOLD 1.3f
#define RED_MIN_VALUE 20

#define EXIT_GREEN_RATIO_THRESHOLD 0.65f
#define EXIT_GREEN_MIN_VALUE 10

#define GREEN_RESULT_LEFT 1
#define GREEN_RESULT_RIGHT 2
#define GREEN_RESULT_DEAD_END 3

// LINE PARAMETERS
#define LINE_FRAME_WIDTH 80
#define LINE_FRAME_HEIGHT 48
#define LINE_CAPTURE_WIDTH 320
#define LINE_CAPTURE_HEIGHT 192

#define LINE_CORNER_WIDTH 7
#define LINE_CORNER_HEIGHT 8

#define LINE_FOLLOW_BASE_SPEED 48
#define LINE_FOLLOW_STRAIGHT_LINE_SPEED 60
#define LINE_FOLLOW_SENSITIVITY 57.0f
#define LINE_FOLLOW_D 0.05f

#define ARM_ANGLE_OFFSET DTOR(25.0f)

#define ENABLE_NO_DIFFERENCE true // should robot increase motor speed when frame = last_frame?

#define GREEN_WEIGHT_SLOPE 0.6f
#define GREEN_DURATION 500

/**
 * Single-pixel thresholding operations
 */
bool is_black(uint8_t b, uint8_t g, uint8_t r);
bool is_green(uint8_t b, uint8_t g, uint8_t r);
bool is_blue(uint8_t b, uint8_t g, uint8_t r);
bool is_red(uint8_t b, uint8_t g, uint8_t r);
bool is_exit_green(uint8_t b, uint8_t g, uint8_t r);

/**
 * A group of pixels with a center and the number of pixels (size).
 * Used to describe green dots.
 */
struct Group {
	float x, y;
	uint32_t num_pixels;
};

class Line {
private:
	cv::VideoCapture cap;
	std::shared_ptr<Robot> robot;

	// Maps
	cv::Mat distance_weight_map;
	cv::Mat pixel_angles_map;

	// Current frame from camera
	cv::Mat frame;
	// Last frame from camera
	cv::Mat last_frame;
	// Clone of frame for drawing debug data
	cv::Mat debug_frame;
	// Thresholded frame
	cv::Mat black;
	cv::Mat green_mat;
	uint32_t green_num_pixels;

	// used for silver testing
	bool test = false;
	
	// used to align before gaps. Currently not working
	bool correct_gap_line_angle = true;

	std::atomic<bool> running;
	std::atomic<bool> obstacle_enabled;
	std::atomic<bool> obstacle_active;

	float last_line_angle;

	uint32_t no_difference_counter = 0;
	long long no_difference_time_stamp = 0;

	float green_weight_slope = 0.0f;
	std::chrono::time_point<std::chrono::high_resolution_clock> green_start_t;
	bool green_active = false;

	std::chrono::time_point<std::chrono::high_resolution_clock> last_frame_t;

	SilverML silver_ml;

	void grab_frame();

	/**
	 * Async method, checks distance
	 */
	void obstacle();

	/**
	 * Goes in a straight line for a maximum duration and stops if the robot hits a black line.
	 */
	bool obstacle_straight_line(int duration);

	/*########################
	METHODS FOR LINE-FOLLOWING
	########################*/

	/**
	 * Function for determining difference weights.
	 * Input range -PI/2 to +PI/2  | TODO: Check
	 * Output range 0 to 1
	 */
	float difference_weight(float x);

	/**
	 * Function for determining the distance weights.
	 * Input range 0 to 1
	 * Output range 0 to 1
	 */
	float distance_weight(float x);

	float green_weight(float x);

	/**
	 * Creates maps to speed up line following.
	 * The distance_weight_map contains the distance weights from 0 to 1
	 * The pixel_angles_map contains the angle of every pixel from -PI/2 to +PI/2
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

	/*####################################
	METHODS FOR INTERSECTIONS & RESCUE KIT
	####################################*/

	/**
	 * Methods to find groups/contours of a single color recursively.
	 * Used to find green dots.
	 */
	void add_to_group_center(int x_pos, int y_pos, cv::Mat ir, uint32_t& num_pixels, float& center_x, float& center_y);
	std::vector<Group> find_groups(cv::Mat frame, cv::Mat& ir, std::function<bool (uint8_t, uint8_t, uint8_t)> f);

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

	/**
	 * Detects rescue kit if there is one, picks it up and returns to initial position.
	 */
	void rescue_kit();

	void red();

	void silver();

public:
	bool found_silver = false;

	Line(std::shared_ptr<Robot> robot);

	void start();
	void check_silver();
	void stop();

	/**
	 * Main line method. Handles cameras and calls methods.
	 */
	void line();
};
