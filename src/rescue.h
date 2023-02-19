#pragma once

#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>
#include <pthread.h>

#include "line.h"
#include "robot.h"
#include "utils.h"
#include "victim_ml.h"
#include "corner_ml.h"

#define RESCUE_FRAME_WIDTH 160
#define RESCUE_FRAME_HEIGHT 120

#define CAM_FAR 0
#define CAM_SHORT 1

#define ARM_UP 70
#define ARM_DOWN -90
#define ARM_ALMOST_DOWN -81
#define ARM_DROP -10

#define GRAB_OPEN -20
#define GRAB_CLOSED -100

#define CORNER_ROI_Y_MIN 0
#define CORNER_ROI_Y_MAX 480
#define CORNER_ROI_X_MIN 0
#define CORNER_ROI_X_MAX 640

#define X_TO_ANGLE(x_res, x) (x / x_res * DTOR(65.0f))

/**
 * Single-pixel thresholding operation for detecting the black corner.
 */
bool is_black2(uint8_t b, uint8_t g, uint8_t r);

class Rescue {
private:
	std::shared_ptr<Robot> robot;

	std::thread::native_handle_type native_handle;

	VictimML victim_ml;
	CornerML corner_ml;

	// Camera control
	cv::VideoCapture cap;
	bool camera_opened = false;

	// Current frame from camera
	cv::Mat frame;
	// Clone of frame for drawing debug data
	cv::Mat debug_frame;


	/**
	 * Open and close camera. Parameters are the resolution send to the camera.
	 */
	void open_camera(int width, int height);
	void close_camera();

	/**
	 * Grab a frame from the camera and resize it to the specified resolution.
	 */
	cv::Mat grab_frame(int width, int height);

	/**
	 * Drive to the center of the evacuation zone.
	 * Three different methods:
	 * 1. Turn and check distance in discrete steps
	 * 2. Turn and check distance, create vector out of average and move accordingly
	 * 3. Turn and check distance continuously
	 */
	void find_center();
	void find_center_new();
	void find_center_new_new();

	/**
	 * Use the corner neural network to find the corner.
	 */
	void find_black_corner();

	/**
	 * Use the victim neural network to find the closest victim.
	 */
	void find_victims(float& x, float& y, bool ignore_dead);

public:
	std::atomic<bool> finished;

	Rescue(std::shared_ptr<Robot> robot);

	/**
	 * Start rescue and all relevant threads.
	 */
	void start();

	/**
	 * Stop and reset rescue.
	 */
	void stop();

	/**
	 * Main rescue method, runs once from start to exit.
	 */
	void rescue();
};