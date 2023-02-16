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
	void open_camera(int width, int height);
	void close_camera();
	cv::Mat grab_frame(int width, int height);
	void find_center();
	void find_center_new();
	void find_black_corner();
	void find_victims(float& x, float& y, bool ignore_dead);

	// Current frame from camera
	cv::Mat frame;
	// Clone of frame for drawing debug data
	cv::Mat debug_frame;


public:
	std::atomic<bool> finished;

	Rescue(std::shared_ptr<Robot> robot);
	void start();
	void stop();
	void rescue();
};