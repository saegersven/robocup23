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

class Rescue {
private:
	std::shared_ptr<Robot> robot;

	std::thread::native_handle_type native_handle;


	void rescue();
public:
	std::atomic<bool> finished;

	Rescue(std::shared_ptr<Robot> robot);
	void start();
	void stop();
};