#include "rescue.h"

#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <pthread.h>

#include "line.h"
#include "robot.h"
#include "utils.h"
Rescue::Rescue(std::shared_ptr<Robot> robot) : finished(false) {
	this->robot = robot;
}

void Rescue::start() {
	std::thread rescue_thread([this]() { this->rescue(); });
	this->native_handle = rescue_thread.native_handle();
	rescue_thread.detach();
}

void Rescue::stop() {
	pthread_cancel(this->native_handle);
}

// main routine for rescue area
void Rescue::rescue() {
	std::cout << "in rescue function" << std::endl;
	/*
	robot->m(127, 127, 500);
	// turn towards the (potential) wall with as little space as possible
	robot->turn(DTOR(22));
	robot->m(127, 127, 100);
	robot->turn(DTOR(15));
	robot->m(127, 127, 80);
	robot->turn(DTOR(15));
	robot->m(127, 127, 50);
	robot->turn(DTOR(15));
	robot->m(127, 127, 40);
	robot->turn(DTOR(45));

	if (robot->distance() < 40) {
		robot->m(-127, -127, 500);
		robot->turn(DTOR(-135));
	} else {
		robot->turn(DTOR(-45));
	}
	robot->m(127, 127, 600);

	// robot is roughly in the centre of the rescue area, no matter where the entrace was
	*/
	// search for victims
	open_camera();
	delay(1000);
	while (1) {
		grab_frame();
		cv::imshow("Frame", frame);
		std::cout << "Width : " << frame.cols << std::endl;
		std::cout << "Height: " << frame.rows << std::endl;
		cv::waitKey(1);
	}
	exit(0);
}

void Rescue::open_camera() {
	if(camera_opened) return;

	cap.open(0, cv::CAP_V4L2);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, RESCUE_FRAME_WIDTH * 4);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, RESCUE_FRAME_HEIGHT * 4);
	cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);
	cap.set(cv::CAP_PROP_FPS, 120);

	if(!cap.isOpened()) {
		std::cerr << "Could not open camera" << std::endl;
		exit(ERRCODE_CAM_SETUP);
	}
	camera_opened = true;
}

void Rescue::close_camera() {
	if(!camera_opened) return;

	cap.release();
	camera_opened = false;
}

void Rescue::grab_frame() {
	if(!camera_opened) {
		std::cerr << "Tried to grab frame with closed camera" << std::endl;
		exit(ERRCODE_CAM_CLOSED);
	}

	cap.grab();
	cap.retrieve(frame);
	cv::resize(frame, frame, cv::Size(RESCUE_FRAME_WIDTH, RESCUE_FRAME_HEIGHT));
	debug_frame = frame.clone();
	cv::flip(debug_frame, frame, 0);
	cv::flip(frame, debug_frame, 1);
	frame = debug_frame.clone();
}

// finds black corner. Important: robot needs to be roughly in centre of rescue area allowing for 360 deg turns
void Rescue::find_black_corner() {
	open_camera();
	// TODO: either implement own algorithm, or use NN
	// ideas to detect black corner:
	// - many black pixels in frame
	// - contours (the one around the black corner) width is larger than height
	// - distance < 120cm
	// - contours width > contours height (at least when robot is far away from black corner)
	// - make us of adjustable cam angle? Maybe start with low angle and incrementally increase angle when theres no large black contour
	// - general problem: prevent the robot from approaching the corner at an oblique angle as it makes unloading the victims hard
}