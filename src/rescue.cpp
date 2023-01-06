#include "rescue.h"

#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <pthread.h>

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
	find_centre();
	*/
	find_black_corner();
	/*
	// search for victims
	open_camera();
	delay(1000);
	while (1) {
		std::cout << "searching for victims" << std::endl;
		grab_frame();
		cv::imshow("Frame", frame);
		cv::waitKey(1);
	}
	
	
	uint8_t rescued_victims = 0;
	bool ignore_dead = false;
	while (rescued_victims < 3) {
		grab_frame()
		if (victim_in_frame(ignore_dead)) {
			drive to victim
			if (victim is close enough) {
				pick up victim
				++rescued_victims
				if (rescued_victims == 2) ignore_dead = false
			}
		}
	}
	find_black_corner();
	unload victims()
	m(127, 127, 500); // drive to centre of rescue area
	find_exit();
	*/
	exit(0);
}

void Rescue::open_camera() {
	if(camera_opened) return;
	cap.open(0, cv::CAP_V4L2);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 480); // 480
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 270); // 270
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
	//cv::resize(frame, frame, cv::Size(RESCUE_FRAME_WIDTH * 4, RESCUE_FRAME_HEIGHT * 4));
	debug_frame = frame.clone();
	cv::flip(debug_frame, frame, 0);
	cv::flip(frame, debug_frame, 1);
	frame = debug_frame.clone();
}

// drives roughly to centre of rescue area
void Rescue::find_centre() {
	for (int8_t i = 0; i < 25; ++i) {
		uint16_t dist = robot->distance_avg(5, 0.2f);
		if (dist < 120) { // no entry or exit ahead
			if (dist < 60) robot->m(-80, -80, 200);
			else robot->m(80, 80, 200);
			robot->turn(DTOR(35));
		} else {
			std::cout << "Entry/Exit detected, ignoring..." << std::endl;
			robot->turn(DTOR(17));
			--i;
		}
	}
	robot->stop();
	exit(0);
}

// finds black corner
void Rescue::find_black_corner() {
	std::cout << "in find_black_corner function" << std::endl;
	robot->servo(SERVO_CAM, 140);
	std::cout << "Adjusted servo position" << std::endl;
	close_camera();
	delay(500);
	open_camera();
	delay(500);
	while (1) {
		grab_frame();
		cv::imshow("Frame", frame);
		
		cv::waitKey(1);
	}




	// TODO: either implement own algorithm, or use NN
	// ideas to detect black corner:
	// - many black pixels in frame
	// - contours (the one around the black corner) width is larger than height
	// - distance < 120cm
	// - contours width > contours height (at least when robot is far away from black corner)
	// - make us of adjustable cam angle? Maybe start with low angle and incrementally increase angle when theres no large black contour
	// - general problem: prevent the robot from approaching the corner at an oblique angle as it makes unloading the victims hard

}