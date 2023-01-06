#include "rescue.h"

#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
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
	*/
	find_black_corner(); // find black corner and unload victims
	// find_exit();
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

// finds black corner and unloads victims
void Rescue::find_black_corner() {
	std::cout << "in find_black_corner function" << std::endl;
	robot->servo(SERVO_CAM, 132);
	std::cout << "Adjusted servo position" << std::endl;
	open_camera();
	while (1) {
		grab_frame();
		uint32_t num_black_pixels = 0;
		cv::Mat black = in_range(frame, &is_black2, &num_black_pixels);
		//cv::imshow("Frame", black);
		std::cout << "Black pixels: " << num_black_pixels << std::endl;
		if (num_black_pixels > 5000) { // possibly black corner
			std::vector<std::vector<cv::Point>> contours;
  			cv::findContours(black, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  			std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
    		return cv::contourArea(c1, false) > cv::contourArea(c2, false);
  			});

  			cv::Rect bounding_box = cv::boundingRect(contours[0]);

  			cv::rectangle(frame, bounding_box, cv::Scalar(0, 255, 0), 2);

  			if (bounding_box.width > bounding_box.height) {
  				std::cout << "Approaching possible corner" << std::endl;
  				save_img(frame, "possible_corner");
				close_camera();

				// TODO: Calculate angle to turn a bit towards centre of corner (take inspiration from last years code)

  				robot->servo(SERVO_CAM, 65);
  				robot->m(127, 127, 1300);
  				open_camera();
  				grab_frame();
  				close_camera();
				cv::Mat black = in_range(frame, &is_black2, &num_black_pixels);
				if (num_black_pixels > 3000 && robot->distance_avg(5, 0.2f) < 45) { // found black corner
  					save_img(frame, "possible_corner");
					std::cout << "Found black corner, black pixels: " << num_black_pixels << std::endl;
					robot->m(-127, -127, 600);
					robot->turn(DTOR(180));
					robot->m(-80, -80, 1000);
					robot->send_byte(CMD_UNLOAD);
					delay(7000);
					robot->m(127, 127, 1200);
					return;
				} else {
					std::cout << "No black corner here" << std::endl;
  					robot->m(-127, -127, 1300);
  					delay(500);
  					robot->turn(DTOR(30));
  					delay(500);
					robot->servo(SERVO_CAM, 132);
  					open_camera();
  					delay(300);
				}
  			}
		} else {
			robot->turn(DTOR(5));
		}
  		//cv::imshow("Frame", frame);
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

bool is_black2(uint8_t b, uint8_t g, uint8_t r) {
	return (uint16_t)b + (uint16_t)g + (uint16_t)r < BLACK_MAX_SUM;
}
