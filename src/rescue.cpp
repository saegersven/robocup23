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
	victim_ml.init();
	corner_ml.init();
}

void Rescue::stop() {
	pthread_cancel(this->native_handle);
	close_camera();
	cv::destroyAllWindows();
}

#define VICTIM_CAP_RES 1280, 960

// main routine for rescue area
void Rescue::rescue() {
	std::cout << "Rescue start." << std::endl;
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

	//find_centre();
	find_black_corner();
	
	float last_x_victim = -1.0f;
	float x_victim = 0.0f;
	float y_victim = 0.0f;
	bool dead = false;

	const float sensitivity = 0.5f;
	const int base_speed = 40;

	int num_frames = 0;
	int cam_angle = CAM_HIGHER_POS;
	robot->servo(SERVO_CAM, CAM_HIGHER_POS);
	delay(20);
	robot->servo(SERVO_CAM, CAM_HIGHER_POS);
	delay(20);
	robot->servo(SERVO_CAM, CAM_HIGHER_POS);
	delay(100);
	open_camera(VICTIM_CAP_RES);

	while(1) {
		find_victims(x_victim, y_victim, dead);
		float angle_victim = DTOR(60.0f) * ((x_victim - 80.0f) / 160.0f);
		if(x_victim != -1.0f) {
			if(last_x_victim == -1.0f) {
				robot->turn(angle_victim);
				close_camera();
				robot->servo(SERVO_CAM, CAM_HIGHER_POS - 25);
				open_camera(VICTIM_CAP_RES);
				for(find_victims(x_victim, y_victim, dead); x_victim == -1.0f; find_victims(x_victim, y_victim, dead)) {
					robot->m(40, 40);
				}
				robot->m(-50, -50, 100);
			}
			std::cout << "Angle: " << RTOD(angle_victim) << std::endl;
			std::cout << "Y: " << y_victim << std::endl;
			if(y_victim > 45.0f) {
				robot->turn(angle_victim / 2.5f);
				if(angle_victim < DTOR(5.0f)) {
					if(num_frames < 10) {
						++num_frames;
					} else {
						num_frames = 0;
						std::cout << "Pick UP!" << std::endl;
						close_camera();
						robot->turn(DTOR(-16.0f));
						robot->send_byte(CMD_PICK_UP);
						delay(4200 + 300);
						robot->servo(SERVO_CAM, CAM_HIGHER_POS);

						find_centre();

						std::cout << "Found centre, finding black corner" << std::endl;
						find_black_corner();
						
						robot->servo(SERVO_CAM, CAM_HIGHER_POS);
						delay(20);
						robot->servo(SERVO_CAM, CAM_HIGHER_POS);
						delay(20);
						robot->servo(SERVO_CAM, CAM_HIGHER_POS);
						delay(100);

						open_camera(VICTIM_CAP_RES);
					}
				}
			} else {
				num_frames = 0;
				robot->m(clamp((float)base_speed + angle_victim * sensitivity, -128.0f, 127.0f),
					clamp((float)base_speed - angle_victim * sensitivity, -128.0f, 127.0f));
			}
		} else if(x_victim != last_x_victim) {
			//robot->m(-70, -70, 300);
		} else  {
			close_camera();
			robot->turn(DTOR(30.0f));
			delay(50);
			open_camera(VICTIM_CAP_RES);
		}
		last_x_victim = x_victim;
	}
}

void Rescue::open_camera(int width, int height) {
	if(camera_opened) return;
	cap.open(0, cv::CAP_V4L2);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
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

cv::Mat Rescue::grab_frame(int width, int height) {
	cv::Mat frame;
	cv::Mat temp;
	if(!camera_opened) {
		std::cerr << "Tried to grab frame with closed camera" << std::endl;
		exit(ERRCODE_CAM_CLOSED);
	}

	cap.grab();
	cap.retrieve(frame);
	cv::resize(frame, frame, cv::Size(width, height));
	temp = frame.clone();
	cv::flip(temp, frame, 0);
	cv::flip(frame, temp, 1);
	frame = temp.clone();
	return frame;
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
}

#define BLACK_CORNER_RES 480, 270

// finds black corner and unloads victims
void Rescue::find_black_corner() {
	open_camera(VICTIM_CAP_RES);
	robot->servo(0, CAM_HIGHER_POS);
	uint8_t deg_per_iteration = 10; // how many degrees should the robot turn after each check for black corner?

	long total_time = 8000;

	float x_corner = 0.0f;
	float last_x_corner = 0.0f;
	float y_corner = 0.0f;

	bool found_corner = false;

	while (!found_corner) {
		deg_per_iteration = 10;
		uint64_t start_time = millis();

		while (millis() - start_time < total_time) {
			cv::Mat frame = grab_frame(160, 120);
			cv::Mat res = corner_ml.invoke(frame);
			cv::imshow("Black corner", res);

			if(corner_ml.extract_corner(res, x_corner, y_corner)) {
				robot->m(35, -35);
				deg_per_iteration = 5;
				x_corner -= (CORNER_IN_WIDTH / 2.0f);
				if(std::abs(x_corner) < 25.0f || x_corner <= 0.0f && last_x_corner > 0.0f) {
					robot->turn(x_corner / CORNER_IN_WIDTH * DTOR(65.0f));
					found_corner = true;
					break;
				}
				std::cout << x_corner << std::endl;
			} else {
				robot->m(45, -45);
			}
			last_x_corner = x_corner;
			robot->turn(DTOR(deg_per_iteration));
		}
		// robot turned full 360 deg and did not find corner
		// recentre and increase cam angle a bit
		if(!found_corner) {
			find_centre();
			robot->servo(0, CAM_HIGHER_POS + 5);
		}
	}
	close_camera();

	robot->turn(R180 + T180_ERR);
	robot->m(-50, -50, 420 * 10);
	delay(42);
	robot->send_byte(CMD_UNLOAD);
	delay(8000);
	robot->m(127, 127, 1500);
}

void Rescue::find_victims(float& x_victim, float& y_victim, bool ignore_dead) {
	x_victim = -1.0f;
	y_victim = -1.0f;
	cv::Mat frame = grab_frame(160, 120);
	cv::Mat res = victim_ml.invoke(frame);
	//cv::resize(res, res, cv::Size(160, 120));
	cv::imshow("Victim result", two_channel_to_three_channel(res));
	std::vector<Victim> victims = victim_ml.extract_victims(res);
	for(int i = 0; i < victims.size(); ++i) {
		cv::circle(frame, cv::Point(victims[i].x, victims[i].y), 10, victims[i].dead ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 5);
	}
	cv::imshow("Victims", frame);
	cv::waitKey(1);

	for(int i = 0; i < victims.size(); ++i) {
		if(ignore_dead && victims[i].dead) continue;
		if(victims[i].y > y_victim) {
			x_victim = victims[i].x;
			y_victim = victims[i].y;
		}
	}
}

bool is_black2(uint8_t b, uint8_t g, uint8_t r) {
	return (uint16_t)b + (uint16_t)g + (uint16_t)r < BLACK_MAX_SUM;
}
