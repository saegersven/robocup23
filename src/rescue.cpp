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
#define X_RES 160

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
	robot->m(127, 127, 300);

	// robot is roughly in the centre of the rescue area, no matter where the entrace was

	find_center_new();
	find_black_corner();
	
	float last_x_victim = -1.0f;
	float x_victim = 0.0f;
	float y_victim = 0.0f;
	bool dead = false;

	const float sensitivity = 0.5f;
	const int base_speed = 40;

	int num_frames = 0;
	robot->servo(SERVO_CAM, CAM_HIGHER_POS);
	delay(20);
	robot->servo(SERVO_CAM, CAM_HIGHER_POS);
	delay(20);
	robot->servo(SERVO_CAM, CAM_HIGHER_POS);
	delay(100);

	while(1) {
		open_camera(VICTIM_CAP_RES);
		find_victims(x_victim, y_victim, dead);
		close_camera();
		delay(50);

		if(x_victim != -1.0f) {
			int cam_angle = CAM_HIGHER_POS;
			robot->turn(X_TO_ANGLE(X_RES, x_victim) - DTOR(30.0f));
			std::cout << "Found victim, approaching..." << std::endl;
			delay(100);
			robot->m(40, 40, 200);
			while(1) {
				bool enable_pickup = false;

				open_camera(VICTIM_CAP_RES);
				find_victims(x_victim, y_victim, dead);
				close_camera();

				float cam_angle_error = 20.0f * (30.0f - y_victim) / (cam_angle - CAM_LOWER_POS);
				int new_cam_angle = (int)(cam_angle + cam_angle_error);

				if(new_cam_angle > CAM_HIGHER_POS) new_cam_angle = CAM_HIGHER_POS;

				if(new_cam_angle < CAM_LOWER_POS + 25) {
					enable_pickup = true;
					new_cam_angle = CAM_LOWER_POS + 25;
				}
				std::cout << "Cam angle: " << cam_angle << std::endl;
				cam_angle = new_cam_angle;
				robot->servo(SERVO_CAM, new_cam_angle, 200);

				if(enable_pickup && y_victim > 40.0f) {
					close_camera();
					robot->turn(-DTOR(25.0f));
					robot->send_byte(CMD_PICK_UP_VICTIM);

					robot->m(-70, -70, 400);

					find_center_new();
					find_black_corner();

					break;
				}
				/*if(x_victim == -1.0f) {
					std::cout << "Lost victims" << std::endl;
					robot->stop();

					robot->servo(SERVO_CAM, cam_angle += 2, 100);

					find_victims(x_victim, y_victim, dead);
					close_camera();

					if(x_victim == -1.0f) {
						std::cout << "Really lost victim" << std::endl;
						break;
					}
				}*/

				float x_error = X_TO_ANGLE(X_RES, (x_victim - 80.0f));
				robot->turn(x_error);
				robot->m(base_speed, base_speed, 300);
				delay(300);
			}
		}

		robot->turn(DTOR(30.0f));
		delay(50);
	}

	/*
	bool camera_not_up = false;
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
				robot->servo(SERVO_CAM, CAM_LOWER_POS + 5);
				for(find_victims(x_victim, y_victim, dead); x_victim == -1.0f; find_victims(x_victim, y_victim, dead)) {
					robot->m(40, 40);
				}
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
						robot->send_byte(CMD_PICK_UP_VICTIM);
						delay(4200 + 300);
						robot->servo(SERVO_CAM, CAM_HIGHER_POS);

						find_center_new();

						std::cout << "Found center, finding black corner" << std::endl;
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
	}*/
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
void Rescue::find_center() {
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

void Rescue::find_center_new() {
	const int NUM_DATA_POINTS = 30;
	const float step = PI2 / NUM_DATA_POINTS;
	const float gamma = 1.8f;
	// Gyro data would be nice here

	// Calculate center of evac zone:
	// For x coordinate:
	// \vec{M}_x = \left[{\frac{1}{n}\sum_{i = 0}^{n} \left[d_i \cos \left(i \alpha\right) \right]^\gamma} \right]^\frac{1}{\gamma}
	// Similarly for y coordinate. Using root mean square instead of mean is fairly accurate, but not 100%.
	float x = 0.0f;
	float y = 0.0f;
	float last_dist = 50.0f;
	for(int i = 0; i < NUM_DATA_POINTS; ++i) {
		robot->m(60, -60, 70); // Tune this so its exactly 12°
		delay(100);
		float dist = (float)robot->distance_avg(5, 0.2f);
		if(dist > 110.0f) {
			std::cout << "Likely entrance or exit, using last distance" << std::endl;
			dist = last_dist;
		}
		std::cout << dist << "," << std::endl;
		float a_x = dist * std::cos(i * step);
		float a_y = dist * std::sin(i * step);

		if(a_x < 0) a_x = -std::pow(-a_x, gamma);
		else a_x = std::pow(a_x, gamma);

		if(a_y < 0) a_y = -std::pow(-a_y, gamma);
		else a_y = std::pow(a_y, gamma);

		x += a_x;
		y += a_y;

		last_dist = dist;
	}
	delay(200);
	x /= NUM_DATA_POINTS;
	y /= NUM_DATA_POINTS;

	if(x < 0) x = -std::pow(-x, 1.0f/gamma);
	else x = std::pow(x, 1.0f/gamma);
	
	if(y < 0) y = -std::pow(-y, 1.0f/gamma);
	else y = std::pow(y, 1.0f/gamma);

	// x and y are now a vector pointing roughly to the center of the evac zone
	float angle = std::atan2(y, x);
	float mag = std::sqrt(x*x + y*y);

	std::cout << "x: " << x << ", y: " << y << std::endl;
	std::cout << "angle: " << RTOD(angle) << ", mag: " << mag << std::endl;

	robot->turn(angle);
	robot->m(127, 127, CM_TO_MS_FULL_SPEED * mag);
}

void Rescue::find_center_new_new() {
	const int MAX_TIME = 5000;

	uint64_t start_time = millis();

	while(millis() - start_time < MAX_TIME) {
		float dist = robot->distance_avg(5, 0.2f);
		if(dist > 80.0f && dist < 110.0f) {
			robot->m(127, 50);
		} else if (dist < 50.0f) {
			robot->m(-50, -127);
		} else {
			robot->m(50, -50);
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
	long max_time = 16000;

	float x_corner = 0.0f;
	float last_x_corner = 0.0f;
	float y_corner = 0.0f;

	bool found_corner = false;

	while (!found_corner) {
		uint64_t start_time = millis();
		uint64_t real_start_time = millis();

		while (millis() - real_start_time < max_time || millis() - start_time < total_time) {
			deg_per_iteration = 10;
			cv::Mat frame = grab_frame(160, 120);
			cv::Mat res = corner_ml.invoke(frame);
			cv::imshow("Black corner", res);

			if(corner_ml.extract_corner(res, x_corner, y_corner)) {
				uint64_t new_start_time = millis() - 2000;
				start_time = start_time > new_start_time ? start_time : new_start_time;
				robot->m(35, -35);
				deg_per_iteration = 5;
				x_corner -= (CORNER_IN_WIDTH / 2.0f);
				std::cout << x_corner << std::endl;
				if(std::abs(x_corner) < 25.0f || x_corner <= 0.0f && last_x_corner > 0.0f) {
					robot->turn(x_corner / CORNER_IN_WIDTH * DTOR(65.0f));
					found_corner = true;
					break;
				}
			} else {
				robot->m(45, -45);
			}
			last_x_corner = x_corner;
			robot->turn(DTOR(deg_per_iteration));
		}
		// robot turned full 360 deg and did not find corner
		// recentre and increase cam angle a bit
		if(!found_corner) {
			find_center_new();
			robot->servo(0, CAM_HIGHER_POS + 5);
		}
	}
	close_camera();
	robot->servo(0, CAM_LOWER_POS);

	open_camera(BLACK_CORNER_RES);

	while(1) {
		robot->m(42, 42);
		frame = grab_frame(BLACK_CORNER_RES);
		uint32_t num_black_pixels = 0;
		in_range(frame, &is_black, &num_black_pixels);

		if(num_black_pixels > 20000) break;
	}
	robot->stop();
	delay(42);
	robot->m(-127, -127, 550);
	robot->turn(R180 + T180_ERR);
	delay(80);
	robot->m(-50, -50, 1500);
	robot->send_byte(CMD_UNLOAD);
	delay(7000);
	robot->m(127, 127, 1000);
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
