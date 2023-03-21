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
	std::cout << "Rescue start!" << std::endl;
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

	find_center_new_new();
	find_black_corner();
	
	float last_x_victim = -1.0f;
	float x_victim = 0.0f;
	float y_victim = 0.0f;

	const float sensitivity = 0.5f;
	const int base_speed = 40;

	int num_frames = 0;
	robot->servo(SERVO_CAM, CAM_HIGHER_POS);
	delay(20);
	robot->servo(SERVO_CAM, CAM_HIGHER_POS);
	delay(20);
	robot->servo(SERVO_CAM, CAM_HIGHER_POS);
	delay(100);

	const int MAX_TURNS = 20;
	int turn_counter = 0;
	int victim_counter = 0;

	while(1) {
		bool ignore_dead = victim_counter < 2;

		open_camera(VICTIM_CAP_RES);
		grab_frame(VICTIM_CAP_RES);
		delay(50);
		grab_frame(VICTIM_CAP_RES);
		delay(50);
		grab_frame(VICTIM_CAP_RES);
		delay(50);
		find_victims(x_victim, y_victim, ignore_dead, true);
		close_camera();
		delay(50);

		bool skip_turn = false;

		if(x_victim != -1.0f) {
			int cam_angle = CAM_HIGHER_POS;
			robot->turn(X_TO_ANGLE(X_RES, x_victim) - DTOR(30.0f));
			std::cout << "Found victim, approaching..." << std::endl;
			delay(100);
			robot->m(40, 40, 200);
			while(1) {
				bool enable_pickup = false;

				open_camera(VICTIM_CAP_RES);
				find_victims(x_victim, y_victim, ignore_dead, cam_angle == CAM_HIGHER_POS);
				close_camera();

				if(x_victim == -1.0f) {
					robot->m(70, 70, 400);
					delay(30);
					robot->turn(DTOR(-15.0f));
					skip_turn = true;
					robot->servo(SERVO_CAM, CAM_HIGHER_POS);
					break;
				}

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

				if(enable_pickup && y_victim > 35.0f) {
					close_camera();
					int dur = (int)(7.0f * (67.0f - y_victim));
					std::cout << "Duration: " << dur << std::endl;
					robot->m(60, 60, dur);
					robot->turn(-DTOR(25.0f));
					robot->send_byte(CMD_PICK_UP_VICTIM);

					robot->m(-70, -70, 400);

					find_center_new_new();
					find_black_corner();

					robot->servo(SERVO_CAM, CAM_HIGHER_POS);
					turn_counter = 0;
					++victim_counter;
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
				delay(50);
			}
		}

		if(!skip_turn) robot->turn(DTOR(30.0f));
		if(!skip_turn) ++turn_counter;

		if(turn_counter == MAX_TURNS) {
			turn_counter = 0;
			robot->m(127, 127, 300);
			find_center_new_new();
			robot->servo(SERVO_CAM, CAM_HIGHER_POS);
		}

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
			robot->m(127, 35);
		} else if (dist < 50.0f) {
			robot->m(-35, -127);
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
	robot->servo(SERVO_CAM, CAM_HIGHER_POS);
	uint8_t deg_per_iteration = 10; // how many degrees should the robot turn after each check for black corner?
	const int SLOW_TURN_SPEED = 35;
	const int FAST_TURN_SPEED = 45;

	int8_t turn_speed = FAST_TURN_SPEED;

	long total_time = 8000;
	long max_time = 10000;

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
			cv::Mat res_resized;
			cv::resize(res, res_resized, cv::Size(160, 120));
			cv::imshow("Black corner", res_resized);
			cv::imshow("frame", frame);
			cv::waitKey(1);

			if(corner_ml.extract_corner(res, x_corner, y_corner)) {
				uint64_t new_start_time = millis() - 2000;
				start_time = start_time > new_start_time ? start_time : new_start_time;
				robot->m(22, -22);
				deg_per_iteration = 5;
				x_corner -= (CORNER_IN_WIDTH / 2.0f);
				std::cout << x_corner << std::endl;
				if(std::abs(x_corner) < 25.0f || x_corner <= 0.0f && last_x_corner > 0.0f) {
					robot->turn(x_corner / CORNER_IN_WIDTH * DTOR(65.0f));
					found_corner = true;
					save_img(frame, "possible_corner");
					break;
				}
			} else {
				robot->m(28, -28);
			}
			last_x_corner = x_corner;
			//robot->turn(DTOR(deg_per_iteration));
			//robot->m(35, -35);
		}
		// robot turned full 360 deg and did not find corner
		// recentre and increase cam angle a bit
		if(!found_corner) {
			find_center_new_new();
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
	robot->m(37, 37, 200);
	robot->stop();
	delay(42);
	robot->m(-127, -127, 550);
	robot->turn(DTOR(190.0f));
	delay(80);
	robot->m(-50, -50, 1500);
	robot->send_byte(CMD_UNLOAD);
	delay(4000);
	robot->m(127, 127, 1000);
}

void Rescue::find_victims(float& x_victim, float& y_victim, bool ignore_dead, bool ignore_top) {
	x_victim = -1.0f;
	y_victim = -1.0f;
	cv::Mat frame = grab_frame(160, 120);
	cv::Mat res = victim_ml.invoke(frame);
	//cv::resize(res, res, cv::Size(160, 120));
	cv::imshow("Victim result", two_channel_to_three_channel(res));
	std::vector<Victim> victims = victim_ml.extract_victims(res, ignore_top);
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

#define EXIT_CAPTURE_RES 480, 270

void Rescue::find_exit() {
	const uint64_t MAX_TIME = 8000;
	const int EXIT_MIN_DISTANCE = 130;
	const int WALL_APPROACH_DISTANCE = 25;
	const uint32_t MIN_NUM_GREEN_PIXELS = 80;

	while(1) {
		uint64_t start_time = millis();
		uint64_t turn_time_on_potential_exit = 8000;
		while(millis() - start_time < MAX_TIME) {
			robot->m(40, -40);

			int dist = robot->distance();
			if(dist > EXIT_MIN_DISTANCE) {
				delay(30); // Turn a few degrees more before stopping
				robot->stop();

				dist = robot->distance_avg(20, 0.2f);
				if(dist > EXIT_MIN_DISTANCE) {
					turn_time_on_potential_exit = millis() - start_time;

					// Get distances of wall directly to the right and to the left of
					// the presumed exit to approximate the distance we have to drive
					int dist_left, dist_right;
					int duration_left, duration_right;
					// Get distance left
					turn_until_wall(&dist_left, &duration_left, EXIT_MIN_DISTANCE, BOOL_DIR_LEFT);

					const int SMALL_TURN_DURATION = 200;
					robot->m(40, -40);
					delay(SMALL_TURN_DURATION);

					// Get distance right
					turn_until_wall(&dist_right, &duration_right, EXIT_MIN_DISTANCE, BOOL_DIR_RIGHT);

					// Turn to center of the exit
					robot->m(-40, 40, (duration_left - SMALL_TURN_DURATION - duration_right) / 2);

					int dist_to_drive = (dist_left + dist_right) / 2;
					int duration_to_drive = dist_to_drive * CM_TO_MS_FULL_SPEED - 500;
					robot->m(127, 127, duration_to_drive);

					robot->servo(SERVO_CAM, CAM_MID_POS);

					open_camera(EXIT_CAPTURE_RES);
					cv::Mat frame = grab_frame(EXIT_CAPTURE_RES);
					close_camera();

					uint32_t num_green_pixels = 0;
					in_range(frame, &is_green, &num_green_pixels);

					if(num_green_pixels > MIN_NUM_GREEN_PIXELS) {
						int dummy_dist, dummy_duration;
						// Reorient the robot based on the wall to the right
						turn_until_wall(&dummy_dist, &dummy_duration, 30, BOOL_DIR_RIGHT);
						robot->turn(DTOR(-8.0f));
						delay(30);
						robot->m(70, 70, 500);
						return;
					}

					// This is likely an entrance, back off again
					robot->m(-127, -127, duration_to_drive);
					robot->turn(DTOR(20.0f));
					start_time = millis() - turn_time_on_potential_exit; // Reset timer
				}
			}
		}
		find_center_new_new();
	}
}

void Rescue::turn_until_wall(int* wall_dist, int* duration, int max_dist, bool direction) {
	const int turn_speed_left = direction == BOOL_DIR_LEFT ? 40 : -40;
	const int turn_speed_right = -turn_speed_left;
	*duration = millis();
	robot->m(turn_speed_left, turn_speed_right);
	while(1) {
		int dist = robot->distance();
		if(dist < max_dist) {
			robot->stop();
			dist = robot->distance_avg(10, 0.2f);
			if(dist < max_dist) break;
			robot->m(turn_speed_left, turn_speed_right);
		}
	}
	robot->m(turn_speed_left, turn_speed_right, 50);
	delay(20);
	robot->stop();
	*wall_dist = robot->distance_avg(20, 0.2f);
	*duration = millis() - *duration;
}

bool is_black2(uint8_t b, uint8_t g, uint8_t r) {
	return (uint16_t)b + (uint16_t)g + (uint16_t)r < BLACK_MAX_SUM;
}
