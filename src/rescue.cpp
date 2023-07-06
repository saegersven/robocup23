// Exposure time adjustment:
// v4l2-ctl -c exposure_time=100

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
	std::cout << "Rescue::stop" << std::endl;
	pthread_cancel(this->native_handle);
	robot->stop_camera();
	cv::destroyAllWindows();
	std::cout << "Windows closed!!!" << std::endl;
}

#define X_RES 160

// main routine for rescue area
void Rescue::rescue() {
	robot = std::make_shared<Robot>();

	robot->capture_width = 1280;
	robot->capture_height = 960;
	robot->frame_width = 160;
	robot->frame_height = 120;

	std::cout << "Rescue start!" << std::endl;
	robot->m(127, 127, 700);

	if (robot->distance_avg(1, 10, 0.3f) < 400) {
		robot->turn(DTOR(130.0f));
		robot->m(-127, -127, 1200);
	} else {
		robot->turn(DTOR(-130.0f));
		robot->m(-127, -127, 1200);
	}
	
	robot->servo(SERVO_CAM, CAM_HIGHER_POS, 1000);

	// Restart camera for exposure adjustment here
	robot->start_camera();

	// find_exit();
	// finished = true;
	// return;
	// delay(100000);

	find_corner(false);

	const int MAX_TURNS = 20;
	int turn_counter = 0;
	int victim_counter = 0;

	float last_x_victim = -1.0f;	
	float x_victim = 0.0f;
	float y_victim = 0.0f;

	bool dead = false;

	int full_rotation_no_victims_count = 0;
	bool find_center_called = false;

	while(1) {
		bool ignore_dead = victim_counter < 2;

		find_victims(x_victim, y_victim, ignore_dead, dead, true);

		bool skip_turn = false;

		if(x_victim != -1.0f) {
			int cam_angle = CAM_HIGHER_POS;
			std::cout << "Starting approach..." << std::endl;
			std::cout << "x_victim: " << x_victim << std::endl;
			std::cout << "angle:    " << X_TO_ANGLE(X_RES, (x_victim - 80.0f)) - DTOR(30.0f) << std::endl;
			robot->turn(X_TO_ANGLE(X_RES, (x_victim - 80.0f)));
			delay(100);
			robot->m(40, 40, 200);

			// Victim approach and capture loop
			while(1) {
				bool enable_pickup = false;

				find_victims(x_victim, y_victim, ignore_dead, dead, cam_angle == CAM_HIGHER_POS);

				// Victim has been lost
				if(x_victim == -1.0f) {
					robot->m(70, 70, 200);
					delay(30);
					robot->turn(DTOR(-15.0f));
					skip_turn = true;
					robot->servo(SERVO_CAM, CAM_HIGHER_POS);
					break;
				}

				// Adjust cam angle
				float cam_angle_error = 20.0f * (30.0f - y_victim) / (cam_angle - CAM_LOWER_POS);
				int new_cam_angle = (int)(cam_angle + cam_angle_error);

				if(new_cam_angle > CAM_HIGHER_POS) new_cam_angle = CAM_HIGHER_POS;

				if(new_cam_angle < CAM_LOWER_POS + 25) {
					// We are very close to victim, enable pickup
					enable_pickup = true;
					new_cam_angle = CAM_LOWER_POS + 25;
				}
				std::cout << "Cam angle: " << cam_angle << std::endl;
				cam_angle = new_cam_angle;
				robot->servo(SERVO_CAM, new_cam_angle, 200);

				if(enable_pickup && y_victim > 35.0f) {
					// Pickup victim
					int dur = (int)(2.0f * (55.0f - y_victim)) - 1+0;
					std::cout << "Duration: " << dur << std::endl;
					robot->m(60, 60, dur);
					robot->turn(-DTOR(25.0f));

					// Collect victim
					robot->gripper(GRIPPER_CLOSE, 200);
					robot->attach_detach_servo(SERVO_ARM); // attach
					robot->servo(SERVO_ARM, ARM_LOWER_POS, 250);
					robot->m(-65, -65, 150);
					delay(20);
					robot->gripper(GRIPPER_OPEN);
					delay(420);
					robot->m(60, 60, 430);
					robot->gripper(GRIPPER_CLOSE);
					delay(80);
					robot->m(30, 30);
					delay(320);
					robot->gripper(GRIPPER_OPEN);
					delay(70);
					robot->gripper(GRIPPER_CLOSE);
					delay(100);
					robot->stop();
					delay(200);
					robot->m(-65, -65, 150);
					robot->servo(SERVO_ARM, ARM_HIGHER_POS, 10);
					robot->m(-50, -50, 350);
					delay(700);
					robot->gripper(GRIPPER_OPEN, 50);
					delay(100);
					robot->gripper(GRIPPER_CLOSE, 200);
					robot->attach_detach_servo(SERVO_ARM); // detach

					find_center_new_new();
					find_corner(dead);

					robot->servo(SERVO_CAM, CAM_HIGHER_POS);
					turn_counter = 0;
					++victim_counter;

					if(dead) {
						find_exit();
						finished = true;
						return;
					}
					break;
				}

				float x_error = X_TO_ANGLE(X_RES, (x_victim - 80.0f));
				std::cout << "X_error: " << x_error << std::endl;
				robot->turn(x_error);
				robot->m(42, 42, 300);
				delay(50);
			}
		}

		if(!skip_turn) robot->turn(DTOR(30.0f));
		if(!skip_turn) ++turn_counter;

		if(turn_counter == MAX_TURNS) {
			if(find_center_called) {
				std::cout << "Found no victims, searching for exit" << std::endl;
				find_corner(true);
				find_exit();
				finished = true;
				return;
			}

			turn_counter = 0;
			robot->m(127, 127, 300);
			find_center_new_new();
			find_center_called = true;
			robot->servo(SERVO_CAM, CAM_HIGHER_POS);

			++full_rotation_no_victims_count;

			if(full_rotation_no_victims_count == 0) {
				break;
			}
		}

		delay(50);
	}
	find_corner(true);
	find_exit();
	finished = true;
}

// drives roughly to centre of rescue area
void Rescue::find_center() {
	for (int8_t i = 0; i < 25; ++i) {
		uint16_t dist = robot->distance_avg(0, 5, 0.2f);
		if (dist < 120) { // no entry or exit ahead
			if (dist < 60) robot->m(-80, -80, 200);
			else robot->m(80, 80, 200);
			robot->turn(DTOR(35));
		} else {
			//std::cout << "Entry/Exit detected, ignoring..." << std::endl;
			robot->turn(DTOR(17));
			--i;
		}
	}
	robot->stop();
}

// TODO: adjust because robot->distance() now returns mm instead of cm
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
		float dist = (float)robot->distance_avg(0, 5, 0.2f);
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

	uint64_t start_time = millis_();

	while(millis_() - start_time < MAX_TIME) {
		float dist = robot->distance_avg(0, 5, 0.2f);
		if(dist > 800.0f && dist < 1100.0f) {
			robot->m(127, 35);
		} else if (dist < 500.0f) {
			robot->m(-35, -127);
		} else {
			robot->m(50, -50);
		}
	}
	robot->stop();
}

#define BLACK_CORNER_RES 480, 270

// if ignore_green = false it only finds green corners.
// if ignore_green = true it only finds red corners
void Rescue::find_corner(bool ignore_green) {
	robot->stop();
	robot = std::make_shared<Robot>(); // quick fix for Nano freezing sometimes
	robot->servo(SERVO_CAM, CAM_HIGHER_POS, 300);
	std::cout << "Searching for corner" << std::endl;
	uint8_t deg_per_iteration = 10; // how many degrees should the robot turn after each check for black corner?
	int SLOW_TURN_SPEED = 35;
	int FAST_TURN_SPEED = 50;

	int8_t turn_speed = FAST_TURN_SPEED;

	long total_time = 8000;
	long max_time = 10000;

	float x_corner = 0.0f;
	float last_x_corner = 0.0f;
	float y_corner = 0.0f;

	bool found_corner = false;

	while (!found_corner) {
		uint64_t start_time = millis_();
		uint64_t real_start_time = millis_();

		while (millis_() - real_start_time < max_time || millis_() - start_time < total_time) {
			deg_per_iteration = 10;
			frame = robot->grab_frame();
			cv::Mat res = corner_ml.invoke(frame);
			cv::Mat res_resized;
			cv::resize(res, res_resized, cv::Size(160, 120));
			//cv::imshow("Corner", two_channel_to_three_channel(res_resized));
			//cv::imshow("frame", frame);
			//cv::waitKey(1);

			if(corner_ml.extract_corner(res, x_corner, y_corner, ignore_green)) {
				uint64_t new_start_time = millis_() - 2000;
				start_time = start_time > new_start_time ? start_time : new_start_time;
				robot->m(SLOW_TURN_SPEED, -SLOW_TURN_SPEED);
				deg_per_iteration = 5;
				x_corner -= (CORNER_IN_WIDTH / 2.0f);
				std::cout << x_corner << std::endl;
				if(std::abs(x_corner) < 25.0f || x_corner <= 0.0f && last_x_corner > 0.0f) {
					robot->stop();
					delay(50);
					robot->turn(-DTOR(20.0f));
					for(int i = 0; i < 2; ++i) {
						frame = robot->grab_frame();
						res = corner_ml.invoke(frame);
						corner_ml.extract_corner(res, x_corner, y_corner, ignore_green);
						robot->turn((x_corner / CORNER_IN_WIDTH - 0.5f) * DTOR(65.0f) / i);
					}
					found_corner = true;
					break;
				}
			} else {
				robot->m(FAST_TURN_SPEED, -FAST_TURN_SPEED);
			}
			last_x_corner = x_corner;
		}
		// robot turned full 360 deg and did not find corner
		// recentre and increase cam angle a bit
		// also drive a bit slower
		if(!found_corner) {
			std::cout << "TURNING SLOWER NOW!!!" << std::endl;
			robot->servo(SERVO_CAM, CAM_HIGHER_POS + 3, 300);
			SLOW_TURN_SPEED -= 10;
			FAST_TURN_SPEED -= 15;
			find_center_new_new();
		}
	}
	// TODO: drive to corner using camera!!!
	robot->m(127, 127, 1100);
	robot->m(-127, -127, 600);
	robot->turn(R180);
	robot->m(-60, -60, 2600);

	for (int i = 0; i < 3; ++i) {
		robot->m(42, 42, 200);
		delay(10);
		robot->m(-127, -127, 150);	
		delay(50);
	}
	robot->m(-60, -60, 300);
	robot->servo(SERVO_GATE, GATE_OPEN, 1500);

	for (int i = 0; i < 2; ++i) {
		robot->m(42, 42, 200);
		delay(10);
		robot->m(-127, -127, 150);	
		delay(50);
	}

	delay(700);
	robot->servo(SERVO_GATE, GATE_CLOSED, 600);
	robot->m(127, 127, 1200);
}

void Rescue::find_victims(float& x_victim, float& y_victim, bool ignore_dead, bool& dead, bool ignore_top) {
	x_victim = -1.0f;
	y_victim = -1.0f;
	dead = false;
	delay(210);
	frame = robot->grab_frame();
	save_img(frame, "captured");
	delay(50);
	cv::Mat res = victim_ml.invoke(frame);
	save_img(frame, "rescue");
	std::vector<Victim> victims = victim_ml.extract_victims(res, ignore_top);

	cv::resize(res, res, cv::Size(160, 120));
	cv::imshow("Victim result", two_channel_to_three_channel(res));

	for(int i = 0; i < victims.size(); ++i) {
		cv::circle(frame, cv::Point(victims[i].x, victims[i].y), 10, victims[i].dead ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 5);
	}
	cv::imshow("Victims", frame);
	cv::waitKey(1);

	for(int i = 0; i < victims.size(); ++i) {
		if(ignore_dead && victims[i].dead) continue;
		if((!victims[i].dead && dead) || victims[i].y > y_victim) {
			x_victim = victims[i].x;
			y_victim = victims[i].y;
			dead = victims[i].dead;
		}
	}
}

bool is_black2(uint8_t b, uint8_t g, uint8_t r) {
	return (uint16_t)b + (uint16_t)g + (uint16_t)r < BLACK_MAX_SUM;
}

#define distance_less_than(dist_variable, sensor, mm) (dist_variable < mm && robot->distance_avg(sensor, 10, 0.2f) < mm)
#define distance_greater_than(dist_variable, sensor, mm) (dist_variable > mm && robot->distance_avg(sensor, 10, 0.2f) > mm)

bool Rescue::check_exit() {
	uint32_t num_black_pixels = 0;
	frame = robot->grab_frame();
	cv::Mat thresh = in_range(frame, &is_black, &num_black_pixels);

	float percentage = (float)num_black_pixels / (float)frame.rows / (float)frame.cols;
	std::cout << "Black Percentage: " << percentage << std::endl;
	if(percentage > 0.1f) {
		return true;
		// not working code:
		/*
	    std::vector<std::vector<cv::Point>> contours;
	    std::vector<cv::Vec4i> hierarchy;
	    cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	    if(contours.size() == 0) return false;

	    cv::Rect bound_rect = cv::boundingRect(contours[0]);

	    std::cout << "Width: " << bound_rect.width << std::endl;
	    if(bound_rect.width > 0.6f * frame.cols || percentage > 0.3f) return true; // in case of many false positives, remove second condition
	    */
	}
	return false;
}

// finds exit
void Rescue::find_exit() {
	int distances[95];
	int second_derivative[93];

	for(int i = 0; i < 95; i++) {
		robot->m(42, -42, 68);

		int dist = robot->distance_avg(0, 5, 0.2f);
		distances[i] = dist;

		if(i >= 2) {
			second_derivative[i - 2] = distances[i - 2] + distances[i - 1] - 2*distances[i];
			std::cout << second_derivative[i - 2] << std::endl;
		}
	}

	int largest_i[2] = {0, 0};

	for(int i = 0; i < 93; i++) {
		if(second_derivative[i] > 120) {
			if(second_derivative[i - 1] < second_derivative[i]
				&& second_derivative[i + 1] < second_derivative[i]) {
				// This is peak
				if(second_derivative[largest_i[0]] < second_derivative[i]) {
					if(second_derivative[largest_i[1]] < second_derivative[largest_i[0]]) {
						largest_i[1] = largest_i[0];
					}
					largest_i[0] = i;
				} else if(second_derivative[largest_i[1]] < second_derivative[i]) {
					largest_i[1] = i;
				}
			}
		}
	}

	float pot_exit_angle_1 = (float)largest_i[0] / (float)93 * 360.0f;
	float pot_exit_angle_2 = (float)largest_i[1] / (float)93 * 360.0f;

	std::cout << "Potential exits at " << pot_exit_angle_1 << "° and " << pot_exit_angle_2 << "°" << std::endl;

	robot->turn(DTOR(pot_exit_angle_1));
	robot->m(127, 127, distances[largest_i[0] + 2] * 2 - 500);

	float largest_dist_angle = 0.0f;
	int largest_dist = 0;
	int average_dist = 0;
	// Look for exit
	robot->turn(-DTOR(31.0f));
	for(int i = 0; i < 15; i++) {
		robot->m(42, -42, 70);
		int dist = robot->distance_avg(0, 10, 0.2f);

		average_dist += dist;
		std::cout << dist << std::endl;
		if(dist > largest_dist) {
			largest_dist = dist;
			largest_dist_angle = i / 15.0f * 60.0f;
		}
	}
	average_dist /= 15;
	std::cout << "Largest dist angle: " << largest_dist_angle - 60.0f << std::endl;
	std::cout << "Angle: " << DTOR(largest_dist_angle - 60.0f) << std::endl;
	robot->turn(DTOR(largest_dist_angle - 60.0f));

	robot->servo(SERVO_CAM, CAM_LOWER_POS, 500);

	long long start_t = millis_();

	robot->m(50, 50);

	bool left_area = false; // did robot reach potential exit and left area a few cm?
	while(!left_area) {
		frame = robot->grab_frame();

		uint32_t num_black_pixels_ = 0;
		in_range(frame, &is_black, &num_black_pixels_);
		std::cout << "Black pixels: " << num_black_pixels_ << std::endl;

		// theres a black line, so robot reached potential exit. Check for exit now
		if (num_black_pixels_ > 0.05 * frame.cols * frame.rows) {
			std::cout << "Possible exit, checking..." << std::endl;
			robot->stop();
			delay(5000);
			robot->m(-60, -60, 100);
			robot->grab_frame();
			delay(10);
			robot->grab_frame();
			delay(10);
			if(check_exit()) {
				std::cout << "Yeah, found exit!" << std::endl;
				robot->stop();
				delay(1000);
				robot->m(60, 60, 300);
				return;
			} else { // robot found entrance instead of exit
				// TODO: drive back to center. Especially use TURN
				std::cout << "Nope, no exit here" << std::endl;
				robot->m(-50, 50, millis_() - start_t); // drive back to center
				std::cout << "Back in center. Checking other exit" << std::endl;

			}
		}
	}

	robot->servo(SERVO_CAM, CAM_LOWER_POS, 300);
	robot->m(-2*42, -2*42, 3000);
	robot->m(60, 60, 420);
	robot->turn(R45);

	while(1) {
		robot->m(50, 50, 300);
		int dist = robot->distance_avg(1, 10, 0.3f);

		std::cout << dist << std::endl;
		if(dist > 1000) {
			// Wand absuchen
			robot->stop();
			delay(5000);
		}

		dist = robot->distance_avg(0, 10, 0.3f);

		if(dist < 100) {
			robot->m(-60, -60, 200);
			robot->turn(R180);
			robot->m(-60, -60, 500);
			robot->m(60, 60, 200);
			robot->turn(R90);
			// Ecke absuchen
		}
	}

	return;

	int counter = 0;

	while(true) {
		int dist = robot->distance_avg(1, 10, 0.3f);

		std::cout << dist << std::endl;

		if(dist > 250) {
			robot->m(60, 60, 150);
			robot->stop();
			if(robot->distance_avg(1, 10, 0.3f) > 400) {
				std::cout << "POTENTIAL EXIT" << std::endl;
				robot->m(-80, -80, 80);
				robot->turn(R90);
				robot->m(80, 80, 300);
				if(check_exit()) {
					std::cout << "FOUND EXIT" << std::endl;
					robot->m(80, 80, 300);
					return;
				}
			} else {
				robot->m(-60, -60, 100);
			}
		}

		if(dist < 110) {
			std::cout << "REALIGN WALL, TOO CLOSE\n";
			// Realign with wall
			robot->turn(-R45);
			robot->m(127, 127, 200);
			robot->turn(R45);
			robot->m(-70, -70, 700);
			robot->m(127, 127, 50);
			robot->turn(R90);
			robot->m(60, 60);
		} else if(dist > 140) {
			std::cout << "REALIGN WALL, TOO FAR AWAY\n";
			robot->turn(-R90);
			robot->m(-70, -70, 900);
			robot->m(127, 127, 50);
			robot->turn(R90);
			robot->m(60, 60);
		}

		// Check front distance
		dist = robot->distance_avg(0, 10, 0.3f);
		if(dist < 250) {
			// Wall in front, turn
			robot->turn(-R90);

		}

		if(counter % 10 == 0) robot->m(60, 60);
		counter++;
	}
}