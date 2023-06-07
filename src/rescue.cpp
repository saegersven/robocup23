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

	const int MAX_TURNS = 20;
	int turn_counter = 0;
	int victim_counter = 0;

	int full_rotation_no_victims_count = 0;
	bool find_center_called = false;

	while(1) {
		bool ignore_dead = victim_counter < 2;

		open_camera(VICTIM_CAP_RES);
		grab_frame(VICTIM_CAP_RES);
		delay(5);
		grab_frame(VICTIM_CAP_RES);
		delay(5);
		grab_frame(VICTIM_CAP_RES);
		delay(5);
		find_victims(x_victim, y_victim, ignore_dead, true);
		close_camera();
		delay(5);

		bool skip_turn = false;

		if(x_victim != -1.0f) {
			int cam_angle = 0;
			robot->turn(X_TO_ANGLE(X_RES, x_victim) - DTOR(30.0f));
			std::cout << "Found victim, approaching..." << std::endl;
			delay(100);
			robot->m(40, 40, 200);
			while(1) {
				bool enable_pickup = false;

				open_camera(VICTIM_CAP_RES);
				find_victims(x_victim, y_victim, ignore_dead, cam_angle == 0);
				close_camera();

				if(x_victim == -1.0f) {
					robot->m(70, 70, 200);
					delay(30);
					robot->turn(DTOR(-15.0f));
					skip_turn = true;
					break;
				}

				float cam_angle_error = 20.0f * (30.0f - y_victim) / (cam_angle - 0);
				int new_cam_angle = (int)(cam_angle + cam_angle_error);

				if(new_cam_angle > 0) new_cam_angle = 0;

				if(new_cam_angle < 0 + 25) {
					enable_pickup = true;
					new_cam_angle = 0 + 25;
				}
				std::cout << "Cam angle: " << cam_angle << std::endl;
				cam_angle = new_cam_angle;

				if(enable_pickup && y_victim > 35.0f) {
					close_camera();
					int dur = (int)(2.0f * (55.0f - y_victim)) - 1+0;
					std::cout << "Duration: " << dur << std::endl;
					robot->m(60, 60, dur);
					robot->turn(-DTOR(25.0f));
					delay(5000);

					find_center_new_new();
					find_black_corner();

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
			if(find_center_called) {
				find_exit();
				finished = true;
				return;
			}

			turn_counter = 0;
			robot->m(127, 127, 300);
			find_center_new_new();
			find_center_called = true;

			++full_rotation_no_victims_count;

			if(full_rotation_no_victims_count == 0) {
				break;
			}
		}

		delay(50);
	}

	std::cout << "Searching for exit" << std::endl;
	find_exit();
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

	uint64_t start_time = millis_();

	while(millis_() - start_time < MAX_TIME) {
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
	uint8_t deg_per_iteration = 10; // how many degrees should the robot turn after each check for black corner?
	const int SLOW_TURN_SPEED = 35;
	const int FAST_TURN_SPEED = 40;

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
			cv::Mat frame = grab_frame(160, 120);
			cv::Mat res = corner_ml.invoke(frame);
			cv::Mat res_resized;
			cv::resize(res, res_resized, cv::Size(160, 120));
			cv::imshow("Black corner", res_resized);
			cv::imshow("frame", frame);
			cv::waitKey(1);

			if(corner_ml.extract_corner(res, x_corner, y_corner)) {
				uint64_t new_start_time = millis_() - 2000;
				start_time = start_time > new_start_time ? start_time : new_start_time;
				robot->m(22, -22);
				deg_per_iteration = 5;
				x_corner -= (CORNER_IN_WIDTH / 2.0f);
				std::cout << x_corner << std::endl;
				if(std::abs(x_corner) < 25.0f || x_corner <= 0.0f && last_x_corner > 0.0f) {
					robot->turn(-DTOR(5.0f));
					frame = grab_frame(160, 120);
					res = corner_ml.invoke(frame);
					corner_ml.extract_corner(res, x_corner, y_corner);
					robot->turn((x_corner / CORNER_IN_WIDTH - 0.5f) * DTOR(65.0f));
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
		}
	}
	close_camera();

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
	robot->turn(DTOR(180.0f));
	delay(80);
	robot->m(-50, -50, 1500);
	for (int i = 0; i < 370; ++i) {
		std::cout << i << std::endl;
		delay(10);
	}
	robot->m(127, 127, 1000);
}

void Rescue::find_victims(float& x_victim, float& y_victim, bool ignore_dead, bool ignore_top) {
	x_victim = -1.0f;
	y_victim = -1.0f;
	bool dead = false;
	cv::Mat frame = grab_frame(160, 120);
	cv::Mat res = victim_ml.invoke(frame);
	save_img(frame, "rescue");
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
		if((!victims[i].dead && dead) || victims[i].y > y_victim) {
			x_victim = victims[i].x;
			y_victim = victims[i].y;
			dead = victims[i].dead;
		}
	}
}


/*void Rescue::find_exit() {
	const uint64_t MAX_TIME = 8000;
	const int EXIT_MIN_DISTANCE = 130;
	const int WALL_APPROACH_DISTANCE = 25;
	const uint32_t MIN_NUM_GREEN_PIXELS = 80;

	while(1) {
		uint64_t start_time = millis();
		uint64_t turn_time_on_potential_exit = 8000;
		while(millis() - start_time < MAX_TIME) {
			robot->m(40, -40, 50);
			delay(20);

			int dist = robot->distance();
			delay(40);
			if(dist > EXIT_MIN_DISTANCE) {
				delay(30); // Turn a few degrees more before stopping
				robot->stop();
				std::cout << "Found possible exit" << std::endl;

				dist = robot->distance_avg(20, 0.2f);
				if(dist > EXIT_MIN_DISTANCE) {
					turn_time_on_potential_exit = millis() - start_time;

					robot->turn(DTOR(25.0f));
					robot->m(70, 70, 500);
					robot->turn(DTOR(-22.0f));

					// Get distances of wall directly to the right and to the left of
					// the presumed exit to approximate the distance we have to drive
					int dist_left, dist_right;
					int duration_left, duration_right;
					// Get distance left
					turn_until_wall(&dist_left, &duration_left, EXIT_MIN_DISTANCE, BOOL_DIR_LEFT);

					std::cout << "LEFT POST" << std::endl;
					delay(50);

					const int SMALL_TURN_DURATION = 150;
					robot->m(35, -35);
					delay(SMALL_TURN_DURATION);
					robot->stop();

					// Get distance right
					turn_until_wall(&dist_right, &duration_right, EXIT_MIN_DISTANCE, BOOL_DIR_RIGHT);

					std::cout << "RIGHT POST" << std::endl;
					delay(50);

					// Turn to center of the exit
					robot->m(-35, 35, (duration_left - SMALL_TURN_DURATION - duration_right) / 2);
					robot->turn(DTOR(8.0f));

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
}*/

#define DRIVE_ONE_TILE robot->m(127, 127, 30 * CM_TO_MS_FULL_SPEED)
#define EXIT_DIST_TO_WALL 12
#define EXIT_CAPTURE_WIDTH 480
#define EXIT_CAPTURE_HEIGHT 270

bool Rescue::check_exit() {
	open_camera(EXIT_CAPTURE_WIDTH, EXIT_CAPTURE_HEIGHT);
	frame = grab_frame(EXIT_CAPTURE_WIDTH, EXIT_CAPTURE_HEIGHT);
	close_camera();

	uint32_t num_green_pixels = 0;
	cv::Mat green = in_range(frame, &is_black, &num_green_pixels);

	cv::imshow("Exit frame", frame);

	cv::imshow("Exit green", green);
	cv::waitKey(500);

	float percentage = (float)num_green_pixels / (float)EXIT_CAPTURE_HEIGHT / (float)EXIT_CAPTURE_WIDTH;
	std::cout << "Exit green percentage: " << percentage << "\n";
	if(percentage > 0.12f) {
		robot->m(60, 60, 500);
		return true;
	}
	return false;
}

void Rescue::turn_wall() {
	// There is a wall here
	robot->m(40, 40, 1200);
	robot->m(-70, -70, 600);
	delay(50);
	robot->turn(R90);
	delay(50);
	robot->m(-60, -60, 300);
	delay(50);
	robot->turn(-R180);
	delay(50);
	robot->m(60, 60, 300);
	delay(50);
}

void Rescue::find_exit() {
	find_black_corner();
	close_camera();

	robot->m(-127, -127, 950);
	robot->m(-42, -42, 500);

	robot->m(70, 70, 200);
	robot->turn(R45);
	robot->m(100, 100, 300);
	robot->turn(R90);
	robot->m(30, 30, 1900);
	robot->m(-100, -100, 500);
	delay(200);

	uint8_t tiles_traveled = 1; // One because of corner

	while(1) {
		// Robot is facing the wall
		int dist = robot->distance_avg(10, 0.2f);
		std::cout << "Dist: " << dist << std::endl;

		if(dist > 30) {
			// There is no wall here, so this can be an entrance or an exit
			robot->m(60, 60, 450);

			// Check for green
			if(check_exit()) {
				return;
			}

			robot->m(-60, -60, 200);
		} else {
			robot->m(35, 35, 1500);
			robot->m(-70, -70, 600);
			delay(200);
		}

		robot->turn(-R90);

		if(tiles_traveled >= 3) {
			// If this is 90cm wall, check for wall
			dist = robot->distance_avg(10, 0.2f);
			if(dist < 30) {
				turn_wall();
				tiles_traveled = 0;
			} else if(dist > 110) {
				turn_wall();
				dist = robot->distance_avg(10, 0.2f);
				robot->turn(R90);
				robot->m(70, 70, 700);
				robot->m(40, 40, 300);
				robot->m(-70, -70, 600);
				robot->turn(-R90);
				if(dist < 25) {
					// There was a wall, check for exit
					robot->m(60, 60, 350);

					if(check_exit()) {
						return;
					}

					// No exit, turn left
					robot->m(-60, -60, 350);
					turn_wall();
					tiles_traveled = 0;
				}
			}
		}

		DRIVE_ONE_TILE;
		++tiles_traveled;

		robot->turn(R90);
	}
}

void Rescue::turn_until_wall(int* wall_dist, int* duration, int max_dist, bool direction) {
	const int turn_speed_left = direction == BOOL_DIR_LEFT ? -35 : 35;
	const int turn_speed_right = -turn_speed_left;
	uint64_t offset = 0;
	*duration = millis_();
	robot->m(turn_speed_left, turn_speed_right);
	while(1) {
		int dist = robot->distance();
		if(dist < max_dist) {
			uint64_t start_time_stop = millis_();
			robot->stop();
			dist = robot->distance_avg(4, 0.2f);
			if(dist < max_dist) break;
			robot->m(turn_speed_left, turn_speed_right);
			offset += millis_() - start_time_stop;
		}
	}
	robot->m(turn_speed_left, turn_speed_right, 50);
	delay(20);
	robot->stop();
	*wall_dist = robot->distance_avg(20, 0.2f);
	*duration = millis_() - *duration - offset;
}

bool is_black2(uint8_t b, uint8_t g, uint8_t r) {
	return (uint16_t)b + (uint16_t)g + (uint16_t)r < BLACK_MAX_SUM;
}
