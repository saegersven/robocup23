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
				for(find_victims(x, y, dead); x == -1.0f;find_victims(x_victim, y_victim, dead)) {
					robot->m(40, 40);
				}
				robot->m(-50, -50, 100);
			}
			std::cout << "Angle: " << RTOD(angle_victim) << std::endl;
			std::cout << "Y: " << y_victim << std::endl;
			if(y > 45.0f) {
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
		} else if(x != last_x_victim) {
			//robot->m(-70, -70, 300);
		} else  {
			close_camera();
			robot->turn(DTOR(30.0f));
			delay(50);
			open_camera(VICTIM_CAP_RES);
		}
		last_x_victim = x;
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
	/*
	### OLD STUFF, COULD BE DELETED ###
	robot->servo(SERVO_CAM, 140);
	close_camera();
	open_camera(BLACK_CORNER_RES);
	while (1) {
		frame = grab_frame(BLACK_CORNER_RES);
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
  				open_camera(BLACK_CORNER_RES);
  				grab_frame(BLACK_CORNER_RES);
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
  					open_camera(BLACK_CORNER_RES);
  					delay(300);
				}
  			}
		} else {
			robot->turn(DTOR(5));
		}
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

	### END OF OLD STUFF ###
	*/
	robot->servo(0, CAM_HIGHER_POS);
	uint8_t deg_per_iteration = 10; // how many degrees should the robot turn after each check for black corner?

	float last_x_victim = -1.0f;
	float x_victim = 0.0f;
	
	while (1) {
		for (int i = 0; i < (int) 360 / deg_per_iteration; ++i) {
			cv::Mat frame = grab_frame(160, 120);
			cv::Mat res = corner_ml.invoke(frame);
			cv::imshow("Black corner", two_channel_to_three_channel(res));

			std::vector<Corner> corner = corner_ml.extract_corner(res, x_corner, y_corner);


			robot->turn(DTOR(deg_per_iteration));
		}
		// robot turned full 360 deg and did not find corner
		// recentre and increase cam angle a bit
		find_centre();
		robot->servo(0, CAM_HIGHER_POS + 5);

	}



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
		if(victims[i].y > y) {
			x_victims = victims[i].x;
			y_victims = victims[i].y;
		}
	}
}

bool is_black2(uint8_t b, uint8_t g, uint8_t r) {
	return (uint16_t)b + (uint16_t)g + (uint16_t)r < BLACK_MAX_SUM;
}
