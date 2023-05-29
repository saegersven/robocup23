#include "line.h"

#include <opencv2/opencv.hpp>
#include <functional>

#include "utils.h"
#include "defines.h"

bool is_black(uint8_t b, uint8_t g, uint8_t r) {
	return (uint16_t)b + (uint16_t)g + (uint16_t)r < BLACK_MAX_SUM;
}

bool is_green(uint8_t b, uint8_t g, uint8_t r) {
	return 1.0f / GREEN_RATIO_THRESHOLD * g > b + r && g > GREEN_MIN_VALUE;
}

bool is_blue(uint8_t b, uint8_t g, uint8_t r) {
	return 1.0f / BLUE_RATIO_THRESHOLD * b > g + r && b > BLUE_MIN_VALUE;
}

bool is_red(uint8_t b, uint8_t g, uint8_t r) {
	return 1.0f / RED_RATIO_THRESHOLD * r > b + g && r > RED_MIN_VALUE;
}

Line::Line(std::shared_ptr<Robot> robot) {
	this->robot = robot;

	create_maps();
}

float Line::difference_weight(float x) {
	x = x / PI * 2.0f;
	return 0.25f + 0.75f * std::exp(-16.0f * x*x);
}

float Line::distance_weight(float x) {
	float e = (3.25f * x - 2.0f);
	float f = std::exp(-e*e) - 0.1f;
	return f > 0.0f ? f : 0.0f;
}

void Line::create_maps() {
	float center_x = LINE_FRAME_WIDTH / 2;
	float center_y = LINE_FRAME_HEIGHT;

	this->distance_weight_map = cv::Mat(LINE_FRAME_HEIGHT, LINE_FRAME_WIDTH, CV_32FC1);
	this->pixel_angles_map = cv::Mat(LINE_FRAME_HEIGHT, LINE_FRAME_WIDTH, CV_32FC1);

	for(int y = 0; y < LINE_FRAME_HEIGHT; ++y) {
		float* p_dwm = this->distance_weight_map.ptr<float>(y);
		float* p_pam = this->pixel_angles_map.ptr<float>(y);
		for(int x = 0; x < LINE_FRAME_WIDTH; ++x) {
			float xdif = x - center_x;
			float ydif = y - center_y;
			float dist = std::sqrt(xdif*xdif + ydif*ydif);
			p_dwm[x] = clamp(distance_weight(dist / LINE_FRAME_HEIGHT), 0.0f, 1.0f);
			p_pam[x] = std::atan2(y - center_y, x - center_x) + PI05;
		}
	}
}

void Line::start() {
	last_line_angle = 0.0f;

	robot->start_camera(142, 80, 120);
	robot->set_blocked(false);

	running = true;
	std::cout << "Line started.\n";
}

void Line::stop() {
	robot->stop();
	robot->set_blocked(false);
	running = false;

	cv::destroyAllWindows();
	std::cout << "Line stopped.\n";
}

void Line::grab_frame() {
	frame = robot->grab_frame();
	cv::transpose(frame, debug_frame);
	debug_frame = debug_frame(cv::Range(94, 142), cv::Range(0, 80));
	frame = debug_frame.clone();
}

float Line::get_line_angle(cv::Mat in) {
	float weighted_line_angle = 0.0f;
	float total_weight = 0.0f;

	uint32_t num_angles = 0;
	// Bottom center coordinates (point close to center of rotation of the robot)
	float center_x = in.cols / 2.0f;
	float center_y = in.rows;

	for(int y = 0; y < in.rows; ++y) {
		uint8_t* p = in.ptr<uint8_t>(y);
		float* p_dwm = this->distance_weight_map.ptr<float>(y);
		float* p_pam = this->pixel_angles_map.ptr<float>(y);

		for(int x = 0; x < in.cols; ++x) {
			// Low-quality servos make the camera see part of the wheels, ignore that part of the image
			if((x < LINE_CORNER_WIDTH || x > (LINE_FRAME_WIDTH - LINE_CORNER_WIDTH))
				&& y > (LINE_FRAME_HEIGHT - LINE_CORNER_HEIGHT)) continue;

			if(p[x]) {
				float distance_weight = p_dwm[x];
				if(distance_weight > 0.0f) {
					++num_angles;

					//float angle = std::atan2(y - center_y, x - center_x) + PI05;
					float angle = p_pam[x];
					float angle_difference_weight = difference_weight(angle - last_line_angle);

					float weight = angle_difference_weight * distance_weight;
					weighted_line_angle += weight * angle;
					total_weight += weight;
				}
			}
		}
	}

	if(num_angles < 40) return 0.0f;
	if(total_weight == 0.0f) return 0.0f;
	weighted_line_angle /= total_weight;

	return weighted_line_angle;
}

void Line::follow() {
	float base_speed = LINE_FOLLOW_BASE_SPEED;
	float line_follow_sensitivity = LINE_FOLLOW_SENSITIVITY;

	uint32_t num_black_pixels = 0;
	black = in_range(frame, &is_black, &num_black_pixels);

	float line_angle = get_line_angle(black);

	if(std::isnan(line_angle)) line_angle = 0.0f;

	// DEBUG
	cv::Point center(debug_frame.cols / 2, debug_frame.rows);
	const int LINE_LENGTH = 20;
	cv::line(debug_frame,
		center,
		cv::Point(std::sin(line_angle) * LINE_LENGTH, -std::cos(line_angle) * LINE_LENGTH) + center,
		cv::Scalar(0, 255, 0), 2
		);
	cv::putText(debug_frame, std::to_string(line_angle * 180 / PI),
		cv::Point(20, 8), cv::FONT_HERSHEY_DUPLEX,
		0.4, cv::Scalar(0, 100, 100));
	// END DEBUG

	float extra_sensitivity = 0.3141f * line_angle * line_angle * line_angle * line_angle + 1.0f;

	float ees_l = line_angle < 0.0f ? 1.5f : 0.2f;
	float ees_r = line_angle > 0.0f ? 1.5f : 0.2f;

	robot->m(
		clamp(base_speed - line_angle * extra_sensitivity * ees_r * line_follow_sensitivity, -128, 127),
		clamp(base_speed + line_angle * extra_sensitivity * ees_l * line_follow_sensitivity, -128, 127)
		);

	last_frame = frame.clone();
	last_line_angle = line_angle;
}

void Line::line() {
	grab_frame();

	follow();

	// DEBUG
	auto now_t = std::chrono::high_resolution_clock::now();
	uint32_t us = std::chrono::duration_cast<std::chrono::microseconds>(now_t - last_frame_t).count();
	int fps = std::round(1.0f / ((float)us / 1000000.0f));
	cv::putText(debug_frame, std::to_string(fps),
		cv::Point(2, 8), cv::FONT_HERSHEY_DUPLEX,
		0.4, cv::Scalar(0, 255, 0));
	last_frame_t = now_t;

	cv::imshow("Debug", debug_frame);
	cv::waitKey(1);
}