#include "line.h"

#include <opencv2/opencv.hpp>
#include <functional>

#include "utils.h"
#include "defines.h"

bool is_black(uint8_t b, uint8_t g, uint8_t r) {
	return (uint16_t)b + (uint16_t)g + (uint16_t)r < BLACK_MAX_SUM;
}

Line::Line(std::shared_ptr<Robot> robot) {
	this->robot = robot;
	last_line_angle = 0.0f;
	create_maps();
}

void Line::start() {
	last_line_angle = 0.0f;

	// Init camera
	cap.open(0, cv::CAP_V4L2);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, LINE_FRAME_WIDTH * 4);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, LINE_FRAME_HEIGHT * 4);
	cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);

	if(!cap.isOpened()) {
		std::cerr << "Could not open camera" << std::endl;
		exit(ERRCODE_CAM_SETUP);
	}
}

void Line::stop() {
	robot->stop();
	cap.release();
}

void Line::grab_frame() {
	cap.grab();
	cap.retrieve(frame);
	cv::resize(frame, frame, cv::Size(LINE_FRAME_WIDTH, LINE_FRAME_HEIGHT));
	debug_frame = frame.clone();
	cv::flip(debug_frame, frame, 0);
	debug_frame = frame.clone();
}

float Line::difference_weight(float x) {
	return 0.25f + 0.75f * std::pow(2, -std::pow(x * 5, 2));
}

float Line::distance_weight(float x) {
	float f = std::pow(2, -std::pow(3.5f * x - 2.6, 2)) - 0.1f;
	if(f < 0.0f) f = 0.0f;
	return f;
}

void Line::create_maps() {
	float center_x = LINE_FRAME_WIDTH / 2;
	float center_y = LINE_FRAME_HEIGHT;

	this->distance_weight_map = cv::Mat(LINE_FRAME_HEIGHT, LINE_FRAME_WIDTH, CV_8UC1);
	this->pixel_angles_map = cv::Mat(LINE_FRAME_HEIGHT, LINE_FRAME_WIDTH, CV_8SC1);

	for(int y = 0; y < LINE_FRAME_HEIGHT; ++y) {
		uint8_t* p_dwm = this->distance_weight_map.ptr<uint8_t>(y);
		int8_t* p_pam = this->pixel_angles_map.ptr<int8_t>(y);
		for(int x = 0; x < LINE_FRAME_WIDTH; ++x) {
			float xdif = x - center_x;
			float ydif = y - center_y;
			float dist = std::sqrt(xdif*xdif + ydif*ydif);
			p_dwm[x] = clamp(distance_weight(dist / LINE_FRAME_HEIGHT), 0.0f, 1.0f) * 255.0f;
			p_pam[x] = std::round((std::atan2(ydif, xdif) + PI05) / PI * 255.0f);
		}
	}
}

float Line::get_line_angle(cv::Mat in) {
	float weighted_line_angle = 0.0f;
	float total_weight = 0.0f;

	uint32_t num_angles = 0;

	for(int i = 0; i < in.rows; ++i) {
		uint8_t* p = in.ptr<uint8_t>(i);
		uint8_t* p_dwm = this->distance_weight_map.ptr<uint8_t>(i);
		uint8_t* p_pam = this->pixel_angles_map.ptr<uint8_t>(i);

		for(int j = 0; j < in.cols; ++j) {
			if(p[j]) {
				uint8_t distance_weight = p_dwm[j];
				if(distance_weight) {
					float pixel_distance_weight = distance_weight / 255.0f;
					float angle = p_pam[j] / 255.0f * PI;
					float angle_difference_weight = difference_weight((angle - last_line_angle) / PI * 2.0f);

					float weight = angle_difference_weight * pixel_distance_weight;
					weighted_line_angle += weight * angle;
					total_weight += weight;

					++num_angles;
				}
			}
		}
	}

	if(num_angles < 40) return 0.0f;
	weighted_line_angle /= total_weight;

	return weighted_line_angle;
}

void Line::follow() {
	float line_angle = get_line_angle(black);

	if(std::isnan(line_angle)) line_angle = 0.0f;
	last_line_angle = line_angle;

#ifdef DEBUG
	cv::Point center(debug_frame.cols / 2, debug_frame.rows);

	const int LINE_LENGTH = 20;

	cv::line(debug_frame,
		center,
		cv::Point(std::sin(line_angle) * LINE_LENGTH, -std::cos(line_angle) * LINE_LENGTH) + center,
		cv::Scalar(0, 255, 0), 2
		);
#endif

	robot->m(
		LINE_FOLLOW_BASE_SPEED - line_angle * LINE_FOLLOW_SENSITIVITY,
		LINE_FOLLOW_BASE_SPEED + line_angle * LINE_FOLLOW_SENSITIVITY
		);
}

void Line::line() {
	grab_frame();

	debug_frame = frame.clone();

	uint32_t num_black_pixels = 0;
	black = in_range(frame, &is_black, &num_black_pixels);

	cv::imshow("Black", black);

	follow();

#ifdef DEBUG
#ifdef DRAW_FPS
	auto now_t = std::chrono::high_resolution_clock::now();
	uint32_t us = std::chrono::duration_cast<std::chrono::microseconds>(now_t - last_frame_t).count();
	int fps = std::round(1.0f / ((float)us / 1'000'000));
	cv::putText(debug_frame, std::to_string(fps),
		cv::Point(2, 8), cv::FONT_HERSHEY_DUPLEX,
		0.2, cv::Scalar(0, 255, 0));
	last_frame_t = now_t;
#endif

	cv::imshow("Debug", debug_frame);
	cv::waitKey(1);
#endif
}