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

Line::Line(std::shared_ptr<Robot> robot) {
	this->robot = robot;
	last_line_angle = 0.0f;
	create_maps();
}

void Line::start() {
	last_line_angle = 0.0f;
	camera_opened = false;
	open_camera();
}

void Line::stop() {
	robot->stop();
	close_camera();
}

void Line::open_camera() {
	if(camera_opened) return;

	cap.open(0, cv::CAP_V4L2);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, LINE_FRAME_WIDTH * 4);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, LINE_FRAME_HEIGHT * 4);
	cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);

	if(!cap.isOpened()) {
		std::cerr << "Could not open camera" << std::endl;
		exit(ERRCODE_CAM_SETUP);
	}
	camera_opened = true;
}

void Line::close_camera() {
	if(!camera_opened) return;

	cap.release();
	camera_opened = false;
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
	uint32_t num_black_pixels = 0;
	black = in_range(frame, &is_black, &num_black_pixels);

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

void add_to_group_center(int x_pos, int y_pos, cv::Mat ir, uint32_t& num_pixels, float& center_x, float& center_y) {
	const int col_limit = ir.cols - 1;
	const int row_limit = ir.rows - 1;

	for(int y = -1; y <= 1; ++y) {
		int y_index = y_pos + y;

		uint8_t* p = ir.ptr<uint8_t>(y_index);

		for(int x = -1; x <= 1; ++x) {
			if(y == 0 && x == 0) continue;

			int x_index = x_pos + x;

			if(p[x_index] == 0xFF) {
				p[x_index] = 0x7F; // Something non-zero different from 0xFF, marking this pixel as found
				center_x += (float)x_index;
				center_y += (float)y_index;
				++num_pixels;

				if(x_index > 0 && x_index < col_limit
					&& y_index > 0 && y_index < row_limit) {
					add_to_group_center(x_index, y_index, ir, num_pixels, center_x, center_y);
				}
			}
		}
	}
}

std::vector<Group> find_groups(cv::Mat frame, cv::Mat ir, std::function<bool (uint8_t, uint8_t, uint8_t)> f) {
	std::vector<Group> groups;

	uint32_t num_pixels = 0;
	ir = in_range(frame, f, &num_pixels);

	if(num_pixels < 50) return groups;

	for(int y = 0; y < ir.rows; ++y) {
		uint8_t* p = ir.ptr<uint8_t>(y);
		for(int x = 0; x < ir.cols; ++x) {
			if(p[x] == 0xFF) {
				uint32_t num_group_pixels = 0;
				float group_center_x = 0.0f;
				float group_center_y = 0.0f;
				add_to_group_center(x, y, ir, num_group_pixels, group_center_x, group_center_y);

				if(num_group_pixels > 130) {
					group_center_x /= num_group_pixels;
					group_center_y /= num_group_pixels;
					groups.push_back({group_center_x, group_center_y, num_group_pixels});
				}
			}
		}
	}

	return groups;
}

uint8_t Line::green_direction(float& global_average_x, float& global_average_y) {
	uint8_t result = 0;

	cv::Mat green;
	std::vector<Group> groups = find_groups(frame, green, &is_green);

	const int cut_width = 30;
	const int cut_height = 30;

	// Check if all of the groups are between certain y values to prevent premature evaluation
	// of a dead-end or late evaluation of green points behind a line, when the lower line is
	// already out of the frame
	for(int i = 0; i < groups.size(); ++i) {
		if(groups[i].y < 10) return 0;
		if(groups[i].y > 35) return 0;
		//if(groups[i].x < 8) return 0;
		//if(groups[i].x > frame.cols - 8) return 0;
	}

	// Global average is the average of all green dots for optimal approach.
	global_average_x = 0.0f;
	global_average_y = 0.0f;
	uint8_t num_green_points = 0;

	// Cut out part of the black matrix around group centers
	for(int i = 0; i < groups.size(); ++i) {
		int x_start = groups[i].x - cut_width / 2;
		int x_end = groups[i].x + cut_width / 2;
		if(x_start < 0) x_start = 0;
		if(x_end > black.cols) x_end = black.cols;

		int y_start = groups[i].y - cut_height / 2;
		int y_end = groups[i].y + cut_height / 2;
		if(y_start < 0) y_start = 0;
		if(y_end > black.rows) y_end = black.rows;

		float average_x = 0.0f;
		float average_y = 0.0f;

		uint32_t num_pixels = 0;

		// Calculate the "average black pixels" around the green dot
		for(int y = y_start; y < y_end; ++y) {
			uint8_t* p = black.ptr<uint8_t>(y);
			uint8_t* p_grn = green.ptr<uint8_t>(y);

			for(int x = x_start; x < x_end; ++x) {
				if(p[x] && !p_grn[x]) {
					average_x += (float)x;
					average_y += (float)y;
					++num_pixels;
				}
			}
		}
		average_x /= num_pixels;
		average_y /= num_pixels;

		// Determine the quadrant of the average pixel
		if(average_y < groups[i].y) {
			// Only consider dot below the line (average is above green dot)
			global_average_x += average_x;
			global_average_y += average_y;
			++num_green_points;

			result |= average_x < groups[i].x ? 0x02 : 0x01;
		}
	}

	global_average_x /= num_green_points;
	global_average_y /= num_green_points;

	return result;
}

void Line::green() {
	float global_average_x, global_average_y;
	uint8_t green_result = green_direction(global_average_x, global_average_y);

	if(green_result != 0) {
		// Approach
		float dif_x = global_average_x - frame.cols / 2.0f;
		float dif_y = global_average_y - (frame.rows + 20.0f);
		float angle = std::atan2(dif_y, dif_x) + PI05;
		float distance = std::sqrt(dif_x*dif_x + dif_y*dif_y);

		close_camera();

		robot->stop();
		robot->turn(angle);
		delay(50);
		robot->m(100, 100, DISTANCE_FACTOR * (distance - 45));

		// Take another frame and reevaluate
		open_camera();
		grab_frame();
		close_camera();

		black = in_range(frame, &is_black);

		uint8_t new_green_result = green_direction(frame, black, global_average_x, global_average_y);

		robot->m(60, 60, 250);

		// TODO: See if checking the previous result is actually necessary
		if(new_green_result == GREEN_RESULT_DEAD_END || green_result == GREEN_RESULT_DEAD_END) {
			robot->turn(R180);
			delay(70);
			robot->m(60, 60, 150);
		} else if(green_result == GREEN_RESULT_LEFT) {
			robot->turn(DTOR(-70.0f));
			delay(70);
		} else if(green_result == GREEN_RESULT_RIGHT) {
			robot->turn(DTOR(70.0f));
			delay(70);
		}
		robot->m(60, 60, 130);
		open_camera();
	}
}

void Line::line() {
	grab_frame();

	debug_frame = frame.clone();

	follow();
	green();

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