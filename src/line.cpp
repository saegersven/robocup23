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
	camera_opened = false;
	create_maps();
}

void Line::start() {
	last_line_angle = 0.0f;
	camera_opened = false;
	open_camera();
	silver_ml.start();
}

void Line::stop() {
	robot->stop();
	close_camera();
	silver_ml.stop();
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
	if(!camera_opened) {
		std::cerr << "Tried to grab frame with closed camera" << std::endl;
		exit(ERRCODE_CAM_CLOSED);
	}

	cap.grab();
	cap.retrieve(frame);
	cv::resize(frame, frame, cv::Size(LINE_FRAME_WIDTH, LINE_FRAME_HEIGHT));
	debug_frame = frame.clone();
	cv::flip(debug_frame, frame, 0);
	cv::flip(frame, debug_frame, 1);
	frame = debug_frame.clone();
}

float Line::difference_weight(float x) {
	return 0.25f + 0.75f * std::pow(2, -std::pow(x * 5, 2));
}

float Line::distance_weight(float x) {
	float f = std::pow(2, -std::pow(4.2f * x - 2.6, 2)) - 0.1f;
	if(f < 0.0f) f = 0.0f;
	return f;
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
			p_pam[x] = std::atan2(y - center_y, x - center_x) + (PI / 2.0f);
		}
	}
}

float Line::get_line_angle(cv::Mat in) {
	cv::imshow("Black", black);
	cv::waitKey(1);

	float weighted_line_angle = 0.0f;
	float total_weight = 0.0f;

	uint32_t num_angles = 0;
	float center_x = in.cols / 2.0f;
	float center_y = in.rows;

	for(int y = 0; y < in.rows; ++y) {
		uint8_t* p = in.ptr<uint8_t>(y);
		float* p_dwm = this->distance_weight_map.ptr<float>(y);
		float* p_pam = this->pixel_angles_map.ptr<float>(y);

		for(int x = 0; x < in.cols; ++x) {
			if(p[x]) {
				float distance_weight = p_dwm[x];
				if(distance_weight > 0.0f) {
					++num_angles;

					float angle = std::atan2(y - center_y, x - center_x) + (PI / 2.0f);
					//float angle = p_pam[x];
					float angle_difference_weight = difference_weight((angle - last_line_angle) / PI * 2.0f);

					float weight = angle_difference_weight * distance_weight;
					weighted_line_angle += weight * angle;
					total_weight += weight;
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
	
	cv::putText(debug_frame, std::to_string(line_angle * 180 / PI),
		cv::Point(20, 8), cv::FONT_HERSHEY_DUPLEX,
		0.4, cv::Scalar(0, 100, 100));
#endif

	robot->m(
		clamp(LINE_FOLLOW_BASE_SPEED + line_angle * LINE_FOLLOW_SENSITIVITY, -128, 127),
		clamp(LINE_FOLLOW_BASE_SPEED - line_angle * LINE_FOLLOW_SENSITIVITY, -128, 127)
		);
}

void Line::add_to_group_center(int x_pos, int y_pos, cv::Mat ir, uint32_t& num_pixels, float& center_x, float& center_y) {
	int col_limit = ir.cols - 1;
	int row_limit = ir.rows - 1;

	for(int y = -1; y <= 1; ++y) {
		int y_index = y_pos + y;

		uint8_t* p = ir.ptr<uint8_t>(y_index);

		for(int x = -1; x <= 1; ++x) {
			if(y == 0 && x == 0) continue;

			int x_index = x_pos + x;

			if(p[x_index] == 0xFF) {
				p[x_index] = 0x7F;
				center_x += (float)x_index;
				center_y += (float)y_index;
				++num_pixels;

				if(x_index > 0 && x_index < col_limit && y_index > 0 && y_index < row_limit)
					add_to_group_center(x_index, y_index, ir, num_pixels, center_x, center_y);
			}
		}
	}
}

std::vector<Group> Line::find_groups(cv::Mat frame, cv::Mat& ir, std::function<bool (uint8_t, uint8_t, uint8_t)> f) {
	std::vector<Group> groups;

	uint32_t num_pixels = 0;
	ir = in_range(frame, f, &num_pixels);

	if(num_pixels < 50) return groups;

	for(int y = 0; y < ir.rows; ++y) {
		uint8_t* p = ir.ptr<uint8_t>(y);
		for(int x = 0; x < ir.cols; ++x) {
			// Don't check for non-zero, as found pixels are set to 0x7F
			// This way, information does not get lost
			// check for non-zero pixels
			if(p[x] == 0xFF) {
				//std::cout << "Creating group at " << x << ", " << y << std::endl;
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

	if(groups.size() == 0) return 0;
	std::cout << groups.size() << std::endl;

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

		cv::circle(debug_frame, cv::Point((int)groups[i].x, (int)groups[i].y),
			2, cv::Scalar(0, 0, 255), 2);
	}

	// Global average is the average of all green dots for optimal approach.
	global_average_x = 0.0f;
	global_average_y = 0.0f;
	uint8_t num_green_points = 0;

	// Cut out part of the black matrix around group centers
	for(int i = 0; i < groups.size(); ++i) {
		// Horizontal range
		int x_start = groups[i].x - cut_width / 2;
		int x_end = groups[i].x + cut_width / 2;
		if(x_start < 0) x_start = 0;
		if(x_end > black.cols) x_end = black.cols;

		// Vertical range
		int y_start = groups[i].y - cut_height / 2;
		int y_end = groups[i].y + cut_height / 2;
		if(y_start < 0) y_start = 0;
		if(y_end > black.rows) y_end = black.rows;

		float average_x = 0.0f;
		float average_y = 0.0f;

		uint32_t num_pixels = 0;

		std::cout << "Green size: " << green.rows << " x " << green.cols << std::endl;
		std::cout << "Black size: " << black.rows << " x " << black.cols << std::endl;

		// Calculate the "average black pixel" around the green dot
		int y, x;
		for(y = y_start; y < y_end; ++y) {
			std::cout << "A" << std::endl;
			uint8_t* p = black.ptr<uint8_t>(y);
			uint8_t* p_grn = green.ptr<uint8_t>(y);
			std::cout << "B" << std::endl;
			for(x = x_start; x < x_end; ++x) {
				std::cout << "C" << std::endl;
				uint8_t p_val = p[x];
				std::cout << "D" << std::endl;
				uint8_t p_grn_val = p_grn[x];
				std::cout << "E" << std::endl;
				if(p_val && !p_grn_val) {
					average_x += (float)x;
					average_y += (float)y;
					++num_pixels;
				}
			}
		}
		average_x /= num_pixels;
		average_y /= num_pixels;

		bool is_considered = average_y < groups[i].y;
		// Determine the quadrant of the average pixel
		if(is_considered) {
			// Only consider dot below the line (average is above green dot)
			global_average_x += average_x;
			global_average_y += average_y;
			++num_green_points;

			result |= average_x < groups[i].x ? 0x02 : 0x01;
		}

#ifdef DEBUG
		cv::circle(debug_frame, cv::Point((int)groups[i].x, (int)groups[i].y),
			2, cv::Scalar(0, 0, is_considered ? 255 : 100), 2);
#endif
	}

	global_average_x /= num_green_points;
	global_average_y /= num_green_points;

	return result;
}

void Line::green() {
	float global_average_x, global_average_y;
	uint8_t green_result = green_direction(global_average_x, global_average_y);

	if(green_result != 0) {
		std::cout << "Approaching intersection (" << std::to_string(green_result) << ")\n";

		// Approach	
		float dif_x = global_average_x - frame.cols / 2.0f;
		float dif_y = global_average_y - (frame.rows + 20.0f);
		float angle = std::atan2(dif_y, dif_x) + PI05;
		float distance = std::sqrt(dif_x*dif_x + dif_y*dif_y);

		close_camera();

		robot->stop();
		//robot->turn(angle);
		delay(50);
		//robot->m(100, 100, DISTANCE_FACTOR * (distance - 45));
		// Take another frame and reevaluate
		open_camera();
		grab_frame();
		close_camera();
		black = in_range(frame, &is_black);
		delay(1000);
		uint8_t new_green_result = green_direction(global_average_x, global_average_y);

		robot->m(60, 60, 250);

		// TODO: See if checking the previous result is actually necessary
		if(new_green_result == GREEN_RESULT_DEAD_END || green_result == GREEN_RESULT_DEAD_END) {
			std::cout << "Result: DEAD END" << std::endl;
			robot->turn(R180);
			delay(70);
			robot->m(60, 60, 150);
		} else if(green_result == GREEN_RESULT_LEFT) {
			std::cout << "Result: LEFT" << std::endl;
			robot->turn(DTOR(-85.0f));
			delay(70);
		} else if(green_result == GREEN_RESULT_RIGHT) {
			std::cout << "Result: RIGHT" << std::endl;
			robot->turn(DTOR(85.0f));
			delay(70);
		}
		robot->m(127, 127, 50);
		open_camera();
	}
}

void Line::line() {
	grab_frame();

	follow();
	green();
	silver_ml.set_frame(frame);
	std::cout << "S: " << silver_ml.get_current_prediction() << std::endl;

#ifdef DEBUG
#ifdef DRAW_FPS
	auto now_t = std::chrono::high_resolution_clock::now();
	uint32_t us = std::chrono::duration_cast<std::chrono::microseconds>(now_t - last_frame_t).count();
	int fps = std::round(1.0f / ((float)us / 1000000));
	cv::putText(debug_frame, std::to_string(fps),
		cv::Point(2, 8), cv::FONT_HERSHEY_DUPLEX,
		0.2, cv::Scalar(0, 255, 0));
	last_frame_t = now_t;
#endif
	cv::waitKey(1);
#endif
}