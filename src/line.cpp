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

float Line::green_weight(float x) {
	float f = green_weight_slope * x + 1.0f;
	return f < 0.0f ? 0.0f : f;	
}

void Line::create_maps() {
	float center_x = LINE_FRAME_WIDTH / 2 - 7.0f;
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

	robot->start_camera();
	robot->set_blocked(false);

	running = true;
	found_silver = false;
	std::cout << "Line started.\n";
}

void Line::stop() {
	robot->stop();
	robot->set_blocked(false);
	robot->stop_camera();
	running = false;

	cv::destroyAllWindows();
	std::cout << "Line stopped.\n";
}

void Line::grab_frame() {
	frame = robot->grab_frame();
	debug_frame = frame.clone();
}

float Line::get_line_angle(cv::Mat in) {
	float weighted_line_angle = 0.0f;
	float total_weight = 0.0f;

	uint32_t num_angles = 0;
	// Bottom center coordinates (point close to center of rotation of the robot)
	float center_x = in.cols / 2.0f - 7.0f;
	float center_y = in.rows;

	for(int y = 0; y < in.rows; ++y) {
		uint8_t* p = in.ptr<uint8_t>(y);
		uint8_t* p_grn = green_mat.ptr<uint8_t>(y);
		float* p_dwm = this->distance_weight_map.ptr<float>(y);
		float* p_pam = this->pixel_angles_map.ptr<float>(y);

		for(int x = 0; x < in.cols; ++x) {
			// Low-quality servos make the camera see part of the wheels, ignore that part of the image
			//if((x < LINE_CORNER_WIDTH || x > (LINE_FRAME_WIDTH - LINE_CORNER_WIDTH))
			//	&& y > (LINE_FRAME_HEIGHT - LINE_CORNER_HEIGHT)) continue;

			if(p[x] && !p_grn[x]) {
				float distance_weight = p_dwm[x];
				if(distance_weight > 0.0f) {
					++num_angles;

					//float angle = std::atan2(y - center_y, x - center_x) + PI05;
					float angle = p_pam[x];
					float angle_difference_weight = difference_weight(angle - last_line_angle);
					float x_green_weight = green_weight(x - center_x);

					float weight = angle_difference_weight * distance_weight * x_green_weight;
					weighted_line_angle += weight * angle;
					total_weight += weight;
				}
			}
		}
	}

	if(num_angles < 50) return 0.0f;
	if(total_weight == 0.0f) return 0.0f;

	std::cout << num_angles << std::endl;

	weighted_line_angle /= total_weight;

	if(num_angles < 300) return weighted_line_angle * num_angles / 300.0f;

	return weighted_line_angle;
}

void Line::follow() {
	float base_speed = LINE_FOLLOW_BASE_SPEED;
	float line_follow_sensitivity = LINE_FOLLOW_SENSITIVITY;

	// If frame does not change much, try to get unstuck by increasing motor speed
	if(!last_frame.empty) {
		float diff = average_difference(frame, last_frame);

		if(diff < 4.2f) {
			++no_difference_counter;
			if(no_difference_counter == 200) {
				if(ENABLE_NO_DIFFERENCE) {
					std::cout << "No difference\n";
					no_difference_time_stamp = millis();
				}
			}
		} else {
			// Slowly decrease counter by 10 at a time
			no_difference_counter = no_difference_counter >= 10 ? no_difference_counter - 10 : 0;
		}
	}

	if(millis() - no_difference_time_stamp < 300) {
		std::cout << "Increasing base motor speed\n";
		base_speed = LINE_FOLLOW_BASE_SPEED * 1.5f;
		if(std::abs(RTOD(last_line_angle)) > 10.0f) {
			std::cout << "Large line angle, trying to get unstuck\n";
			robot->m(127, 127, 42*2);
			robot->turn(last_line_angle);
		}
	}

	uint32_t num_black_pixels = 0;
	black = in_range(frame, &is_black, &num_black_pixels);

	cv::imshow("Black", black);

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

	float us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - last_frame_t).count();
	float line_angle_d = last_line_angle / us * 1000000.0f;

	robot->m(
		clamp(base_speed + line_angle * line_follow_sensitivity * ees_l * extra_sensitivity, -128, 127),
		clamp(base_speed - line_angle * line_follow_sensitivity * ees_r * extra_sensitivity, -128, 127)
		);


	last_frame = frame.clone();
	last_line_angle = line_angle;
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

	uint32_t num_pixels = green_num_pixels;
	//ir = in_range(frame, f, &num_pixels);

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

	std::vector<Group> groups = find_groups(frame, green_mat, &is_green);

	cv::imshow("Green", green_mat);
	cv::waitKey(1);

	if(groups.size() == 0) return 0;
	
	std::cout << groups.size() << " Groups\n";

	const int cut_width = 30;
	const int cut_height = 30;

	// Check if all of the groups are between certain y values to prevent premature evaluation
	// of a dead-end or late evaluation of green points behind a line, when the lower line is
	// already out of the frame
	for(int i = 0; i < groups.size(); ++i) {
		if(groups[i].y < 17) return 0;
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

		// Calculate the "average black pixel" around the green dot
		int y, x;
		for(y = y_start; y < y_end; ++y) {
			uint8_t* p = black.ptr<uint8_t>(y);
			uint8_t* p_grn = green_mat.ptr<uint8_t>(y);
			for(x = x_start; x < x_end; ++x) {
				uint8_t p_val = p[x];
				uint8_t p_grn_val = p_grn[x];
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

		cv::circle(debug_frame, cv::Point((int)groups[i].x, (int)groups[i].y),
			2, cv::Scalar(is_considered ? 255 : 0, 0, is_considered ? 0 : 255), 2);
	}

	global_average_x /= num_green_points;
	global_average_y /= num_green_points;

	return result;
}

void Line::green() {
	float global_average_x, global_average_y;

	// Green active time
	auto now = std::chrono::high_resolution_clock::now();
	uint32_t time_since_last_green = std::chrono::duration_cast<std::chrono::milliseconds>(now - green_start_t).count();

	green_active = time_since_last_green < GREEN_DURATION;
	if(green_active) std::cout << "GREEN ACTIVE\n";
	if(!green_active) green_weight_slope = 0.0f;

	uint8_t green_result = green_direction(global_average_x, global_average_y);

	if(green_result != 0) {
		obstacle_enabled = false;
		obstacle_active = false;

		if(green_result == GREEN_RESULT_DEAD_END) {
			// Dead-End regardless of green_active
			robot->m(127, 127, 200);
			robot->turn(DTOR(180.0f));
			robot->m(127, 127, 160);
			return;
		}

		if(green_active) return;

		float dx = global_average_x - frame.cols / 2.0f;
		float dy = global_average_y - (frame.rows + 20.0f);
		float angle = std::atan2(dy, dx) + PI05;
		float distance = std::sqrt(dx*dx + dy*dy);

		robot->m(50 + (last_line_angle + angle) * 40, 50 - (last_line_angle + angle) * 40);
		delay((int)(distance * 2.5f));

		green_start_t = now;

		green_weight_slope = GREEN_WEIGHT_SLOPE * (green_result == GREEN_RESULT_LEFT ? -1.0f : 1.0f);
	}
}

void Line::rescue_kit() {
	cv::Mat blue;
	std::vector<Group> groups = find_groups(frame, blue, &is_blue);

	if(groups.size() > 0) {
		// Use biggest group
		Group group = groups[0];

		if(groups.size() > 1) {
			for(int i = 1; i < groups.size(); ++i) {
				if(groups[i].num_pixels > group.num_pixels) {
					group = groups[i];
				}
			}
		}

		if(group.num_pixels < 100) return;

		robot->stop();

		// Approach rescue kit
		float center_x = frame.cols / 2.0f;
		float center_y = frame.rows + 20.0f;
		float dy = group.y - center_y;
		float dx = group.x - center_x;
		float angle = std::atan2(dy, dx) + R90;
		float distance = std::sqrt(dy*dy + dx*dx);

		// We can easily tolerate +-5° when picking up
		// With higher angles, we risk driving off an edge
		// So turn only as much as needed
		const float ANGLE_TOLERANCE = DTOR(5.0f);
		float to_turn = angle - ARM_ANGLE_OFFSET;
		if(std::abs(to_turn) > ANGLE_TOLERANCE) {
			if(to_turn > 0) to_turn -= ANGLE_TOLERANCE;
			else to_turn += ANGLE_TOLERANCE;
		}
		robot->turn(to_turn);

		int dur = distance * 2 - 120;
		robot->m(70, 70, dur);

		// Collect rescue kit
		robot->gripper(GRIPPER_CLOSE, 200);
		robot->attach_detach_servo(SERVO_ARM); // attach
		robot->servo(SERVO_ARM, ARM_LOWER_POS, 150);
		robot->m(-65, -65, 150);
		delay(20);
		robot->gripper(GRIPPER_OPEN);
		delay(420);
		robot->m(60, 60, 260);
		robot->gripper(GRIPPER_CLOSE);
		delay(50);
		robot->m(30, 30);
		delay(320);
		robot->gripper(GRIPPER_OPEN);
		delay(50);
		robot->gripper(GRIPPER_CLOSE);
		delay(150);
		robot->m(-65, -65, 100);
		robot->servo(SERVO_ARM, ARM_HIGHER_POS, 10);
		robot->m(-50, -50, 350);
		delay(700);
		robot->gripper(GRIPPER_OPEN, 50);
		delay(100);
		robot->gripper(GRIPPER_CLOSE, 200);
		robot->attach_detach_servo(SERVO_ARM); // detach

		robot->turn(-angle + ARM_ANGLE_OFFSET);
	}
}

void Line::red() {
	uint32_t num_red_pixels = 0;
	in_range(frame, &is_red, &num_red_pixels);

	uint32_t num_pixels = LINE_FRAME_HEIGHT * LINE_FRAME_WIDTH;
	float red_percentage = (float)num_red_pixels / (float)num_pixels;
	std::cout << "Red: " << red_percentage << std::endl;
	if(red_percentage > 0.23f) {
		std::cout << "Detected red" << std::endl;
		robot->m(100, 100, 300);
		delay(8000);
	}
}

void Line::silver() {
	silver_ml.set_frame(frame);

	if(silver_ml.get_current_prediction()) {
		obstacle_enabled = false;
		robot->set_blocked(false);
		delay(50);
		std::cout << "NN detected silver" << std::endl;
		robot->turn(last_line_angle / 3.0f);
		robot->stop();

		found_silver = true;
	}
}

void Line::line() {
	grab_frame();

	// Do green thresholding globally so we can exclude green points
	// during line following (dark green is often mistaken for black)
	// This improves asymmetric intersections where the robot has to go straight
	green_mat = in_range(frame, &is_green, &green_num_pixels);
	follow();
	green();
	rescue_kit();
	red();
	silver();

	// DEBUG, show fps and current frame
	auto now_t = std::chrono::high_resolution_clock::now();
	uint32_t us = std::chrono::duration_cast<std::chrono::microseconds>(now_t - last_frame_t).count();
	int fps = std::round(1.0f / ((float)us / 1000000.0f));
	//std::cout << fps << std::endl;
	cv::putText(debug_frame, std::to_string(fps),
		cv::Point(2, 8), cv::FONT_HERSHEY_DUPLEX,
		0.4, cv::Scalar(0, 255, 0));
	last_frame_t = now_t;

	cv::imshow("Debug", debug_frame);
	cv::waitKey(1);
}