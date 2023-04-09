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
	last_line_angle = 0.0f;
	camera_opened = false;
	create_maps();
}

void Line::start() {
	last_line_angle = 0.0f;
	camera_opened = false;
	found_silver = false;
	open_camera(320, 192);
	silver_ml.start();
	robot->set_blocked(false);
	// Start obstacle thread
	running = true;
	obstacle_enabled = true;
	obstacle_active = false;
	std::thread t([this] {this->obstacle();});
	t.detach();
}

void Line::check_silver() {
	obstacle_enabled = false;
	delay(20);
	std::cout << "Checking silver before line..." << std::endl;
	grab_frame(80, 48);
	std::cout << "Grabbed frame" << std::endl;
	int dist = 100; //robot->distance_avg(10, 0.2f);
	std::cout << "Distance: " << dist << std::endl;
	if (dist < 135 && dist > 90) { // dist must be between 120 and 90cm for rescue area
		std::cout << "Distance within range, counting black pixels..." << std::endl;
		close_camera();

		robot->servo(SERVO_CAM, (int)((CAM_LOWER_POS + CAM_HIGHER_POS) / 2), 100);
		robot->servo(SERVO_CAM, (int)((CAM_LOWER_POS + CAM_HIGHER_POS) / 2), 100);
		delay(200);

		open_camera();
		grab_frame(80, 48);

		uint32_t num_black_pixels = 0;
		black = in_range(frame, &is_black, &num_black_pixels);
		std::cout << "Black pixels: " << num_black_pixels << std::endl;
		if(num_black_pixels < 200) {
			std::cout << "Few black pixels, should be rescue" << std::endl;
			close_camera();
			found_silver = true;
			return;
		}
	}
	obstacle_enabled = true;
	robot->servo(SERVO_CAM, CAM_LOWER_POS, 300);
	delay(20);
}

void Line::stop() {
	robot->stop();
	close_camera();
	silver_ml.stop();
	robot->set_blocked(false);
	running = false; // Stop obstacle thread
	obstacle_enabled = true;
	obstacle_active = false;
	cv::destroyAllWindows();
	std::cout << "Line stopped." << std::endl;
}

void Line::open_camera(int width, int height) {
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

void Line::close_camera() {
	if(!camera_opened) return;

	cap.release();
	camera_opened = false;
}

void Line::grab_frame(int width, int height) {
	if(!camera_opened) {
		std::cerr << "Tried to grab frame with closed camera" << std::endl;
		exit(ERRCODE_CAM_CLOSED);
	}

	cap.grab();
	cap.retrieve(frame);
	cv::resize(frame, frame, cv::Size(width, height));
	debug_frame = frame.clone();
	cv::flip(debug_frame, frame, 0);
	cv::flip(frame, debug_frame, 1);
	frame = debug_frame.clone();
}

void Line::obstacle() {
	while(running) {
		continue;
		if((obstacle_enabled == false) || (obstacle_active == true)) continue;

		int dist = robot->distance();
		std::cout << dist << std::endl;
		delay(30);

		if(dist < 10) {
			robot->stop();
			robot->set_blocked(true);
			delay(10);

			if(robot->distance_avg(10, 0.2f) < 12) {
				delay(50);
				if(robot->distance_avg(10, 0.2f) < 12) {
					robot->set_blocked(false);
					delay(20);
					obstacle_active = true;

					/*std::cout << "OBSTACLE!" << std::endl;
					obstacle_active = false;
					delay(3000);*/
				}
			}
			robot->set_blocked(false);
		}
	}
}

bool Line::obstacle_straight_line(int duration) {
	auto start_t = std::chrono::high_resolution_clock::now();

	close_camera();
	open_camera();

	while(1) {
		robot->m(50, 50);

		grab_frame();
		cv::imshow("Obstacle", frame);
		cv::waitKey(1);

		cv::Mat roi = frame(cv::Range(0, 48), cv::Range(50, 80));
		uint32_t roi_size = roi.cols * roi.rows;

		uint32_t num_black = 0;
		in_range(roi, &is_black, &num_black);

		float p = (float)num_black / roi_size;
		std::cout << p << std::endl;

		if(p > 0.08f) {
			// Abort
			save_img(roi, "obstacle_roi");
			save_img(frame, "obstacle_frame");

			robot->stop();
			return true;
		}

		auto now = std::chrono::high_resolution_clock::now();
		uint32_t elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_t).count();
		if(elapsed >= duration) return false;
	}
}

float Line::difference_weight(float x) {
	// Uncomment for everything other than linear
	x = x / PI * 2.0f;
	
	
	// Exponential
	return 0.25f + 0.75f * std::exp(-16.0f * x*x);


	// Rectified Linear (should be fastest)
	/*if(x < 0.0f) x = -x;
	if(x > 0.6283185307f) return 0.25f;
	return -1.1936620731892f * x;*/

	/*
	// Rectified Quartic Polynomial (probably slowest, does not work anymore)
	if(x < 0.0f) x = -x;
	if(x > 0.387298332f) return 0.25f;
	float xx = x*x;
	return 33.333333f * xx*xx - 10.0f * xx + 1.0f;
	*/
}

float Line::distance_weight(float x) {
	// Precalculated, performance does not matter
	/*// Old Function
	float f = std::pow(2, -std::pow(4.2f * x - 2.6, 2)) - 0.1f;
	if(f < 0.0f) f = 0.0f;
	return f;
	*/

	
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

float Line::get_line_angle(cv::Mat in) {
	cv::imshow("Black", black);
	cv::waitKey(1);

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
	
	/*
	if(num_black_pixels < 40 && std::abs(RTOD(last_line_angle)) > 5.0f) { // TODO: why is second condition always false?
		std::cout << "Last line angle: " << last_line_angle << std::endl;
		robot->stop();
		close_camera();
		delay(100);
		robot->m(-LINE_FOLLOW_BASE_SPEED, -LINE_FOLLOW_BASE_SPEED, 250);
		robot->stop();
		line_follow_sensitivity = LINE_FOLLOW_SENSITIVITY / 2.0f;
		open_camera();
		delay(100);
		grab_frame();
	}
	*/
	

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
	//if(std::abs(RTOD(line_angle)) < 20.0f) base_speed = LINE_FOLLOW_STRAIGHT_LINE_SPEED;

	float ll = line_angle * line_angle;
	//float extra_sensitivity = std::abs(line_angle) * (0.3f*ll + 1.0f); //(0.1f * ll*ll - 0.02f*ll + 1.0f));
	float extra_sensitivity = 0.3141f * line_angle * line_angle * line_angle * line_angle + 1.0f;

	float ees_l = line_angle < 0.0f ? 1.5f : 0.2f;
	float ees_r = line_angle > 0.0f ? 1.5f : 0.2f;

	robot->m(
		clamp(base_speed + line_angle * extra_sensitivity * ees_l * line_follow_sensitivity, -128, 127),
		clamp(base_speed - line_angle * extra_sensitivity * ees_r * line_follow_sensitivity, -128, 127)
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

		// Calculate the "average black pixel" around the green dot
		int y, x;
		for(y = y_start; y < y_end; ++y) {
			uint8_t* p = black.ptr<uint8_t>(y);
			uint8_t* p_grn = green.ptr<uint8_t>(y);
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
		obstacle_enabled = false;
		obstacle_active = false;

		std::cout << "Approaching intersection (" << std::to_string(green_result) << ")\n";

		robot->stop();

		// Approach	
		float dif_x = global_average_x - frame.cols / 2.0f;
		float dif_y = global_average_y - (frame.rows + 20.0f);
		float angle = std::atan2(dif_y, dif_x) + PI05;
		float distance = std::sqrt(dif_x*dif_x + dif_y*dif_y);

		close_camera();
		robot->turn(angle);
		delay(50);
		robot->m(100, 100, DISTANCE_FACTOR * (distance - 50));

		// Take another frame and reevaluate
		open_camera();
		grab_frame(80, 48);
		save_img(frame, "green");
		close_camera();
		black = in_range(frame, &is_black);
		uint8_t new_green_result = green_direction(global_average_x, global_average_y);
		//uint8_t new_green_result = green_result;
		robot->m(60, 60, 250);

		// TODO: See if checking the previous result is actually necessary
		if(new_green_result == GREEN_RESULT_DEAD_END || green_result == GREEN_RESULT_DEAD_END) {
			std::cout << "Result: DEAD END" << std::endl;
			robot->turn(DTOR(170.0f));
			robot->m(127, 127, 150);
		} else if(green_result == GREEN_RESULT_LEFT) {
			std::cout << "Result: LEFT" << std::endl;
			robot->turn(DTOR(-73.0f));			
			delay(70);
		} else if(green_result == GREEN_RESULT_RIGHT) {
			std::cout << "Result: RIGHT" << std::endl;
			robot->turn(DTOR(73.0f));
			delay(70);
		}
		robot->m(127, 127, 70);
		open_camera();
		grab_frame(80, 48);
		uint32_t num_black_pixels = 0;
		black = in_range(frame, &is_black, &num_black_pixels);
		
		if(num_black_pixels < 200) {
			std::cout << "Searching left and right\n";
			// Search for line left and right
			uint32_t num_black_pixels_right = 0;
			uint32_t num_black_pixels_left = 0;

			grab_frame(80, 48);
			close_camera();

			robot->turn(DTOR(40.0f));
			open_camera();
			grab_frame(80, 48);
			close_camera();

			black = in_range(frame, &is_black, &num_black_pixels_right);

			robot->turn(DTOR(-80.0f));
			open_camera();
			grab_frame(80, 48);
			close_camera();

			black = in_range(frame, &is_black, &num_black_pixels_left);

			std::cout << num_black_pixels_left << " | " << num_black_pixels_right << "\n";
			if(num_black_pixels_right > num_black_pixels_left) {
				robot->turn(DTOR(85.0f));
			}
			open_camera();
		}
		obstacle_enabled = true;
	}
}

void Line::rescue_kit() {
	cv::Mat blue;
	std::vector<Group> groups = find_groups(frame, blue, &is_blue);

	if(groups.size() > 0) {
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

		float center_x = frame.cols / 2.0f;
		float center_y = frame.rows + 20.0f;
		float angle = std::atan2(group.y - center_y, group.x - center_x) + R90;
		float distance = std::sqrt(std::pow(group.y - center_y, 2) + std::pow(group.x - center_x, 2));
		save_img(frame, "rescue_kit");
		close_camera();
		robot->turn(angle - ARM_ANGLE_OFFSET);
		robot->send_byte(CMD_PICK_UP_RESCUE_KIT);
		delay(2400);
		robot->turn(-angle + ARM_ANGLE_OFFSET);
		open_camera();
	}
}

void Line::line() {
	//auto start_time = std::chrono::high_resolution_clock::now();

	//auto main_start_time = std::chrono::high_resolution_clock::now();
	grab_frame(80, 48);
	follow();
	green();
	rescue_kit();

	//auto silver_start_time = std::chrono::high_resolution_clock::now();

	silver_ml.set_frame(frame);

	//auto end_time = std::chrono::high_resolution_clock::now();

	/*uint32_t total_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
	uint32_t frame_time = std::chrono::duration_cast<std::chrono::microseconds>(main_start_time - start_time).count();
	uint32_t main_time = std::chrono::duration_cast<std::chrono::microseconds>(silver_start_time - main_start_time).count();
	uint32_t silver_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - silver_start_time).count();

	std::cout << frame_time << "\t|\t" << main_time << "\t|\t" << silver_time << "\n" << total_time << "\n";*/

	if(silver_ml.get_current_prediction()) {
		obstacle_enabled = false;
		robot->set_blocked(false);
		delay(50);
		std::cout << "NN detected silver, checking distance..." << std::endl;
		save_img(frame, "potential_silver");
		robot->turn(last_line_angle / 3.0f);
		robot->stop();
		delay(200);
		int dist = 100; //robot->distance_avg(10, 0.2f);
		std::cout << "Distance: " << dist << std::endl;
		if (dist < 135 && dist > 90) { // dist must be between 120 and 90cm for rescue area
			std::cout << "Distance within range, counting black pixels..." << std::endl;
			close_camera();

			robot->servo(SERVO_CAM, (int)((CAM_LOWER_POS + CAM_HIGHER_POS) / 2), 150);
			delay(200);

			open_camera();
			grab_frame(80, 48);

			uint32_t num_black_pixels = 0;
			black = in_range(frame, &is_black, &num_black_pixels);
			std::cout << "Black pixels: " << num_black_pixels << std::endl;
			if(num_black_pixels < 200) {
				std::cout << "Few black pixels, should be rescue" << std::endl;
				close_camera();
				found_silver = true;
				return;
			}
		}
		robot->servo(SERVO_CAM, CAM_LOWER_POS, 300);
		// while delaying for servo movement reopen camera prophylactically to clear frame buffer
		close_camera();
		delay(200);
		robot->m(80, 80, 50);
		open_camera();
		obstacle_enabled = true;
	}

	if(obstacle_active) {
		std::cout << "OBSTACLE ACTIVE" << std::endl;
		close_camera();

		robot->m(-100, -100, 120);
		delay(20);
		robot->turn(-R90);
		robot->m(100, 100, 500);
		delay(20);
		robot->turn(R90);
		open_camera();

		const uint32_t durations[] = {1250, 1250, 1250, 675};

		for(int i = 0; i < 4; ++i) {
			if(obstacle_straight_line(durations[i])) break;

			close_camera();
			robot->turn(R90);
			open_camera();
		}

		close_camera();

		robot->m(-40, -40, 180);
		robot->m(-40, 40, 200);
		robot->m(80, 80, 330);
		robot->m(-80, 80, 180);
		robot->m(-40, -40, 260);

		open_camera();

		obstacle_active = false;
	}

#ifdef DEBUG
#ifdef DRAW_FPS
	auto now_t = std::chrono::high_resolution_clock::now();
	uint32_t us = std::chrono::duration_cast<std::chrono::microseconds>(now_t - last_frame_t).count();
	int fps = std::round(1.0f / ((float)us / 1000000.0f));
	cv::putText(debug_frame, std::to_string(fps),
		cv::Point(2, 8), cv::FONT_HERSHEY_DUPLEX,
		0.4, cv::Scalar(0, 255, 0));
	last_frame_t = now_t;
#endif
	cv::imshow("Debug", debug_frame);
	cv::waitKey(1);
#endif
}