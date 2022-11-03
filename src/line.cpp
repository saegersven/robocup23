#include "line.h"

#include "defines.h"

Line::Line() {
	this->last_line_angle = 0.0f;


}

Line::start() {
	this->last_line_angle = 0.0f;
}

Line::create_maps() {
	float center_x = LINE_FRAME_WIDTH / 2;
	float center_y = LINE_FRAME_HEIGHT;

	this->distance_weight_map = cv::Mat(LINE_FRAME_HEIGHT, LINE_FRAME_WIDTH, cv::CV_8UC1);
	this->pixel_angles_map = cv::Mat(LINE_FRAME_HEIGHT, LINE_FRAME_WIDTH, cv::CV_8SC1);

	for(int y = 0; y < LINE_FRAME_HEIGHT; ++y) {
		uint8_t* p_dwm = this->distance_weight_map.ptr<uint8_t>(y);
		int8_t* p_pam = this->pixel_angles_map.ptr<int8_t>(y);
		for(int x = 0; x < LINE_FRAME_WIDTH; ++x) {
			float xdif = x - center_x;
			float ydif = y - center_y;
			p_dwm[x] = std::round(std::sqrt(xdif*xdif + ydif*ydif));
			p_dam[x] = std::round(std::atan2(ydif, xdif) / PI * 255.0f);
		}
	}
}

float Line::difference_weight(float x) {
	return 0.25f + 0.75f * std::pow(2, -std::pow(x * 5, 2));
}

float Line::distance_weight(float x) {
	return std::pow(2, -std::pow(((x - 0.65) * 4), 2));
}

float Line::line_angle(cv::Mat in) {
	float weighted_line_angle = 0.0f;
	float total_weight = 0.0f;

	uint32_t num_angles = 0;

	for(int i = 0; i < in.rows; ++i) {
		uint8_t* p = in.ptr<uint8_t>(i);
		uint8_t* p_dwm = in.ptr<uint8_t>(i);

		for(int j = 0; j < in.cols; ++j) {
			if(p[j]) {

			}
		}
	}
}