#include "utils.h"

#include <algorithm>

cv::Mat in_range(cv::Mat in, std::function<bool (uint8_t, uint8_t, uint8_t)> f, uint32_t* num_pixels) {
	uint32_t num_p = 0;

	int rows = in.rows;
	int cols = in.cols;

	cv::Mat out(rows, cols, CV_8UC1);

	int i, j;
	for(i = 0; i < rows; ++i) {
		cv::Vec3b* p = in.ptr<cv::Vec3b>(i);
		uint8_t* p_out = out.ptr<uint8_t>(i);
		for(j = 0; j < cols; j++) {
			if(f(p[j][0], p[j][1], p[j][2])) {
				p_out[j] = 0xFF;
				++num_p;
			} else {
				p_out[j] = 0x00;
			}
		}
	}
	if(num_pixels != nullptr) {
		*num_pixels = num_p;
	}
	return out;
}

float clamp(float n, float min, float max) {
	return (n < min ? min : (n > max ? max : n));
}

/*
 * Saves .png image to subfolder in /home/pi/Desktop/images
 */
void save_img(cv::Mat img, const std::string& subfolder) {
	cv::imwrite("/home/pi/Desktop/images/" + subfolder + std::to_string(millis()) + ".png", img);
}