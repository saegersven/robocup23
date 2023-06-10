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
 * Saves .png image to subfolder in /home/pi/Desktop/iamges
 */
void save_img(cv::Mat& img, const std::string& subfolder) {
    std::string filename = "/home/pi/Desktop/iamges/" + subfolder + "/" + std::to_string(millis_()) + ".png";
    //std::cout << filename << std::endl;
	cv::imwrite(filename, img);
}

cv::Mat two_channel_to_three_channel(cv::Mat in) {
	cv::Mat out(in.rows, in.cols, CV_32FC3);
	for(int i = 0; i < in.rows; ++i) {
		float* p_in = in.ptr<float>(i);
		float* p_out = out.ptr<float>(i);
		for(int j = 0; j < in.cols; ++j) {
			p_out[j * 3] = p_in[j * 2];
			p_out[j * 3 + 1] = p_in[j * 2 + 1];
			p_out[j * 3 + 2] = 0.0f;
		}
	}
	return out;
}

float average_difference(cv::Mat a, cv::Mat b) {
	CV_Assert(a.depth() == CV_8U);
	CV_Assert(b.depth() == CV_8U);
	CV_Assert(a.cols == b.cols);
	CV_Assert(a.rows == b.rows);
	CV_Assert(a.channels() == b.channels());

	float difference = 0.0f;

	int i, j;
	for(i = 0; i < a.rows; ++i) {
		uint8_t* a_ptr = a.ptr<uint8_t>(i);
		uint8_t* b_ptr = b.ptr<uint8_t>(i);
		for(j = 0; j < a.cols * a.channels(); j++) {
			int16_t diff = a_ptr[j] - b_ptr[j];
			difference += std::abs(diff);
		}
	}
	return difference / (a.cols * a.rows * a.channels());
}

float map(float s, float a1, float a2, float b1, float b2) {
	return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
}