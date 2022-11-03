#include "utils.h"

#include <algortithm>

cv::Mat in_range(cv::Mat& in, std::function<bool (uint8_t, uint8_t, uint8_t)> f, uint32_t* num_pixels) {
	uint32_t num_p = 0;

	cv::Mat out(in.rows, in.cols, CV_8UC1);

	for(int i = 0; i < in.rows; ++i) {
		cv::Vec3b* p = in.ptr<cv::Vec3b>(i);
		uint8_t* p_out = in.ptr<uint8_t>(i);

		for(int j = 0; j < in.cols; ++j) {
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