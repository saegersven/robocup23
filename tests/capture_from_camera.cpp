#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>

int main(int, char**) {
    cap.open(0, cv::CAP_ANY);
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    for (;;) {
        cap.read(frame);
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        cv::imshow("Live", frame);
        save_img(frame, "silver")
        cv::waitKey(50);
    }
    return 0;
}

void save_img(cv::Mat img, const std::string& subfolder) {
	cv::imwrite("/home/pi/Desktop/images/" + subfolder + "/" + std::to_string(millis()) + ".png", img);
}