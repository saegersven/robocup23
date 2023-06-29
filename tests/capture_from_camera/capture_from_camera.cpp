#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <chrono>

// compiling & executing:

// navigate to capture_from_camera dir
// mkdir -p build/default
// cd build/default
// cmake -GNinja ../..
// ninja
// ./capture_from_camera


/**
 * Milliseconds since January 1st, 1970 at 00:00:00
 */
#define millis() std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()

// Line/Silver: 80x48
// Rescue victims: 160x120
// Rescue corner: 160x120
// Rescue exit: 640x480
#define WIDTH 80
#define HEIGHT 48

//#define TIME

void save_img(cv::Mat& img, const std::string& subfolder) {
    std::string filename = "/home/pi/Desktop/iamges/" + subfolder + "/" + std::to_string(millis()) + ".png";
    //std::cout << filename << std::endl;
	cv::imwrite(filename, img);
}

int main(int, char**) {
    std::cout << WIDTH << " " << HEIGHT << std::endl;
    cv::VideoCapture cap;
    cap.open(0, cv::CAP_ANY);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, WIDTH * 4);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, HEIGHT * 4);
	cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);
    cap.set(cv::CAP_PROP_FPS, 120);
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    uint64_t last_time = millis();
    cv::Mat frame;
    for (;;) {
        cap.read(frame);
        cv::Mat debug_frame;
        cv::resize(frame, frame, cv::Size(WIDTH, HEIGHT));
        debug_frame = frame.clone();
        cv::flip(debug_frame, frame, 0);
        cv::flip(frame, debug_frame, 1);

        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";ls
            break;
        }
        cv::imshow("Live", frame);
        if(cv::waitKey(1) == 'c') {
            save_img(frame, "captured");
            std::cout << "Saved img (" << WIDTH << "x" << HEIGHT << ")" << std::endl;
        }
        
        //save_img(frame, "captured");
        //cv::waitKey(5);
    }
    return 0;
}