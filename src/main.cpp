#include <iostream>
#include <chrono>
#include <thread>

#include "line.h"
#include "robot.h"
#include "rescue.h"
#include "rescue.cpp"

enum class State {
	line,
	rescue
};

int main() {
	std::cout << "Program started." << std::endl;

	std::shared_ptr<Robot> robot = std::make_shared<Robot>();

	/*
	robot->start_camera(142, 80, 120);
	cv::Mat frame;
	long long start_time = millis();
	long frame_counter = 0;
	while(1) {
		frame = robot->grab_frame();
		cv::imshow("Frame", frame);
		cv::waitKey(1);

		auto now = std::chrono::high_resolution_clock::now();
		std::cout << frame_counter * 1000.0f / (millis() - start_time) << std::endl;
		++frame_counter;
	}
	return 0;*/

	/*while(true) {
		auto start_time = millis();
		std::cout << robot->button() << "\n";
		auto receive_time = millis();
		std::cout << receive_time - start_time<< std::endl;
	}*/

	State state = State::line;
	robot->stop();

	/*
	// obstacle motor values:
	robot->m(50, 50, 0);
	while (robot->distance() > 8);
	robot->m(-50, -50, 100);
	robot->turn(DTOR(92));
	delay(3000);
	robot->m(54, 127, 0);
	delay(3000);
	robot->stop();
	*/
	Line line(robot);
	
	Rescue rescue(robot);
	
	// TODO: SET SERVOS TO DEFAULT POS

	robot->set_cam_angle(90);
	std::cout << "Init." << std::endl;
	robot->toggle_led();
	while(!robot->button()) {
		delay(10);
	}
	while(robot->button());

	line.start();
	robot->toggle_led();
	delay(100);

	auto last_started = millis_(); // time at which robot has been restarted
	
	// MAIN LOOP
	while(1) {
		uint64_t duration = millis_();
		while(robot->button()) delay(10);
		duration = millis_() - duration;
		if(duration > 20 && millis_() - last_started > 300) { // if Restart_btn is pressed and main program has been running for at least 300ms:
			while(robot->button());
			robot->stop();
			switch(state) {
				case State::line:
					line.stop();
					break;
				case State::rescue:
					rescue.stop();
					state = State::line;
					break;
			}
			
			std::cout << "Stop." << std::endl;
			delay(300);
			robot->toggle_led();

			while(!robot->button()) {
				delay(10);
			} 
			while(robot->button());
			std::cout << "Start." << std::endl;
			robot->toggle_led();
			//line.check_silver();
			line.start();
		}

		switch(state) {
			case State::line: {
				if(line.found_silver) {
					std::cout << "Starting rescue" << std::endl;
					line.stop();
					rescue.start();
					state = State::rescue;
				} else {
					line.line();					
				}
				break;
			}
			case State::rescue: {
				// Monitor rescue thread
				if(rescue.finished) {
					std::cout << "Starting line" << std::endl;
					rescue.stop();
					line.start();
					state = State::line;
				}
				break;
			}
		}
	}

	return 0;
}
