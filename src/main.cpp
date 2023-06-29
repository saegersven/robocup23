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
	std::shared_ptr<Robot> robot = std::make_shared<Robot>();

	/*robot->capture_height = 192;
	robot->capture_width = 320;
	robot->frame_height = 48;
	robot->frame_width = 80;
	robot->start_camera();
	cv::Mat frame;
	while(1) {
		frame = robot->grab_frame();
		cv::imshow("Frame", frame);
		cv::waitKey(1);
	}*/

	State state = State::line;
	Line line(robot);

	Rescue rescue(robot);

	// SET SERVOS TO DEFAULT POS
	robot->attach_detach_servo(SERVO_CAM); // attach cam servo, necessary???
	robot->gripper(GRIPPER_CLOSE, 200);
	robot->servo(2, CAM_LOWER_POS, 300); // don't detach so cam stays in position
	robot->servo(1, GATE_CLOSED, 300);
	robot->servo(0, ARM_HIGHER_POS, 300);

	while (!robot->button()) {
		robot->send_ready();
		delay(50);
	}
	while (robot->button());

	line.start();

	auto last_started = millis_(); // time at which robot has been restarted
	
	// MAIN LOOP
	while(1) {
		if(robot->button() && millis_() - last_started > 300) { // if Restart_btn is pressed and main program has been running for at least 300ms:
			while(robot->button());
			robot->stop();
			switch(state) {
				case State::line:
					line.stop();
					line.obstacle_direction = !line.obstacle_direction;
					break;
				case State::rescue:
					rescue.stop();
					state = State::line;
					break;
			}
			
			std::cout << "Stop." << std::endl;
			// SET SERVOS TO DEFAULT POS
			robot->servo(2, CAM_LOWER_POS, 300); // don't detach so cam stays in position
			robot->servo(1, GATE_CLOSED, 300);
			robot->servo(0, ARM_HIGHER_POS, 300);
			delay(300);

			while(!robot->button()) {
				robot->send_ready();
				delay(50);
			} 
			while(robot->button());
			std::cout << "Start." << std::endl;
			line.start();
			//line.check_silver();
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
