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

	while (true) {
		std::cout << robot->distance(0) << std::endl;
		delay(100);
	}

	State state = State::rescue;
	Rescue rescue(robot);
	rescue.start();



	while (1) {
		
	}





	//Rescue rescue(robot);

	// SET SERVOS TO DEFAULT POS
	//robot->attach_detach_servo(SERVO_CAM); // attach cam servo, necessary???
	//robot->servo(2, CAM_LOWER_POS, 300); // don't detach so cam stays in position
	//robot->servo(1, GATE_CLOSED, 300);
	//robot->servo(0, ARM_HIGHER_POS, 300);
	/*
	robot->toggle_led();
	while(!robot->button()) {
		delay(10);
	}
	while(robot->button());
	line.start();
	robot->toggle_led();

	auto last_started = millis_(); // time at which robot has been restarted
	
	// MAIN LOOP
	while(1) {
		uint64_t duration = millis_();
		while(robot->button() && robot->button() && robot->button()) delay(10);
		duration = millis_() - duration;
		if(duration > 20 && millis_() - last_started > 300) { // if Restart_btn is pressed and main program has been running for at least 300ms:
			while(robot->button() && robot->button() && robot->button());
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
			// SET SERVOS TO DEFAULT POS
			robot->attach_detach_servo(SERVO_CAM); // attach cam servo
			robot->servo(2, CAM_LOWER_POS, 300); // don't detach so cam stays in position
			robot->servo(1, GATE_CLOSED, 300);
			robot->servo(0, ARM_HIGHER_POS, 300);
			delay(300);
			robot->toggle_led();

			while(!(robot->button() && robot->button() && robot->button())) {
				delay(10);
			} 
			while(robot->button() && robot->button() && robot->button());
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
	*/
}
