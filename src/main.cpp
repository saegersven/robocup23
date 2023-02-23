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
	
	// set servos to default position
	delay(50);
	robot->send_byte(CMD_SERVOS_HOME_POS);
	delay(20);
	robot->send_byte(CMD_SERVOS_HOME_POS);
	delay(20);
	robot->send_byte(CMD_SERVOS_HOME_POS);
	delay(810);

	std::cout << "Init." << std::endl;
	while(!robot->button(BTN_RESTART)) {
		robot->send_byte(CMD_READY);
		delay(1);	
	}
	while(robot->button(BTN_RESTART));
	line.start();
	delay(40);
	
	auto last_started = millis(); // time at which robot has been restarted
	
	// MAIN LOOP
	while(1) {
		if(robot->button(BTN_RESTART) && millis() - last_started > 300) { // if Restart_btn is pressed and main program has been running for at least 300ms:
			while(robot->button(BTN_RESTART));
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
			robot->send_byte(CMD_SERVOS_HOME_POS);
			delay(300);

			while(!robot->button(BTN_RESTART)) {
				robot->send_byte(CMD_READY);
				delay(1);
			} 
			while(robot->button(BTN_RESTART));
			std::cout << "Start." << std::endl;
			line.start();
			line.check_silver();
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
