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
	
	robot->stop();
	/* Camera is not being closed after stopping of line. Cant restart cam from rescue thread
	Line line(robot);
	line.start();
	line.stop();
	*/
	Rescue rescue(robot);
	rescue.start();
	while (1);
	// set servos to default position
	delay(50);
	robot->send_byte(CMD_SERVOS_HOME_POS);
	delay(810);
	std::cout << "Init." << std::endl;
	while(!robot->button(BTN_RESTART)) {
		robot->send_byte(CMD_READY);
		delay(10);
		//std::cout << robot->read_distance() << std::endl;
	}
	while(robot->button(BTN_RESTART));
	delay(40);
	
	auto last_started = millis(); // time at which robot has been restarted

	/*
	// TODO: Testing for errors
	// MAIN LOOP
	while(1) {
		if(robot->button(BTN_RESTART) && millis() - last_started > 300) { // if Restart_btn is pressed and main program has been running for at least 300ms:
			while(robot->button(BTN_RESTART));
			switch(state) {
				case State::line:
					line.stop();
					break;
				case State::rescue:
					rescue.stop();
					state = State::line;
					break;
			}
			line.stop();
			robot->stop();
			
			std::cout << "Stop." << std::endl;
			robot->send_byte(CMD_SERVOS_HOME_POS);
			delay(300);

			while(!robot->button(BTN_RESTART)) {
				robot->send_byte(CMD_READY);
				delay(1);
			} 
			while(robot->button(BTN_RESTART));
			std::cout << "Start." << std::endl;
			
			last_started = millis();
			line.start();
		}

		switch(state) {
			case State::line: {
				line.line();
				break;
			}
			case State::rescue: {
				// Monitor rescue thread
				if(rescue.finished) {
					std::cout << "Starting line" << std::endl;
					//rescue.stop();
					line.start();
					state = State::line;
				}
				break;
			}
		}
	}
	*/

	return 0;
}
