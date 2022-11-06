#include <iostream>
#include <chrono>
#include <thread>

#include "line.h"
#include "robot.h"

int main() {
	std::cout << "Init" << std::endl;

	std::shared_ptr<Robot> robot = std::make_shared<Robot>();

	robot->stop();
	robot->send_byte(0x08);

	Line line(robot);
	line.start();

	while(!robot->button(BTN_RESTART));
	while(robot->button(BTN_RESTART));
	
	robot->send_byte(0x09);
	delay(200);

	// MAIN LOOP
	while(1) {
		if(robot->button(BTN_RESTART)) {
			while(robot->button(BTN_RESTART));
			line.stop();
			robot->stop();
			std::cout << "Stop." << std::endl;
			delay(50);

			while(!robot->button(BTN_RESTART));
			while(robot->button(BTN_RESTART));
			delay(100);
			std::cout << "Start." << std::endl;
			
			line.start();
		}

		line.line();
	}

	return 0;
}