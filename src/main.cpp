#include <iostream>
#include <chrono>
#include <thread>

#include "line.h"
#include "robot.h"

int main() {
	std::cout << "Init" << std::endl;

	std::shared_ptr<Robot> robot = std::make_shared<Robot>();

	robot->stop();

	/*while(1) {
		std::cout << robot->read_heading() << std::endl;
	}
	exit(0);*/

	Line line(robot);
	line.start();

	while(!robot->button(BTN_RESTART));
	while(robot->button(BTN_RESTART));

	robot->send_byte(CMD_START);
	delay(200);

	// MAIN LOOP
	while(1) {
		if(robot->button(BTN_RESTART)) {
			while(robot->button(BTN_RESTART));
			line.stop();
			robot->stop();
			std::cout << "Stop." << std::endl;
			delay(200);

			while(!robot->button(BTN_RESTART));
			while(robot->button(BTN_RESTART));
			robot->send_byte(CMD_START);
			delay(200);
			std::cout << "Start." << std::endl;
			
			line.start();
		}

		line.line();
	}

	return 0;
}
