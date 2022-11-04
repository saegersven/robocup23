#include <iostream>
#include <chrono>

#include "line.h"
#include "robot.h"

int main() {
	std::cout << "Init" << std::endl;

	std::shared_ptr<Robot> robot = std::make_shared<Robot>();

	robot->stop();

	Line line(robot);
	line.start();

	while(!robot->button(BTN_RESTART));
	while(robot->button(BTN_RESTART));
	delay(100);

	// MAIN LOOP
	while(1) {
		if(robot->button(BTN_RESTART)) {
			while(robot->button(BTN_RESTART));
			line.stop();
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