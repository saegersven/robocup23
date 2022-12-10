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
		std::cout << robot->read_distance() << std::endl;
	}
	exit(0);*/

	// max power usage test
	/*
	robot->servo(0, 120, 0);
	robot->servo(1, 170, 0);
	robot->servo(2, 85, 0);
	robot->servo(4, 40, 0);
	delay(500)
	robot->servo(0, 50, 0);	
	robot->servo(1, 20, 0);
	robot->servo(2, 0, 0);
	robot->servo(4, 100, 0);
	*/
	
	Line line(robot);
	line.start();

	while(!robot->button(BTN_RESTART)) robot->send_byte(CMD_READY);
	while(robot->button(BTN_RESTART));

	auto last_started = millis(); // time at which robot has been restarted

	// MAIN LOOP
	while(1) {
		if(robot->button(BTN_RESTART) && millis() - last_started > 300) { // if Restart_btn is pressed and main program has been running for at least 300ms:
			while(robot->button(BTN_RESTART));
			
			line.stop();
			robot->stop();

			std::cout << "Stop." << std::endl;
			delay(300);

			while(!robot->button(BTN_RESTART)) robot->send_byte(CMD_READY);
			while(robot->button(BTN_RESTART));
			std::cout << "Start." << std::endl;
			
			last_started = millis();
			line.start();
		}

		line.line();
	}

	return 0;
}
