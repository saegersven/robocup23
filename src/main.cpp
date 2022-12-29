#include <iostream>
#include <chrono>
#include <thread>

#include "line.h"
#include "robot.h"

int main() {
	std::cout << "Program started." << std::endl;

	std::shared_ptr<Robot> robot = std::make_shared<Robot>();
	
	robot->stop();

	Line line(robot);
	line.start();

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
	/*
	// unloading victims test:
	delay(1000);
	robot->m(127, 127, 1000); //is not received by Teensy?! TODO: DEBUG (hardware)
	exit(0);
	
	delay(1000);
	robot->servo(4, GATE_OPEN, 500);
	delay(500);
	for (int i = 0; i < 5; ++i) {
		robot->m(127, 127, 50);
		delay(50);
		robot->m(-127, -127, 50);
		delay(50);
	}
	exit(0);
	*/
	auto last_started = millis(); // time at which robot has been restarted

	// MAIN LOOP
	while(1) {
		if(robot->button(BTN_RESTART) && millis() - last_started > 300) { // if Restart_btn is pressed and main program has been running for at least 300ms:
			while(robot->button(BTN_RESTART));
			
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

		line.line();
	}

	return 0;
}
