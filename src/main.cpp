#include <iostream>
#include <chrono>
#include <thread>

#include "line.h"
#include "robot.h"

int main() {
	std::cout << "Init" << std::endl;

	std::shared_ptr<Robot> robot = std::make_shared<Robot>();
	
	/*while(true) {
		//robot->m(50, 50, 500);
		//robot->m(-50, -50, 500);
		std::cout << robot->read_distance() << std::endl;
	}*/

	robot->stop();

	Line line(robot);
	line.start();

	robot->send_byte(CMD_READY);

	while(!robot->button(BTN_RESTART)) {
		delay(5);
		//std::cout << robot->read_heading() << std::endl;
	}
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

			while(!robot->button(BTN_RESTART)) {
				robot->send_byte(CMD_READY);
				delay(5);
				robot->send_byte(CMD_STOP);
				delay(5);
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
