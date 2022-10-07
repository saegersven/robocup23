#pragma once

class Protocol {
private:
	const uint8_t PACKET_ID_CONFIRM 		= 0x01;
	const uint8_t PACKET_ID_SET_SPEED 		= 0x02;
	const uint8_t PACKET_ID_TURN			= 0x03;
	const uint8_t PACKET_ID_TURN_TO			= 0x04;
	const uint8_t PACKET_ID_STOP			= 0x05;
	const uint8_t PACKET_ID_SERVO			= 0x06;
	const uint8_t PACKET_ID_GET_DISTANCE	= 0x07;
	const uint8_t PACKET_ID_GET_ORIENTATION = 0x08;

	const uint8_t[8] packet_lengths = {
		0, // No packet
		3, // Confirm
		4, // Set Speed
		6, // Turn
		6, // Turn to
		3, // Stop
		4, // Servo
		5, // Get distance
		7  // Get orientation
	};

	const uint8_t CONFIRM_POSITIVE = 0xFF;

	int fd;
public:
	bool transfer(uint8_t* buf, uint32_t len);

	bool confirm();
	void set_speed(int8_t left, int8_t right);
	void turn(float angle);
	void turn_to(float heading);
	void stop(uint8_t brake);
	void servo(uint8_t servo_id, uint8_t angle);
	uint16_t get_distance(uint8_t sensor_id);
	float get_heading(uint8_t axis_number);
};