#include "robot.h"

extern "C" {
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
}

#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>

#include "defines.h"
#include "utils.h"

Robot::Robot() : blocked(false)
#ifdef ENABLE_VL53L0X
	, vl53l0x(VL53L0X_FORWARD_XSHUT)
#endif // ENABLE_VL53L0X
{
	wiringPiSetupGpio();

	pinMode(BTN_RESTART, INPUT);
	pinMode(STOP_PIN, OUTPUT);
	digitalWrite(STOP_PIN, LOW);
	pinMode(HCSR04_TRIGGER, OUTPUT);
	pinMode(HCSR04_ECHO, INPUT);
	digitalWrite(HCSR04_TRIGGER, LOW);

	delay(2000); // Wait for Nano to boot up
}

void Robot::init_serial() {
	termios options;
	speed_t baud_rate = B115200;

	int status;

	if((serial_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) < 0) {
		std::cout << "Could not open Serial." << std::endl;
	}

	fcntl(serial_fd, F_SETFL, O_RDWR);

	tcgetattr(serial_fd, &options);

	cfmakeraw(&options);
	cfsetispeed(&options, baud_rate);
	cfsetospeed(&options, baud_rate);

	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
	options.c_oflag &= ~OPOST;

	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 10; // One second (10 deciseconds)

	tcsetattr(serial_fd, TCSANOW, &options);
	ioctl(serial_fd, TIOCMGET, &status);

	status |= TIOCM_DTR;
	status |= TIOCM_RTS;

	ioctl(serial_fd, TIOCMSET, &status);
}

void Robot::start_camera(int width, int height, int framerate) {
	camera.options->video_width = width;
	camera.options->video_height = height;
	camera.options->framerate = framerate;
	camera.options->verbose = true;

	camera.startVideo();
	camera_running = true;
}

void Robot::stop_camera() {
	camera.stopVideo();
	camera_running = false;
}

cv::Mat Robot::grab_frame() {
	if(!camera_running) std::cout << "Grabbing frame with closed camera" << std::endl;

	cv::Mat frame;
	if(!camera.getVideoFrame(frame, 1000)) {
		std::cout << "Camera timed out" << std::endl;
	}

	return frame;
}

void Robot::set_blocked(bool blocked) {
	this->blocked = blocked;
}

bool Robot::button() {
	char msg[2] = {0x05, 3};

	write(serial_fd, msg, 2);
	if(read(serial_fd, msg, 2) != 2) return false;
	return msg[0] || msg[1];
}

void Robot::m(int8_t left, int8_t right, int32_t duration) {
	if(blocked) return;
	if(duration < 0) {
		duration = -duration;
		left = -left;
		right = -right;
	}
	if(duration > 65535) duration = 65535;
	uint16_t dur = duration;
	char msg[5] = {0x01, 60, 60, 0, 0};
	memcpy(&msg[3], &dur, 2);

	std::cout << (int)write(serial_fd, msg, 5) << std::endl;
	delay(duration + 10);
}

void Robot::stop() {
	if(blocked) return;

	char msg = 0;
	write(serial_fd, &msg, 1);
}

// turns given angle in radians
void Robot::turn(float angle) {
	if(blocked) return;

	int16_t angle_mrad = angle * 1000;
	char msg[3] = {0x07, 0, 0};
	memcpy(&msg[1], &angle_mrad, 2);

	std::cout << (int)write(serial_fd, msg, 3) << std::endl;

	delay((int)(RTOD(angle) * MS_PER_DEGREE + 10));
}

void Robot::send_byte(char b) {
	if(blocked) return;
	//i2c_write_byte_single(TEENSY_I2C_ADDR, b);
	uint8_t msg[1] = {b};
	//spi_write(msg, 1);
}

void Robot::servo(uint8_t servo_id, uint8_t angle, uint16_t delay_ms) {
	if(blocked) return;

	uint8_t msg[3] = {CMD_SERVO, servo_id, angle};
	//i2c_write(TEENSY_I2C_ADDR, CMD_SERVO, msg, 2);
	//spi_write(msg, 3);
	if(delay_ms != 0) delay(delay_ms);
}

float Robot::read_heading() {
	int16_t data = 0.0;

	/*if(bno055_read_euler_h(&data) != 0) {
		std::cerr << "Error reading euler data" << std::endl;
		exit(ERRCODE_BNO055);
	}*/
	return DTOR((float)data / 16.0f);
}

float Robot::read_pitch() {
	int16_t data = 0.0;

	/*if(bno055_read_euler_r(&data) != 0) {
		std::cerr << "Error reading euler data" << std::endl;
		exit(ERRCODE_BNO055);
	}*/
	return DTOR((float)data / 16.0f);
}

// returns front distance in cm
int Robot::distance() {
	return 0;
}

int Robot::distance_avg(uint8_t num_measurements, float remove_percentage) {
	float arr[num_measurements];

	// take measurements
	for(int i = 0; i < sizeof(arr) / sizeof(arr[0]); i++) {
		float dist = distance();
		arr[i] = dist;
		delay(15);
	}

	// calculate avg after removing n percent of exteme measurements
	int arr_len = sizeof(arr) / sizeof(arr[0]);

	std::sort(arr, arr + arr_len);
	int kthPercent = (arr_len * remove_percentage);
	float sum = 0;

	for(int i = 0; i < arr_len; i++)
		if (i >= kthPercent && i < (arr_len - kthPercent))
			sum += arr[i];

	float avg = sum / (arr_len - 2 * kthPercent);

	return (int)avg;
}
