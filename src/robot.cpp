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
#include <fstream>
#include <sstream>
#include <string>

#include <wiringPi.h>
#include <softPwm.h>

#include "defines.h"
#include "utils.h"

Robot::Robot() : blocked(false), has_frame(false) {
	init_serial();

	delay(2000); // Wait for Nano to boot up (can probably be shorter)
}

void Robot::init_serial() {
	termios options;
	speed_t baud_rate = B115200;

	std::string dev_filename = "/dev/ttyUSB0";
	std::string dev_name = "ttyUSB0";

	int status;

	while((serial_fd = open(dev_filename.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) < 0) {
		std::cout << "Could not open Serial." << std::endl;
		std::cout << "Trying USB1..." << std::endl;
		dev_filename = "/dev/ttyUSB1";
		dev_name = "ttyUSB1";
	}

	std::cout << "Opened serial:" << dev_name << std::endl;

	std::stringstream ss;
	ss << "/sys/bus/usb-serial/drivers/ftdi_sio/" << dev_name << "/latency_timer";
	std::string latency_file_name = ss.str();
	std::cout << "Set latency to 2ms in " << latency_file_name << std::endl;

	std::ofstream out(latency_file_name);
	out << "2" << std::endl;
	out.close();


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
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;

	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 5; // 0.5 seconds (5 deciseconds)

	tcsetattr(serial_fd, TCSANOW, &options);
	ioctl(serial_fd, TIOCMGET, &status);

	status |= TIOCM_DTR;
	status |= TIOCM_RTS;

	ioctl(serial_fd, TIOCMSET, &status);

	delay(10);
}

void Robot::start_camera() {
	cap.open(0, cv::CAP_V4L2);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 192);
	cap.set(cv::CAP_PROP_FORMAT, CV_8UC3);
	cap.set(cv::CAP_PROP_FPS, 120);

	if(!cap.isOpened()) {
		std::cout << "Could not open camera." << std::endl;
		exit(ERRCODE_CAM_SETUP);
	}

	camera_running = true;

	std::thread t([this] {this->camera_thread();});
	t.detach();
}
// constantly grabs frame from camera
void Robot::camera_thread() {
	while(camera_running) {
		cap.grab();
		frame_lock.lock();
		cap.retrieve(curr_frame);
		cv::resize(curr_frame, curr_frame, cv::Size(80, 48));
		frame_lock.unlock();
		has_frame = true;
	}
}

void Robot::stop_camera() {
	cap.release();
	camera_running = false;
	has_frame = false;
}

cv::Mat Robot::grab_frame() {
	if(!camera_running) std::cout << "Grabbing frame with closed camera" << std::endl;

	while(!has_frame);
	has_frame = false;

	cv::Mat frame;
	
	frame_lock.lock();
	cv::flip(curr_frame, frame, -1);
	frame_lock.unlock();

	return frame;
}

void Robot::set_cam_angle(uint8_t angle) {
	int pw = (int)(((float)angle / 180.0f + 1.0f) / 0.1f * 1024.0f);
	std::cout << "Pulse width: " << pw << std::endl;
	if(pw < 0) pw = 0;
	if(pw > 1024) pw = 1024;
	//pwmWrite(SERVO_CAM_PIN, pw);
}

void Robot::set_blocked(bool blocked) {
	this->blocked = blocked;
}

int Robot::serial_available() {
	int res;
	if(ioctl(serial_fd, FIONREAD, &res) == -1) return -1;

	return res;
}

bool Robot::button() {
	char msg[2] = {CMD_SENSOR, 2};

	write(serial_fd, msg, 2);

	if(read(serial_fd, &msg, 2) != 2) return false;
	return msg[0] || msg[1];
}

void Robot::m(int8_t left, int8_t right, int32_t duration) {
	//std::cout << "left: " << static_cast<int>(left) << "  right: " << static_cast<int>(right) << "  dur: " << duration << std::endl;
	if(blocked) return;
	if(duration < 0) {
		duration = -duration;
		left = -left;
		right = -right;
	}
	if(duration > 65535) duration = 65535;
	uint16_t dur = duration;
	char msg[5] = {CMD_MOTOR, *((char*)&left), *((char*)&right), 0, 0};
	memcpy(&msg[3], &dur, 2);

	write(serial_fd, msg, 5);
	delay(duration);
}

void Robot::stop() {
	if(blocked) return;

	char msg = CMD_STOP;
	write(serial_fd, &msg, 1);
}

void Robot::send_ready() {
	char msg = CMD_READY;
	write(serial_fd, &msg, 1);
}

// turns given angle in radians
void Robot::turn(float angle) {
	if(blocked) return;

	int16_t angle_mrad = angle * 1000;
	char msg[3] = {CMD_TURN, 0, 0};
	memcpy(&msg[1], &angle_mrad, 2);

	write(serial_fd, msg, 3);


	// wait until turn is finished. Currently not working @saegersven
	while (1) {
		std::cout << "Awaiting done turning" << std::endl;
		char msg[1] = {0};
		read(serial_fd, &msg, 1);
		std::cout << msg[0] << std::endl;
		if (10 == msg[0]) {
			std::cout << "Received done, msg is: " << msg[0] << std::endl;
			break;
		}
	}
	std::cout << "Received done turning" << std::endl;

	delay(2000);
}

void Robot::attach_detach_servo(uint8_t servo_id) {
	if(blocked) return;

	uint8_t msg[2] = {CMD_SERVO_ATTACH_DETACH, servo_id};

	write(serial_fd, msg, 2);
}

void Robot::servo(uint8_t servo_id, uint8_t angle, uint16_t delay_ms) {
	if(blocked) return;

	uint8_t msg[3] = {CMD_SERVO_WRITE, servo_id, angle};
	write(serial_fd, msg, 3);
	if(delay_ms != 0) delay(delay_ms);
}

void Robot::gripper(int8_t gripper_direction, uint16_t delay_ms) {
	if(blocked) return;

	uint8_t msg[2] = {CMD_GRIPPER, *((uint8_t*)&gripper_direction)};

	write(serial_fd, msg, 2);
	if(delay_ms != 0) {
		delay(delay_ms);
		msg[1] = GRIPPER_OFF;
		write(serial_fd, msg, 2);
	}
}

float Robot::read_heading() {
	int16_t data = 0.0;
	// TODO: serial to NANO
	return DTOR((float)data / 16.0f);
}

float Robot::read_pitch() {
	int16_t data = 0.0;
	// TODO: serial to NANO
	return DTOR((float)data / 16.0f);
}

// returns front distance in cm
int Robot::distance(uint8_t sensor_id) {
	uint8_t msg[2] = {CMD_SENSOR, sensor_id};
	write(serial_fd, msg, 2);

	if(read(serial_fd, msg, 2) != 2) return 0xFFFF;

	uint16_t dist;
	memcpy(&dist, msg, 2);

	return dist;
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