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

#include <pigpio.h>

#include "defines.h"
#include "utils.h"

Robot::Robot() : blocked(false), has_frame(false) {
	init_serial();

	if(gpioInitialise() < 0) {
		std::cout << "GPIO initialization failed" << std::endl;
	}

	gpioSetMode(PIN_BTN, PI_INPUT);
	gpioSetPullUpDown(PIN_BTN, PI_PUD_DOWN);

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
	cap.set(cv::CAP_PROP_FRAME_WIDTH, capture_width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, capture_height);
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
	//auto time = std::chrono::high_resolution_clock::now();
	while(camera_running) {
		//auto now_t = std::chrono::high_resolution_clock::now();
		//long long dt = std::chrono::duration_cast<std::chrono::microseconds>(now_t - time).count();
		//std::cout << 1000000.0f / (float)dt << std::endl;
		//time = now_t;
		cap.grab();
		frame_lock.lock();
		cap.retrieve(curr_frame);
		cv::resize(curr_frame, curr_frame, cv::Size(frame_width, frame_height));
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
    /*bool reading = gpioRead(PIN_BTN); // Read the state of the button

    if (reading != lastButtonState) {
        lastDebounceTime = millis_(); // Reset the debounce timer
    }

    if ((millis_() - lastDebounceTime) > 50) { // change 50ms if needed
        if (reading != buttonState) {
            buttonState = reading;
        }
    }

    lastButtonState = reading;*/
    for(int i = 0; i < 8; i++) {
    	if(!gpioRead(PIN_BTN)) {
    		std::cout << i << std::endl;
    		return false;
    	}
    }

    return true;
}

void Robot::m(int8_t left, int8_t right, int32_t duration) {
	std::cout << "left: " << static_cast<int>(left) << "  right: " << static_cast<int>(right) << "  dur: " << duration << std::endl;
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

	/*
	if(std::abs(RTOD(angle)) < 30.0f) {
		float sign = angle / std::abs(angle);
    	m(sign * 50, -sign * 50, (int)(RTOD(std::abs(angle)) * MS_PER_DEG));
		return;
	}
	*/

	int16_t angle_mrad = angle * 1000;
	char msg[3] = {CMD_TURN, 0, 0};
	memcpy(&msg[1], &angle_mrad, 2);

	write(serial_fd, msg, 3);

	tcflush(serial_fd, TCIFLUSH);

	uint64_t start_t = millis_();

	// wait until turn is finished. Currently not working @saegersven
	while (1) {
		char msg[1] = {0};
		read(serial_fd, &msg, 1);
		std::cout << msg[0] << std::endl;
		if (CMD_TURN_DONE == msg[0]) {
			std::cout << "Received done, msg is: " << (int)msg[0] << std::endl;
			break;
		}
		if(start_t - millis_() > 6000) break;
	}

	tcflush(serial_fd, TCIFLUSH);
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

// Returns distance in mm. id 0 is front sensor, id 1 is side sensor
int Robot::distance(uint8_t sensor_id) {
    uint8_t msg[2] = {CMD_SENSOR, sensor_id};
    write(serial_fd, msg, 2);

	tcflush(serial_fd, TCIFLUSH);

    if (read(serial_fd, msg, 2) != 2) return 0xFFFF;

    uint16_t dist;
    //dist = (msg[0] << 8) | msg[1];  // Convert bytes to little-endian. Sometimes necessary
    dist = (msg[0]) | (msg[1] << 8);

    return dist;
}

// avg distance in mm. Highest and lowest remove_percentage is removed
int Robot::distance_avg(uint8_t sensor_id, uint8_t num_measurements, float remove_percentage) {
	float arr[num_measurements];

	// take measurements
	for(int i = 0; i < sizeof(arr) / sizeof(arr[0]); i++) {
		float dist = distance(sensor_id);
		arr[i] = dist;
		delay(35);
	}

	// calculate avg after removing n percent of exteme measurements
	int arr_len = sizeof(arr) / sizeof(arr[0]);

	std::sort(arr, arr + arr_len);
	int kthPercent = (arr_len * remove_percentage);

	int num_considered_measurements = (arr_len - 2 * kthPercent);

	float sum = 0;

	for(int i = 0; i < arr_len; i++) {
		if (i >= kthPercent && i < (arr_len - kthPercent)) {
			if(arr[i] == 0xFFFF) {
				// False measurement, dont include
				--num_considered_measurements;
			} else {
				sum += arr[i];
			}
		}
	}

	float avg = sum / num_considered_measurements;

	return (int)avg;
}

// returns 1 for ramp up, 2 for ramp down, 0 for no ramp
int Robot::ramp() {
	uint8_t msg[2] = {CMD_SENSOR, 3}; // 3 means gyro
    write(serial_fd, msg, 2);

	tcflush(serial_fd, TCIFLUSH);

    int ramp_state;

    if (read(serial_fd, msg, 2) != 2) ramp_state = -1;

    //dist = (msg[0] << 8) | msg[1];  // Convert bytes to little-endian. Sometimes necessary
    ramp_state = (msg[0]) | (msg[1] << 8);
    return ramp_state;
}