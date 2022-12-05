#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

#include <opencv2/opencv.hpp>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"

#define SILVER_MODEL_PATH "/home/pi/robocup23/runtime_data/silver.tflite"

class SilverML {
private:
	std::unique_ptr<tflite::FlatBufferModel> model;
	std::unique_ptr<tflite::Interpreter> interpreter;

	float* input_layer;
	float* output_layer;

	std::atomic<bool> running;
	std::atomic<bool> current_prediction;
	std::atomic<bool> has_new_frame;
	std::mutex frame_lock;
	cv::Mat current_frame;

public:
	SilverML();

	/**
	 * Load the model and start the detection thread.
	 */
	void start();

	/**
	 * Notifies the internal loop to stop.
	 */
	void stop();

	/**
	 * Sets current_frame to a new image. Once the internal loop is done processing,
	 * it will copy the current frame into the input layer of the neural network.
	 */
	void set_frame(cv::Mat new_frame);

	/**
	 * Get the last prediction of the neural network. Returns true if the NN detected silver.
	 */
	bool get_current_prediction();

	/**
	 * Performs copying operations and invokes the neural network.
	 */
	void internal_loop();
}