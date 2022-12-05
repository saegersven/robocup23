#include "silver_ml.h"

void SilverML::start() {
	running = true;
	has_new_frame = false;
	current_prediction = false;

	// Load and initialize model
	model = tflite::FlatBufferModel::BuildFromFile(SILVER_MODEL_PATH);
	tflite::ops::builtin::BuiltinOpResolver resolver;
	tflite::InterpreterBuilder builder(*model, resolver);
	builder(&interpreter);
	interpreter->AllocateTensors();

	input_layer = interpreter->typed_input_tensor<float>(0);
	output_layer = interpreter->typed_output_tensor<float>(0);

	std::thread t([this] { this->internal_loop(); });
	t.detach();
}

void SilverML::stop() {
	running = false;
}

void SilverML::internal_loop() {
	while(running) {
		if(!has_new_frame) continue;

		frame_lock.lock();
		// Image is three bytes (BGR), need to convert to one float (grayscale)
		uint8_t* p;
		for(int i = 0; i < image.rows; ++i) {
			p = image.ptr<uint8_t>(i);
			for(int j = 0; j < image.cols; ++j) {
				// Compute average of all three channels
				float f = 0.0f;
				for(int k = 0; k < image.channels; ++k) {
					f += (float)p[j][k] / 255.0f;
				}
				f /= image.channels;
				// Put into input layer of NN
				input_layer[i * image.cols + j * image.channels + k] = f;
			}
		}
		frame_lock.unlock();

		interpreter->Invoke();

		current_prediction = output_layer[0] > output_layer[1];
		if(current_prediction) {
			std::cout << "Detected silver (" << output_layer[0] << ")" << std::endl;
		}
		has_new_frame = false;
	}
}

void SilverML::set_frame(cv::Mat new_frame) {
	frame_lock.lock();
	current_frame = new_frame.clone();
	frame_lock.unlock();
	has_new_frame = true;
}

bool SilverML::get_current_prediction() {
	return current_prediction;
}