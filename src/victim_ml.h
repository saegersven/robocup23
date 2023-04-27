#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>

#include <opencv2/opencv.hpp>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"

#include "utils.h"

#define IN_WIDTH 160
#define IN_HEIGHT 120

#define OUT_WIDTH 32
#define OUT_HEIGHT 20
#define OUT_CHANNELS 2

#define TOP_Y 3

#define VICTIM_MODEL_PATH "/home/pi/robocup23/runtime_data/victims.tflite"

struct Victim {
    bool dead;
    float x;
    float y;
};

class VictimML {
private:
    std::unique_ptr<tflite::FlatBufferModel> model;
    std::unique_ptr<tflite::Interpreter> interpreter;

    float* input_layer;
    float* output_layer;

public:
    VictimML();
    void init();

    /**
     * Invokes the neural net on the input image.
     * Returns a probability map of alive or dead victims.
     */
    cv::Mat invoke(cv::Mat image);

    /**
     * Extracts victims out of a probability map generated by the
     * neural net using cv::findContours and thresholding operations.
     */
    std::vector<Victim> extract_victims(cv::Mat probability_map, bool ignore_top = false);
};