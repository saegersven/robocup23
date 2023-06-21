#include "corner_ml.h"

CornerML::CornerML() {}

void CornerML::init() {
    model = tflite::FlatBufferModel::BuildFromFile(CORNER_MODEL_PATH);
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(*model, resolver);
    if(builder(&interpreter) != kTfLiteOk) {
        std::cout << "Failed building interpreter (Corner)" << std::endl;
    }
	
	if(interpreter->AllocateTensors() != kTfLiteOk) {
		std::cerr << "Failed allocating tensors (Corner)" << std::endl;
	}

    input_layer = interpreter->typed_input_tensor<float>(0);
}

cv::Mat CornerML::invoke(cv::Mat image) {
    cv::resize(image, image, cv::Size(CORNER_IN_WIDTH, CORNER_IN_HEIGHT));

    for(int i = 0; i < image.rows; ++i) {
        cv::Vec3b* p = image.ptr<cv::Vec3b>(i);
        for(int j = 0; j < image.cols; ++j) {
            for(int k = 0; k < CORNER_IN_CHANNELS; k++) {
                float d = (float)p[j][k];
                input_layer[i * image.cols * CORNER_IN_CHANNELS + j * CORNER_IN_CHANNELS + k] = d;
            }
        }
    }

    interpreter->Invoke();
    output_layer = interpreter->typed_output_tensor<float>(0);

    cv::Mat out(CORNER_OUT_HEIGHT, CORNER_OUT_WIDTH, CV_32FC2);

    for(int i = 0; i < CORNER_OUT_HEIGHT; ++i) {
        float* p = out.ptr<float>(i);
        for(int j = 0; j < CORNER_OUT_WIDTH; ++j) {
            for(int k = 0; k < CORNER_OUT_CHANNELS; ++k) {
                float val = output_layer[i * CORNER_OUT_WIDTH * CORNER_OUT_CHANNELS + j * CORNER_OUT_CHANNELS + k];
                if(val > 1.0f) val = 1.0f;
                else if(val < 0.0f) val = 0.0f;
                if(i == 0) val = 0.0f;
                p[j * CORNER_OUT_CHANNELS + k] = val;
            }
        }
    }
    return out;
}

bool CornerML::extract_corner(cv::Mat probability_map, float& x, float& y, bool red) {
    const float THRESHOLD_RED = 0.4f;
    const float THRESHOLD_GREEN = 0.4f;

    cv::Mat blurred;
    cv::GaussianBlur(probability_map, blurred, cv::Size(3, 3), 0);

    cv::Mat thresh;
    cv::Scalar threshold = cv::Scalar(THRESHOLD_RED, 0.0f);

    if(!red) {
        threshold = cv::Scalar(0.0f, THRESHOLD_GREEN);
    }

    cv::inRange(blurred, threshold, cv::Scalar(1.0f, 1.0f), thresh_red);
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if(contours.size() == 0) return false;

    // Find largest contour
    float largest_contour_area = 10000.0f;
    int largest_contour_idx = 0;

    for(int i = 0; i < contours_red.size(); ++i) {
        float area = cv::contourArea(contours_red[i]);
        if(area > largest_contour_area) {
            largest_contour_area = area;
            largest_contour_idx = i;
        }
    }

    if(largest_contour_area < 8.0f) return false; // TODO: Minimum area value may not be perfec

    const float CHUNK_WIDTH = (float)CORNER_IN_WIDTH / CORNER_OUT_WIDTH;
    const float CHUNK_HEIGHT = (float)CORNER_IN_HEIGHT / CORNER_OUT_HEIGHT;

    cv::Rect rect = cv::boundingRect(contours[largest_contour_idx]);

    x = (rect.x + rect.width / 2.0f) * CHUNK_WIDTH;
    y = (rect.y + rect.height / 2.0f) * CHUNK_HEIGHT;

    return true;
}