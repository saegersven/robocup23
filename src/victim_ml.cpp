#include "victim_ml.h"

VictimML::VictimML() {}

void VictimML::init() {
    model = tflite::FlatBufferModel::BuildFromFile(VICTIM_MODEL_PATH);
    tflite::ops::builtin::BuiltinOpResolver resolver;
    tflite::InterpreterBuilder builder(*model, resolver);
    if(builder(&interpreter) != kTfLiteOk) {
        std::cout << "Failed building interpreter (Victims)" << std::endl;
    }
	
	if(interpreter->AllocateTensors() != kTfLiteOk) {
		std::cerr << "Failed allocating tensors (Victims)" << std::endl;
	}

    input_layer = interpreter->typed_input_tensor<float>(0);
}

cv::Mat VictimML::invoke(cv::Mat image) {
    cv::resize(image, image, cv::Size(IN_WIDTH, IN_HEIGHT));

    for(int i = 0; i < image.rows; ++i) {
        cv::Vec3b* p = image.ptr<cv::Vec3b>(i);
        for(int j = 0; j < image.cols; ++j) {
            float d = ((float)p[j][0] + p[j][1] + p[j][2]) / 3.0f;
            input_layer[i * image.cols + j] = d;
        }
    }

    interpreter->Invoke();
    output_layer = interpreter->typed_output_tensor<float>(0);

    cv::Mat out(OUT_HEIGHT, OUT_WIDTH, CV_32FC2);

    for(int i = 0; i < OUT_HEIGHT; ++i) {
        float* p = out.ptr<float>(i);
        for(int j = 0; j < OUT_WIDTH; ++j) {
            for(int k = 0; k < OUT_CHANNELS; ++k) {
                float val = output_layer[i * OUT_WIDTH * OUT_CHANNELS + j * OUT_CHANNELS + k];
                if(val > 1.0f) val = 1.0f;
                else if(val < 0.0f) val = 0.0f;
                p[j * OUT_CHANNELS + k] = val;
            }
        }
    }
    return out;
}

std::vector<Victim> VictimML::extract_victims(cv::Mat probability_map) {
    const float THRESHOLD_DEAD = 0.3f;
    const float THRESHOLD_ALIVE = 0.4f;

    cv::Mat blurred;
    cv::GaussianBlur(probability_map, blurred, cv::Size(3, 3), 0);

    cv::Mat thresh1;
    cv::Mat thresh2;
    cv::inRange(blurred, cv::Scalar(THRESHOLD_DEAD, 0.0f), cv::Scalar(1.0f, 1.0f), thresh1);
    cv::inRange(blurred, cv::Scalar(0.0f, THRESHOLD_ALIVE), cv::Scalar(1.0f, 1.0f), thresh2);
    cv::Mat thresh;
    cv::bitwise_or(thresh1, thresh2, thresh);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Victim> victims;

    for(int i = 0; i < contours.size(); ++i) {
        cv::Mat mask(blurred.rows, blurred.cols, CV_8UC1, cv::Scalar(0));
        cv::drawContours(mask, contours, i, cv::Scalar(255), -1);
        cv::Scalar mean = cv::mean(blurred, mask);

        bool is_dead = mean[0] > mean[1];

        const float CHUNK_WIDTH = (float)IN_WIDTH / OUT_WIDTH;
        const float CHUNK_HEIGHT = (float)IN_HEIGHT / OUT_HEIGHT;

        cv::Rect rect = cv::boundingRect(contours[i]);
        float min_area = (180.0f / 60.0f) * (rect.y + rect.height / 2.0f) + 20.0f;

        if((float)rect.height * CHUNK_HEIGHT * (float)rect.width * CHUNK_WIDTH < min_area) {
            continue;
        }

        victims.push_back({
            is_dead,
            ((float)rect.x + rect.width / 2.0f) * CHUNK_WIDTH, 
            ((float)rect.y + rect.height / 2.0f) * CHUNK_HEIGHT
        });
    }
    return victims;
}