/* Copyright 2022 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
#include <cstring>
#include <string>

#include "pybind11/pybind11.h"
#include "pybind11/pytypes.h"
#include "tensorflow/compiler/mlir/quantization/tensorflow/python/quantize_model_wrapper.h"
#include "tensorflow/python/lib/core/pybind11_lib.h"

PYBIND11_MODULE(pywrap_quantize_model, m) {
  m.def(
      "clear_calibrator",
      [] {
        tensorflow::quantization::ClearCollectedInformationFromCalibrator();
      },
      R"pbdoc(
      Clears the collected metrics from the calibrator.
    )pbdoc");
  m.def(
      "clear_data_from_calibrator",
      [](const absl::string_view id) {
        tensorflow::quantization::ClearDataFromCalibrator(id);
      },
      R"pbdoc(
      Clears the collected data of the given id from calibrator.
    )pbdoc");
  m.def(
      "get_max_from_calibrator",
      [](const absl::string_view id) {
        return tensorflow::quantization::GetMaxFromCalibrator(id);
      },
      R"pbdoc(
      Return the tuple with the min value of the given id.
    )pbdoc");
  m.def(
      "get_min_from_calibrator",
      [](const absl::string_view id) {
        return tensorflow::quantization::GetMinFromCalibrator(id);
      },
      R"pbdoc(
      Return the tuple with the min value of the given id.
    )pbdoc");
  m.def(
      "quantize_qat_model",
      [](const absl::string_view saved_model_path,
         const absl::string_view exported_names_str,
         const absl::string_view tags,
         const absl::string_view quant_opts_serialized) {
        const std::string exported_model_serialized =
            tensorflow::quantization::QuantizeQatModel(saved_model_path,
                                                       exported_names_str, tags,
                                                       quant_opts_serialized);

        return py::bytes(exported_model_serialized);
      },
      R"pbdoc(
      Returns serialized ExportedModel that contains the quantized model's
      GraphDef and metadata.
    )pbdoc");
  m.def(
      "quantize_ptq_dynamic_range",
      [](const absl::string_view saved_model_path,
         const absl::string_view exported_names_str,
         const absl::string_view tags,
         const absl::string_view quant_opts_serialized) {
        const std::string exported_model_serialized =
            tensorflow::quantization::QuantizePtqDynamicRange(
                saved_model_path, exported_names_str, tags,
                quant_opts_serialized);

        return py::bytes(exported_model_serialized);
      },
      R"pbdoc(
      Returns serialized ExportedModel that contains the quantized model's
      GraphDef and metadata.
    )pbdoc");
  m.def(
      "quantize_ptq_model_pre_calibration",
      [](const absl::string_view saved_model_path,
         const absl::string_view exported_names_str,
         const absl::string_view tags,
         const absl::string_view quant_opts_serialized) {
        const std::string exported_model_serialized =
            tensorflow::quantization::QuantizePtqModelPreCalibration(
                saved_model_path, exported_names_str, tags,
                quant_opts_serialized);

        return py::bytes(exported_model_serialized);
      },
      R"pbdoc(
      Returns serialized ExportedModel that contains the model's GraphDef and
      metadata. The GraphDef contains extra ops required for calibration.
    )pbdoc");
  m.def(
      "quantize_ptq_model_post_calibration",
      [](const absl::string_view saved_model_path,
         const absl::string_view exported_names_str,
         const absl::string_view tags,
         const absl::string_view quant_opts_serialized) {
        const std::string exported_model_serialized =
            tensorflow::quantization::QuantizePtqModelPostCalibration(
                saved_model_path, exported_names_str, tags,
                quant_opts_serialized);

        return py::bytes(exported_model_serialized);
      },
      R"pbdoc(
      Returns serialized ExportedModel that contains the quantized model's
      GraphDef and metadata.
    )pbdoc");
}
