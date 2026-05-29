// Copyright (c) 2022-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "aruco_max.h"

#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/opencv.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

class ArucoMaxDetectorPy {
private:
  aruco_max::ArucoMaxDetector detector;

public:
  ArucoMaxDetectorPy(int dict_id)
      : detector(cv::aruco::getPredefinedDictionary(
            static_cast<cv::aruco::PredefinedDictionaryType>(dict_id))) {}

  // Accepts a NumPy array (grayscale or color image) from Python
  py::tuple detect(py::array_t<uint8_t> input_image) {
    py::buffer_info buf = input_image.request();

    int type = CV_8UC1;
    if (buf.ndim == 3) {
      if (buf.shape[2] == 1)
        type = CV_8UC1;
      else if (buf.shape[2] == 3)
        type = CV_8UC3;
      else if (buf.shape[2] == 4)
        type = CV_8UC4;
      else
        throw std::runtime_error("Unsupported number of channels");
    } else if (buf.ndim != 2) {
      throw std::runtime_error("Input image must be grayscale or color");
    }

    // Map the numpy array directly to a cv::Mat to avoid deep copying
    cv::Mat img(buf.shape[0], buf.shape[1], type, (void *)buf.ptr);

    cv::Mat gray_img;
    if (type == CV_8UC3) {
      cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
    } else if (type == CV_8UC4) {
      cv::cvtColor(img, gray_img, cv::COLOR_BGRA2GRAY);
    } else {
      gray_img = img;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    // Detect markers using Aruco Max
    detector.detectMarkers(gray_img, corners, ids);

    py::list py_corners;
    for (const auto &marker : corners) {
      py::array_t<float> marker_arr({1, 4, 2});
      auto r = marker_arr.mutable_unchecked<3>();
      for (size_t i = 0; i < marker.size(); ++i) {
        r(0, i, 0) = marker[i].x;
        r(0, i, 1) = marker[i].y;
      }
      py_corners.append(marker_arr);
    }

    py::list py_ids;
    for (int id : ids) {
      py_ids.append(id);
    }

    return py::make_tuple(py_corners, py_ids);
  }
};

PYBIND11_MODULE(aruco_max, m) {
  m.doc() = "FRC Team 6328's optimized Aruco detector based on Aruco Nano.";

  py::class_<ArucoMaxDetectorPy>(m, "ArucoMaxDetector")
      .def(py::init<int>(), py::arg("dict_id"))
      .def("detect", &ArucoMaxDetectorPy::detect,
           "Detect markers in a grayscale numpy array");
}