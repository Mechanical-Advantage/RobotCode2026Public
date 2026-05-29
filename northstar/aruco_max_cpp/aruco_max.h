// Copyright (c) 2022-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "metal_context.h"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <sstream>
#include <vector>

namespace aruco_max {

/**
 * @brief A detected ArUco marker.
 *
 * Inherits from std::vector<cv::Point2f> where each element represents
 * a subpixel-accurate corner of the detected marker.
 */
class Marker : public std::vector<cv::Point2f> {
public:
  /// Identifier of the marker (-1 if unidentified).
  int id = -1;

  /// Dictionary index (defaults to 0 for a single dictionary configuration).
  int dict = 0;

  /**
   * @brief Draws the marker border, corners, and ID on a given image.
   * * @param image The image to draw the marker on.
   * @param color The base color for the marker's outline.
   */
  inline void draw(cv::Mat &image,
                   const cv::Scalar color = cv::Scalar(0, 0, 255)) const;
};

/**
 * @brief Configuration parameters for the ArucoMaxDetector.
 */
struct DetectorParameters {
  // -----------------------------------------------------------------------
  // Thresholding & Filtering
  // -----------------------------------------------------------------------
  /// Filter size for adaptive thresholding.
  int boxFilterSize = 15;

  /// Threshold value for adaptive thresholding.
  int thres = 3;

  // -----------------------------------------------------------------------
  // Contour & Candidate Extraction
  // -----------------------------------------------------------------------
  /// Minimum size of a contour side to be considered as a candidate.
  int minSize = 10;

  /// Maximum number of times a contour can revisit any of its pixels.
  /// (1 is traditional tracing; higher values allow complex/noisy shapes).
  float maxTimesRevisited = 0.05f;

  /// Number of attempts to identify a candidate by slightly altering corners.
  int maxAttemptsPerCandidate = 5;

  // -----------------------------------------------------------------------
  // Dictionary & Identification
  // -----------------------------------------------------------------------
  /// The ArUco dictionary used for detection.
  cv::aruco::Dictionary dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_MIP_36h12);

  /// Width of the marker border in bits.
  float markerBorderBits = 1.0f;

  /// Maximum allowed error correction rate. Default 0 is recommended to avoid
  /// false positives.
  double errorCorrectionRate = 0.0;

  /// Maximum rate of erroneous bits in the border. Default 0 means no errors
  /// allowed.
  double maxErroneousBitsInBorderRate = 0.0;

  /// Set to true if the markers are printed inverted (white over black
  /// background).
  bool detectInvertedMarker = false;
};

namespace _private {

/**
 * @brief Utility for computing and applying perspective transformations.
 */
struct Homographer {
  /**
   * @brief Constructs a homography matrix from standard square coordinates to
   * marker corners.
   * @param out The four corners of the marker in the image.
   */
  Homographer(const std::vector<cv::Point2f> &out) {
    std::vector<cv::Point2f> in = {cv::Point2f(0, 0), cv::Point2f(1, 0),
                                   cv::Point2f(1, 1), cv::Point2f(0, 1)};
    H = cv::getPerspectiveTransform(in, out);
  }

  /**
   * @brief Applies the perspective transform to a given point.
   */
  cv::Point2f operator()(const cv::Point2f &p) {
    double *m = H.ptr<double>(0);
    double c = m[6] * p.x + m[7] * p.y + m[8];
    return cv::Point2f((m[0] * p.x + m[1] * p.y + m[2]) / c,
                       (m[3] * p.x + m[4] * p.y + m[5]) / c);
  }

  cv::Mat H;
};

} // namespace _private

/**
 * @brief High-performance Aruco marker detector.
 */
class ArucoMaxDetector {
public:
  /**
   * @brief Default constructor using standard parameters.
   */
  ArucoMaxDetector() {}

  /**
   * @brief Constructs the detector with a specific dictionary and parameters.
   * * @param dict The ArUco dictionary to use for identification.
   * @param params Detection parameters.
   */
  ArucoMaxDetector(const cv::aruco::Dictionary &dict,
                   const DetectorParameters &params = {}) {
    _params = params;
    _params.dictionary = dict;
  }

  /**
   * @brief Detects ArUco markers in an image.
   * * @param image Input image.
   * @param corners Output array of detected marker corners.
   * @param ids Output array of identifiers for the detected markers.
   * @param rejectedImgPoints Output array of candidates that were rejected
   * during identification.
   */
  void detectMarkers(
      cv::InputArray image, cv::OutputArrayOfArrays corners,
      cv::OutputArray ids,
      cv::OutputArrayOfArrays rejectedImgPoints = cv::noArray()) const;

private:
  DetectorParameters _params;
  mutable ArucoMaxMetalContext _metalCtx;

  std::vector<Marker>
  detectImpl(const cv::Mat &img,
             std::vector<Marker> *candidatesOut = nullptr) const;

  static Marker sort(const Marker &marker);
  static float getSubpixelValue(const cv::Mat &im_grey, const cv::Point2f &p);

  static int getMarkerId(cv::Mat candidateBits, int &idx, int &nrotations,
                         const DetectorParameters &params);
  static int getBorderErrors(const cv::Mat &bits, int markerSize,
                             int borderSize);

  static int isInto(const std::vector<cv::Point2f> &a,
                    const std::vector<cv::Point2f> &b);
  static void thres255Adaptive(cv::Mat &in, cv::Mat &out, int off = 2,
                               int thres = 5);

  static std::vector<std::vector<cv::Point>>
  visitedAwareTracingContour(cv::Mat &padded, size_t minSize = 1,
                             float maxRevisited = 0.1f);

  void copyVector2Output(const std::vector<Marker> &vec,
                         cv::OutputArrayOfArrays out) const;
};

// =========================================================================================
// Inline Implementations
// =========================================================================================

void Marker::draw(cv::Mat &in, const cv::Scalar color) const {
  auto _to_string = [](int i) {
    std::stringstream str;
    str << i;
    return str.str();
  };

  float flineWidth = std::max(1.f, std::min(5.f, float(in.cols) / 500.f));
  int lineWidth = std::round(flineWidth);

  // Draw edges
  for (int i = 0; i < 4; i++) {
    cv::line(in, (*this)[i], (*this)[(i + 1) % 4], color, lineWidth);
  }

  // Highlight corners
  auto p2 = cv::Point2f(2.f * static_cast<float>(lineWidth),
                        2.f * static_cast<float>(lineWidth));
  cv::rectangle(in, (*this)[0] - p2, (*this)[0] + p2,
                cv::Scalar(0, 0, 255, 255), -1);
  cv::rectangle(in, (*this)[1] - p2, (*this)[1] + p2,
                cv::Scalar(0, 255, 0, 255), lineWidth);
  cv::rectangle(in, (*this)[2] - p2, (*this)[2] + p2,
                cv::Scalar(255, 0, 0, 255), lineWidth);

  // Compute centroid and display ID
  cv::Point2f cent(0, 0);
  for (const auto &p : *this)
    cent += p;
  cent /= 4.0f;

  float fsize = std::min(3.0f, flineWidth * 0.75f);
  cv::putText(in, _to_string(id), cent - cv::Point2f(10 * flineWidth, 0),
              cv::FONT_HERSHEY_SIMPLEX, fsize,
              cv::Scalar(255, 255, 255) - color, lineWidth, cv::LINE_AA);
}

void ArucoMaxDetector::detectMarkers(
    cv::InputArray _image, cv::OutputArrayOfArrays _corners,
    cv::OutputArray _ids, cv::OutputArrayOfArrays _rejectedImgPoints) const {
  cv::Mat image = _image.getMat();
  CV_Assert(!image.empty());

  std::vector<Marker> rejectedImgPointsVec;

  // 1. Execute core detection
  std::vector<Marker> markers = this->detectImpl(image, &rejectedImgPointsVec);

  // 2. Format marker outputs
  copyVector2Output(markers, _corners);
  if (_rejectedImgPoints.needed()) {
    copyVector2Output(rejectedImgPointsVec, _rejectedImgPoints);
  }

  // 3. Format ID outputs
  std::vector<int> idsVec;
  idsVec.reserve(markers.size());
  for (const auto &m : markers) {
    idsVec.push_back(m.id);
  }

  _ids.create((int)idsVec.size(), 1, CV_32SC1);
  cv::Mat(idsVec).copyTo(_ids);
}

std::vector<Marker>
ArucoMaxDetector::detectImpl(const cv::Mat &img,
                             std::vector<Marker> *candidatesOut) const {
  cv::Mat bwimage, thresImage;
  std::vector<Marker> detectedMarkers;

  // 1. Grayscale Conversion
  if (img.channels() == 3) {
    cv::cvtColor(img, bwimage, cv::COLOR_BGR2GRAY);
  } else {
    bwimage = img;
  }

  // 2. Adaptive Thresholding
  ArucoMaxMetalContext *metalCtxPtr = &_metalCtx;
  if (metalCtxPtr != nullptr) {
    uint8_t *outData = metalCtxPtr->adaptiveThreshold(
        bwimage.ptr<uint8_t>(), bwimage.cols, bwimage.rows,
        _params.boxFilterSize, _params.thres);
    thresImage = cv::Mat(bwimage.rows, bwimage.cols, CV_8UC1, outData);
  } else {
    cv::boxFilter(bwimage, thresImage, bwimage.type(),
                  cv::Size(_params.boxFilterSize, _params.boxFilterSize),
                  cv::Point(-1, -1), true,
                  cv::BORDER_REPLICATE | cv::BORDER_ISOLATED);
    thresImage = thresImage - bwimage;
    cv::threshold(thresImage, thresImage, _params.thres, 255,
                  cv::THRESH_BINARY);
  }

  // 3. Contour Detection
  int minSizeSq = _params.minSize * _params.minSize;
  int minSize4 = 4 * _params.minSize;

  std::vector<std::vector<cv::Point>> contours = visitedAwareTracingContour(
      thresImage, minSize4, _params.maxTimesRevisited);

  std::vector<Marker> localCandidates;
  if (candidatesOut == nullptr) {
    candidatesOut = &localCandidates;
  } else {
    candidatesOut->clear();
  }

  // 4. Rectangle Approximation & Candidate Formatting
  std::vector<cv::Point> approxCurve;
  for (const auto &contour : contours) {
    cv::approxPolyDP(contour, approxCurve, double(contour.size()) * 0.03, true);

    if (approxCurve.size() != 4 || !cv::isContourConvex(approxCurve))
      continue;

    // Verify distance between corners
    if (((approxCurve[0].x - approxCurve[1].x) *
             (approxCurve[0].x - approxCurve[1].x) +
         (approxCurve[0].y - approxCurve[1].y) *
             (approxCurve[0].y - approxCurve[1].y)) < minSizeSq)
      continue;
    if (((approxCurve[1].x - approxCurve[2].x) *
             (approxCurve[1].x - approxCurve[2].x) +
         (approxCurve[1].y - approxCurve[2].y) *
             (approxCurve[1].y - approxCurve[2].y)) < minSizeSq)
      continue;
    if (((approxCurve[2].x - approxCurve[3].x) *
             (approxCurve[2].x - approxCurve[3].x) +
         (approxCurve[2].y - approxCurve[3].y) *
             (approxCurve[2].y - approxCurve[3].y)) < minSizeSq)
      continue;
    if (((approxCurve[3].x - approxCurve[0].x) *
             (approxCurve[3].x - approxCurve[0].x) +
         (approxCurve[3].y - approxCurve[0].y) *
             (approxCurve[3].y - approxCurve[0].y)) < minSizeSq)
      continue;

    Marker marker;
    marker.reserve(4);
    for (int j = 0; j < 4; j++) {
      marker.emplace_back(cv::Point2f(approxCurve[j].x, approxCurve[j].y));
    }

    marker = sort(marker);
    candidatesOut->push_back(marker);
  }

  // 5. Candidate Validation & ID Extraction
  cv::Mat bits(_params.dictionary.markerSize + 2,
               _params.dictionary.markerSize + 2, CV_8UC1);
  cv::Mat bitadaptive(_params.dictionary.markerSize + 2,
                      _params.dictionary.markerSize + 2, CV_8UC1);
  cv::RNG rand;

  for (auto it = candidatesOut->begin(); it != candidatesOut->end();) {
    auto marker = *it;

    for (int i = 0; i < _params.maxAttemptsPerCandidate && marker.id == -1;
         i++) {
      auto marker2 = marker;

      // Alter corner locations slightly on subsequent attempts
      if (i != 0) {
        for (int c = 0; c < 4; c++) {
          marker2[c].x += rand.gaussian(0.75);
          marker2[c].y += rand.gaussian(0.75);
        }
      }

      _private::Homographer hom(marker2);
      for (int r = 0; r < bits.rows; r++) {
        for (int c = 0; c < bits.cols; c++) {
          bits.at<uchar>(r, c) = uchar(
              0.5 + getSubpixelValue(
                        bwimage,
                        hom(cv::Point2f(float(c + 0.5f) / float(bits.cols),
                                        float(r + 0.5f) / float(bits.rows)))));
        }
      }

      // Fallback adaptive threshold to improve robustness
      if (i == 2) {
        thres255Adaptive(bits, bitadaptive);
        bitadaptive.copyTo(bits);
      } else {
        cv::threshold(bits, bits, 0, 255, cv::THRESH_OTSU);
      }

      int nrotations = 0;
      if (getMarkerId(bits, marker.id, nrotations, _params) == 0)
        continue;

      std::rotate(marker.begin(), marker.begin() + 4 - nrotations,
                  marker.end());
    }

    if (marker.id != -1) {
      marker.dict = 0;
      detectedMarkers.push_back(marker);
      it = candidatesOut->erase(it); // Remove from rejected candidates list
    } else {
      it++;
    }
  }

  // 6. Duplicate Detection Removal (Inner & Outer borders)
  std::sort(detectedMarkers.begin(), detectedMarkers.end(),
            [](const Marker &a, const Marker &b) { return a.id < b.id; });

  std::vector<Marker> finalMarkers;
  std::vector<bool> toRemove(detectedMarkers.size(), false);

  for (size_t i = 0; i < detectedMarkers.size(); i++) {
    if (toRemove[i])
      continue;
    for (size_t j = i + 1; j < detectedMarkers.size(); j++) {
      if (detectedMarkers[i].id == detectedMarkers[j].id) {
        int res = isInto(detectedMarkers[i], detectedMarkers[j]);
        if (res == 1)
          toRemove[i] = true;
        else if (res == 2)
          toRemove[j] = true;
      }
    }
  }

  for (size_t i = 0; i < detectedMarkers.size(); i++) {
    if (!toRemove[i])
      finalMarkers.push_back(detectedMarkers[i]);
  }

  // 7. Subpixel Corner Refinement
  if (!finalMarkers.empty()) {
    for (auto &marker : finalMarkers) {
      // Calculate the perimeter of the current marker to estimate its size
      float perimeter = 0.0f;
      for (int i = 0; i < 4; i++) {
        cv::Point2f diff = marker[i] - marker[(i + 1) % 4];
        perimeter += std::sqrt(diff.x * diff.x + diff.y * diff.y);
      }

      // Dynamically scale the window size
      // Matches OpenCV's window sizes for 36h11 tags
      int dynamicHalfWinSize =
          std::max(1, std::min(5, static_cast<int>(perimeter / 106.6f)));

      // Extract corners for just this marker
      std::vector<cv::Point2f> markerCorners(marker.begin(), marker.end());

      // Refine corners with the dynamically sized window
      cv::cornerSubPix(
          bwimage, markerCorners,
          cv::Size(dynamicHalfWinSize, dynamicHalfWinSize), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                           30, 0.01));

      // Update the marker with the refined corners
      for (int c = 0; c < 4; c++) {
        marker[c] = markerCorners[c];
      }
    }
  }

  return finalMarkers;
}

int ArucoMaxDetector::getMarkerId(cv::Mat candidateBits, int &idx,
                                  int &nrotations,
                                  const DetectorParameters &params) {
  uint8_t typ = 1;

  if (params.detectInvertedMarker) {
    candidateBits = ~candidateBits;
  }

  // Analyze border errors
  int maximumErrorsInBorder =
      int(params.dictionary.markerSize * params.dictionary.markerSize *
          params.maxErroneousBitsInBorderRate);
  int borderSize = int(params.markerBorderBits);
  int borderErrors =
      getBorderErrors(candidateBits, params.dictionary.markerSize, borderSize);

  if (borderErrors > maximumErrorsInBorder)
    return 0;

  // Extract core bits, convert, and identify
  cv::Mat onlyBits =
      candidateBits.rowRange(borderSize, candidateBits.rows - borderSize)
          .colRange(borderSize, candidateBits.cols - borderSize);

  onlyBits /= 255;

  if (!params.dictionary.identify(onlyBits, idx, nrotations,
                                  params.errorCorrectionRate)) {
    return 0;
  }

  return typ;
}

int ArucoMaxDetector::getBorderErrors(const cv::Mat &bits, int markerSize,
                                      int borderSize) {
  int sizeWithBorders = markerSize + 2 * borderSize;
  int totalErrors = 0;

  // Top and Bottom borders
  for (int y = 0; y < sizeWithBorders; y++) {
    for (int k = 0; k < borderSize; k++) {
      if (bits.ptr<unsigned char>(y)[k] != 0)
        totalErrors++;
      if (bits.ptr<unsigned char>(y)[sizeWithBorders - 1 - k] != 0)
        totalErrors++;
    }
  }

  // Left and Right borders
  for (int x = borderSize; x < sizeWithBorders - borderSize; x++) {
    for (int k = 0; k < borderSize; k++) {
      if (bits.ptr<unsigned char>(k)[x] != 0)
        totalErrors++;
      if (bits.ptr<unsigned char>(sizeWithBorders - 1 - k)[x] != 0)
        totalErrors++;
    }
  }

  return totalErrors;
}

float ArucoMaxDetector::getSubpixelValue(const cv::Mat &im_grey,
                                         const cv::Point2f &p) {
  const int ix = static_cast<int>(p.x);
  const int iy = static_cast<int>(p.y);

  if (ix < 0 || iy < 0 || ix >= im_grey.cols - 1 || iy >= im_grey.rows - 1) {
    return 0.0f;
  }

  const float dx = p.x - ix;
  const float dy = p.y - iy;

  const uchar *ptr = im_grey.ptr<uchar>(iy) + ix;
  const size_t step = im_grey.step;

  const float p00 = static_cast<float>(ptr[0]);
  const float p01 = static_cast<float>(ptr[1]);
  const float p10 = static_cast<float>(ptr[step]);
  const float p11 = static_cast<float>(ptr[step + 1]);

  const float top = p00 + dx * (p01 - p00);
  const float bot = p10 + dx * (p11 - p10);

  return top + dy * (bot - top);
}

Marker ArucoMaxDetector::sort(const Marker &marker) {
  Marker res_marker = marker;

  double dx1 = res_marker[1].x - res_marker[0].x;
  double dy1 = res_marker[1].y - res_marker[0].y;
  double dx2 = res_marker[2].x - res_marker[0].x;
  double dy2 = res_marker[2].y - res_marker[0].y;
  double o = (dx1 * dy2) - (dy1 * dx2);

  if (o < 0.0)
    std::swap(res_marker[1], res_marker[3]);

  return res_marker;
}

int ArucoMaxDetector::isInto(const std::vector<cv::Point2f> &a,
                             const std::vector<cv::Point2f> &b) {
  auto countInside = [](const std::vector<cv::Point2f> &source,
                        const std::vector<cv::Point2f> &target) -> int {
    int count = 0;
    for (const auto &pt : source) {
      bool inside = false;
      for (int i = 0, j = 3; i < 4; j = i++) {
        if (((target[i].y > pt.y) != (target[j].y > pt.y)) &&
            (pt.x < (target[j].x - target[i].x) * (pt.y - target[i].y) /
                            (target[j].y - target[i].y) +
                        target[i].x)) {
          inside = !inside;
        }
      }
      if (inside)
        count++;
    }
    return count;
  };

  int aInB = countInside(a, b);
  int bInA = countInside(b, a);

  if (aInB == 0 && bInA == 0)
    return 0;
  if (aInB > bInA)
    return 1;
  if (bInA > aInB)
    return 2;

  return 0;
}

std::vector<std::vector<cv::Point>>
ArucoMaxDetector::visitedAwareTracingContour(cv::Mat &padded, size_t minSize,
                                             float maxRevisited) {
  if (padded.empty() || padded.type() != CV_8UC1)
    return {};

  int rows = padded.rows;
  int cols = padded.cols;
  int32_t step = padded.step;
  uchar *data = padded.data;

  // Fast boundary clear
  memset(data, 0, cols);
  memset(data + (rows - 1) * step, 0, cols);
  for (int r = 1; r < rows - 1; ++r) {
    uchar *row_ptr = data + r * step;
    row_ptr[0] = 0;
    row_ptr[cols - 1] = 0;
  }

  const int offsets[16] = {
      -1, -step - 1, -step, -step + 1, 1, step + 1, step, step - 1,
      -1, -step - 1, -step, -step + 1, 1, step + 1, step, step - 1,
  };

  const int dx[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
  const int dy[8] = {0, -1, -1, -1, 0, 1, 1, 1};

  std::vector<std::vector<cv::Point>> contours;
  contours.reserve(2048);
  std::vector<cv::Point> buffer;
  buffer.reserve(2048);

  const uchar FOREGROUND = 255, BACKGROUND = 0, VISITED = 100;

  for (int r = 1; r < rows - 1; r++) {
    uchar *row_ptr = data + r * step;
    for (int c = 1; c < cols - 1;) {

      // SIMD acceleration block for scanning foreground pixels
#if (CV_SIMD || CV_SIMD_SCALABLE)
      cv::v_uint8 v_zero = cv::vx_setzero_u8();
      for (; c <= cols - cv::VTraits<cv::v_uint8>::vlanes();
           c += cv::VTraits<cv::v_uint8>::vlanes()) {
        cv::v_uint8 vmask =
            (cv::v_ne(cv::vx_load((uchar *)(row_ptr + c)), v_zero));
        if (v_check_any(vmask)) {
          c += v_scan_forward(vmask);
          break;
        }
      }
#endif
      for (; c < cols && !row_ptr[c]; ++c)
        ;

      if (c == cols)
        break;

      if (row_ptr[c] == FOREGROUND) {
        buffer.clear();
        int curr_x = c, curr_y = r, search_idx = 1;
        uchar *curr_ptr = row_ptr + c, *start_ptr = curr_ptr;
        size_t ntimesRevisited = 0;

        do {
          buffer.emplace_back(curr_x, curr_y);
          *curr_ptr = VISITED;

          for (int i = 0; i < 8; ++i) {
            int idx = search_idx + i;
            uchar *neighbor = curr_ptr + offsets[idx];
            if (*neighbor != BACKGROUND) {
              curr_ptr = neighbor;
              int dir = (idx & 7);
              int next_x = curr_x + dx[dir], next_y = curr_y + dy[dir];

              ntimesRevisited += int(*neighbor == VISITED);
              curr_x = next_x;
              curr_y = next_y;
              search_idx = (dir + 5) & 7;
              break;
            }
          }
        } while (curr_ptr != start_ptr);

        size_t bufsize = buffer.size();
        if (ntimesRevisited <= float(bufsize) * maxRevisited &&
            bufsize >= minSize) {
          contours.push_back(buffer);
        }
      }
      c++;

      // SIMD block for clearing scanned memory block
      if (row_ptr[c]) {
#if (CV_SIMD || CV_SIMD_SCALABLE)
        cv::v_uint8 v_zero = cv::vx_setzero_u8();
        for (; c <= cols - cv::VTraits<cv::v_uint8>::vlanes();
             c += cv::VTraits<cv::v_uint8>::vlanes()) {
          cv::v_uint8 vmask =
              (cv::v_eq(cv::vx_load((uchar *)(row_ptr + c)), v_zero));
          if (cv::v_check_any(vmask)) {
            c += cv::v_scan_forward(vmask);
            break;
          }
        }
#endif
        for (; c < cols && row_ptr[c]; ++c)
          ;
      }
    }
  }
  return contours;
}

void ArucoMaxDetector::thres255Adaptive(cv::Mat &in, cv::Mat &out, int off,
                                        int thres) {
  cv::boxFilter(in, out, in.type(), cv::Size(off * 2 + 1, off * 2 + 1),
                cv::Point(-1, -1), true, 4);

  for (int i = 0; i < in.rows; i++) {
    const uchar *sdata = in.ptr(i);
    uchar *ddata = out.ptr(i);
    for (int j = 0; j < in.cols; j++) {
      ddata[j] = ((ddata[j] - thres) < sdata[j]) * 255;
    }
  }
}

void ArucoMaxDetector::copyVector2Output(const std::vector<Marker> &vec,
                                         cv::OutputArrayOfArrays out) const {
  out.create((int)vec.size(), 1, CV_32FC2);
  if (out.isMatVector()) {
    for (unsigned int i = 0; i < vec.size(); i++) {
      out.create(4, 1, CV_32FC2, i);
      cv::Mat &m = out.getMatRef(i);
      cv::Mat(cv::Mat(vec[i]).t()).copyTo(m);
    }
  } else if (out.isUMatVector()) {
    for (unsigned int i = 0; i < vec.size(); i++) {
      out.create(4, 1, CV_32FC2, i);
      cv::UMat &m = out.getUMatRef(i);
      cv::Mat(cv::Mat(vec[i]).t()).copyTo(m);
    }
  } else if (out.kind() == cv::_OutputArray::STD_VECTOR_VECTOR) {
    for (unsigned int i = 0; i < vec.size(); i++) {
      out.create(4, 1, CV_32FC2, i);
      cv::Mat m = out.getMat(i);
      cv::Mat(cv::Mat(vec[i]).t()).copyTo(m);
    }
  } else {
    CV_Error(cv::Error::StsNotImplemented,
             "Only Mat vector, UMat vector, and vector<vector> OutputArrays "
             "are currently supported.");
  }
}

} // namespace aruco_max