#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace ecal::core {

struct CheckerboardOrderResult {
  bool success = false;
  std::vector<cv::Point2f> ordered; // row-major (rows*cols)
  float score = 0.0f;               // lower is better
};

CheckerboardOrderResult
orderCheckerboardCorners(const std::vector<cv::Point2f> &points, int rows,
                         int cols);

bool isCheckerboardValid(const std::vector<cv::Point2f> &ordered, int rows,
                         int cols, float tor_spacing = 0.25f,
                         float tor_orth = 0.2f);

std::vector<cv::Point3f> buildObjectPoints(int rows, int cols,
                                           float square_size);

cv::Mat drawCheckerboardRowSnake(const cv::Mat &gray_or_bgr,
                                 const std::vector<cv::Point2f> &ordered,
                                 int rows, int cols, int radius = 3,
                                 bool draw_points = true, int thickness = 1);

} // namespace ecal::core
