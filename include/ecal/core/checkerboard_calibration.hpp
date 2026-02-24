#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace ecal::core {

struct CalibrationResult {
  bool success = false;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  double reprojection_error = 0.0;
};

struct CalibrationOptions {
  int max_iter = 50;
  double eps = 1e-12;
  bool fix_k3plus = true;
  bool use_intrinsic_guess = true;
};

struct CalibrationBootstrapResult {
  bool success = false;
  size_t used_runs = 0;
  cv::Mat K_mean;
  cv::Mat K_std;
  cv::Mat dist_mean;
  cv::Mat dist_std;
  double reproj_mean = 0.0;
  double reproj_std = 0.0;
  CalibrationResult best;
  std::vector<int> best_indices;
};

double
computeReprojectionError(const std::vector<std::vector<cv::Point3f>> &objpoints,
                         const std::vector<std::vector<cv::Point2f>> &imgpoints,
                         const cv::Mat &K, const cv::Mat &dist,
                         const std::vector<cv::Mat> &rvecs,
                         const std::vector<cv::Mat> &tvecs);

CalibrationResult
calibrateCheckerboard(const std::vector<std::vector<cv::Point3f>> &objpoints,
                      const std::vector<std::vector<cv::Point2f>> &imgpoints,
                      const cv::Size &image_size,
                      const CalibrationOptions &opt = CalibrationOptions());

CalibrationBootstrapResult calibrateCheckerboardBootstrap(
    const std::vector<std::vector<cv::Point3f>> &objpoints,
    const std::vector<std::vector<cv::Point2f>> &imgpoints,
    const cv::Size &image_size, const CalibrationOptions &opt, int calib_B,
    int calib_R);

} // namespace ecal::core
