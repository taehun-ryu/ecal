#pragma once

#include <string>

#include "ecal/viz/calibration_viz.hpp"

namespace ecal::io {

std::string prepareCalibrationRunOutputDir(const std::string &base_out_dir);

void ensureCalibrationOutputDirs(const std::string &out_dir);

void saveCalibrationOutputs(const std::string &out_dir, size_t window_idx,
                            const ecal::viz::WindowVis &vis);

void saveCalibrationYaml(const std::string &out_dir, size_t used_windows,
                         size_t total_windows, int board_rows, int board_cols,
                         float square_size, int calib_B, int calib_R,
                         size_t used_runs, const cv::Mat &K_mean,
                         const cv::Mat &K_std, const cv::Mat &dist_mean,
                         const cv::Mat &dist_std, double reproj_mean,
                         double reproj_std);

} // namespace ecal::io
