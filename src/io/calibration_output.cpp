#include "ecal/io/calibration_output.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace ecal::io {

static std::string zeroPad(size_t v, int width) {
  std::string s = std::to_string(v);
  if (static_cast<int>(s.size()) >= width) {
    return s;
  }
  return std::string(static_cast<size_t>(width - s.size()), '0') + s;
}

static std::string formatNowTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf {};
#if defined(_WIN32)
  localtime_s(&tm_buf, &now_tt);
#else
  localtime_r(&now_tt, &tm_buf);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
  return oss.str();
}

std::string prepareCalibrationRunOutputDir(const std::string &base_out_dir) {
  if (base_out_dir.empty()) {
    return "";
  }

  const std::filesystem::path base_path(base_out_dir);
  std::filesystem::create_directories(base_path);

  const std::string timestamp = formatNowTimestamp();
  std::filesystem::path run_path = base_path / ("run_" + timestamp);
  int suffix = 1;
  while (std::filesystem::exists(run_path)) {
    run_path =
        base_path / ("run_" + timestamp + "_" + std::to_string(suffix++));
  }

  ensureCalibrationOutputDirs(run_path.string());
  return run_path.string();
}

void ensureCalibrationOutputDirs(const std::string &out_dir) {
  if (out_dir.empty()) {
    return;
  }
  std::filesystem::create_directories(out_dir);
  std::filesystem::create_directories(std::filesystem::path(out_dir) / "raw");
  std::filesystem::create_directories(std::filesystem::path(out_dir) / "iwe");
  std::filesystem::create_directories(std::filesystem::path(out_dir) / "piwe");
  std::filesystem::create_directories(std::filesystem::path(out_dir) /
                                      "corner_detection");
}

void saveCalibrationOutputs(const std::string &out_dir, size_t window_idx,
                            const ecal::viz::WindowVis &vis) {
  if (out_dir.empty()) {
    return;
  }
  const std::string idx_str = zeroPad(window_idx, 6);
  const std::string raw_path = (std::filesystem::path(out_dir) / "raw" /
                                ("raw_events_" + idx_str + ".png"))
                                   .string();
  const std::string iwe_path =
      (std::filesystem::path(out_dir) / "iwe" / ("iwe_" + idx_str + ".png"))
          .string();
  const std::string piwe_path =
      (std::filesystem::path(out_dir) / "piwe" / ("piwe_" + idx_str + ".png"))
          .string();
  const std::string corner_path =
      (std::filesystem::path(out_dir) / "corner_detection" /
       ("corners_" + idx_str + ".png"))
          .string();

  if (!vis.raw_vis.empty()) {
    cv::imwrite(raw_path, vis.raw_vis);
  }
  if (!vis.piwe_vis.empty()) {
    cv::imwrite(piwe_path, vis.piwe_vis);
  }
  if (!vis.iwe_vis.empty()) {
    cv::imwrite(iwe_path, vis.iwe_vis);
  }
  if (!vis.corners_vis.empty()) {
    cv::imwrite(corner_path, vis.corners_vis);
  }
}

static void writeMatYaml(std::ofstream &ofs, const std::string &key,
                         const cv::Mat &m) {
  ofs << key << ":\n";
  if (m.empty()) {
    ofs << "  []\n";
    return;
  }
  for (int r = 0; r < m.rows; ++r) {
    ofs << "  - [";
    for (int c = 0; c < m.cols; ++c) {
      const double v = m.at<double>(r, c);
      ofs << v;
      if (c + 1 < m.cols)
        ofs << ", ";
    }
    ofs << "]\n";
  }
}

void saveCalibrationYaml(const std::string &out_dir, size_t used_windows,
                         size_t total_windows, int board_rows, int board_cols,
                         float square_size, int calib_B, int calib_R,
                         size_t used_runs, const cv::Mat &K_mean,
                         const cv::Mat &K_std, const cv::Mat &dist_mean,
                         const cv::Mat &dist_std, double reproj_mean,
                         double reproj_std) {
  if (out_dir.empty()) {
    return;
  }
  const std::string path =
      (std::filesystem::path(out_dir) / "calibration.yaml").string();
  std::ofstream ofs(path);
  if (!ofs) {
    return;
  }

  ofs << "used_windows: " << used_windows << "\n";
  ofs << "total_windows: " << total_windows << "\n";
  ofs << "checkerboard:\n";
  ofs << "  rows: " << board_rows << "\n";
  ofs << "  cols: " << board_cols << "\n";
  ofs << "  square_size: " << square_size << "\n";
  ofs << "bootstrap:\n";
  ofs << "  B: " << calib_B << "\n";
  ofs << "  R: " << calib_R << "\n";
  ofs << "  used_runs: " << used_runs << "\n";
  ofs << "reprojection_error_mean: " << reproj_mean << "\n";
  ofs << "reprojection_error_std: " << reproj_std << "\n";

  cv::Mat Kd_mean, Kd_std, distd_mean, distd_std;
  K_mean.convertTo(Kd_mean, CV_64F);
  K_std.convertTo(Kd_std, CV_64F);
  dist_mean.convertTo(distd_mean, CV_64F);
  dist_std.convertTo(distd_std, CV_64F);
  writeMatYaml(ofs, "camera_matrix_mean", Kd_mean);
  writeMatYaml(ofs, "camera_matrix_std", Kd_std);

  auto writeVec = [&](const std::string &key, const cv::Mat &m) {
    ofs << key << ": [";
    if (!m.empty()) {
      for (int i = 0; i < m.total(); ++i) {
        ofs << m.at<double>(static_cast<int>(i));
        if (i + 1 < m.total())
          ofs << ", ";
      }
    }
    ofs << "]\n";
  };
  writeVec("dist_coeffs_mean", distd_mean);
  writeVec("dist_coeffs_std", distd_std);
}

} // namespace ecal::io
