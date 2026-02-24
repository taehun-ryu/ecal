#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ecal/core/checkerboard_calibration.hpp"
#include "ecal/core/checkerboard_validity.hpp"
#include "ecal/core/cm_tracker_2d.hpp"
#include "ecal/core/corner_init.hpp"
#include "ecal/core/corner_refinement.hpp"
#include "ecal/core/patch_extractor.hpp"
#include "ecal/core/simple_event.hpp"
#include "ecal/io/calibration_config.hpp"
#include "ecal/io/calibration_output.hpp"
#include "ecal/io/h5_events.hpp"
#include "ecal/viz/calibration_report.hpp"
#include "ecal/viz/calibration_viz.hpp"

namespace {
bool parseArgs(int argc, char **argv, ecal::io::CalibrationConfig &cfg) {
  // ecal_checkerboard_calibrate [config_path]
  std::string config_path = "config/checkerboard.yaml";
  if (argc >= 2) {
    config_path = argv[1];
  }
  std::string err;
  if (!ecal::io::loadCalibrationConfig(config_path, cfg, &err)) {
    std::cerr << err << "\n";
    return false;
  }
  return true;
}

inline double usToSec(uint64_t t_us) {
  return static_cast<double>(t_us) * 1e-6;
}

bool parseBoolEnv(const char *name, bool default_value) {
  const char *raw = std::getenv(name);
  if (raw == nullptr || *raw == '\0') {
    return default_value;
  }
  std::string value(raw);
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  return default_value;
}

bool shouldEnableGui() {
  const bool has_display = (std::getenv("DISPLAY") != nullptr) ||
                           (std::getenv("WAYLAND_DISPLAY") != nullptr);
  return parseBoolEnv("ECAL_ENABLE_GUI", has_display);
}

} // namespace

int main(int argc, char **argv) {
  ecal::io::CalibrationConfig cfg;
  if (!parseArgs(argc, argv, cfg)) {
    return 1;
  }

  ecal::io::H5Events h5;
  std::string h5_err;
  if (!ecal::io::loadH5Events(cfg.h5_path, h5, &h5_err)) {
    std::cerr << "H5 load failed: " << h5_err << "\n";
    return 1;
  }
  if (h5.ts_us.empty()) {
    std::cerr << "No events in file\n";
    return 1;
  }

  cfg.out_dir = ecal::io::prepareCalibrationRunOutputDir(cfg.out_dir);
  if (!cfg.out_dir.empty()) {
    std::cout << "[out] saving to: " << cfg.out_dir << "\n";
  }

  bool gui_enabled = shouldEnableGui();
  if (!gui_enabled) {
    std::cout << "[viz] GUI disabled (set ECAL_ENABLE_GUI=1 to force enable)\n";
  }

  // Tracker config
  ecal::core::CmIweOptions iwe_opt;
  iwe_opt.sigma = cfg.cm_iwe_sigma;
  iwe_opt.cutoff_factor = cfg.cm_iwe_cutoff_factor;
  iwe_opt.patch_radius_override = cfg.cm_iwe_patch_radius_override;
  iwe_opt.use_variance = cfg.use_variance; // L2 or IWE

  ecal::core::CmTracker2DOptions track_opt;
  track_opt.max_iterations = cfg.tracker_max_iterations;
  track_opt.verbose = false;
  track_opt.compute_final_iwe = true;
  track_opt.final_use_full_events = true;

  ecal::core::CmTracker2D tracker(cfg.width, cfg.height,
                                  cfg.tracker_num_threads, iwe_opt);

  const uint64_t t0_us = h5.ts_us.front();
  const uint64_t window_len_us =
      static_cast<uint64_t>(cfg.window_sec * 1e6 + 0.5);
  const uint64_t max_window_len_us =
      static_cast<uint64_t>(cfg.max_window_sec * 1e6 + 0.5);

  size_t idx = 0;
  size_t window_idx = 0;
  uint64_t window_start_us = h5.ts_us.front();
  uint64_t window_end_us = window_start_us + window_len_us;

  double vx = 0.0;
  double vy = 0.0;

  std::vector<ecal::core::TimedEventNs> window;
  window.reserve(100000);

  const int board_rows = cfg.board_h;
  const int board_cols = cfg.board_w;
  int expected = cfg.expected_corners;
  if (expected <= 0 && board_rows > 0 && board_cols > 0) {
    expected = board_rows * board_cols;
  }
  const std::vector<cv::Point3f> objp =
      ecal::core::buildObjectPoints(board_rows, board_cols, cfg.square_size);
  std::vector<std::vector<cv::Point3f>> objpoints;
  std::vector<std::vector<cv::Point2f>> imgpoints;

  while (idx < h5.ts_us.size()) {
    const uint64_t t_us = h5.ts_us[idx];
    if (t_us < window_end_us) {
      ecal::core::TimedEventNs ev;
      ev.x = h5.xs[idx];
      ev.y = h5.ys[idx];
      ev.polarity = (h5.ps[idx] != 0);
      ev.t_ns = static_cast<int64_t>((t_us - t0_us) * 1000ULL);
      window.push_back(ev);
      idx++;
      if (window.size() >= cfg.max_events) {
        window_end_us = t_us;
      }
      continue;
    }

    const uint64_t window_dur_us = window_end_us - window_start_us;
    if (window.size() < cfg.min_events && window_dur_us < max_window_len_us) {
      window_end_us = std::min(window_start_us + max_window_len_us,
                               window_end_us + window_len_us);
      continue;
    }

    if (!window.empty()) {
      // Convert to SimpleEvent (seconds relative to window start)
      const uint64_t win_t0_us = window.front().t_ns / 1000ULL;
      std::vector<ecal::core::SimpleEvent> frame_events;
      frame_events.reserve(window.size());
      for (const auto &e : window) {
        ecal::core::SimpleEvent s;
        s.x = e.x;
        s.y = e.y;
        s.polarity = e.polarity;
        const uint64_t t_us_rel =
            static_cast<uint64_t>(e.t_ns / 1000ULL) - win_t0_us;
        s.t = usToSec(t_us_rel);
        frame_events.push_back(s);
      }

      const auto res = tracker.track(frame_events, track_opt, vx, vy);

      vx = res.vx;
      vy = res.vy;

      // Debug stats: dt range, polarity ratio
      uint64_t min_dt = std::numeric_limits<uint64_t>::max();
      uint64_t max_dt = 0;
      size_t pos_cnt = 0;
      const uint64_t t0_ns = static_cast<uint64_t>(window.front().t_ns);
      for (const auto &ev : window) {
        const uint64_t dt = static_cast<uint64_t>(ev.t_ns) - t0_ns;
        min_dt = std::min(min_dt, dt);
        max_dt = std::max(max_dt, dt);
        if (ev.polarity) {
          pos_cnt++;
        }
      }
      const double pos_ratio =
          window.empty() ? 0.0 : static_cast<double>(pos_cnt) / window.size();

      std::cout << "[cm] window=" << window_idx << " events=" << window.size()
                << " v=(" << vx << "," << vy << ")"
                << " dt_s=[" << (min_dt * 1e-9) << "," << (max_dt * 1e-9) << "]"
                << " pos=" << pos_ratio << " ok=" << res.success << "\n";

      // Patch extraction + corner init
      std::vector<cv::Point> patch_points;
      std::vector<ecal::core::PatchBox> boxes;
      std::vector<cv::Point2f> init_corners;
      std::vector<cv::Point2f> refined_corners;
      std::vector<cv::Point2f> filtered_corners;
      bool filtered_valid = false;

      ecal::core::CornerInitOptions corner_opt;
      corner_opt.seed_thr = 10.0f;
      corner_opt.seed_percentile = 90.0f;
      corner_opt.tau = 0.5f;
      corner_opt.tau_percentile = 70.0f;
      corner_opt.max_grow_iters = 20;
      corner_opt.min_component_area = 2;
      corner_opt.weighted_line_fit = true;

      ecal::core::CornerRefineOptions ref_opt;
      ref_opt.enable = true;
      ref_opt.lr = cfg.corner_refine_lr;
      ref_opt.max_iter = cfg.corner_refine_max_iter;
      ref_opt.gtol = 1e-6f;
      ref_opt.armijo_c = 1e-4f;
      ref_opt.min_step = 1e-6f;
      ref_opt.strip_half_width0 = cfg.corner_refine_strip_half_width0;
      ref_opt.strip_half_width1 = cfg.corner_refine_strip_half_width1;

      float global_seed_thr = corner_opt.seed_thr;
      float global_tau_thr = corner_opt.tau;

      if (!res.iwe.empty() && !res.piwe.empty()) {
        ecal::core::computeGlobalIweThresholds(
            res.iwe, corner_opt, &global_seed_thr, &global_tau_thr);

        ecal::core::PatchPointsOptions pp_opt;
        pp_opt.radius = cfg.pp_radius;

        ecal::core::PatchClusterOptions cl_opt;
        cl_opt.eps = 0;
        cl_opt.min_component_area = 1;

        ecal::core::PatchExtractor extractor(pp_opt, cl_opt);
        patch_points = extractor.computePatchPoints(res.piwe);
        boxes = extractor.clusterToBoxes(patch_points, res.piwe.cols,
                                         res.piwe.rows);

        init_corners.reserve(boxes.size());
        refined_corners.reserve(boxes.size());
        for (const auto &b : boxes) {
          const int w = b.x1 - b.x0;
          const int h = b.y1 - b.y0;
          if (w <= 1 || h <= 1) {
            continue;
          }
          const cv::Rect roi(b.x0, b.y0, w, h);
          const cv::Mat iwe_patch = res.iwe(roi);
          const auto init_res = ecal::core::initializeCornerFromIwePatch(
              iwe_patch, corner_opt, global_seed_thr, global_tau_thr);
          if (!init_res.success) {
            continue;
          }
          auto ref_res = ecal::core::refineCornerInIwePatch(
              iwe_patch, init_res.init_corner_xy, init_res.line0,
              init_res.line1, ref_opt);
          const cv::Point2f corner_global(
              static_cast<float>(b.x0) + init_res.corner_xy.x,
              static_cast<float>(b.y0) + init_res.corner_xy.y);
          init_corners.push_back(corner_global);
          if (ref_res.success) {
            refined_corners.emplace_back(
                static_cast<float>(b.x0) + ref_res.refined_xy.x,
                static_cast<float>(b.y0) + ref_res.refined_xy.y);
          }
        }

        const std::vector<cv::Point2f> &cand =
            refined_corners.empty() ? init_corners : refined_corners;
        if (expected > 0 && board_rows > 0 && board_cols > 0 &&
            static_cast<int>(cand.size()) >= expected) {
          auto ord = ecal::core::orderCheckerboardCorners(cand, board_rows,
                                                          board_cols);
          if (ord.success && static_cast<int>(ord.ordered.size()) == expected) {
            if (ecal::core::isCheckerboardValid(
                    ord.ordered, board_rows, board_cols,
                    cfg.checkerboard_tor_spacing, cfg.checkerboard_tor_orth)) {
              filtered_corners = ord.ordered;
              filtered_valid = true;
              imgpoints.push_back(filtered_corners);
              objpoints.push_back(objp);
            }
          }
        }
      }

      std::cout << "[corner] window=" << window_idx
                << " init=" << init_corners.size()
                << " refined=" << refined_corners.size()
                << " valid=" << (filtered_valid ? "yes" : "no") << "\n";

      const double window_dt_s =
          window.size() > 1
              ? (static_cast<double>(window.back().t_ns - window.front().t_ns) *
                 1e-9)
              : 0.0;

      // Visualization
      const auto vis = ecal::viz::buildWindowVis(
          window, cfg.width, cfg.height, res.iwe, res.piwe, patch_points, boxes,
          init_corners, refined_corners, filtered_corners, static_cast<float>(vx),
          static_cast<float>(vy), window_dt_s, board_rows, board_cols,
          cfg.viz_zoom);
      if (gui_enabled) {
        try {
          ecal::viz::showWindowVis(vis);
          cv::waitKey(1);
        } catch (const cv::Exception &e) {
          std::cerr
              << "[viz] GUI unavailable, disabling window rendering: "
              << e.what() << "\n";
          gui_enabled = false;
        }
      }
      ecal::io::saveCalibrationOutputs(cfg.out_dir, window_idx, vis);

      window_idx++;
    }

    while (t_us >= window_end_us) {
      window_start_us = window_end_us;
      window_end_us = window_start_us + window_len_us;
    }

    window.clear();
  }
  if (gui_enabled) {
    cv::destroyAllWindows();
  }

  if (!imgpoints.empty() && !objpoints.empty()) {
    const size_t total_windows = window_idx;
    const size_t used_windows = imgpoints.size();
    std::cout << "[calib] Calibrating with " << used_windows << "/"
              << total_windows << " windows...\n";
    const cv::Size image_size(cfg.width, cfg.height);
    ecal::core::CalibrationOptions calib_opt;
    calib_opt.max_iter = cfg.calib_max_iter;
    calib_opt.fix_k3plus = cfg.calib_fix_k3plus;
    calib_opt.use_intrinsic_guess = cfg.calib_use_intrinsic_guess;
    std::cout << "[calib] bootstrap B=" << cfg.calib_B
              << " R=" << cfg.calib_R << "\n";
    const auto boot = ecal::core::calibrateCheckerboardBootstrap(
        objpoints, imgpoints, image_size, calib_opt, cfg.calib_B, cfg.calib_R);
    if (boot.success) {
      std::cout << "[calib] success runs=" << boot.used_runs << "\n";
      std::cout << "[calib] reproj_mean=" << boot.reproj_mean
                << " std=" << boot.reproj_std << "\n";
      ecal::io::saveCalibrationYaml(
          cfg.out_dir, used_windows, total_windows, board_rows, board_cols,
          cfg.square_size, cfg.calib_B, cfg.calib_R, boot.used_runs,
          boot.K_mean, boot.K_std, boot.dist_mean, boot.dist_std,
          boot.reproj_mean, boot.reproj_std);
      if (boot.best.success) {
        std::vector<std::vector<cv::Point3f>> best_obj;
        std::vector<std::vector<cv::Point2f>> best_img;
        if (!boot.best_indices.empty()) {
          best_obj.reserve(boot.best_indices.size());
          best_img.reserve(boot.best_indices.size());
          for (int idx : boot.best_indices) {
            if (idx < 0 ||
                static_cast<size_t>(idx) >= objpoints.size()) {
              continue;
            }
            best_obj.push_back(objpoints[static_cast<size_t>(idx)]);
            best_img.push_back(imgpoints[static_cast<size_t>(idx)]);
          }
        } else {
          best_obj = objpoints;
          best_img = imgpoints;
        }
        ecal::viz::saveCalibrationReportImages(
            cfg.out_dir, image_size, boot.best.camera_matrix,
            boot.best.dist_coeffs, boot.best.rvecs, boot.best.tvecs, best_obj,
            best_img);
      }
    } else {
      std::cout << "[calib] failed\n";
    }
  } else {
    std::cout << "[calib] no valid views\n";
  }

  return 0;
}
