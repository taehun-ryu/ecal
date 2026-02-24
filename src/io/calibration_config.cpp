#include "ecal/io/calibration_config.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

namespace ecal::io {

static inline std::string trim(const std::string &s) {
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {
    return "";
  }
  size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

bool loadCalibrationConfig(const std::string &path, CalibrationConfig &cfg,
                           std::string *err) {
  std::ifstream ifs(path);
  if (!ifs) {
    if (err) {
      *err = "Failed to open config: " + path;
    }
    return false;
  }

  std::unordered_map<std::string, std::string> kv;
  std::string line;
  while (std::getline(ifs, line)) {
    const size_t hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    const size_t colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    std::string key = trim(line.substr(0, colon));
    std::string val = trim(line.substr(colon + 1));
    if (!val.empty() && val.front() == '\"' && val.back() == '\"') {
      val = val.substr(1, val.size() - 2);
    }
    kv[key] = val;
  }

  auto has = [&](const char *k) { return kv.find(k) != kv.end(); };
  auto get = [&](const char *k) { return kv[k]; };

  if (has("h5_path"))
    cfg.h5_path = get("h5_path");
  if (has("out_dir"))
    cfg.out_dir = get("out_dir");
  if (has("width"))
    cfg.width = std::stoi(get("width"));
  if (has("height"))
    cfg.height = std::stoi(get("height"));
  if (has("window_sec"))
    cfg.window_sec = std::stod(get("window_sec"));
  if (has("min_events"))
    cfg.min_events = static_cast<size_t>(std::stoll(get("min_events")));
  if (has("max_events"))
    cfg.max_events = static_cast<size_t>(std::stoll(get("max_events")));
  if (has("min_window_sec"))
    cfg.min_window_sec = std::stod(get("min_window_sec"));
  if (has("max_window_sec"))
    cfg.max_window_sec = std::stod(get("max_window_sec"));

  if (has("expected_corners"))
    cfg.expected_corners = std::stoi(get("expected_corners"));
  if (has("board_w"))
    cfg.board_w = std::stoi(get("board_w"));
  if (has("board_h"))
    cfg.board_h = std::stoi(get("board_h"));
  if (has("square_size"))
    cfg.square_size = std::stof(get("square_size"));
  if (has("viz_zoom"))
    cfg.viz_zoom = std::stof(get("viz_zoom"));
  if (has("pp_radius"))
    cfg.pp_radius = std::stoi(get("pp_radius"));
  if (has("use_variance"))
    cfg.use_variance = (get("use_variance") == "true");
  if (has("cm_iwe.sigma"))
    cfg.cm_iwe_sigma = std::stof(get("cm_iwe.sigma"));
  if (has("cm_iwe.cutoff_factor"))
    cfg.cm_iwe_cutoff_factor = std::stof(get("cm_iwe.cutoff_factor"));
  if (has("cm_iwe.patch_radius_override"))
    cfg.cm_iwe_patch_radius_override =
        std::stoi(get("cm_iwe.patch_radius_override"));
  if (has("tracker.num_threads"))
    cfg.tracker_num_threads = std::stoi(get("tracker.num_threads"));
  if (has("tracker.max_iterations"))
    cfg.tracker_max_iterations = std::stoi(get("tracker.max_iterations"));
  if (has("checkerboard_validity.tor_spacing"))
    cfg.checkerboard_tor_spacing =
        std::stof(get("checkerboard_validity.tor_spacing"));
  if (has("checkerboard_validity.tor_orth"))
    cfg.checkerboard_tor_orth =
        std::stof(get("checkerboard_validity.tor_orth"));
  if (has("corner_refine.max_iter"))
    cfg.corner_refine_max_iter = std::stoi(get("corner_refine.max_iter"));
  if (has("corner_refine.lr"))
    cfg.corner_refine_lr = std::stof(get("corner_refine.lr"));
  if (has("corner_refine.strip_half_width0"))
    cfg.corner_refine_strip_half_width0 =
        std::stof(get("corner_refine.strip_half_width0"));
  if (has("corner_refine.strip_half_width1"))
    cfg.corner_refine_strip_half_width1 =
        std::stof(get("corner_refine.strip_half_width1"));
  if (has("calib_max_iter"))
    cfg.calib_max_iter = std::stoi(get("calib_max_iter"));
  if (has("calib_fix_k3plus"))
    cfg.calib_fix_k3plus = (get("calib_fix_k3plus") == "true");
  if (has("calib_use_intrinsic_guess"))
    cfg.calib_use_intrinsic_guess =
        (get("calib_use_intrinsic_guess") == "true");
  if (has("calib_B")) cfg.calib_B = std::stoi(get("calib_B"));
  if (has("calib_R")) cfg.calib_R = std::stoi(get("calib_R"));

  if (cfg.h5_path.empty()) {
    if (err)
      *err = "h5_path is empty in config";
    return false;
  }
  if (cfg.width <= 0 || cfg.height <= 0) {
    if (err)
      *err = "width/height must be > 0";
    return false;
  }
  if (cfg.window_sec <= 0.0) {
    if (err)
      *err = "window_sec must be > 0";
    return false;
  }
  if (cfg.min_events == 0 || cfg.max_events == 0 ||
      cfg.min_events > cfg.max_events) {
    if (err)
      *err = "min_events/max_events invalid";
    return false;
  }
  if (cfg.min_window_sec <= 0.0 || cfg.max_window_sec <= 0.0 ||
      cfg.min_window_sec > cfg.max_window_sec) {
    if (err)
      *err = "min_window_sec/max_window_sec invalid";
    return false;
  }

  if (cfg.expected_corners < 0 && cfg.board_w > 0 && cfg.board_h > 0) {
    cfg.expected_corners = cfg.board_w * cfg.board_h;
  }

  return true;
}

} // namespace ecal::io
