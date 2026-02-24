#include "ecal/core/checkerboard_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>

#include <opencv2/calib3d.hpp>

namespace ecal::core {

double
computeReprojectionError(const std::vector<std::vector<cv::Point3f>> &objpoints,
                         const std::vector<std::vector<cv::Point2f>> &imgpoints,
                         const cv::Mat &K, const cv::Mat &dist,
                         const std::vector<cv::Mat> &rvecs,
                         const std::vector<cv::Mat> &tvecs) {
  double total_err = 0.0;
  size_t total_pts = 0;
  std::vector<cv::Point2f> proj;
  for (size_t i = 0; i < objpoints.size(); ++i) {
    cv::projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist, proj);
    double err = 0.0;
    for (size_t j = 0; j < proj.size(); ++j) {
      const cv::Point2f d = proj[j] - imgpoints[i][j];
      err += std::sqrt(d.dot(d));
    }
    total_err += err;
    total_pts += proj.size();
  }
  return (total_pts > 0) ? (total_err / total_pts) : 0.0;
}

static int buildFlags(const CalibrationOptions &opt) {
  int flags = 0;
  if (opt.fix_k3plus) {
    flags |= cv::CALIB_FIX_K3;
    flags |= cv::CALIB_FIX_K4;
    flags |= cv::CALIB_FIX_K5;
    flags |= cv::CALIB_FIX_K6;
  }
  return flags;
}

CalibrationResult
calibrateCheckerboard(const std::vector<std::vector<cv::Point3f>> &objpoints,
                      const std::vector<std::vector<cv::Point2f>> &imgpoints,
                      const cv::Size &image_size,
                      const CalibrationOptions &opt) {
  CalibrationResult out;
  if (objpoints.empty() || imgpoints.empty()) {
    return out;
  }
  const int flags_base = buildFlags(opt);
  const cv::TermCriteria criteria(cv::TermCriteria::EPS +
                                      cv::TermCriteria::MAX_ITER,
                                  std::max(1, opt.max_iter), opt.eps);

  cv::Mat K, dist;
  std::vector<cv::Mat> rvecs, tvecs;

  double rms = cv::calibrateCamera(objpoints, imgpoints, image_size, K, dist,
                                   rvecs, tvecs, flags_base, criteria);

  if (opt.use_intrinsic_guess) {
    const int flags2 = flags_base | cv::CALIB_USE_INTRINSIC_GUESS;
    rms = cv::calibrateCamera(objpoints, imgpoints, image_size, K, dist, rvecs,
                              tvecs, flags2, criteria);
  }

  out.success = std::isfinite(rms) && rms > 0.0;
  out.camera_matrix = K;
  out.dist_coeffs = dist;
  out.rvecs = rvecs;
  out.tvecs = tvecs;
  out.reprojection_error =
      computeReprojectionError(objpoints, imgpoints, K, dist, rvecs, tvecs);
  return out;
}

static cv::Mat toRow64(const cv::Mat &m) {
  cv::Mat md;
  m.convertTo(md, CV_64F);
  return md.reshape(1, 1);
}

CalibrationBootstrapResult calibrateCheckerboardBootstrap(
    const std::vector<std::vector<cv::Point3f>> &objpoints,
    const std::vector<std::vector<cv::Point2f>> &imgpoints,
    const cv::Size &image_size, const CalibrationOptions &opt, int calib_B,
    int calib_R) {
  CalibrationBootstrapResult out;
  const size_t M = imgpoints.size();
  if (M == 0) {
    return out;
  }
  int B = std::max(1, calib_B);
  int R = std::max(1, calib_R);

  if (M <= static_cast<size_t>(B) || R <= 1) {
    const auto calib =
        calibrateCheckerboard(objpoints, imgpoints, image_size, opt);
    if (!calib.success) {
      return out;
    }
    out.success = true;
    out.used_runs = 1;
    out.best = calib;
    out.best_indices.resize(M);
    std::iota(out.best_indices.begin(), out.best_indices.end(), 0);
    out.K_mean = calib.camera_matrix.clone();
    out.dist_mean = toRow64(calib.dist_coeffs);
    out.K_std = cv::Mat::zeros(out.K_mean.size(), CV_64F);
    out.dist_std = cv::Mat::zeros(out.dist_mean.size(), CV_64F);
    out.reproj_mean = calib.reprojection_error;
    out.reproj_std = 0.0;
    return out;
  }

  B = std::min(B, static_cast<int>(M));
  std::vector<int> indices(M);
  std::iota(indices.begin(), indices.end(), 0);
  std::mt19937 rng(static_cast<unsigned int>(std::random_device{}()));

  std::vector<cv::Mat> Ks;
  std::vector<cv::Mat> dists;
  std::vector<double> errs;

  for (int r = 0; r < R; ++r) {
    std::shuffle(indices.begin(), indices.end(), rng);
    std::vector<std::vector<cv::Point3f>> obj_sub;
    std::vector<std::vector<cv::Point2f>> img_sub;
    std::vector<int> sample_idx;
    obj_sub.reserve(static_cast<size_t>(B));
    img_sub.reserve(static_cast<size_t>(B));
    sample_idx.reserve(static_cast<size_t>(B));
    for (int i = 0; i < B; ++i) {
      const int idx = indices[static_cast<size_t>(i)];
      obj_sub.push_back(objpoints[static_cast<size_t>(idx)]);
      img_sub.push_back(imgpoints[static_cast<size_t>(idx)]);
      sample_idx.push_back(idx);
    }
    const auto calib = calibrateCheckerboard(obj_sub, img_sub, image_size, opt);
    if (!calib.success) {
      continue;
    }
    out.used_runs++;
    Ks.push_back(calib.camera_matrix.clone());
    dists.push_back(toRow64(calib.dist_coeffs));
    errs.push_back(calib.reprojection_error);
    if (!out.best.success ||
        calib.reprojection_error < out.best.reprojection_error) {
      out.best = calib;
      out.best_indices = sample_idx;
    }
  }

  if (out.used_runs == 0) {
    return out;
  }

  out.success = true;

  const cv::Mat K0 = Ks.front();
  const cv::Mat d0 = dists.front();
  cv::Mat K_sum = cv::Mat::zeros(K0.size(), CV_64F);
  cv::Mat d_sum = cv::Mat::zeros(d0.size(), CV_64F);
  for (size_t i = 0; i < Ks.size(); ++i) {
    cv::Mat Kd, dd;
    Ks[i].convertTo(Kd, CV_64F);
    dists[i].convertTo(dd, CV_64F);
    K_sum += Kd;
    d_sum += dd;
  }
  out.K_mean = K_sum / static_cast<double>(out.used_runs);
  out.dist_mean = d_sum / static_cast<double>(out.used_runs);

  if (out.used_runs > 1) {
    cv::Mat K_var = cv::Mat::zeros(out.K_mean.size(), CV_64F);
    cv::Mat d_var = cv::Mat::zeros(out.dist_mean.size(), CV_64F);
    for (size_t i = 0; i < Ks.size(); ++i) {
      cv::Mat Kd, dd;
      Ks[i].convertTo(Kd, CV_64F);
      dists[i].convertTo(dd, CV_64F);
      cv::Mat Kdiff = Kd - out.K_mean;
      cv::Mat ddiff = dd - out.dist_mean;
      K_var += Kdiff.mul(Kdiff);
      d_var += ddiff.mul(ddiff);
    }
    K_var /= static_cast<double>(out.used_runs);
    d_var /= static_cast<double>(out.used_runs);
    cv::sqrt(K_var, out.K_std);
    cv::sqrt(d_var, out.dist_std);
  } else {
    out.K_std = cv::Mat::zeros(out.K_mean.size(), CV_64F);
    out.dist_std = cv::Mat::zeros(out.dist_mean.size(), CV_64F);
  }

  double err_sum = 0.0;
  for (double e : errs) {
    err_sum += e;
  }
  out.reproj_mean = err_sum / static_cast<double>(out.used_runs);
  if (out.used_runs > 1) {
    double var = 0.0;
    for (double e : errs) {
      const double d = e - out.reproj_mean;
      var += d * d;
    }
    out.reproj_std = std::sqrt(var / static_cast<double>(out.used_runs));
  } else {
    out.reproj_std = 0.0;
  }

  return out;
}

} // namespace ecal::core
