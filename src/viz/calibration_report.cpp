#include "ecal/viz/calibration_report.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace ecal::viz {

static cv::Scalar colorFromIndex(int idx, int total) {
  if (total <= 1) {
    return cv::Scalar(0, 255, 255);
  }
  const double t = static_cast<double>(idx) / (total - 1);
  const int h = static_cast<int>(t * 179.0);
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, 255, 255));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const cv::Vec3b c = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(c[0], c[1], c[2]);
}

static cv::Mat drawPoseProjections(const std::vector<cv::Mat> &rvecs,
                                   const std::vector<cv::Mat> &tvecs) {
  const int n = static_cast<int>(tvecs.size());
  const int w = 900;
  const int h = 300;
  cv::Mat canvas(h, w, CV_8UC3, cv::Scalar(20, 20, 20));
  const int panel_w = w / 3;
  const int panel_h = h;

  if (n == 0) {
    return canvas;
  }

  std::vector<cv::Point3d> tv(n);
  for (int i = 0; i < n; ++i) {
    cv::Mat t64;
    tvecs[i].convertTo(t64, CV_64F);
    tv[i] =
        cv::Point3d(t64.at<double>(0), t64.at<double>(1), t64.at<double>(2));
  }
  cv::Point3d mn = tv[0];
  cv::Point3d mx = tv[0];
  for (const auto &p : tv) {
    mn.x = std::min(mn.x, p.x);
    mn.y = std::min(mn.y, p.y);
    mn.z = std::min(mn.z, p.z);
    mx.x = std::max(mx.x, p.x);
    mx.y = std::max(mx.y, p.y);
    mx.z = std::max(mx.z, p.z);
  }
  const cv::Point3d diff(std::max(1e-9, mx.x - mn.x),
                         std::max(1e-9, mx.y - mn.y),
                         std::max(1e-9, mx.z - mn.z));

  auto norm = [&](const cv::Point3d &p) {
    return cv::Point3d((p.x - mn.x) / diff.x, (p.y - mn.y) / diff.y,
                       (p.z - mn.z) / diff.z);
  };

  auto draw_panel = [&](int panel, int ax0, int ax1, const std::string &label) {
    const int x0 = panel * panel_w;
    cv::rectangle(canvas, cv::Rect(x0, 0, panel_w, panel_h),
                  cv::Scalar(60, 60, 60), 1);
    cv::putText(canvas, label, cv::Point(x0 + 8, 18), cv::FONT_HERSHEY_SIMPLEX,
                0.5, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);

    cv::Point prev(-1, -1);
    for (int i = 0; i < n; ++i) {
      const cv::Point3d p = norm(tv[i]);
      const double v0 = (ax0 == 0) ? p.x : (ax0 == 1 ? p.y : p.z);
      const double v1 = (ax1 == 0) ? p.x : (ax1 == 1 ? p.y : p.z);
      const int px = x0 + static_cast<int>(20 + v0 * (panel_w - 40));
      const int py = static_cast<int>(panel_h - 20 - v1 * (panel_h - 40));
      const cv::Point pt(px, py);
      if (prev.x >= 0) {
        cv::line(canvas, prev, pt, cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
      }
      prev = pt;
      cv::circle(canvas, pt, 3, colorFromIndex(i, n), -1, cv::LINE_AA);

      if (i < static_cast<int>(rvecs.size())) {
        cv::Mat r64, R;
        rvecs[i].convertTo(r64, CV_64F);
        cv::Rodrigues(r64, R);
        const double axis_len = 0.1;
        cv::Point3d ax(R.at<double>(0, 0) * axis_len,
                       R.at<double>(1, 0) * axis_len,
                       R.at<double>(2, 0) * axis_len);
        cv::Point3d ay(R.at<double>(0, 1) * axis_len,
                       R.at<double>(1, 1) * axis_len,
                       R.at<double>(2, 1) * axis_len);
        cv::Point3d az(R.at<double>(0, 2) * axis_len,
                       R.at<double>(1, 2) * axis_len,
                       R.at<double>(2, 2) * axis_len);
        const cv::Point3d pn = norm(tv[i]);
        auto proj = [&](const cv::Point3d &v) {
          const double q0 = (ax0 == 0) ? v.x : (ax0 == 1 ? v.y : v.z);
          const double q1 = (ax1 == 0) ? v.x : (ax1 == 1 ? v.y : v.z);
          const int ex = x0 + static_cast<int>(20 + q0 * (panel_w - 40));
          const int ey = static_cast<int>(panel_h - 20 - q1 * (panel_h - 40));
          return cv::Point(ex, ey);
        };
        const cv::Point origin = proj(pn);
        cv::line(canvas, origin, proj(pn + ax), cv::Scalar(0, 0, 255), 1);
        cv::line(canvas, origin, proj(pn + ay), cv::Scalar(0, 255, 0), 1);
        cv::line(canvas, origin, proj(pn + az), cv::Scalar(255, 0, 0), 1);
      }
    }
  };

  draw_panel(0, 0, 1, "XY");
  draw_panel(1, 0, 2, "XZ");
  draw_panel(2, 1, 2, "YZ");
  return canvas;
}

static cv::Mat
drawReprojectionReport(const cv::Size &image_size, const cv::Mat &K,
                       const cv::Mat &dist, const std::vector<cv::Mat> &rvecs,
                       const std::vector<cv::Mat> &tvecs,
                       const std::vector<std::vector<cv::Point3f>> &objpoints,
                       const std::vector<std::vector<cv::Point2f>> &imgpoints) {
  const int panel_w = 600;
  const int panel_h = 500;
  const int w = panel_w * 2;
  const int h = panel_h;
  cv::Mat canvas(h, w, CV_8UC3, cv::Scalar(20, 20, 20));

  cv::Rect left(0, 0, panel_w, panel_h);
  cv::Rect right(panel_w, 0, panel_w, panel_h);
  cv::rectangle(canvas, left, cv::Scalar(60, 60, 60), 1);
  cv::rectangle(canvas, right, cv::Scalar(60, 60, 60), 1);

  const int n_imgs = static_cast<int>(imgpoints.size());
  std::vector<double> all_dx;
  std::vector<double> all_dy;
  std::vector<int> all_idx;

  for (int i = 0; i < n_imgs; ++i) {
    std::vector<cv::Point2f> proj;
    cv::projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist, proj);
    const cv::Scalar col = colorFromIndex(i, n_imgs);
    for (size_t k = 0; k < proj.size(); ++k) {
      const cv::Point2f g = imgpoints[i][k];
      const cv::Point2f p = proj[k];
      const int gx = left.x + static_cast<int>(
                                  g.x / image_size.width * (panel_w - 40) + 20);
      const int gy =
          left.y +
          static_cast<int>(g.y / image_size.height * (panel_h - 40) + 20);
      const int px = left.x + static_cast<int>(
                                  p.x / image_size.width * (panel_w - 40) + 20);
      const int py =
          left.y +
          static_cast<int>(p.y / image_size.height * (panel_h - 40) + 20);
      cv::line(canvas, cv::Point(gx, gy), cv::Point(px, py), col, 1,
               cv::LINE_AA);
      cv::circle(canvas, cv::Point(gx, gy), 2, col, -1, cv::LINE_AA);
      cv::circle(canvas, cv::Point(px, py), 2, col, 1, cv::LINE_AA);

      all_dx.push_back(p.x - g.x);
      all_dy.push_back(p.y - g.y);
      all_idx.push_back(i);
    }
  }

  double max_abs = 1.0;
  for (size_t i = 0; i < all_dx.size(); ++i) {
    max_abs =
        std::max(max_abs, std::max(std::abs(all_dx[i]), std::abs(all_dy[i])));
  }
  const double scale = (panel_h - 60) / (2.0 * max_abs);
  const cv::Point center(right.x + panel_w / 2, right.y + panel_h / 2);

  cv::line(canvas, cv::Point(right.x + 20, center.y),
           cv::Point(right.x + panel_w - 20, center.y), cv::Scalar(80, 80, 80),
           1);
  cv::line(canvas, cv::Point(center.x, right.y + 20),
           cv::Point(center.x, right.y + panel_h - 20), cv::Scalar(80, 80, 80),
           1);

  for (size_t i = 0; i < all_dx.size(); ++i) {
    const int px = static_cast<int>(center.x + all_dx[i] * scale);
    const int py = static_cast<int>(center.y - all_dy[i] * scale);
    cv::circle(canvas, cv::Point(px, py), 2, colorFromIndex(all_idx[i], n_imgs),
               -1, cv::LINE_AA);
  }

  cv::putText(canvas, "Reprojection lines", cv::Point(left.x + 10, left.y + 20),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(220, 220, 220), 1);
  cv::putText(canvas, "Error scatter (px)",
              cv::Point(right.x + 10, right.y + 20), cv::FONT_HERSHEY_SIMPLEX,
              0.5, cv::Scalar(220, 220, 220), 1);

  return canvas;
}

void saveCalibrationReportImages(
    const std::string &out_dir, const cv::Size &image_size, const cv::Mat &K,
    const cv::Mat &dist, const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs,
    const std::vector<std::vector<cv::Point3f>> &objpoints,
    const std::vector<std::vector<cv::Point2f>> &imgpoints) {
  if (out_dir.empty()) {
    return;
  }
  std::filesystem::create_directories(out_dir);

  cv::Mat poses = drawPoseProjections(rvecs, tvecs);
  cv::Mat reproj = drawReprojectionReport(image_size, K, dist, rvecs, tvecs,
                                          objpoints, imgpoints);

  const std::string poses_path =
      (std::filesystem::path(out_dir) / "calibration_poses.png").string();
  const std::string reproj_path =
      (std::filesystem::path(out_dir) / "calibration_reprojection.png")
          .string();

  cv::imwrite(poses_path, poses);
  cv::imwrite(reproj_path, reproj);
}

} // namespace ecal::viz
