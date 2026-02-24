#include "ecal/viz/calibration_viz.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ecal/core/checkerboard_validity.hpp"
#include "ecal/viz/event_render.hpp"

namespace ecal::viz {

static cv::Mat normalizeToU8(const cv::Mat &m) {
  CV_Assert(m.type() == CV_32F);
  cv::Mat norm;
  cv::normalize(m, norm, 0, 255, cv::NORM_MINMAX);
  cv::Mat u8;
  norm.convertTo(u8, CV_8U);
  return u8;
}

static cv::Mat normalizeSignedToU8(const cv::Mat &m) {
  CV_Assert(m.type() == CV_32F);
  double mn = 0.0;
  double mx = 0.0;
  cv::minMaxLoc(m, &mn, &mx);
  const double max_abs = std::max(std::abs(mn), std::abs(mx));
  if (max_abs <= 1e-12) {
    return cv::Mat(m.size(), CV_8U, cv::Scalar(128));
  }
  cv::Mat out(m.size(), CV_8U);
  for (int y = 0; y < m.rows; ++y) {
    const float *src = m.ptr<float>(y);
    uint8_t *dst = out.ptr<uint8_t>(y);
    for (int x = 0; x < m.cols; ++x) {
      const double v = static_cast<double>(src[x]);
      const double u = 128.0 + 127.0 * (v / max_abs);
      const double uc = std::min(255.0, std::max(0.0, u));
      dst[x] = static_cast<uint8_t>(uc + 0.5);
    }
  }
  return out;
}

static cv::Mat toBgr(const cv::Mat &m) {
  if (m.empty()) {
    return cv::Mat();
  }
  cv::Mat out;
  if (m.channels() == 1) {
    cv::cvtColor(m, out, cv::COLOR_GRAY2BGR);
  } else {
    out = m.clone();
  }
  return out;
}

WindowVis buildWindowVis(const std::vector<ecal::core::TimedEventNs> &events,
                         int width, int height, const cv::Mat &iwe,
                         const cv::Mat &piwe,
                         const std::vector<cv::Point> &patch_points,
                         const std::vector<ecal::core::PatchBox> &boxes,
                         const std::vector<cv::Point2f> &init_corners,
                         const std::vector<cv::Point2f> &refined_corners,
                         const std::vector<cv::Point2f> &filtered_corners,
                         float vx, float vy, double window_dt_s,
                         int board_rows, int board_cols, float zoom_factor) {
  WindowVis out;

  const float vis_zoom = (zoom_factor > 0.0f) ? zoom_factor : 1.0f;
  const auto scaledSize = [&](int base) {
    return std::max(1, static_cast<int>(std::lround(base * vis_zoom)));
  };
  const auto scaledRadius = [&](float base) {
    return std::max(1, static_cast<int>(std::lround(base * vis_zoom)));
  };

  out.raw_vis =
      ecal::viz::eventsToImage(events, width, height, vis_zoom, true, 50);

  if (!out.raw_vis.empty()) {
    const cv::Point center(out.raw_vis.cols / 2, out.raw_vis.rows / 2);
    const double dx = static_cast<double>(vx) * window_dt_s * vis_zoom;
    const double dy = static_cast<double>(vy) * window_dt_s * vis_zoom;
    const double norm = std::sqrt(dx * dx + dy * dy);
    if (std::isfinite(norm) && norm > 1e-6) {
      const cv::Point tip(static_cast<int>(std::round(center.x + dx)),
                          static_cast<int>(std::round(center.y + dy)));
      cv::arrowedLine(out.raw_vis, center, tip, cv::Scalar(0, 255, 0), 2,
                      cv::LINE_AA, 0, 0.25);
      const std::string label =
          "v=(" + std::to_string(vx) + "," + std::to_string(vy) + ") px/s";
      cv::putText(out.raw_vis, label, cv::Point(10, 20),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1,
                  cv::LINE_AA);
    }
  }

  if (iwe.empty() || piwe.empty()) {
    return out;
  }

  const cv::Mat iwe_u8 = normalizeToU8(iwe);
  const cv::Mat piwe_u8 = normalizeSignedToU8(piwe);

  cv::resize(iwe_u8, out.iwe_vis,
             cv::Size(scaledSize(iwe_u8.cols), scaledSize(iwe_u8.rows)), 0, 0,
             cv::INTER_NEAREST);
  cv::resize(piwe_u8, out.piwe_vis,
             cv::Size(scaledSize(piwe_u8.cols), scaledSize(piwe_u8.rows)), 0, 0,
             cv::INTER_NEAREST);

  out.iwe_vis = toBgr(out.iwe_vis);
  out.piwe_vis = toBgr(out.piwe_vis);

  ecal::core::PatchExtractor::drawPatchPoints(
      out.piwe_vis, patch_points, vis_zoom, cv::Scalar(0, 255, 0), 1, -1);
  ecal::core::PatchExtractor::drawPatchBoxes(out.piwe_vis, boxes, vis_zoom,
                                             cv::Scalar(255, 0, 255), 2);

  const int corner_outline = std::max(1, static_cast<int>(std::lround(vis_zoom)));
  const int init_radius = scaledRadius(3.0f);
  const int refined_radius = scaledRadius(4.0f);
  const int filtered_radius = scaledRadius(4.0f);

  for (const auto &c : init_corners) {
    const cv::Point cc(static_cast<int>(std::lround(c.x * vis_zoom)),
                       static_cast<int>(std::lround(c.y * vis_zoom)));
    cv::circle(out.iwe_vis, cc, init_radius, cv::Scalar(0, 255, 255), -1,
               cv::LINE_AA);
  }
  for (const auto &c : refined_corners) {
    const cv::Point cc(static_cast<int>(std::lround(c.x * vis_zoom)),
                       static_cast<int>(std::lround(c.y * vis_zoom)));
    cv::circle(out.iwe_vis, cc, refined_radius, cv::Scalar(0, 0, 255), -1,
               cv::LINE_AA);
  }
  for (const auto &c : filtered_corners) {
    const cv::Point cc(static_cast<int>(std::lround(c.x * vis_zoom)),
                       static_cast<int>(std::lround(c.y * vis_zoom)));
    cv::circle(out.iwe_vis, cc, filtered_radius, cv::Scalar(255, 255, 0), -1,
               cv::LINE_AA);
    cv::circle(out.iwe_vis, cc, filtered_radius, cv::Scalar(0, 0, 0),
               corner_outline, cv::LINE_AA);
  }

  if (!filtered_corners.empty() && board_rows > 0 && board_cols > 0) {
    const int corner_radius = scaledRadius(3.0f);
    const int line_thickness = std::max(1, static_cast<int>(std::lround(vis_zoom)));
    if (std::abs(vis_zoom - 1.0f) > 1e-6f) {
      cv::Mat corners_base;
      cv::resize(iwe_u8, corners_base,
                 cv::Size(scaledSize(iwe_u8.cols), scaledSize(iwe_u8.rows)), 0,
                 0, cv::INTER_NEAREST);
      std::vector<cv::Point2f> scaled;
      scaled.reserve(filtered_corners.size());
      for (const auto &p : filtered_corners) {
        scaled.emplace_back(p.x * vis_zoom, p.y * vis_zoom);
      }
      out.corners_vis = ecal::core::drawCheckerboardRowSnake(
          corners_base, scaled, board_rows, board_cols, corner_radius, true,
          line_thickness);
    } else {
      out.corners_vis = ecal::core::drawCheckerboardRowSnake(
          iwe_u8, filtered_corners, board_rows, board_cols, corner_radius, true,
          line_thickness);
    }
  }

  return out;
}

void showWindowVis(const WindowVis &vis) {
  if (!vis.raw_vis.empty()) {
    cv::imshow("cm_window_events", vis.raw_vis);
  }
  if (!vis.piwe_vis.empty()) {
    cv::imshow("piwe_u8", vis.piwe_vis);
  }
  if (!vis.iwe_vis.empty()) {
    cv::imshow("iwe_u8", vis.iwe_vis);
  }
  if (!vis.corners_vis.empty()) {
    cv::imshow("corners_matched", vis.corners_vis);
  }
}

} // namespace ecal::viz
