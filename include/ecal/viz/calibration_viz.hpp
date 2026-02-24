#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "ecal/core/patch_extractor.hpp"
#include "ecal/core/simple_event.hpp"

namespace ecal::viz {

struct WindowVis {
  cv::Mat raw_vis;
  cv::Mat iwe_vis;
  cv::Mat piwe_vis;
  cv::Mat corners_vis;
};

WindowVis buildWindowVis(const std::vector<ecal::core::TimedEventNs> &events,
                         int width, int height, const cv::Mat &iwe,
                         const cv::Mat &piwe,
                         const std::vector<cv::Point> &patch_points,
                         const std::vector<ecal::core::PatchBox> &boxes,
                         const std::vector<cv::Point2f> &init_corners,
                         const std::vector<cv::Point2f> &refined_corners,
                         const std::vector<cv::Point2f> &filtered_corners,
                         float vx, float vy, double window_dt_s,
                         int board_rows, int board_cols,
                         float zoom_factor = 2.0f);

void showWindowVis(const WindowVis &vis);

} // namespace ecal::viz
