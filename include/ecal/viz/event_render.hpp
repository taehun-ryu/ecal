#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "ecal/core/simple_event.hpp"

namespace ecal::viz {

cv::Mat eventsToImage(const std::vector<ecal::core::TimedEventNs> &events,
                      int width, int height, float zoom_factor = 1.0f,
                      bool non_max_suppression = false, int add_value = 50);

} // namespace ecal::viz
