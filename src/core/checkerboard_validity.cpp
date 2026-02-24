#include "ecal/core/checkerboard_validity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace ecal::core {

static void kmeans1d(const std::vector<double> &x, int k,
                     std::vector<int> *labels_out,
                     std::vector<double> *centers_out) {
  const int n = static_cast<int>(x.size());
  labels_out->assign(n, 0);
  centers_out->assign(k, 0.0);
  if (k <= 1) {
    double mean = 0.0;
    for (double v : x)
      mean += v;
    mean /= std::max(1, n);
    (*centers_out)[0] = mean;
    return;
  }

  std::vector<double> centers(k, 0.0);
  for (int i = 0; i < k; ++i) {
    const double q = (i + 1.0) / (k + 1.0);
    const int idx = static_cast<int>(std::round(q * (n - 1)));
    centers[i] = x[std::max(0, std::min(n - 1, idx))];
  }

  for (int it = 0; it < 60; ++it) {
    bool changed = false;
    for (int i = 0; i < n; ++i) {
      double best = std::numeric_limits<double>::infinity();
      int best_k = 0;
      for (int j = 0; j < k; ++j) {
        const double d = std::abs(x[i] - centers[j]);
        if (d < best) {
          best = d;
          best_k = j;
        }
      }
      if ((*labels_out)[i] != best_k) {
        (*labels_out)[i] = best_k;
        changed = true;
      }
    }
    std::vector<double> new_centers(k, 0.0);
    std::vector<int> counts(k, 0);
    for (int i = 0; i < n; ++i) {
      new_centers[(*labels_out)[i]] += x[i];
      counts[(*labels_out)[i]]++;
    }
    for (int j = 0; j < k; ++j) {
      if (counts[j] > 0) {
        new_centers[j] /= counts[j];
      } else {
        new_centers[j] = centers[j];
      }
    }
    if (!changed) {
      centers = new_centers;
      break;
    }
    centers = new_centers;
  }

  *centers_out = centers;
}

static float scoreGrid(const std::vector<cv::Point2f> &ordered, int rows,
                       int cols) {
  const int N = rows * cols;
  if (static_cast<int>(ordered.size()) != N) {
    return std::numeric_limits<float>::infinity();
  }
  std::vector<cv::Point2f> grid = ordered;
  auto idx = [&](int r, int c) { return r * cols + c; };

  std::vector<float> row_d;
  std::vector<float> col_d;
  row_d.reserve(rows * (cols - 1));
  col_d.reserve((rows - 1) * cols);

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols - 1; ++c) {
      cv::Point2f v = grid[idx(r, c + 1)] - grid[idx(r, c)];
      row_d.push_back(std::sqrt(v.dot(v)));
    }
  }
  for (int r = 0; r < rows - 1; ++r) {
    for (int c = 0; c < cols; ++c) {
      cv::Point2f v = grid[idx(r + 1, c)] - grid[idx(r, c)];
      col_d.push_back(std::sqrt(v.dot(v)));
    }
  }

  auto mean_std = [](const std::vector<float> &v) {
    double mean = 0.0;
    for (float x : v)
      mean += x;
    mean /= std::max<size_t>(1, v.size());
    double var = 0.0;
    for (float x : v) {
      const double d = x - mean;
      var += d * d;
    }
    var /= std::max<size_t>(1, v.size());
    return std::pair<double, double>(mean, std::sqrt(var));
  };

  auto [rm, rs] = mean_std(row_d);
  auto [cm, cs] = mean_std(col_d);
  const float rcv = static_cast<float>(rs / (rm + 1e-9));
  const float ccv = static_cast<float>(cs / (cm + 1e-9));

  std::vector<float> ortho;
  for (int r = 0; r < rows - 1; ++r) {
    for (int c = 0; c < cols - 1; ++c) {
      cv::Point2f dx = grid[idx(r, c + 1)] - grid[idx(r, c)];
      cv::Point2f dy = grid[idx(r + 1, c)] - grid[idx(r, c)];
      const float ndx = std::sqrt(dx.dot(dx));
      const float ndy = std::sqrt(dy.dot(dy));
      if (ndx < 1e-9f || ndy < 1e-9f) {
        continue;
      }
      const float cosang = std::abs(dx.dot(dy) / (ndx * ndy));
      ortho.push_back(cosang);
    }
  }
  float oerr = 1.0f;
  if (!ortho.empty()) {
    double sum = 0.0;
    for (float v : ortho)
      sum += v;
    oerr = static_cast<float>(sum / ortho.size());
  }
  return rcv + ccv + oerr;
}

static std::vector<cv::Point2f>
reshapeGrid(const std::vector<cv::Point2f> &ordered, int rows, int cols) {
  std::vector<cv::Point2f> out = ordered;
  if (rows <= 0 || cols <= 0) {
    return out;
  }
  // enforce left->right, top->bottom
  if (out[0].x > out[cols - 1].x) {
    for (int r = 0; r < rows; ++r) {
      std::reverse(out.begin() + r * cols, out.begin() + (r + 1) * cols);
    }
  }
  if (out[0].y > out[(rows - 1) * cols].y) {
    for (int r = 0; r < rows / 2; ++r) {
      for (int c = 0; c < cols; ++c) {
        std::swap(out[r * cols + c], out[(rows - 1 - r) * cols + c]);
      }
    }
  }
  return out;
}

CheckerboardOrderResult
orderCheckerboardCorners(const std::vector<cv::Point2f> &points, int rows,
                         int cols) {
  CheckerboardOrderResult out;
  if (rows <= 0 || cols <= 0) {
    return out;
  }
  const int N = static_cast<int>(points.size());
  const int K = rows * cols;
  if (N < K) {
    return out;
  }

  cv::Point2d mean(0.0, 0.0);
  for (const auto &p : points) {
    mean.x += p.x;
    mean.y += p.y;
  }
  mean.x /= std::max(1, N);
  mean.y /= std::max(1, N);

  cv::Mat X(N, 2, CV_64F);
  for (int i = 0; i < N; ++i) {
    X.at<double>(i, 0) = points[i].x - mean.x;
    X.at<double>(i, 1) = points[i].y - mean.y;
  }
  cv::SVD svd(X, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
  cv::Vec2d u(svd.vt.at<double>(0, 0), svd.vt.at<double>(0, 1));
  cv::Vec2d v(svd.vt.at<double>(1, 0), svd.vt.at<double>(1, 1));
  if (u.dot(cv::Vec2d(1.0, 0.0)) < 0.0)
    u = -u;
  if (v.dot(cv::Vec2d(0.0, 1.0)) < 0.0)
    v = -v;

  auto try_axis = [&](int axis_rows, std::vector<cv::Point2f> *ordered,
                      float *score) -> bool {
    std::vector<double> proj_u(N), proj_v(N);
    for (int i = 0; i < N; ++i) {
      const double x = X.at<double>(i, 0);
      const double y = X.at<double>(i, 1);
      proj_u[i] = x * u[0] + y * u[1];
      proj_v[i] = x * v[0] + y * v[1];
    }

    const std::vector<double> &row_coord = (axis_rows == 1) ? proj_v : proj_u;
    const std::vector<double> &col_coord = (axis_rows == 1) ? proj_u : proj_v;

    std::vector<int> rlab;
    std::vector<double> rcent;
    kmeans1d(row_coord, rows, &rlab, &rcent);

    std::vector<int> row_order(rows);
    std::iota(row_order.begin(), row_order.end(), 0);
    std::sort(row_order.begin(), row_order.end(),
              [&](int a, int b) { return rcent[a] < rcent[b]; });

    std::vector<int> index_map;
    index_map.reserve(K);
    for (int rid : row_order) {
      std::vector<int> idxs;
      for (int i = 0; i < N; ++i) {
        if (rlab[i] == rid)
          idxs.push_back(i);
      }
      if (idxs.empty()) {
        return false;
      }
      std::sort(idxs.begin(), idxs.end(),
                [&](int a, int b) { return col_coord[a] < col_coord[b]; });
      if (static_cast<int>(idxs.size()) > cols) {
        const int mid = static_cast<int>(idxs.size()) / 2;
        const int half = cols / 2;
        int start = std::max(0, mid - half);
        int end = std::min(static_cast<int>(idxs.size()), start + cols);
        start = end - cols;
        idxs = std::vector<int>(idxs.begin() + start, idxs.begin() + end);
      } else if (static_cast<int>(idxs.size()) < cols) {
        return false;
      }
      for (int idx : idxs) {
        index_map.push_back(idx);
      }
    }
    if (static_cast<int>(index_map.size()) != K) {
      return false;
    }

    ordered->resize(K);
    for (int i = 0; i < K; ++i) {
      (*ordered)[i] = points[index_map[i]];
    }
    *ordered = reshapeGrid(*ordered, rows, cols);
    *score = scoreGrid(*ordered, rows, cols);
    return true;
  };

  std::vector<cv::Point2f> cand1, cand2;
  float s1 = std::numeric_limits<float>::infinity();
  float s2 = std::numeric_limits<float>::infinity();
  bool ok1 = try_axis(1, &cand1, &s1);
  bool ok2 = try_axis(0, &cand2, &s2);

  if (!ok1 && !ok2) {
    return out;
  }
  if (!ok2 || (ok1 && s1 <= s2)) {
    out.success = true;
    out.ordered = cand1;
    out.score = s1;
    return out;
  }
  out.success = true;
  out.ordered = cand2;
  out.score = s2;
  return out;
}

bool isCheckerboardValid(const std::vector<cv::Point2f> &ordered, int rows,
                         int cols, float tor_spacing, float tor_orth) {
  const int N = rows * cols;
  if (static_cast<int>(ordered.size()) != N) {
    return false;
  }
  if (rows < 2 || cols < 2) {
    return false;
  }

  std::vector<cv::Point2f> grid = ordered;
  // Build ideal grid coordinates
  std::vector<cv::Point2f> G;
  G.reserve(N);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      G.emplace_back(static_cast<float>(c), static_cast<float>(r));
    }
  }

  cv::Mat H = cv::findHomography(G, grid, 0);
  if (H.empty()) {
    return false;
  }
  cv::Mat Hinv = H.inv();

  std::vector<cv::Point2f> canon;
  canon.reserve(N);
  for (const auto &p : grid) {
    cv::Matx31d v(p.x, p.y, 1.0);
    cv::Matx31d q = cv::Matx33d(Hinv) * v;
    const double w = q(2, 0);
    if (std::abs(w) < 1e-9) {
      return false;
    }
    canon.emplace_back(static_cast<float>(q(0, 0) / w),
                       static_cast<float>(q(1, 0) / w));
  }

  auto idx = [&](int r, int c) { return r * cols + c; };
  std::vector<cv::Point2f> canon_grid = canon;

  // Monotonicity check (and flip if needed)
  bool mono_x = true;
  bool mono_y = true;
  for (int c = 0; c < cols - 1; ++c) {
    if (canon_grid[idx(0, c + 1)].x < canon_grid[idx(0, c)].x - 1e-3f) {
      mono_x = false;
    }
  }
  for (int r = 0; r < rows - 1; ++r) {
    if (canon_grid[idx(r + 1, 0)].y < canon_grid[idx(r, 0)].y - 1e-3f) {
      mono_y = false;
    }
  }
  if (!(mono_x && mono_y)) {
    std::vector<cv::Point2f> flipped = reshapeGrid(canon_grid, rows, cols);
    canon_grid = flipped;
    mono_x = true;
    mono_y = true;
    for (int c = 0; c < cols - 1; ++c) {
      if (canon_grid[idx(0, c + 1)].x < canon_grid[idx(0, c)].x - 1e-3f) {
        mono_x = false;
      }
    }
    for (int r = 0; r < rows - 1; ++r) {
      if (canon_grid[idx(r + 1, 0)].y < canon_grid[idx(r, 0)].y - 1e-3f) {
        mono_y = false;
      }
    }
    if (!(mono_x && mono_y)) {
      return false;
    }
  }

  // Spacing CV
  std::vector<float> row_d;
  std::vector<float> col_d;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols - 1; ++c) {
      cv::Point2f v = canon_grid[idx(r, c + 1)] - canon_grid[idx(r, c)];
      row_d.push_back(std::sqrt(v.dot(v)));
    }
  }
  for (int r = 0; r < rows - 1; ++r) {
    for (int c = 0; c < cols; ++c) {
      cv::Point2f v = canon_grid[idx(r + 1, c)] - canon_grid[idx(r, c)];
      col_d.push_back(std::sqrt(v.dot(v)));
    }
  }
  auto mean_std = [](const std::vector<float> &v) {
    double mean = 0.0;
    for (float x : v)
      mean += x;
    mean /= std::max<size_t>(1, v.size());
    double var = 0.0;
    for (float x : v) {
      const double d = x - mean;
      var += d * d;
    }
    var /= std::max<size_t>(1, v.size());
    return std::pair<double, double>(mean, std::sqrt(var));
  };
  auto [rm, rs] = mean_std(row_d);
  auto [cm, cs] = mean_std(col_d);
  const float row_cv = static_cast<float>(rs / (rm + 1e-6));
  const float col_cv = static_cast<float>(cs / (cm + 1e-6));
  const bool row_ok = row_cv < tor_spacing;
  const bool col_ok = col_cv < tor_spacing;

  // Orthogonality
  std::vector<float> orth;
  for (int r = 0; r < rows - 1; ++r) {
    for (int c = 0; c < cols - 1; ++c) {
      cv::Point2f dx = canon_grid[idx(r, c + 1)] - canon_grid[idx(r, c)];
      cv::Point2f dy = canon_grid[idx(r + 1, c)] - canon_grid[idx(r, c)];
      const float ndx = std::sqrt(dx.dot(dx));
      const float ndy = std::sqrt(dy.dot(dy));
      if (ndx < 1e-8f || ndy < 1e-8f) {
        continue;
      }
      const float cosang = std::abs(dx.dot(dy) / (ndx * ndy));
      orth.push_back(cosang);
    }
  }
  float orth_mean = 1.0f;
  if (!orth.empty()) {
    double sum = 0.0;
    for (float v : orth)
      sum += v;
    orth_mean = static_cast<float>(sum / orth.size());
  }
  const bool orth_ok = orth_mean < tor_orth;

  // Range guard
  float xmin = canon_grid[0].x;
  float xmax = canon_grid[0].x;
  float ymin = canon_grid[0].y;
  float ymax = canon_grid[0].y;
  for (const auto &p : canon_grid) {
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
  }
  const float eps_range = 0.35f;
  const bool range_ok =
      (xmin >= -eps_range && ymin >= -eps_range &&
       xmax <= (cols - 1) + eps_range && ymax <= (rows - 1) + eps_range);

  // Off-lattice distance
  float dx_mean = 0.0f;
  float dy_mean = 0.0f;
  float dx_max = 0.0f;
  float dy_max = 0.0f;
  for (const auto &p : canon_grid) {
    const float dx = std::abs(p.x - std::round(p.x));
    const float dy = std::abs(p.y - std::round(p.y));
    dx_mean += dx;
    dy_mean += dy;
    dx_max = std::max(dx_max, dx);
    dy_max = std::max(dy_max, dy);
  }
  dx_mean /= N;
  dy_mean /= N;
  const bool near_mean_ok = (dx_mean < 0.18f) && (dy_mean < 0.18f);
  const bool near_max_ok = (dx_max < 0.45f) && (dy_max < 0.45f);

  // Linearity (MAD)
  auto mad = [](const std::vector<float> &v) {
    if (v.empty())
      return 0.0f;
    std::vector<float> tmp = v;
    std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
    const float med = tmp[tmp.size() / 2];
    for (float &x : tmp)
      x = std::abs(x - med);
    std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
    return tmp[tmp.size() / 2];
  };
  float max_row_mad = 0.0f;
  float max_col_mad = 0.0f;
  for (int r = 0; r < rows; ++r) {
    std::vector<float> ys;
    ys.reserve(cols);
    for (int c = 0; c < cols; ++c) {
      ys.push_back(canon_grid[idx(r, c)].y);
    }
    max_row_mad = std::max(max_row_mad, mad(ys));
  }
  for (int c = 0; c < cols; ++c) {
    std::vector<float> xs;
    xs.reserve(rows);
    for (int r = 0; r < rows; ++r) {
      xs.push_back(canon_grid[idx(r, c)].x);
    }
    max_col_mad = std::max(max_col_mad, mad(xs));
  }
  const bool linearity_ok = (max_row_mad < 0.20f) && (max_col_mad < 0.20f);

  // Cell orientation consistency
  auto signed_area = [](const cv::Point2f &a, const cv::Point2f &b,
                        const cv::Point2f &c) {
    return 0.5f * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
  };
  float sign0 = 0.0f;
  bool area_ok = true;
  bool sign_set = false;
  for (int r = 0; r < rows - 1; ++r) {
    for (int c = 0; c < cols - 1; ++c) {
      const cv::Point2f a = canon_grid[idx(r, c)];
      const cv::Point2f b = canon_grid[idx(r, c + 1)];
      const cv::Point2f cpt = canon_grid[idx(r + 1, c)];
      const float s = signed_area(a, b, cpt);
      const float sign = (s >= 0.0f) ? 1.0f : -1.0f;
      if (!sign_set) {
        sign0 = sign;
        sign_set = true;
      } else if (sign != sign0) {
        area_ok = false;
      }
    }
  }

  return (row_ok && col_ok && orth_ok && range_ok && near_mean_ok &&
          near_max_ok && linearity_ok && area_ok);
}

std::vector<cv::Point3f> buildObjectPoints(int rows, int cols,
                                           float square_size) {
  std::vector<cv::Point3f> obj;
  obj.reserve(rows * cols);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      obj.emplace_back(static_cast<float>(c) * square_size,
                       static_cast<float>(r) * square_size, 0.0f);
    }
  }
  return obj;
}

cv::Mat drawCheckerboardRowSnake(const cv::Mat &gray_or_bgr,
                                 const std::vector<cv::Point2f> &ordered,
                                 int rows, int cols, int radius,
                                 bool draw_points, int thickness) {
  cv::Mat vis;
  if (gray_or_bgr.channels() == 1) {
    cv::cvtColor(gray_or_bgr, vis, cv::COLOR_GRAY2BGR);
  } else {
    vis = gray_or_bgr.clone();
  }

  const int line_thickness = std::max(1, thickness);
  const int point_radius = std::max(1, radius);

  const cv::Scalar colors[] = {cv::Scalar(0, 0, 255),   cv::Scalar(0, 165, 255),
                               cv::Scalar(0, 255, 255), cv::Scalar(0, 255, 0),
                               cv::Scalar(255, 0, 0),   cv::Scalar(130, 0, 75),
                               cv::Scalar(211, 0, 148)};

  auto idx = [&](int r, int c) { return r * cols + c; };
  std::vector<cv::Point2f> grid = reshapeGrid(ordered, rows, cols);

  for (int r = 0; r < rows; ++r) {
    const cv::Scalar color = colors[r % (sizeof(colors) / sizeof(colors[0]))];
    std::vector<cv::Point> row_pts;
    row_pts.reserve(cols);
    for (int c = 0; c < cols; ++c) {
      const cv::Point2f p = grid[idx(r, c)];
      row_pts.emplace_back(static_cast<int>(std::round(p.x)),
                           static_cast<int>(std::round(p.y)));
    }
    cv::polylines(vis, row_pts, false, color, line_thickness, cv::LINE_AA);
    if (draw_points) {
      for (const auto &p : row_pts) {
        cv::circle(vis, p, point_radius, color, -1, cv::LINE_AA);
        cv::circle(vis, p, point_radius, cv::Scalar(0, 0, 0), line_thickness,
                   cv::LINE_AA);
      }
    }
    if (r < rows - 1) {
      const cv::Point p_end = row_pts.back();
      const cv::Point p_next =
          cv::Point(static_cast<int>(std::round(grid[idx(r + 1, 0)].x)),
                    static_cast<int>(std::round(grid[idx(r + 1, 0)].y)));
      cv::line(vis, p_end, p_next, color, line_thickness, cv::LINE_AA);
    }
  }
  return vis;
}

} // namespace ecal::core
