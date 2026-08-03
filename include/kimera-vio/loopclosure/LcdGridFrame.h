#pragma once

#include <opencv2/core.hpp>
#include <optional>
#include <vector>

#include "kimera-vio/loopclosure/LandmarkManager.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"

namespace VIO {

// ---------------------------------------------------------------------------
// Grid-based feature packaging
// ---------------------------------------------------------------------------

/** Per-landmark stats (aggregate over one grid frame). */
struct LcdLmkStats {
  LmkIdToNumObsMap num_obs;
  LmkIdToResidualMap residuals;
};

/** Data stored in a single occupied cell of the LCD feature grid. */
struct LcdGridCell {
  cv::KeyPoint keypoint;
  Landmark landmark;   ///< 3-D position in camera frame
  cv::Mat descriptor;  ///< single-row descriptor matrix
  BearingVector bearing_vector;
  LandmarkId landmark_id = -1;
  size_t num_obs = 0;
  double residual = 0.0;
};

/**
 * @brief 2-D grid packaging all per-keypoint outputs of
 * augmentAndFilterFrameFeatures. At most one keypoint occupies each cell.
 * Cells are addressed by (row, col).
 */
class LcdGridFrame {
 public:
  int grid_cols = 0;
  int grid_rows = 0;
  int img_width = 0;
  int img_height = 0;

  /// cells_[row][col] — nullopt when the cell has no keypoint.
  std::vector<std::vector<std::optional<LcdGridCell>>> cells_;

  LcdGridFrame() = default;
  LcdGridFrame(int cols, int rows, int w, int h)
      : grid_cols(cols),
        grid_rows(rows),
        img_width(w),
        img_height(h),
        cells_(rows,
               std::vector<std::optional<LcdGridCell>>(cols, std::nullopt)) {}

  const std::optional<LcdGridCell>& cell(int row, int col) const {
    return cells_[row][col];
  }
  std::optional<LcdGridCell>& cell(int row, int col) {
    return cells_[row][col];
  }

  /// Number of occupied cells.
  size_t size() const {
    size_t n = 0;
    for (const auto& row : cells_)
      for (const auto& c : row)
        if (c) ++n;
    return n;
  }

  // --- flat accessors (row-major order, occupied cells only) ---------------

  std::vector<cv::KeyPoint> getKeypoints() const {
    std::vector<cv::KeyPoint> out;
    for (const auto& row : cells_)
      for (const auto& c : row)
        if (c) out.push_back(c->keypoint);
    return out;
  }

  Landmarks getLandmarks() const {
    Landmarks out;
    for (const auto& row : cells_)
      for (const auto& c : row)
        if (c) out.push_back(c->landmark);
    return out;
  }

  cv::Mat getDescriptors() const {
    cv::Mat out;
    for (const auto& row : cells_)
      for (const auto& c : row)
        if (c && !c->descriptor.empty()) out.push_back(c->descriptor);
    return out;
  }

  BearingVectors getBearingVectors() const {
    BearingVectors out;
    for (const auto& row : cells_)
      for (const auto& c : row)
        if (c) out.push_back(c->bearing_vector);
    return out;
  }

  std::vector<LandmarkId> getLandmarkIds() const {
    std::vector<LandmarkId> out;
    for (const auto& row : cells_)
      for (const auto& c : row)
        if (c) out.push_back(c->landmark_id);
    return out;
  }

  LcdLmkStats getLmkStats() const {
    LcdLmkStats stats;
    for (const auto& row : cells_)
      for (const auto& c : row)
        if (c && c->landmark_id >= 0) {
          stats.num_obs[c->landmark_id] = c->num_obs;
          stats.residuals[c->landmark_id] = c->residual;
        }
    return stats;
  }

  /**
   * @brief Compute descriptor diversity for repetitive-frame rejection.
   *
   * A low score means that the local descriptors within this keyframe are
   * mutually similar. Such frames are unsuitable for place recognition
   * because their global descriptor is likely to produce ambiguous loop
   * candidates.
   */
  static double computeDescriptorDiversityScore(
      const cv::Mat& descriptors) {
    // tau_dist: saturation threshold calibrated for XFeat 64-D L2-normalised
    // unit vectors.  For such descriptors V_app = (1/N)*Σ||d_k - d_bar|| ≈
    // sqrt(1 - ||d_bar||²), which ranges from ~0.44 (very repetitive scene,
    // ||d_bar||≈0.9) to ~1.0 (fully random/diverse). Setting tau_dist=0.8
    // means V_app must reach 0.8 before the penalty saturates, giving useful
    // discrimination between repetitive and diverse frames.
    // (Previously 0.5 caused perpetual saturation to 1.0.)
    static constexpr double tau_dist = 0.8;

    const int N = descriptors.rows;
    if (N == 0) return 0.0;

    cv::Mat descs;
    descriptors.convertTo(descs, CV_32F);

    // Mean descriptor: d_bar = (1/N) * sum_k d_k
    cv::Mat mean_desc;
    cv::reduce(descs, mean_desc, /*dim=*/0, cv::REDUCE_AVG, CV_32F);

    // V_app = (1/N) * sum_k || d_k - d_bar ||_2
    double V_app = 0.0;
    for (int i = 0; i < N; ++i) {
      V_app += cv::norm(descs.row(i) - mean_desc, cv::NORM_L2);
    }
    V_app /= N;

    VLOG(5) << "DescriptorDiversity: N=" << N << " V_app=" << V_app
            << " tau=" << tau_dist
            << " score=" << std::min(1.0, V_app / tau_dist);

    // S_app = min(1, V_app / tau_dist)
    return std::min(1.0, V_app / tau_dist);
  }

  double computeDescriptorDiversityScore() const {
    return computeDescriptorDiversityScore(getDescriptors());
  }
};

}  // namespace VIO
