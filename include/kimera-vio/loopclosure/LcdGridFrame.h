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

  double computeConfidenceWeight() const {
    // w_k = \frac{\log(1+N_{obs}^{(k)})}{1+\epsilon_{reproj}^{(k)}},
    return std::log(1 + num_obs) / (1 + residual);
  }

  double computeFeatureMass() const {
    // m_c = \sum_{k \in c}{w_k},

    // since there only 1 landmark per cell
    return computeConfidenceWeight();
  }
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

  double computeDescriptorVariancePenalty() const {
    // tau_dist: saturation threshold calibrated for XFeat 64-D L2-normalised
    // unit vectors.  For such descriptors V_app = (1/N)*Σ||d_k - d_bar|| ≈
    // sqrt(1 - ||d_bar||²), which ranges from ~0.44 (very repetitive scene,
    // ||d_bar||≈0.9) to ~1.0 (fully random/diverse). Setting tau_dist=0.8
    // means V_app must reach 0.8 before the penalty saturates, giving useful
    // discrimination between repetitive and diverse frames.
    // (Previously 0.5 caused perpetual saturation to 1.0.)
    static constexpr double tau_dist = 0.8;

    cv::Mat descs = getDescriptors();  // N × D, CV_32F
    const int N = descs.rows;
    if (N == 0) return 0.0;

    // Mean descriptor: d_bar = (1/N) * sum_k d_k
    cv::Mat mean_desc;
    cv::reduce(descs, mean_desc, /*dim=*/0, cv::REDUCE_AVG, CV_32F);

    // V_app = (1/N) * sum_k || d_k - d_bar ||_2
    double V_app = 0.0;
    for (int i = 0; i < N; ++i) {
      V_app += cv::norm(descs.row(i) - mean_desc, cv::NORM_L2);
    }
    V_app /= N;

    VLOG(5) << "DescriptorVariancePenalty: N=" << N << " V_app=" << V_app
            << " tau=" << tau_dist
            << " penalty=" << std::min(1.0, V_app / tau_dist);

    // S_app = min(1, V_app / tau_dist)
    return std::min(1.0, V_app / tau_dist);
  }

  double computeCoverageScore() const {
    std::vector<double> feature_masses;
    for (const auto& row : cells_) {
      for (const auto& c : row) {
        feature_masses.push_back(c ? c->computeFeatureMass() : 0.0);
      }
    }
    CHECK_EQ(feature_masses.size(), grid_cols * grid_rows);

    double total_mass =
        std::accumulate(feature_masses.begin(), feature_masses.end(), 0.0);
    std::vector<double> P_c;
    if (total_mass == 0.0) {
      LOG(WARNING)
          << "Total feature mass is zero in coverage score computation.";
      return 0.0;
    } else {
      CHECK_GT(total_mass, 0.0) << "Total feature mass must be positive.";
    }
    // devide feature mass by total mass to get feature mass distribution
    for (const auto& m_c : feature_masses) {
      CHECK_GE(m_c, 0.0);
      P_c.push_back(m_c / total_mass);
    }

    // compute shannon entropy; by convention 0*log2(0) = 0 (limit as p->0)
    double H_cover = 0.0;
    for (const auto& p : P_c) {
      CHECK_GE(p, 0.0);
      if (p > 0.0) H_cover += -p * std::log2(p);
    }
    double H_cover_max = std::log2(feature_masses.size());
    return H_cover / H_cover_max;
  }

  double computeStructureScore() const {
    std::vector<double> w_k;
    for (const auto& row : cells_) {
      for (const auto& c : row) {
        w_k.push_back(c ? c->computeConfidenceWeight() : 0.0);
      }
    }

    double total_w = std::accumulate(w_k.begin(), w_k.end(), 0.0);
    if (total_w == 0.0) {
      LOG(WARNING) << "Total weight is zero in structure score computation "
                      "(empty grid frame).";
      return 0.0;
    }
    gtsam::Point3 bar_P{0.0, 0.0, 0.0};
    for (size_t i = 0; i < w_k.size(); ++i) {
      const auto& c = cells_[i / grid_cols][i % grid_cols];
      if (c) {
        bar_P += w_k[i] * c->landmark;
      }
    }
    bar_P /= total_w;

    Eigen::Matrix3d C_3D = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < w_k.size(); ++i) {
      const auto& c = cells_[i / grid_cols][i % grid_cols];
      if (c) {
        auto Pk = c->landmark - bar_P;
        C_3D += w_k[i] * Pk * Pk.transpose();
      }
    }
    C_3D /= total_w;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(C_3D);
    if (eigensolver.info() != Eigen::Success) {
      LOG(WARNING) << "Failed to compute eigenvalues for structure score.";
      return 0.0;
    }
    Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
    double lambda_1 = eigenvalues(2);  // largest eigenvalue
    double lambda_2 = eigenvalues(1);
    double lambda_3 = eigenvalues(0);
    CHECK_GE(lambda_1, lambda_2);
    CHECK_GE(lambda_2, lambda_3);

    Eigen::Vector3d v1 =
        eigensolver.eigenvectors().col(2);  // eigenvector of largest eigenvalue
    Eigen::Vector3d v2 = eigensolver.eigenvectors().col(1);
    Eigen::Vector3d v3 = eigensolver.eigenvectors().col(
        0);  // eigenvector of smallest eigenvalue

    const static Eigen::Vector3d z{
        0.0, 0.0, 1.0};  // assuming camera looks along +Z in its own frame

    double c = std::abs(
        v3.dot(z));  // cosine of angle between v3 and camera viewing direction

    if (lambda_1 <= 0.0) {
      LOG(WARNING) << "Non-positive largest eigenvalue in structure score.";
      return 0.0;
    }

    double S_geom = (lambda_2 / lambda_1) *
                    (lambda_3 / lambda_2 + (1 - lambda_3 / lambda_2) * c);
    S_geom = std::sqrt(S_geom);  // make it more linear
    return S_geom;
  }
};

}  // namespace VIO
