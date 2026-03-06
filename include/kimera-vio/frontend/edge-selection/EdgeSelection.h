/**
 * @file   EdgeSelection.h
 * @brief  Golden Edge Selection: identify the single best candidate pair of
 *         keyframes to send to a deep feature matcher by maximising the
 *         expected increase in the pose-graph algebraic connectivity (λ₂).
 *
 * Algorithm (K=1 Algebraic Connectivity Maximisation):
 *   E[Δλ₂] = p_ij · (v₂ᵢ − v₂ⱼ)²
 * where
 *   - p_ij is the geometric overlap probability between keyframes i and j,
 *   - v₂   is the Fiedler vector of the current graph Laplacian L = D − A.
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>

#include <optional>
#include <vector>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

// ----------------------------------------------------------------------------
// Parameters
// ----------------------------------------------------------------------------

/**
 * @brief Tuning parameters for the golden edge selection algorithm.
 */
struct EdgeSelectionParams {
  //! Maximum baseline distance [m] between camera centres for a feasible match.
  double max_depth_range = 20.0;

  //! Minimum baseline distance [m] between camera centres for a feasible match.
  //! Pairs whose camera centres are closer than this are rejected, which
  //! effectively enforces a minimum temporal separation between selected frames.
  //! Default 0.0 (no minimum). Increase (e.g. 1.0–3.0 m) to avoid picking
  //! temporally adjacent keyframes.
  double min_baseline = 0.0;

  //! Minimum cosine of the angle between principal camera Z-axes.
  //! Default cos(120°) = -0.5 — pairs whose viewing directions diverge by more
  //! than 120° are considered geometrically infeasible.
  //! Setting this to exactly 1.0 restricts matches to perfectly parallel
  //! cameras; the division-by-zero that would result is handled internally.
  double min_cos_angle = -0.5;

  //! Camera-in-body extrinsic transform (B_T_C).
  //! In Kimera-VIO the poses stored in KeyframeNode are IMU body poses
  //! (W_T_B), not camera poses. Supplying the body-to-camera transform here
  //! lets calculateOverlapProbability derive the true camera principal axis and
  //! camera centre in world coordinates via W_T_C = W_T_B ∘ B_T_C.
  //! Defaults to identity (body frame ≡ camera frame), which is correct when
  //! the pose already represents the camera (e.g. in unit tests).
  gtsam::Pose3 b_T_c = gtsam::Pose3();
};

// ----------------------------------------------------------------------------
// Data structures
// ----------------------------------------------------------------------------

/**
 * @brief A single keyframe entry inside the VIO sliding window.
 */
struct KeyframeNode {
  FrameId frame_id;  //!< Global keyframe identifier.
  gtsam::Pose3 pose; //!< Body-in-world pose (W_T_B).
};

/** @brief Ordered list of keyframes currently inside the sliding window. */
using SlidingWindow = std::vector<KeyframeNode>;

/**
 * @brief The result produced by selectGoldenEdge().
 *
 * id_i / id_j are zero-based indices into the SlidingWindow vector.
 * frame_id_i / frame_id_j are the corresponding global FrameIds.
 * score is the expected Fiedler gain: p_ij · (v₂ᵢ − v₂ⱼ)².
 */
struct EdgeCandidate {
  int id_i = -1;           //!< Sliding-window index of the first  keyframe.
  int id_j = -1;           //!< Sliding-window index of the second keyframe.
  FrameId frame_id_i = 0;  //!< Global FrameId of the first  keyframe.
  FrameId frame_id_j = 0;  //!< Global FrameId of the second keyframe.
  double score = -1.0;     //!< Expected Fiedler gain.

  /** @return True iff a valid candidate was found. */
  bool isValid() const noexcept { return id_i >= 0 && id_j >= 0; }
};

// ----------------------------------------------------------------------------
// EdgeSelection
// ----------------------------------------------------------------------------

/**
 * @brief Stateless utility class implementing the Golden Edge Selection
 *        algorithm for K=1 algebraic connectivity maximisation.
 */
class EdgeSelection {
 public:
  KIMERA_POINTER_TYPEDEFS(EdgeSelection);

  // Non-instantiable — all methods are static.
  EdgeSelection() = delete;

  /**
   * @brief Compute the geometric overlap probability between two camera poses.
   *
   * Both input poses are assumed to be IMU body poses in the world frame
   * (W_T_B). The camera poses are derived internally as:
   *   W_T_C = W_T_B ∘ params.b_T_c
   * so the correct camera principal axis and camera centre are used regardless
   * of how the camera is mounted relative to the IMU body.
   *
   * The probability is computed in two stages:
   *  1. Hard rejection: returns 0 if the camera-centre baseline exceeds
   *     params.max_depth_range or if the cosine of the angle between the
   *     camera Z-axes is below params.min_cos_angle (default cos 60° = 0.5).
   *  2. Linear scaling: maps the cosine from the feasible range
   *     [min_cos_angle, 1] onto [0, 1]:
   *       p = (cos_angle − min_cos_angle) / (1 − min_cos_angle)
   *     When min_cos_angle == 1.0 (perfectly parallel only), the denominator
   *     would be zero; in that case the pair already passed the threshold and
   *     p = 1.0 is returned directly.
   *
   * @param pose_i  World body pose of the first  keyframe (W_T_B).
   * @param pose_j  World body pose of the second keyframe (W_T_B).
   * @param params  Algorithm parameters (including b_T_c extrinsic).
   * @return        Geometric overlap probability ∈ [0, 1].
   */
  static double calculateOverlapProbability(
      const gtsam::Pose3& pose_i,
      const gtsam::Pose3& pose_j,
      const EdgeSelectionParams& params = EdgeSelectionParams{});

  /**
   * @brief Select the single best candidate edge for deep feature matching.
   *
   * Steps:
   *  1. Build the graph Laplacian L = D − A from the adjacency matrix.
   *  2. Compute the Fiedler vector v₂ (eigenvector of the second-smallest
   *     eigenvalue of L) via a symmetric eigen-decomposition.
   *  3. For every non-adjacent pair (i, j):
   *       expected_gain = calculateOverlapProbability(i,j) × (v₂ᵢ − v₂ⱼ)²
   *  4. Return the pair with the highest expected gain.
   *
   * @param window        Ordered list of keyframes in the sliding window.
   * @param adjacency     N×N symmetric adjacency matrix of the current pose
   *                      graph. Entry (i,j) > 0 means an edge already exists.
   * @param params        Algorithm parameters.
   * @param required_node If set, only candidate pairs that include this
   *                      zero-based node index are considered. Useful when the
   *                      caller wants to anchor one endpoint (e.g. the current
   *                      keyframe) without manipulating the adjacency matrix.
   *                      Out-of-range values are ignored with a warning.
   * @return              Best EdgeCandidate; isValid() == false if none found.
   */
  static EdgeCandidate selectGoldenEdge(
      const SlidingWindow& window,
      const Eigen::MatrixXd& adjacency,
      const EdgeSelectionParams& params = EdgeSelectionParams{},
      std::optional<int> required_node = std::nullopt);

 private:
  /**
   * @brief Compute the Fiedler vector of the graph Laplacian.
   *
   * The Laplacian is constructed internally as L = diag(A·1) − A.
   * A symmetric eigen-decomposition (Eigen::SelfAdjointEigenSolver) is used;
   * eigenvalues are returned in ascending order, so the Fiedler vector is
   * column 1 of the eigenvector matrix.
   *
   * @param adjacency  N×N symmetric adjacency matrix.
   * @return           Fiedler vector of length N, or a zero vector on failure.
   */
  static Eigen::VectorXd computeFiedlerVector(const Eigen::MatrixXd& adjacency);
};

// ============================================================================
// Inline implementations
// ============================================================================

inline double EdgeSelection::calculateOverlapProbability(
    const gtsam::Pose3& pose_i,
    const gtsam::Pose3& pose_j,
    const EdgeSelectionParams& params) {

  // Apply body-to-camera extrinsics: W_T_C = W_T_B ∘ B_T_C.
  // When b_T_c is identity (default) this is a no-op, preserving backwards
  // compatibility with callers that already supply camera poses directly.
  const gtsam::Pose3 W_T_C_i = pose_i.compose(params.b_T_c);
  const gtsam::Pose3 W_T_C_j = pose_j.compose(params.b_T_c);

  // --- 1. Baseline distance check (between camera centres) ------------------
  const double distance =
      (W_T_C_i.translation() - W_T_C_j.translation()).norm();
  if (distance < params.min_baseline || distance > params.max_depth_range) {
    return 0.0;
  }

  // --- 2. Viewing-direction check (camera Z-axes in world frame) ------------
  // col(2) of the rotation matrix is the camera principal axis expressed in
  // world coordinates — the true "where is the camera looking" vector.
  const Eigen::Vector3d z_i = W_T_C_i.rotation().matrix().col(2);
  const Eigen::Vector3d z_j = W_T_C_j.rotation().matrix().col(2);
  const double cos_angle = z_i.dot(z_j);

  if (cos_angle < params.min_cos_angle) {
    return 0.0;  // Viewing directions diverge beyond the threshold angle.
  }

  // --- 3. Linear probability scaling ----------------------------------------
  // Maps [min_cos_angle, 1.0] → [0.0, 1.0].
  // Guard: if min_cos_angle == 1.0 the denominator is zero. The pair already
  // passed the threshold above (cos_angle >= 1.0), so return full probability.
  const double denominator = 1.0 - params.min_cos_angle;
  if (denominator <= 0.0) {
    return 1.0;
  }

  return (cos_angle - params.min_cos_angle) / denominator;
}

inline Eigen::VectorXd EdgeSelection::computeFiedlerVector(
    const Eigen::MatrixXd& adjacency) {

  const int N = static_cast<int>(adjacency.rows());

  if (N < 2) {
    VLOG(2) << "EdgeSelection::computeFiedlerVector: graph too small (N=" << N
            << "); returning zero vector.";
    return Eigen::VectorXd::Zero(std::max(N, 0));
  }

  // Build the graph Laplacian L = D - A, where D = diag(A * 1).
  const Eigen::VectorXd degree = adjacency.rowwise().sum();
  const Eigen::MatrixXd laplacian =
      degree.asDiagonal().toDenseMatrix() - adjacency;

  // Symmetric eigen-decomposition — eigenvalues sorted in ascending order.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(laplacian);
  if (solver.info() != Eigen::Success) {
    LOG(WARNING)
        << "EdgeSelection::computeFiedlerVector: eigen-decomposition failed.";
    return Eigen::VectorXd::Zero(N);
  }

  // Column 0 → trivial zero eigenvalue (all-ones vector for connected graphs).
  // Column 1 → Fiedler vector (second-smallest eigenvalue λ₂).
  return solver.eigenvectors().col(1);
}

inline EdgeCandidate EdgeSelection::selectGoldenEdge(
    const SlidingWindow& window,
    const Eigen::MatrixXd& adjacency,
    const EdgeSelectionParams& params,
    std::optional<int> required_node) {

  const int N = static_cast<int>(window.size());
  DCHECK_EQ(static_cast<int>(adjacency.rows()), N)
      << "Adjacency matrix row count must match the sliding window size.";
  DCHECK_EQ(static_cast<int>(adjacency.cols()), N)
      << "Adjacency matrix must be square.";

  // Validate required_node if provided.
  if (required_node.has_value() &&
      (*required_node < 0 || *required_node >= N)) {
    LOG(WARNING) << "EdgeSelection::selectGoldenEdge: required_node "
                 << *required_node << " is out of range [0, " << N
                 << "); ignoring.";
    required_node = std::nullopt;
  }

  if (N < 2) {
    LOG(FATAL) << "EdgeSelection::selectGoldenEdge: sliding window too small "
                  "(N=" << N << "); this is a bug — VO cannot be running "
                  "without at least two keyframes.";
  }

  // ---- Step 1 & 2: Compute the Fiedler vector --------------------------------
  const Eigen::VectorXd v2 = computeFiedlerVector(adjacency);
  const bool fiedler_valid = !v2.isZero(0.0);
  if (!fiedler_valid) {
    LOG(WARNING)
        << "EdgeSelection::selectGoldenEdge: Fiedler vector is zero; "
           "will fall back to most-recent geometrically feasible edge.";
  }

  // ---- Helper: evaluate a candidate pair and update best_edge if better -----
  EdgeCandidate best_edge;
  double max_expected_gain = -1.0;
  bool has_feasible_pair = false;  // any pair passed the geometric check

  auto try_pair = [&](int i, int j) {
    // Do not skip pairs that already have covisibility — VO tracking
    // connections are not loop-closure edges. The Fiedler score naturally
    // deprioritises well-connected pairs, so they only win when truly best.
    const double p_ij =
        calculateOverlapProbability(window[i].pose, window[j].pose, params);
    if (p_ij <= 0.0) return;
    has_feasible_pair = true;
    const double delta_v2 = fiedler_valid ? (v2(i) - v2(j)) : 0.0;
    const double expected_gain = p_ij * delta_v2 * delta_v2;
    if (expected_gain > max_expected_gain) {
      max_expected_gain    = expected_gain;
      best_edge.id_i       = i;
      best_edge.id_j       = j;
      best_edge.frame_id_i = window[i].frame_id;
      best_edge.frame_id_j = window[j].frame_id;
      best_edge.score      = expected_gain;
    }
  };

  // ---- Step 3: Evaluate all non-adjacent candidate pairs --------------------
  if (fiedler_valid) {
    for (int i = 0; i < N; ++i) {
      for (int j = i + 1; j < N; ++j) {
        if (required_node.has_value() &&
            i != *required_node && j != *required_node) {
          continue;
        }
        try_pair(i, j);
      }
    }
  }

  // ---- Fallback: Fiedler unavailable — scan all pairs by p_ij only ----------
  if (!best_edge.isValid()) {
    for (int j = N - 1; j >= 1 && !best_edge.isValid(); --j) {
      for (int i = j - 1; i >= 0 && !best_edge.isValid(); --i) {
        if (required_node.has_value() &&
            i != *required_node && j != *required_node) {
          continue;
        }
        const double p_ij =
            calculateOverlapProbability(window[i].pose, window[j].pose, params);
        if (p_ij <= 0.0) continue;
        has_feasible_pair = true;
        best_edge.id_i       = i;
        best_edge.id_j       = j;
        best_edge.frame_id_i = window[i].frame_id;
        best_edge.frame_id_j = window[j].frame_id;
        best_edge.score      = 0.0;
      }
    }
  }

  if (!best_edge.isValid()) {
    if (has_feasible_pair) {
      LOG(FATAL) << "EdgeSelection::selectGoldenEdge: geometrically feasible "
                    "pairs exist but none were selected — this is a bug.";
    }
    VLOG(2) << "EdgeSelection::selectGoldenEdge: no geometrically feasible "
               "pair found (all poses too far or divergent); skipping.";
  }

  VLOG(2) << "EdgeSelection::selectGoldenEdge: best edge ("
          << best_edge.frame_id_i << ", " << best_edge.frame_id_j
          << ") with score " << best_edge.score;

  return best_edge;
}

}  // namespace VIO
