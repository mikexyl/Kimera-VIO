/**
 * @file   testEdgeSelection.cpp
 * @brief  Standalone integration test for EdgeSelection::selectGoldenEdge().
 *
 * Scenario: 5-frame U-shaped trajectory with chain odometry edges.
 *
 *   Frame positions (XY plane, all cameras face +Z / identity rotation):
 *
 *     (0,1)  (2,1)
 *      [4]----[3]
 *       |      |
 *      [0]----[1]----[2]
 *     (0,0)  (1,0)  (2,0)
 *
 *   Existing odometry edges: 0-1, 1-2, 2-3, 3-4  (chain)
 *
 * Expected result:
 *   The Fiedler vector of a path graph assigns its most extreme values to
 *   the two endpoints (nodes 0 and 4).  All cameras share the same
 *   viewing direction (identity rotation → Z-axis = [0,0,1]), so the
 *   geometric overlap probability p_ij = 1.0 for every non-adjacent pair.
 *   Therefore the pair (0, 4) — which is also geometrically close at 1 m —
 *   maximises E[Δλ₂] = p_ij · (v₂ᵢ − v₂ⱼ)² and is the correct answer.
 */

#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Dense>

#include <cstdlib>
#include <iostream>

#include "kimera-vio/frontend/edge-selection/EdgeSelection.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Build a KeyframeNode with identity rotation and given translation. */
static VIO::KeyframeNode makeNode(VIO::FrameId id, double x, double y,
                                  double z) {
  return VIO::KeyframeNode{id,
                           gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, y, z))};
}

/** Simple check macro — prints PASS/FAIL and returns EXIT_FAILURE on failure. */
#define CHECK_TRUE(cond, msg)                                          \
  do {                                                                 \
    if (!(cond)) {                                                     \
      std::cerr << "[FAIL] " << (msg) << "\n";                        \
      std::cerr << "       Condition: " #cond "\n";                   \
      return EXIT_FAILURE;                                             \
    }                                                                  \
    std::cout << "[PASS] " << (msg) << "\n";                          \
  } while (false)

// ---------------------------------------------------------------------------
// Test: U-shaped trajectory
// ---------------------------------------------------------------------------

static int testUShapedTrajectory() {
  std::cout << "\n=== testUShapedTrajectory ===\n";

  constexpr int N = 5;

  // ---- Sliding window (all cameras face +Z, i.e. identity rotation) --------
  VIO::SlidingWindow window;
  window.reserve(N);
  window.push_back(makeNode(0, 0.0, 0.0, 0.0));  // start of U-left leg
  window.push_back(makeNode(1, 1.0, 0.0, 0.0));
  window.push_back(makeNode(2, 2.0, 0.0, 0.0));  // bottom-right
  window.push_back(makeNode(3, 2.0, 1.0, 0.0));  // top-right
  window.push_back(makeNode(4, 0.0, 1.0, 0.0));  // close to frame 0

  // ---- Chain adjacency matrix (odometry: 0-1, 1-2, 2-3, 3-4) --------------
  Eigen::MatrixXd adjacency = Eigen::MatrixXd::Zero(N, N);
  for (int i = 0; i < N - 1; ++i) {
    adjacency(i, i + 1) = 1.0;
    adjacency(i + 1, i) = 1.0;
  }

  std::cout << "Running Golden Edge Selection on " << N << " frames...\n";

  VIO::EdgeCandidate best = VIO::EdgeSelection::selectGoldenEdge(window, adjacency);

  // ---- Basic validity checks -----------------------------------------------
  CHECK_TRUE(best.isValid(), "A valid candidate edge must be found");
  CHECK_TRUE(best.score > 0.0, "Expected gain must be positive");

  // ---- Identity checks between window indices and global FrameIds ----------
  CHECK_TRUE(best.frame_id_i == window[best.id_i].frame_id,
             "frame_id_i matches window entry");
  CHECK_TRUE(best.frame_id_j == window[best.id_j].frame_id,
             "frame_id_j matches window entry");

  // ---- Correctness: endpoints (0, 4) should win ----------------------------
  //
  // Reasoning:
  //  - All p_ij = 1.0 (identity rotation → cos_angle = 1, baseline < 10 m).
  //  - The Fiedler vector of a path graph has monotonically extreme values at
  //    its two endpoints, so (v₂₀ − v₂₄)² is the maximum topological gain.
  //  - Therefore E[Δλ₂](0,4) > E[Δλ₂] for every other non-adjacent pair.
  const bool correct_pair =
      (best.id_i == 0 && best.id_j == 4) ||
      (best.id_i == 4 && best.id_j == 0);

  CHECK_TRUE(correct_pair,
             "Best edge is the loop-closure candidate (node 0, node 4)");

  std::cout << "\nResult: node " << best.id_i << " (FrameId=" << best.frame_id_i
            << ") <-> node " << best.id_j << " (FrameId=" << best.frame_id_j
            << ")  |  score = " << best.score << "\n";

  return EXIT_SUCCESS;
}

// ---------------------------------------------------------------------------
// Test: no candidates when graph is fully connected
// ---------------------------------------------------------------------------

static int testFullyConnectedGraph() {
  std::cout << "\n=== testFullyConnectedGraph ===\n";

  constexpr int N = 3;
  VIO::SlidingWindow window;
  window.push_back(makeNode(0, 0.0, 0.0, 0.0));
  window.push_back(makeNode(1, 1.0, 0.0, 0.0));
  window.push_back(makeNode(2, 0.5, 0.5, 0.0));

  // Complete graph — all edges already exist, nothing to add.
  Eigen::MatrixXd adjacency = Eigen::MatrixXd::Ones(N, N);
  adjacency.diagonal().setZero();

  VIO::EdgeCandidate best = VIO::EdgeSelection::selectGoldenEdge(window, adjacency);

  CHECK_TRUE(!best.isValid(),
             "No candidate should be returned for a fully-connected graph");

  return EXIT_SUCCESS;
}

// ---------------------------------------------------------------------------
// Test: geometric pruning rejects opposite-facing cameras
// ---------------------------------------------------------------------------

static int testGeometricPruning() {
  std::cout << "\n=== testGeometricPruning ===\n";

  // Two frames: one faces +Z, the other faces -Z (180° apart → cos = -1).
  VIO::SlidingWindow window;
  window.push_back(
      makeNode(0, 0.0, 0.0, 0.0));  // faces +Z (identity rotation)

  // Rotation that flips Z: Rot3 with 180° around X-axis.
  const gtsam::Rot3 flip_z = gtsam::Rot3::Rx(M_PI);
  window.push_back(
      VIO::KeyframeNode{1u, gtsam::Pose3(flip_z, gtsam::Point3(1.0, 0.0, 0.0))});

  Eigen::MatrixXd adjacency = Eigen::MatrixXd::Zero(2, 2);
  // No existing edge between them.

  VIO::EdgeCandidate best = VIO::EdgeSelection::selectGoldenEdge(window, adjacency);

  CHECK_TRUE(!best.isValid(),
             "Opposite-facing cameras must be rejected by geometric pruning");

  return EXIT_SUCCESS;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  int status = EXIT_SUCCESS;
  status |= testUShapedTrajectory();
  status |= testFullyConnectedGraph();
  status |= testGeometricPruning();

  std::cout << "\n";
  if (status == EXIT_SUCCESS) {
    std::cout << "All tests PASSED.\n";
  } else {
    std::cout << "One or more tests FAILED.\n";
  }
  return status;
}
