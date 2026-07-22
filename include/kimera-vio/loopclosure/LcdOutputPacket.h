/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LcdOutputPacket.h
 * @brief  Loop closure output packet
 * @author Marcus Abate
 * @author Antoni Rosinol
 * @author Luca Carlone
 * @author Nathan Hughes
 * @author Yun Chang
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"
#include "kimera-vio/pipeline/PipelinePayload.h"
#include "kimera-vio/utils/Macros.h"

namespace DBoW2 {
class BowVector;  // forward declare to avoid public dbow dependency
}

namespace VIO {

typedef std::unordered_map<FrameId, Timestamp> FrameIDTimestampMap;

struct LcdOutput : PipelinePayload {
  KIMERA_POINTER_TYPEDEFS(LcdOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LcdOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LcdOutput(LCDStatus lcd_status,
            const Timestamp& timestamp_kf,
            const std::vector<Timestamp>& timestamp_query,
            const std::vector<Timestamp>& timestamp_match,
            const std::vector<FrameId>& id_match,
            const std::vector<FrameId>& id_recent,
            const std::vector<gtsam::Pose3>& relative_pose);

  explicit LcdOutput(const Timestamp& timestamp_kf);

  explicit LcdOutput(LCDStatus lcd_status, const Timestamp& timestamp_kf);

  void setMapInformation(const gtsam::Pose3& W_Pose_Map,
                         const gtsam::Pose3& Map_Pose_Odom,
                         const gtsam::Values& states,
                         const gtsam::NonlinearFactorGraph& nfg);

  void setFrameInformation(const KeypointsCV& keypoints_2d,
                           const Landmarks& keypoints_3d,
                           const BearingVectors& versors,
                           const LandmarkIds& landmark_ids,
                           const std::map<int, double>& bow_vec,
                           const cv::Mat& descriptors_mat);

  // TODO(marcus): inlude stats/score of match
  LCDStatus lcd_status_;
  std::vector<Timestamp> timestamp_query_;
  std::vector<Timestamp> timestamp_match_;
  std::vector<FrameId> id_match_;
  std::vector<FrameId> id_recent_;
  std::vector<gtsam::Pose3> relative_pose_;
  // map information
  gtsam::Pose3 W_Pose_Map_;
  gtsam::Pose3 Map_Pose_Odom_;  // Map frame is the optimal (RPGO) global frame
                                // and odom is the VIO estimate global frame
  gtsam::Values states_;
  gtsam::NonlinearFactorGraph nfg_;
  // frame information
  Timestamp timestamp_kf_;
  // Stable VIO keyframe identifier for the descriptor/verification payload.
  // This is intentionally independent from callback ordering: the VLAD path
  // can delay outputs by its local window and may omit invalid frames.
  FrameId keyframe_id_{0};
  KeypointsCV keypoints_2d_;
  Landmarks keypoints_3d_;
  BearingVectors versors_;
  LandmarkIds landmark_ids_;
  std::map<int, double> bow_vec_;
  cv::Mat descriptors_mat_;
  FrameIDTimestampMap timestamp_map_;
  Pose3 T_base_cam_;
  Pose3 T_world_odom_;

  Landmarks landmarks_;
  std::map<FrameId, FrameIdSet> covis_graph_;
  FrameId query_frame_;
  FrameIdSet global_candidates_;

  // Frame cache statistics
  size_t frame_cache_memory_bytes_{0};  ///< Total memory used by frame cache
  size_t frame_cache_size_{0};          ///< Number of frames stored in cache

  std::vector<std::vector<FrameId>> seq_frames;  ///< Sequence of frame IDs for each sequence of frames added to the database
  std::pair<FrameId, cv::Mat> debug_seq_frame;
  bool is_seq_frame{false};

  double coverage_score{0.0};   ///< Shannon-entropy coverage score of the feature grid
  double structure_score{0.0};  ///< Geometric structure score of the feature grid
  double covisibility_score{0.0};  ///< Covisibility score between the current frame and the previous frame
  double similarity_penalty{0.0};  ///< Covisibility score between the current frame and the previous frame
};

}  // namespace VIO
