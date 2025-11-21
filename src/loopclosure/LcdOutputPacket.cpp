/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   LcdOutputPacket.cpp
 * @brief  Loop closure output packet
 * @author Marcus Abate
 * @author Antoni Rosinol
 * @author Luca Carlone
 * @author Nathan Hughes
 * @author Yun Chang
 */

#include "kimera-vio/loopclosure/LcdOutputPacket.h"

#include <DBoW2/DBoW2.h>

namespace VIO {

LcdOutput::LcdOutput(LCDStatus lcd_status,
                     const Timestamp& timestamp_kf,
                     const std::vector<Timestamp>& timestamp_query,
                     const std::vector<Timestamp>& timestamp_match,
                     const std::vector<FrameId>& id_match,
                     const std::vector<FrameId>& id_recent,
                     const std::vector<gtsam::Pose3>& relative_pose)
    : PipelinePayload(timestamp_kf),
      lcd_status_(lcd_status),
      timestamp_query_(timestamp_query),
      timestamp_match_(timestamp_match),
      id_match_(id_match),
      id_recent_(id_recent),
      relative_pose_(relative_pose) {}

LcdOutput::LcdOutput(const Timestamp& timestamp_kf)
    : PipelinePayload(timestamp_kf),
      lcd_status_(LCDStatus::NO_MATCHES),
      timestamp_query_({}),
      timestamp_match_({}),
      id_match_({}),
      id_recent_({}) {}

LcdOutput::LcdOutput(LCDStatus lcd_status, const Timestamp& timestamp_kf)
    : PipelinePayload(timestamp_kf),
      lcd_status_(lcd_status),
      timestamp_query_({}),
      timestamp_match_({}),
      id_match_({}),
      id_recent_({}) {}

void LcdOutput::setMapInformation(const gtsam::Pose3& W_Pose_Map,
                                  const gtsam::Pose3& Map_Pose_Odom,
                                  const gtsam::Values& states,
                                  const gtsam::NonlinearFactorGraph& nfg) {
  W_Pose_Map_ = W_Pose_Map;
  Map_Pose_Odom_ = Map_Pose_Odom;
  states_ = states;
  nfg_ = nfg;
}

void LcdOutput::setFrameInformation(const KeypointsCV& keypoints_2d,
                                    const Landmarks& keypoints_3d,
                                    const BearingVectors& versors,
                                    const std::map<int, double>& bow_vec,
                                    const cv::Mat& descriptors_mat) {
  keypoints_2d_ = keypoints_2d;
  keypoints_3d_ = keypoints_3d;
  versors_ = versors;
  bow_vec_ = bow_vec;
  descriptors_mat_ = descriptors_mat;
}

}  // namespace VIO
