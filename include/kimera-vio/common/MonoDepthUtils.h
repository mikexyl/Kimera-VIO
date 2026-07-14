#pragma once

#include <cstddef>
#include <opencv2/core.hpp>
#include <string>

#include "kimera-vio/common/MonoDepthTypes.h"

namespace VIO {

struct Da3OverlapScaleEstimate {
  bool valid = false;
  double scale_ratio = 1.0;
  std::size_t candidate_count = 0u;
  std::size_t inlier_count = 0u;
  double log_rmse = 0.0;
  std::string failure_reason;
};

/** Robustly estimate the multiplier mapping next_context into the scale of
 * previous_current. Both packets must be canonical predictions of one image.
 */
Da3OverlapScaleEstimate estimateDa3OverlapScale(
    const MonoDepthRawPacket& previous_current,
    const MonoDepthRawPacket& next_context,
    int point_stride);

cv::Mat scaleMonoDepthImage(const cv::Mat& canonical_depth, double depth_scale);

cv::Mat makeMonoDepthWeightImage(const cv::Mat& depth,
                                 const cv::Mat& valid_mask,
                                 const MonoDepthIntrinsics& intrinsics,
                                 const MonoDepthParams& params);

cv::Mat makeMonoDepthWeightImageAtSize(const cv::Mat& depth,
                                       const cv::Mat& valid_mask,
                                       const MonoDepthIntrinsics& intrinsics,
                                       const cv::Size& weight_size,
                                       const MonoDepthParams& params);

float sampleMonoDepthWeight(const cv::Mat& weight_image,
                            const cv::Size& depth_size,
                            int u,
                            int v);

}  // namespace VIO
