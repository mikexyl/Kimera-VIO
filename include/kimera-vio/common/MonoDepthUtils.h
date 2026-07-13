#pragma once

#include <opencv2/core.hpp>

#include "kimera-vio/common/MonoDepthTypes.h"

namespace VIO {

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
