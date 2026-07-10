#pragma once

#include <xfeat-cpp/mono_depth/mono_depth.h>

#include <memory>
#include <optional>

#include "kimera-vio/common/MonoDepthTypes.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class MonoDepthInference {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(MonoDepthInference);
  KIMERA_POINTER_TYPEDEFS(MonoDepthInference);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit MonoDepthInference(const MonoDepthParams& params);
  ~MonoDepthInference() = default;

  MonoDepthRawPacket::ConstPtr inferKeyframe(const Frame& frame) const;

 private:
  static cv::Mat toBgrImage(const cv::Mat& image);
  static cv::Mat makeValidMask(const cv::Mat& depth, const cv::Mat& sky_mask);
  static Eigen::Vector3d backprojectDepthPixel(
      int u,
      int v,
      float z,
      const MonoDepthIntrinsics& intrinsics);
  cv::Mat makeWeightImage(const cv::Mat& depth,
                          const cv::Mat& valid_mask,
                          const MonoDepthIntrinsics& intrinsics) const;

 private:
  MonoDepthParams params_;
  std::unique_ptr<xfeat::MonoDepth> mono_depth_;
};

}  // namespace VIO
