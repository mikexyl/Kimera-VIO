#pragma once

#include <glog/logging.h>

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <functional>
#include <string>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

enum class DenseMapBackend { kGaussianVoxelMap = 0 };

inline std::string denseMapBackendToString(const DenseMapBackend backend) {
  switch (backend) {
    case DenseMapBackend::kGaussianVoxelMap:
      return "gaussian_voxel_map";
    default:
      LOG(FATAL) << "Unknown dense map backend: "
                 << static_cast<int>(backend);
  }
  return "gaussian_voxel_map";
}

inline DenseMapBackend denseMapBackendFromString(std::string backend) {
  std::transform(backend.begin(),
                 backend.end(),
                 backend.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  if (backend == "gaussian_voxel_map" ||
      backend == "gaussian-voxel-map" ||
      backend == "gaussian_voxel" ||
      backend == "gaussian-voxel") {
    return DenseMapBackend::kGaussianVoxelMap;
  }
  LOG(FATAL) << "Unsupported dense_map.backend: " << backend
             << ". Expected gaussian_voxel_map.";
  return DenseMapBackend::kGaussianVoxelMap;
}

struct DenseMapParams {
  bool enabled = false;
  DenseMapBackend backend = DenseMapBackend::kGaussianVoxelMap;
  double voxel_resolution = 0.15;
  float point_radius = 0.025f;

  bool operator==(const DenseMapParams& rhs) const {
    return enabled == rhs.enabled && backend == rhs.backend &&
           voxel_resolution == rhs.voxel_resolution &&
           point_radius == rhs.point_radius;
  }
};

struct DenseMapInputPacket {
  KIMERA_POINTER_TYPEDEFS(DenseMapInputPacket);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrameId keyframe_id = 0u;
  Timestamp timestamp = 0u;
  Point3VectorConstPtr points;
  RgbaColorVectorConstPtr colors;
};

using DenseMapPointVisitor =
    std::function<void(const Point3& point, const Eigen::Vector4f& color)>;

class DenseMapView {
 public:
  KIMERA_POINTER_TYPEDEFS(DenseMapView);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~DenseMapView() = default;

  virtual std::string backendName() const = 0;
  virtual std::size_t pointCount() const = 0;
  virtual void visitPoints(const DenseMapPointVisitor& visitor) const = 0;
};

struct DenseMapOutput {
  KIMERA_POINTER_TYPEDEFS(DenseMapOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FrameId target_frame_id = 0u;
  Timestamp target_timestamp = 0u;
  std::string backend_name;
  std::size_t active_submap_id = 0u;
  std::size_t submap_count = 0u;
  std::size_t inserted_keyframes = 0u;
  std::size_t inserted_points = 0u;
  std::size_t map_points = 0u;
  DenseMapView::ConstPtr dense_map;
  float point_radius = 0.025f;
};

}  // namespace VIO
