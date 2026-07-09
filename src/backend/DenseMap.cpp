#include "kimera-vio/backend/DenseMap.h"

#include <glog/logging.h>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <utility>

#include "kimera-vio/utils/Timer.h"

namespace VIO {
namespace {

double elapsedMs(
    const std::chrono::high_resolution_clock::time_point& start_time) {
  return static_cast<double>(
             utils::Timer::toc<std::chrono::microseconds>(start_time).count()) /
         1000.0;
}

constexpr double kSlowDenseMapInsertMs = 20.0;
constexpr double kSlowDenseMapProcessMs = 30.0;

double colorToIntensity(const Eigen::Vector4f& color) {
  const double r = static_cast<double>(color[0]);
  const double g = static_cast<double>(color[1]);
  const double b = static_cast<double>(color[2]);
  return std::clamp((0.299 * r + 0.587 * g + 0.114 * b) / 255.0, 0.0, 1.0);
}

Eigen::Vector4f intensityToColor(const double intensity) {
  const float value =
      static_cast<float>(255.0 * std::clamp(intensity, 0.0, 1.0));
  return Eigen::Vector4f(value, value, value, 210.0f);
}

class GaussianVoxelDenseMapView final : public DenseMapView {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GaussianVoxelDenseMapView(
      gtsam_points::GaussianVoxelMapCPU::ConstPtr voxel_map,
      std::shared_ptr<std::mutex> mutex)
      : voxel_map_(std::move(voxel_map)), mutex_(std::move(mutex)) {}

  std::string backendName() const override {
    return denseMapBackendToString(DenseMapBackend::kGaussianVoxelMap);
  }

  std::size_t pointCount() const override {
    if (!voxel_map_) {
      return 0u;
    }
    const std::lock_guard<std::mutex> lock(*mutex_);
    return voxel_map_->num_voxels();
  }

  void visitPoints(const DenseMapPointVisitor& visitor) const override {
    if (!voxel_map_ || !visitor) {
      return;
    }
    const std::lock_guard<std::mutex> lock(*mutex_);
    const std::size_t num_voxels = voxel_map_->num_voxels();
    for (std::size_t voxel_id = 0u; voxel_id < num_voxels; ++voxel_id) {
      const std::size_t point_index = voxel_map_->calc_index(voxel_id, 0u);
      const Eigen::Vector4d& p = voxel_map_->point(point_index);
      const double intensity = voxel_map_->has_intensities()
                                   ? voxel_map_->intensity(point_index)
                                   : 0.7;
      visitor(Point3(p.x(), p.y(), p.z()), intensityToColor(intensity));
    }
  }

 private:
  gtsam_points::GaussianVoxelMapCPU::ConstPtr voxel_map_;
  std::shared_ptr<std::mutex> mutex_;
};

class GaussianVoxelDenseMap final : public DenseMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit GaussianVoxelDenseMap(const DenseMapParams& params)
      : params_(params),
        voxel_map_(std::make_shared<gtsam_points::GaussianVoxelMapCPU>(
            params.voxel_resolution)),
        mutex_(std::make_shared<std::mutex>()) {
    CHECK_GT(params_.voxel_resolution, 0.0);
  }

  std::string backendName() const override {
    return denseMapBackendToString(DenseMapBackend::kGaussianVoxelMap);
  }

  void insert(const DenseMapInputPacket::ConstPtr& packet) override {
    if (!packet || !packet->points || packet->points->empty()) {
      return;
    }

    const auto total_tic = utils::Timer::tic();
    const auto convert_tic = utils::Timer::tic();
    const Point3Vector& packet_points = *packet->points;
    const RgbaColorVector* packet_colors = packet->colors.get();
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
        points;
    points.reserve(packet_points.size());
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
        covariances;
    covariances.reserve(packet_points.size());
    std::vector<double> intensities;
    intensities.reserve(packet_points.size());
    const double covariance_sigma =
        std::max(1e-4, 0.5 * params_.voxel_resolution);
    Eigen::Matrix4d point_covariance = Eigen::Matrix4d::Zero();
    point_covariance.topLeftCorner<3, 3>().setIdentity();
    point_covariance.topLeftCorner<3, 3>() *=
        covariance_sigma * covariance_sigma;

    for (std::size_t i = 0u; i < packet_points.size(); ++i) {
      const Point3& p = packet_points[i];
      if (!std::isfinite(p.x()) || !std::isfinite(p.y()) ||
          !std::isfinite(p.z())) {
        continue;
      }
      points.emplace_back(p.x(), p.y(), p.z(), 1.0);
      covariances.push_back(point_covariance);
      if (packet_colors && i < packet_colors->size()) {
        intensities.push_back(colorToIntensity((*packet_colors)[i]));
      } else {
        intensities.push_back(0.7);
      }
    }

    if (points.empty()) {
      return;
    }
    const double convert_ms = elapsedMs(convert_tic);

    const auto cloud_tic = utils::Timer::tic();
    gtsam_points::PointCloudCPU cloud(points);
    cloud.add_covs(covariances);
    cloud.add_intensities(intensities);
    const double cloud_ms = elapsedMs(cloud_tic);

    const auto lock_tic = utils::Timer::tic();
    std::unique_lock<std::mutex> lock(*mutex_);
    const double lock_wait_ms = elapsedMs(lock_tic);

    const auto insert_tic = utils::Timer::tic();
    voxel_map_->insert(cloud);
    const double voxel_insert_ms = elapsedMs(insert_tic);
    ++inserted_keyframes_;
    inserted_points_ += points.size();
    const std::size_t map_points = voxel_map_->num_voxels();
    const double total_ms = elapsedMs(total_tic);

    if (total_ms > kSlowDenseMapInsertMs ||
        lock_wait_ms > kSlowDenseMapInsertMs ||
        voxel_insert_ms > kSlowDenseMapInsertMs) {
      LOG(WARNING) << "Dense map insert timing [slow]: keyframe_id="
                   << packet->keyframe_id
                   << ", input_points=" << packet_points.size()
                   << ", inserted_points=" << points.size()
                   << ", map_points=" << map_points
                   << ", convert_ms=" << convert_ms
                   << ", cloud_ms=" << cloud_ms
                   << ", lock_wait_ms=" << lock_wait_ms
                   << ", voxel_insert_ms=" << voxel_insert_ms
                   << ", total_ms=" << total_ms;
    } else {
      LOG_EVERY_N(INFO, 10)
          << "Dense map insert timing: keyframe_id=" << packet->keyframe_id
          << ", input_points=" << packet_points.size()
          << ", inserted_points=" << points.size()
          << ", map_points=" << map_points
          << ", convert_ms=" << convert_ms
          << ", cloud_ms=" << cloud_ms
          << ", lock_wait_ms=" << lock_wait_ms
          << ", voxel_insert_ms=" << voxel_insert_ms
          << ", total_ms=" << total_ms;
    }
  }

  DenseMapOutput::ConstPtr buildOutput(
      const FrameId target_frame_id,
      const Timestamp timestamp,
      const std::size_t active_submap_id,
      const std::size_t submap_count) const override {
    auto output = std::make_shared<DenseMapOutput>();
    output->target_frame_id = target_frame_id;
    output->target_timestamp = timestamp;
    output->backend_name = backendName();
    output->active_submap_id = active_submap_id;
    output->submap_count = submap_count;
    output->inserted_keyframes = inserted_keyframes_;
    output->inserted_points = inserted_points_;
    {
      const std::lock_guard<std::mutex> lock(*mutex_);
      output->map_points = voxel_map_->num_voxels();
    }
    output->dense_map =
        std::make_shared<GaussianVoxelDenseMapView>(voxel_map_, mutex_);
    output->point_radius = params_.point_radius;

    return output;
  }

 private:
  DenseMapParams params_;
  gtsam_points::GaussianVoxelMapCPU::Ptr voxel_map_;
  std::shared_ptr<std::mutex> mutex_;
  std::size_t inserted_keyframes_ = 0u;
  std::size_t inserted_points_ = 0u;
};

}  // namespace

DenseMapModule::DenseMapModule(const DenseMapParams& params)
    : params_(params) {
  CHECK(params_.enabled);
  CHECK_GT(params_.voxel_resolution, 0.0);
  submaps_.push_back(makeDenseMap(params_));
}

DenseMapOutput::ConstPtr DenseMapModule::process(
    const DenseMapInputPacket::ConstPtr& packet) {
  if (!params_.enabled || submaps_.empty() || !packet || !packet->points ||
      packet->points->empty()) {
    return nullptr;
  }

  const auto total_tic = utils::Timer::tic();
  const std::size_t active_submap_id = activeSubmapId(packet);
  CHECK_LT(active_submap_id, submaps_.size());
  const auto insert_tic = utils::Timer::tic();
  submaps_[active_submap_id]->insert(packet);
  const double insert_call_ms = elapsedMs(insert_tic);
  const auto output_tic = utils::Timer::tic();
  DenseMapOutput::ConstPtr output = submaps_[active_submap_id]->buildOutput(
      packet->keyframe_id,
      packet->timestamp,
      active_submap_id,
      submaps_.size());
  const double build_output_ms = elapsedMs(output_tic);
  const double total_ms = elapsedMs(total_tic);

  if (output) {
    if (total_ms > kSlowDenseMapProcessMs) {
      LOG(WARNING) << "Dense map process timing [slow]: keyframe_id="
                   << packet->keyframe_id
                   << ", input_points=" << packet->points->size()
                   << ", map_points=" << output->map_points
                   << ", active_submap=" << output->active_submap_id << "/"
                   << output->submap_count
                   << ", insert_call_ms=" << insert_call_ms
                   << ", build_output_ms=" << build_output_ms
                   << ", total_ms=" << total_ms;
    } else {
      LOG_EVERY_N(INFO, 10)
          << "Dense map process timing: keyframe_id=" << packet->keyframe_id
          << ", input_points=" << packet->points->size()
          << ", map_points=" << output->map_points
          << ", active_submap=" << output->active_submap_id << "/"
          << output->submap_count
          << ", insert_call_ms=" << insert_call_ms
          << ", build_output_ms=" << build_output_ms
          << ", total_ms=" << total_ms;
    }
    VLOG(1) << "Dense map [" << output->backend_name
            << "] active submap: " << output->active_submap_id << "/"
            << output->submap_count << ", keyframes: "
            << output->inserted_keyframes << ", inserted points: "
            << output->inserted_points << ", map points: "
            << output->map_points;
  }
  return output;
}

std::size_t DenseMapModule::activeSubmapId(
    const DenseMapInputPacket::ConstPtr&) const {
  // Placeholder submapping policy: keep every depth keyframe in a single map.
  return 0u;
}

DenseMap::UniquePtr DenseMapModule::makeDenseMap(
    const DenseMapParams& params) {
  switch (params.backend) {
    case DenseMapBackend::kGaussianVoxelMap:
      return std::make_unique<GaussianVoxelDenseMap>(params);
    default:
      LOG(FATAL) << "Unsupported dense map backend: "
                 << denseMapBackendToString(params.backend);
  }
  return nullptr;
}

}  // namespace VIO
