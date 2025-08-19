#pragma once

#include <kimera-vio/loopclosure/FrameCache.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/frontend/Frame.h"

namespace VIO {
class LcdLandmarkManager : public std::unordered_map<LandmarkId, Landmark> {
 public:
  using Base = std::unordered_map<LandmarkId, Landmark>;

  KIMERA_POINTER_TYPEDEFS(LcdLandmarkManager);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LcdLandmarkManager);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LcdLandmarkManager() : Base() {}
  virtual ~LcdLandmarkManager() = default;

  void updateLandmarks(const PointsWithIdMap& W_points_with_ids) {
    for (auto const& point_with_id : W_points_with_ids) {
      LandmarkId lmk_id = point_with_id.first;
      Landmark lmk = point_with_id.second;
      if (this->find(lmk_id) == this->end()) {
        // If the landmark does not exist, add it.
        this->emplace(lmk_id, lmk);
      } else {
        // If it exists, update the landmark.
        this->at(lmk_id) = lmk;
      }
    }
  }

  std::optional<Landmark> getLandmark(const LandmarkId& lmk_id) const {
    auto it = this->find(lmk_id);
    if (it != this->end()) {
      return it->second;
    }
    return std::nullopt;  // Return empty optional if not found.
  }

  void updateObsFrames(const FrameId& frame_id,
                       const std::vector<LandmarkId>& lmk_ids) {
    std::set<FrameId> covis_frames;
    for (LandmarkId lmk_id : lmk_ids) {
      if (lmk_id < 0) continue;
      auto it = landmark_obs_frame_ids_.find(lmk_id);
      if (it != landmark_obs_frame_ids_.end()) {
        for (auto const& covis_frame : it->second) {
          covis_frames.insert(covis_frame);
        }

        // If the landmark exists, update its observation frame ids.
        it->second.insert(frame_id);
      } else {
        landmark_obs_frame_ids_.emplace(lmk_id, FrameIdSet{frame_id});
      }
    }

    // update covis_graph_
    for (auto const& covis_frame : covis_frames) {
      covis_graph_[frame_id].insert(covis_frame);
      covis_graph_[covis_frame].insert(frame_id);
    }
  }

  // Checks if a landmark is in the camera's field of view.
  bool isLandmarkInFov(const Point3& lmk_in_cam,
                       const CameraParams& cam_params,
                       cv::Point2f* uv) const {
    // Example implementation: checks if the landmark is in front of the camera
    // and within image bounds. You may need to adjust this logic based on your
    // CameraParams and projection model.
    if (lmk_in_cam.z() <= 0) return false;  // Landmark behind camera

    double fx = cam_params.K_.at<double>(0, 0);
    double fy = cam_params.K_.at<double>(1, 1);
    double cx = cam_params.K_.at<double>(0, 2);
    double cy = cam_params.K_.at<double>(1, 2);

    // Project to image plane (simple pinhole model)
    double u = fx * lmk_in_cam.x() / lmk_in_cam.z() + cx;
    double v = fy * lmk_in_cam.y() / lmk_in_cam.z() + cy;

    if (uv) {
      uv->x = static_cast<float>(u);
      uv->y = static_cast<float>(v);
    }

    return (u >= 0 && u < cam_params.image_size_.width) && v >= 0 &&
           v < cam_params.image_size_.height;
  }

  bool shouldObserve(Landmark lmk, const LCDFrame& lcd_frame, cv::Point2f* uv) {
    Pose3 B_Pose_Cam = lcd_frame.cam_params_.body_Pose_cam_;
    Point3 lmk_in_cam = (lcd_frame.W_Pose_Blkf_ * B_Pose_Cam).inverse() * lmk;

    // check if lmk is in fov
    if (isLandmarkInFov(lmk_in_cam, lcd_frame.cam_params_, uv)) {
      return true;
    }
    return false;
  }

  int checkAndCullingLandmarks(const std::vector<LandmarkId>& lmk_ids,
                               const FrameCache& frame_cache,
                               double min_obs_ratio,
                               float min_parallex) {
    int culled = 0;
    for (const auto& lmk_id : lmk_ids) {
      // already checked, and it's valid
      if (landmarks_valid_.find(lmk_id) != landmarks_valid_.end() and
          landmarks_valid_[lmk_id]) {
        continue;
      }

      if (landmark_obs_frame_ids_.find(lmk_id) ==
          landmark_obs_frame_ids_.end()) {
        continue;
      }
      if (this->find(lmk_id) == this->end()) {
        continue;  // skip if no observation frames
      }
      Landmark lmk = this->at(lmk_id);
      FrameIdSet& obs_frames = landmark_obs_frame_ids_[lmk_id];
      FrameIdSet covis_frames;
      // collect all covis frames
      for (const auto& obs_frame : obs_frames) {
        CHECK(covis_graph_.find(obs_frame) != covis_graph_.end());
        for (const auto& covis_frame : covis_graph_[obs_frame]) {
          covis_frames.insert(covis_frame);
        }
      }

      // find the range of frames to check
      FrameId first_frame = *covis_frames.begin(),
              last_frame = *covis_frames.rbegin();

      int should_observe = 0;

      std::vector<cv::Point2f> uvs;

      for (FrameId q_frame = first_frame; q_frame <= last_frame; ++q_frame) {
        auto q_frame_ptr = frame_cache.getFrame(q_frame);

        // if not already observed, project to see if it should observe.
        cv::Point2f uv;
        if (shouldObserve(lmk, *q_frame_ptr, &uv)) {
          should_observe++;
          uvs.emplace_back(uv);
        }
      }
      VLOG(1) << "lmk: " << lmk_id
              << ", covis frame size: " << last_frame - first_frame
              << ", should observe: " << should_observe
              << ", observed: " << obs_frames.size();

      if (should_observe != 0) {
        double obs_ratio =
            obs_frames.size() / static_cast<double>(should_observe);
        if (obs_ratio < min_obs_ratio) {
          // Cull the landmark
          this->erase(lmk_id);
          landmark_obs_frame_ids_.erase(lmk_id);
          culled++;
          continue;
        }
      } else {
        // Cull the landmark
        this->erase(lmk_id);
        landmark_obs_frame_ids_.erase(lmk_id);
        culled++;
        continue;
      }

      CHECK((!uvs.empty()) or (uvs.empty() and should_observe == 0))
          << "LoopClosureDetector: No uvs found for landmark " << lmk_id
          << ", should_observe: " << should_observe;

      // compute max parallex from uv
      float max_u_diff = 0.0f, max_v_diff = 0.0f;
      std::vector<float> us, vs;
      for (const auto& uv : uvs) {
        us.emplace_back(uv.x);
        vs.emplace_back(uv.y);
      }
      max_u_diff = *std::max_element(us.begin(), us.end()) -
                   *std::min_element(us.begin(), us.end());
      max_v_diff = *std::max_element(vs.begin(), vs.end()) -
                   *std::min_element(vs.begin(), vs.end());
      if (max_u_diff < min_parallex and max_v_diff < min_parallex) {
        // Cull the landmark
        this->erase(lmk_id);
        landmark_obs_frame_ids_.erase(lmk_id);
        culled++;
        continue;
      }

      // If we reach here, the landmark is valid
      landmarks_valid_[lmk_id] = true;
    }
    return culled;
  }

  Landmarks getLandmarks() const {
    Landmarks landmarks;
    for (const auto& [lmk_id, lmk] : *this) {
      landmarks.emplace_back(lmk);
    }
    return landmarks;
  }

  auto const& getCovisGraph() const { return covis_graph_; }

  std::map<LandmarkId, bool> landmarks_valid_{};
  std::map<LandmarkId, FrameIdSet> landmark_obs_frame_ids_{};  // lcd frame ids
  std::map<FrameId, FrameIdSet> covis_graph_{};
};
}  // namespace VIO