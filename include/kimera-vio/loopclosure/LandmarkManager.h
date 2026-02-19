#pragma once

#include <kimera-vio/loopclosure/FrameCache.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/UndistorterRectifier.h"

namespace VIO {
class LcdLandmarkManager : public std::unordered_map<LandmarkId, Landmark> {
 public:
  using Base = std::unordered_map<LandmarkId, Landmark>;

  KIMERA_POINTER_TYPEDEFS(LcdLandmarkManager);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LcdLandmarkManager);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LcdLandmarkManager() : Base() {}
  virtual ~LcdLandmarkManager() = default;

  void updateLandmarks(const PointsWithIdMap& smoother_points_with_ids,
                       const gtsam::Pose3& W_Pose_smoother = gtsam::Pose3(),
                       const LmkMapWithStats* backend_stats = nullptr) {
    for (auto const& point_with_id : smoother_points_with_ids) {
      LandmarkId lmk_id = point_with_id.first;
      Landmark lmk_in_smoother = point_with_id.second;
      if (this->find(lmk_id) == this->end()) {
        // If the landmark does not exist, add it.
        this->emplace(lmk_id, W_Pose_smoother * lmk_in_smoother);
      } else {
        // If it exists, update the landmark.
        this->at(lmk_id) = W_Pose_smoother * lmk_in_smoother;
      }

      if (backend_stats) {
        auto obs_it = backend_stats->num_observations.find(lmk_id);
        if (obs_it != backend_stats->num_observations.end()) {
          landmark_backend_num_obs_[lmk_id] = obs_it->second;
        }
        auto res_it = backend_stats->residuals.find(lmk_id);
        if (res_it != backend_stats->residuals.end()) {
          landmark_backend_residuals_[lmk_id] = res_it->second;
        }
      }
    }
  }

  size_t getNumObs(const LandmarkId& lmk_id) const {
    auto it = landmark_backend_num_obs_.find(lmk_id);
    return it != landmark_backend_num_obs_.end() ? it->second : 0u;
  }

  double getResidual(const LandmarkId& lmk_id) const {
    auto it = landmark_backend_residuals_.find(lmk_id);
    return it != landmark_backend_residuals_.end() ? it->second : 0.0;
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

  int checkAndCullingLandmarks(const std::set<LandmarkId>& lmk_ids,
                               const FrameCache& frame_cache,
                               double min_obs_ratio,
                               float min_parallax,
                               float max_reproj_error,
                               int min_obs_cnt) {
    int culled_obs_ratio = 0, culled_parallax = 0, culled_reproj_error = 0;
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
      FrameIdSet obs_frames = landmark_obs_frame_ids_[lmk_id];
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
      bool bad_reproj = false;
      for (FrameId q_frame = first_frame; q_frame <= last_frame; ++q_frame) {
        auto q_frame_ptr = frame_cache.getFrame(q_frame);
        if (!q_frame_ptr) continue;  // Skip frames that have been cleaned

        // if not already observed, project to see if it should observe.
        cv::Point2f uv;
        if (shouldObserve(lmk, *q_frame_ptr, &uv)) {
          should_observe++;
          uvs.emplace_back(uv);

          auto obs_it = obs_frames.find(q_frame);
          if (obs_it != obs_frames.end()) {
            auto obs_frame_ptr = frame_cache.getFrame(*obs_it);
            if (!obs_frame_ptr) continue;  // Skip if frame has been cleaned
            std::vector<cv::Point2f> kp;
            bool found_kp = false;
            for (size_t lmk_i = 0; lmk_i < obs_frame_ptr->landmark_ids.size();
                 lmk_i++) {
              if (obs_frame_ptr->landmark_ids[lmk_i] == lmk_id) {
                UndistorterRectifier::UndistortRectifyKeypoints(
                    {obs_frame_ptr->keypoints_[lmk_i].pt},
                    &kp,
                    obs_frame_ptr->cam_params_);
                kp[0].x =
                    kp[0].x * obs_frame_ptr->cam_params_.K_.at<double>(0, 0) +
                    obs_frame_ptr->cam_params_.K_.at<double>(0, 2);
                kp[0].y =
                    kp[0].y * obs_frame_ptr->cam_params_.K_.at<double>(1, 1) +
                    obs_frame_ptr->cam_params_.K_.at<double>(1, 2);
                found_kp = true;
                break;
              }
            }
            CHECK(found_kp);
            double reproj_error =
                (uv - kp[0]).dot(uv - kp[0]);  // squared error
            if (std::sqrt(reproj_error) >= max_reproj_error) {
              bad_reproj = true;
              break;
            }
          }
        }
      }

      if (should_observe != 0) {
        double obs_ratio =
            obs_frames.size() / static_cast<double>(should_observe);
        if (obs_ratio < min_obs_ratio or
            obs_frames.size() < static_cast<size_t>(min_obs_cnt)) {
          // Cull the landmark
          eraseLandmark(lmk_id);
          culled_obs_ratio++;
          VLOG(1) << "Culled landmark " << lmk_id
                  << " obs cnt: " << obs_frames.size()
                  << ", covis: " << should_observe
                  << ", obs ratio: " << obs_ratio
                  << " threshold: " << min_obs_ratio << " , " << min_obs_cnt;
          continue;
        }
      } else {
        // Cull the landmark
        eraseLandmark(lmk_id);
        culled_obs_ratio++;
        continue;
      }

      CHECK((!uvs.empty()) or (uvs.empty() and should_observe == 0))
          << "LoopClosureDetector: No uvs found for landmark " << lmk_id
          << ", should_observe: " << should_observe;

      // compute max parallax from uv
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
      if ((max_u_diff < min_parallax and max_v_diff < min_parallax) or
          bad_reproj) {
        // Cull the landmark
        eraseLandmark(lmk_id);
        if (bad_reproj) {
          culled_reproj_error++;
        } else {
          culled_parallax++;
        }
        continue;
      }

      // If we reach here, the landmark is valid
      landmarks_valid_[lmk_id] = true;
    }
    VLOG(1) << "culled_obs_ratio: " << culled_obs_ratio
            << ", culled_parallax: " << culled_parallax
            << ", culled_reproj_error: " << culled_reproj_error;
    return culled_obs_ratio + culled_parallax + culled_reproj_error;
  }

  Landmarks getLandmarks() const {
    Landmarks landmarks;
    for (const auto& [lmk_id, lmk] : *this) {
      landmarks.emplace_back(lmk);
    }
    return landmarks;
  }

  size_t removeLandmarksUntil(FrameId frame_id) {
    size_t removed_count = 0;
    for (auto it = this->begin(); it != this->end();) {
      auto lmk_id = it->first;
      if (landmark_obs_frame_ids_.count(lmk_id)) {
        auto obs_frames = landmark_obs_frame_ids_[lmk_id];
        auto earliest_obs_frame =
            *std::min_element(obs_frames.begin(), obs_frames.end());
        if (earliest_obs_frame < frame_id) {
          it = this->erase(it);
          landmark_obs_frame_ids_.erase(lmk_id);
          landmark_backend_num_obs_.erase(lmk_id);
          landmark_backend_residuals_.erase(lmk_id);
          removed_count++;
        } else {
          ++it;
        }
      } else {
        ++it;
      }
    }
    return removed_count;
  }

  FrameId getOldestCovisFrame(FrameId frame_id) const {
    if (covis_graph_.find(frame_id) == covis_graph_.end()) {
      return frame_id;
    } else {
      return *std::min_element(covis_graph_.at(frame_id).begin(),
                               covis_graph_.at(frame_id).end());
    }
  }

  auto const& getCovisGraph() const { return covis_graph_; }

  double computeCovisibilityScore(const FrameId& source_frame,
                                  const FrameId& target_frame) const {
    // compute what percentage of landmarks observed in source_frame are also
    // observed in target_frame

    // Find all landmarks observed in source_frame
    std::set<LandmarkId> source_landmarks;
    for (const auto& [lmk_id, frame_ids] : landmark_obs_frame_ids_) {
      if (frame_ids.find(source_frame) != frame_ids.end()) {
        source_landmarks.insert(lmk_id);
      }
    }

    if (source_landmarks.empty()) {
      return 0.0;
    }

    // Count how many of these landmarks are also observed in target_frame
    size_t shared_count = 0;
    for (const auto& lmk_id : source_landmarks) {
      const auto& frame_ids = landmark_obs_frame_ids_.at(lmk_id);
      if (frame_ids.find(target_frame) != frame_ids.end()) {
        shared_count++;
      }
    }

    return static_cast<double>(shared_count) /
           static_cast<double>(source_landmarks.size());
  }

 private:
  void eraseLandmark(const LandmarkId& lmk_id) {
    this->erase(lmk_id);
    landmark_obs_frame_ids_.erase(lmk_id);
    landmark_backend_num_obs_.erase(lmk_id);
    landmark_backend_residuals_.erase(lmk_id);
  }

 public:
  std::map<LandmarkId, bool> landmarks_valid_{};
  std::map<LandmarkId, FrameIdSet> landmark_obs_frame_ids_{};  // lcd frame ids
  std::map<FrameId, FrameIdSet> covis_graph_{};
  LmkIdToNumObsMap landmark_backend_num_obs_{};
  LmkIdToResidualMap landmark_backend_residuals_{};
};
}  // namespace VIO