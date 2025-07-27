#pragma once

#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/frontend/Frame.h"

namespace VIO {
class LcdLandmarkManager : public std::map<LandmarkId, Landmark> {
 public:
  KIMERA_POINTER_TYPEDEFS(LcdLandmarkManager);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LcdLandmarkManager);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LcdLandmarkManager() : std::map<LandmarkId, Landmark>() {}
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
};
}  // namespace VIO