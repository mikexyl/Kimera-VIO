#pragma once

#include <memory>

#include "kimera-vio/common/DenseMapTypes.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class DenseMap {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(DenseMap);
  KIMERA_POINTER_TYPEDEFS(DenseMap);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DenseMap() = default;
  virtual ~DenseMap() = default;

  virtual std::string backendName() const = 0;
  virtual void insert(const DenseMapInputPacket::ConstPtr& packet) = 0;
  virtual DenseMapOutput::ConstPtr buildOutput(FrameId target_frame_id,
                                               Timestamp timestamp,
                                               std::size_t active_submap_id,
                                               std::size_t submap_count) const = 0;
};

class DenseMapModule {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(DenseMapModule);
  KIMERA_POINTER_TYPEDEFS(DenseMapModule);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit DenseMapModule(const DenseMapParams& params);
  ~DenseMapModule() = default;

  DenseMapOutput::ConstPtr process(const DenseMapInputPacket::ConstPtr& packet);

 private:
  std::size_t activeSubmapId(const DenseMapInputPacket::ConstPtr& packet) const;
  static DenseMap::UniquePtr makeDenseMap(const DenseMapParams& params);

 private:
  DenseMapParams params_;
  std::vector<DenseMap::UniquePtr> submaps_;
};

}  // namespace VIO
