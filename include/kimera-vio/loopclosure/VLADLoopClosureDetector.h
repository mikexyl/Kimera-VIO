#pragma once

#include "kimera-vio/loopclosure/LoopClosureDetector.h"

namespace VIO {

class VLADLoopClosureDetector : public LoopClosureDetector {
 public:
  KIMERA_POINTER_TYPEDEFS(VLADLoopClosureDetector);
  KIMERA_DELETE_COPY_CONSTRUCTORS(VLADLoopClosureDetector);

  template <typename... Args>
  VLADLoopClosureDetector(Args&&... args)
      : LoopClosureDetector(std::forward<Args>(args)...) {}

  /* ------------------------------------------------------------------------
   */
  virtual ~VLADLoopClosureDetector() override = default;
};

}  // namespace VIO