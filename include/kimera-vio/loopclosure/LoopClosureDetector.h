#pragma once

#include "kimera-vio/loopclosure/LcdOutputPacket.h"
#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"

namespace VIO {

// ---------------------------------------------------------------------------

class LoopClosureDetectorBase {
 public:
  KIMERA_POINTER_TYPEDEFS(LoopClosureDetectorBase);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LoopClosureDetectorBase);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using IsBackendQueueFilledCallback = std::function<bool()>;

  LoopClosureDetectorBase() = default;
  virtual ~LoopClosureDetectorBase() = default;

  /* ------------------------------------------------------------------------
   */
  /**
   * @brief Register a loop closure between two frames in a threadsafe manner
   * @param[in] query_id most recent frame
   * @param[in] match_id previous frame that was matched to query
   */
  virtual LoopResult registerFrames(FrameId query_id, FrameId match_id) = 0;

  virtual LcdOutput::UniquePtr spinOnce(const LcdInput& input) = 0;

  /* ------------------------------------------------------------------------
   */
  /** @brief Register callback for checking the size of the input queue.
   * Knowing this can help determine when to optimize the factor graph and
   * when to wait for additional inputs to be added first.
   * @param[in] cb A callback function.
   */
  virtual inline void registerIsBackendQueueFilledCallback(
      const IsBackendQueueFilledCallback& cb) = 0;
};

}  // namespace VIO
