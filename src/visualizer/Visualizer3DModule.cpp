/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Visualizer3DModule.cpp
 * @brief  Pipeline Module for the 3D visualizer.
 * @author Antoni Rosinol
 */

#include "kimera-vio/visualizer/Visualizer3DModule.h"

#include <string>
#include <utility>

namespace VIO {

VisualizerModule::VisualizerModule(OutputQueue* output_queue,
                                   bool parallel_run,
                                   bool use_lcd,
                                   Visualizer3D::UniquePtr visualizer)
    : MISOPipelineModule<VisualizerInput, DisplayInputBase>(output_queue,
                                                            "Visualizer",
                                                            parallel_run),
      frontend_queue_("visualizer_frontend_queue"),
      backend_queue_("visualizer_backend_queue"),
      visualizer_(std::move(visualizer)) {
  if (visualizer_->visualization_type_ ==
      VisualizationType::kMesh2dTo3dSparse) {
    throw std::runtime_error(
        "Meshing is deleted for removing opencv contrib "
        "dependency.");
  }
}

VisualizerModule::InputUniquePtr VisualizerModule::getInputPacket() {
  bool queue_state = false;
  VizBackendInput backend_payload = nullptr;
  if (PIO::parallel_run_) {
    queue_state = backend_queue_.popBlocking(backend_payload);
  } else {
    queue_state = backend_queue_.pop(backend_payload);
  }

  if (!queue_state) {
    std::string msg = "Module: " + name_id_ + " - " + backend_queue_.queue_id_ +
                      " queue is down";
    LOG_IF(WARNING, PIO::parallel_run_) << msg;
    VLOG_IF(1, !PIO::parallel_run_) << msg;
    return nullptr;
  }

  CHECK(backend_payload);
  const Timestamp& timestamp = backend_payload->timestamp_;

  // Look for the synchronized packet in Frontend payload queue
  // This should always work, because it should not be possible to have
  // a Backend payload without having a Frontend one first!
  VizFrontendInput frontend_payload = nullptr;
  PIO::syncQueue(timestamp, &frontend_queue_, &frontend_payload);
  CHECK(frontend_payload);
  CHECK(frontend_payload->is_keyframe_);

  // Push the synced messages to the visualizer's input queue
  return std::make_unique<VisualizerInput>(
      timestamp, backend_payload, frontend_payload);
}

VisualizerModule::OutputUniquePtr VisualizerModule::spinOnce(
    VisualizerInput::UniquePtr input) {
  CHECK(input);
  return visualizer_->spinOnce(*input);
}

void VisualizerModule::shutdownQueues() {
  LOG(INFO) << "Shutting down queues for: " << name_id_;
  frontend_queue_.shutdown();
  backend_queue_.shutdown();
  // This shutdowns the output queue as well.
  MISO::shutdownQueues();
}

//! Checks if the module has work to do (should check input queues are empty)
bool VisualizerModule::hasWork() const { return false; }
}  // namespace VIO
