/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FrameCache.h
 * @brief  Disk-backed cache for frame features
 * @author Nathan Hughes
 */

#pragma once
#include <vector>

#include "kimera-vio/loopclosure/LoopClosureDetector-definitions.h"

namespace VIO {

struct FrameCacheConfig {
  int64_t max_frames = -1;
  std::string cache_path;
  std::string cache_name = ".kimera_lcd_frames";
  size_t num_frames_per_file = 15;
  bool remove_cache_on_exit = false;
};

struct FrameCacheImpl {
  virtual ~FrameCacheImpl() = default;

  virtual size_t addFrame(const LCDFrame::Ptr& frame) = 0;

  virtual LCDFrame::Ptr getFrame(size_t index) const = 0;

  virtual size_t size() const = 0;

  /// Returns the total memory usage of the cache and all stored frames in bytes
  virtual size_t getMemoryUsage() const = 0;

  /// Removes a frame from the cache by index
  virtual void removeFrame(size_t index) = 0;
};

class FrameCache {
 public:
  static constexpr size_t NEW_ID = 0;

  FrameCache();

  explicit FrameCache(const FrameCacheConfig& config);

  virtual ~FrameCache() = default;

  size_t addFrame(const LCDFrame::Ptr& frame) { return impl_->addFrame(frame); }

  LCDFrame::Ptr getFrame(size_t index) const { return impl_->getFrame(index); }

  size_t size() const { return impl_->size(); }

  /// Returns the total memory usage of the cache and all stored frames in bytes
  size_t getMemoryUsage() const { return impl_->getMemoryUsage(); }

  /// Removes a frame from the cache by index
  void removeFrame(size_t index) { impl_->removeFrame(index); }

 private:
  std::unique_ptr<FrameCacheImpl> impl_;
};

class InMemoryCacheImpl : public FrameCacheImpl {
 public:
  InMemoryCacheImpl();

  virtual ~InMemoryCacheImpl() = default;

  virtual size_t addFrame(const LCDFrame::Ptr& frame);

  virtual LCDFrame::Ptr getFrame(size_t index) const;

  virtual size_t size() const;

  virtual size_t getMemoryUsage() const;

  virtual void removeFrame(size_t index);

 private:
  std::vector<LCDFrame::Ptr> frames_;
};

class LRUCacheImpl : public FrameCacheImpl {
 public:
  struct CacheEntry {
    size_t slot;
    size_t last_used;
  };

  explicit LRUCacheImpl(const FrameCacheConfig& config);

  virtual ~LRUCacheImpl();

  virtual size_t addFrame(const LCDFrame::Ptr& frame);

  virtual LCDFrame::Ptr getFrame(size_t index) const;

  virtual size_t size() const;

  virtual size_t getMemoryUsage() const;

  virtual void removeFrame(size_t index);

 public:
  const FrameCacheConfig config;

 private:
  std::string getCacheFilepath(size_t i) const;

  void cacheLastFrame();

  size_t getNextSlot() const;

 private:
  size_t total_ = 0;
  LCDFrame::Ptr last_added_;
  std::list<LCDFrame::Ptr> to_archive_;

  mutable std::vector<std::vector<LCDFrame::Ptr>> loaded_;
  mutable std::map<size_t, CacheEntry> entries_;
};

}  // namespace VIO
