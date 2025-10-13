#include "FrameBufferManager.h"

FrameBufferManager::FrameBufferManager(int size) 
    : size_(size)
{
    frames_.resize(size);
    frame_in_obs_.resize(size);
    frame_linestatus_.resize(size);
    frame_ids_.resize(size);
    frame_timestamps_.resize(size);
    system_timestamps_.resize(size);
}

void FrameBufferManager::storeFrame(int index, const cv::Mat& frame, 
                                    const FrameMetadata& metadata, bool in_obs)
{
    if (index < 0 || index >= size_) return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    frames_[index] = frame;  // Shallow copy is fine - OpenCV Mat is reference-counted
    frame_in_obs_[index] = in_obs;
    frame_linestatus_[index] = metadata.lineStatus;
    frame_ids_[index] = metadata.frameID;
    frame_timestamps_[index] = metadata.timestamp;
    system_timestamps_[index] = metadata.systemTime;
}

bool FrameBufferManager::copyFrame(int index, cv::Mat& frame, 
                                   FrameMetadata& metadata, bool& in_obs) const
{
    if (index < 0 || index >= size_) return false;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (frames_[index].empty()) {
        return false;
    }
    
    frame = frames_[index].clone();  // Deep copy
    metadata.frameID = frame_ids_[index];
    metadata.timestamp = frame_timestamps_[index];
    metadata.systemTime = system_timestamps_[index];
    metadata.lineStatus = frame_linestatus_[index];
    in_obs = frame_in_obs_[index];
    
    return true;
}

FrameBufferManager::FrameAccess FrameBufferManager::accessFrame(int index) const
{
    FrameAccess access;
    access.lock = std::unique_lock<std::mutex>(mutex_);
    
    if (index >= 0 && index < size_ && !frames_[index].empty()) {
        access.frame = &frames_[index];
        access.in_obs = frame_in_obs_[index];
        access.linestatus = frame_linestatus_[index];
    } else {
        access.frame = nullptr;
        access.in_obs = false;
        access.linestatus = false;
    }
    
    return access;  // Lock moves with the return value
}

bool FrameBufferManager::hasFrame(int index) const
{
    if (index < 0 || index >= size_) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    return !frames_[index].empty();
}

bool FrameBufferManager::isInObs(int index) const {
  if (index < 0 || index >= size_) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_in_obs_[index];
}

bool FrameBufferManager::getLineStatus(int index) const {
  if (index < 0 || index >= size_) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_linestatus_[index];
}

FrameBufferManager::ObservationPair 
FrameBufferManager::getObservationPair(int cur_idx, int prev_idx) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  ObservationPair result;
  result.cur_valid = (cur_idx >= 0 && cur_idx < size_ && !frames_[cur_idx].empty());
  result.prev_valid = (prev_idx >= 0 && prev_idx < size_ && !frames_[prev_idx].empty());
  result.cur_in_obs = result.cur_valid ? frame_in_obs_[cur_idx] : false;
  result.prev_in_obs = result.prev_valid ? frame_in_obs_[prev_idx] : false;
  
  return result;
}

