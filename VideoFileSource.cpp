#include <thread>
#include "VideoFileSource.h"
#include <iostream>
#include <dynio.h>

extern void fireEvent(const std::string& type, const std::string& data);

VideoFileSource::VideoFileSource(const std::string& videoFile,
                                 const std::string& dgzFile,
                                 float playbackSpeed,
                                 bool rateLimited,
				 bool loopPlayback)
    : metadata_dg(nullptr, [](DYN_GROUP*){ dgCloseBuffer(); })
    , stored_frameIDs(nullptr)
    , stored_timestamps(nullptr)
    , stored_linestatus(nullptr)
    , metadata_length(0)
    , current_idx(0)
    , playback_speed(playbackSpeed)
    , rate_limit(rateLimited)
    , loop_playback(loopPlayback)
    , default_frameID(0)
    , has_metadata(false)
{
    cap.open(videoFile);
    if (!cap.isOpened()) {
        throw std::runtime_error("Failed to open video file: " + videoFile);
    }
    
    width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    fps = cap.get(cv::CAP_PROP_FPS);
    if (fps == 0.0) fps = 30.0;
    
    // Try to determine if color (read first frame and check)
    cv::Mat test_frame;
    cap >> test_frame;
    color = (test_frame.channels() > 1);
    cap.set(cv::CAP_PROP_POS_FRAMES, 0); // Rewind
    
    if (!dgzFile.empty()) {
      //        has_metadata = loadMetadata(dgzFile);
    }
    
    playback_start = std::chrono::high_resolution_clock::now();
}

bool VideoFileSource::loadMetadata(const std::string& dgzFile) {
  return true;
}

bool VideoFileSource::getNextFrame(cv::Mat& frame, FrameMetadata& metadata) {
  // Rate limiting for realistic playback
  if (rate_limit && current_idx > 0) {
    auto target_time = playback_start + 
      std::chrono::microseconds((int64_t)(current_idx * 1e6 / (fps * playback_speed)));
    std::this_thread::sleep_until(target_time);
  }
  
  cap >> frame;
  if (frame.empty()) {
    if (loop_playback) {
      rewind();
      cap >> frame;
      
      // If still empty after rewind, file is unreadable
      if (frame.empty()) {
	return false;
      }
    } else {
      return false;
    }    
  }
  
  metadata.systemTime = std::chrono::high_resolution_clock::now();
  
  if (has_metadata && current_idx < metadata_length) {
    // Use stored metadata
    metadata.frameID = stored_frameIDs[current_idx];
    metadata.timestamp = stored_timestamps[current_idx];
    metadata.lineStatus = stored_linestatus ? 
      (bool)stored_linestatus[current_idx] : false;
  } else {
    // Generate default metadata
    metadata.frameID = default_frameID++;
    metadata.timestamp = (int64_t)(current_idx * 1e9 / fps);
    metadata.lineStatus = false;
  }
  
  current_idx++;
  return true;
}

void VideoFileSource::rewind() {
  cap.set(cv::CAP_PROP_POS_FRAMES, 0);
  current_idx = 0;
  default_frameID = 0;
  playback_start = std::chrono::high_resolution_clock::now();
  
  // Fire event to notify plugins/UI
  fireEvent("video_source_rewind", "");
}

VideoFileSource::~VideoFileSource() {
  close();
  // metadata_dg will be cleaned up by unique_ptr with custom deleter
}
