#include <tcl.h>
#include "opencv2/opencv.hpp"
#include <thread>
#include <mutex>
#include <chrono>
#include <random>
#include <unordered_map>

#include "Widget.h"
#include "WidgetManager.h"
#include "SharedQueue.hpp"
#include "SourceManager.h"
#include "FrameBufferManager.h"
#include "SamplingManager.h"
#include "VideoStream.h"

// SampleManager implementation

void SamplingManager::start(int n_frames, int interval_ms, bool random) {
  if (m_bActive) {
    stop();
  }

  if (m_thread.joinable()) {
    m_thread.join();
  }
    
  n_frames_ = n_frames;
  interval_ms_ = interval_ms;
  random_intervals_ = random;
  m_bDone = false;
  m_bActive = true;
  
  m_thread = std::thread(&SamplingManager::samplingLoop, this);
}

void SamplingManager::stop() {
  m_bDone = true;
  if (m_thread.joinable()) {
    m_thread.join();
  }
  m_bActive = false;
  m_bDone = false;
}

bool SamplingManager::isActive() { return m_bActive; }
    
void SamplingManager::samplingLoop() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(interval_ms_ / 2, interval_ms_ * 3 / 2);
  
  int sampled = 0;
  
  while (!m_bDone && sampled < n_frames_) {
    // Sample current frame
    cv::Mat frame_copy;
    FrameMetadata metadata;
    bool in_obs;
    
    if (prog_info_->frameBuffer->copyFrame(*(prog_info_->curFrame), 
					   frame_copy, metadata, in_obs)) {
      if (prog_info_->sourceManager->sampleCurrentFrame(frame_copy, metadata)) {
	sampled++;
	std::cout << "Sampled frame " << sampled << "/" << n_frames_ << std::endl;
      }
    }
    
    // Wait for next sample
    if (sampled < n_frames_) {
      int wait_time = random_intervals_ ? dis(gen) : interval_ms_;
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
    }
  }
  
  m_bActive = false;
  
  // Thread-safe read of callback
  std::string callback_copy;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    callback_copy = completion_callback_;
  }
  
  if (!callback_copy.empty()) {
    extern SharedQueue<std::string> mouse_queue;
    mouse_queue.push_back(callback_copy);
  }  
}
