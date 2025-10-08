// ReviewModeSource.cpp
#include "ReviewModeSource.h"
#include <iostream>

ReviewModeSource::ReviewModeSource()
    : current_index_(0)
    , loop_(false)
    , playback_rate_(1.0f)
    , auto_advance_(false)
    , last_frame_time_(std::chrono::high_resolution_clock::now())
{
}

void ReviewModeSource::addFrame(const cv::Mat& frame, const FrameMetadata& metadata)
{
    if (frame.empty()) {
        std::cerr << "Warning: Attempted to add empty frame to review source" << std::endl;
        return;
    }
    
    frames_.push_back(frame.clone());
    metadata_.push_back(metadata);
    
    std::cout << "Added frame to review source (total: " << frames_.size() << ")" << std::endl;
}

void ReviewModeSource::clearFrames()
{
    frames_.clear();
    metadata_.clear();
    current_index_ = 0;
    std::cout << "Review source cleared" << std::endl;
}

void ReviewModeSource::removeFrame(int index)
{
    if (index < 0 || index >= (int)frames_.size()) {
        std::cerr << "Invalid frame index: " << index << std::endl;
        return;
    }
    
    frames_.erase(frames_.begin() + index);
    metadata_.erase(metadata_.begin() + index);
    
    // Adjust current index if needed
    if (current_index_ >= (int)frames_.size() && !frames_.empty()) {
        current_index_ = frames_.size() - 1;
    } else if (frames_.empty()) {
        current_index_ = 0;
    }
    
    std::cout << "Removed frame " << index << " (remaining: " << frames_.size() << ")" << std::endl;
}

void ReviewModeSource::nextFrame()
{
    if (frames_.empty()) return;
    
    current_index_++;
    if (current_index_ >= (int)frames_.size()) {
        if (loop_) {
            current_index_ = 0;
        } else {
            current_index_ = frames_.size() - 1;
        }
    }
}

void ReviewModeSource::previousFrame()
{
    if (frames_.empty()) return;
    
    current_index_--;
    if (current_index_ < 0) {
        if (loop_) {
            current_index_ = frames_.size() - 1;
        } else {
            current_index_ = 0;
        }
    }
}

void ReviewModeSource::jumpToFrame(int index)
{
    if (frames_.empty()) return;
    
    if (index < 0) {
        current_index_ = 0;
    } else if (index >= (int)frames_.size()) {
        current_index_ = frames_.size() - 1;
    } else {
        current_index_ = index;
    }
}

void ReviewModeSource::setAutoAdvance(bool enable, float fps)
{
    auto_advance_ = enable;
    if (fps > 0) {
        playback_rate_ = fps;
    }
    last_frame_time_ = std::chrono::high_resolution_clock::now();
}

bool ReviewModeSource::getNextFrame(cv::Mat& frame, FrameMetadata& metadata)
{
    if (frames_.empty() || current_index_ < 0 || current_index_ >= (int)frames_.size()) {
        return false;
    }
    
    // Handle auto-advance timing
    if (auto_advance_ && playback_rate_ > 0) {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_frame_time_).count();
        
        int frame_interval_ms = (int)(1000.0f / playback_rate_);
        
        if (elapsed >= frame_interval_ms) {
            nextFrame();
            last_frame_time_ = now;
        }
    }
    
    frame = frames_[current_index_].clone();
    metadata = metadata_[current_index_];
    
    return true;
}

bool ReviewModeSource::isOpen() const
{
    return !frames_.empty();
}

void ReviewModeSource::close()
{
    clearFrames();
}

int ReviewModeSource::getWidth() const
{
    if (frames_.empty()) return 0;
    return frames_[0].cols;
}

int ReviewModeSource::getHeight() const
{
    if (frames_.empty()) return 0;
    return frames_[0].rows;
}

float ReviewModeSource::getFrameRate() const
{
    return playback_rate_;
}

bool ReviewModeSource::isColor() const
{
    if (frames_.empty()) return true;
    return frames_[0].channels() > 1;
}
