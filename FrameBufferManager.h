#ifndef FRAME_BUFFER_MANAGER_H
#define FRAME_BUFFER_MANAGER_H

#include <vector>
#include <mutex>
#include <chrono>
#include "opencv2/opencv.hpp"
#include "IFrameSource.h"

class FrameBufferManager {
private:
    std::vector<cv::Mat> frames_;
    std::vector<bool> frame_in_obs_;
    std::vector<bool> frame_linestatus_;
    std::vector<int64_t> frame_ids_;
    std::vector<int64_t> frame_timestamps_;
    std::vector<std::chrono::high_resolution_clock::time_point> system_timestamps_;
    
    mutable std::mutex mutex_;
    int size_;
    
public:
    explicit FrameBufferManager(int size);
    
    // Thread-safe write
    void storeFrame(int index, const cv::Mat& frame, const FrameMetadata& metadata, bool in_obs);
    
    // Thread-safe read with copy (for sampling, saving, etc.)
    bool copyFrame(int index, cv::Mat& frame, FrameMetadata& metadata, bool& in_obs) const;
    
    // Thread-safe read without copy (for display - returns locked reference)
    // Caller must use the returned lock to keep the frame valid
    struct FrameAccess {
        const cv::Mat* frame;
        bool in_obs;
        bool linestatus;
        std::unique_lock<std::mutex> lock;
        
        bool isValid() const { return frame && !frame->empty(); }
    };

    
    /**
     * Access frame with lock held.
     * WARNING: The returned FrameAccess holds a lock until it's destroyed.
     * DO NOT call accessFrame() multiple times in the same expression or
     * while another FrameAccess is in scope!
     * 
     * WRONG (DEADLOCK):
     *   auto a = accessFrame(i); auto b = accessFrame(j);
     * 
     * RIGHT:
     *   { auto a = accessFrame(i); use(a); }  // Lock released
     *   { auto b = accessFrame(j); use(b); }  // Now safe
     * 
     * For simple queries, prefer convenience methods like getObservationPair().
     */
    FrameAccess accessFrame(int index) const;

    bool isInObs(int index) const;
    bool getLineStatus(int index) const;

    struct ObservationPair {
        bool cur_in_obs;
        bool prev_in_obs;
        bool cur_valid;
        bool prev_valid;
    };
    ObservationPair getObservationPair(int cur_idx, int prev_idx) const;
  
    // Get buffer size
    int size() const { return size_; }
    
    // Check if frame exists without locking (may race)
    bool hasFrame(int index) const;
};

#endif // FRAME_BUFFER_MANAGER_H
