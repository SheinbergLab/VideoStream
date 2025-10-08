// ReviewModeSource.h
#ifndef REVIEW_MODE_SOURCE_H
#define REVIEW_MODE_SOURCE_H

#include "IFrameSource.h"
#include <vector>
#include <opencv2/opencv.hpp>

class ReviewModeSource : public IFrameSource {
private:
    std::vector<cv::Mat> frames_;
    std::vector<FrameMetadata> metadata_;
    int current_index_;
    bool loop_;
    float playback_rate_;
    bool auto_advance_;
    
    // For auto-advance timing
    std::chrono::high_resolution_clock::time_point last_frame_time_;
    
public:
    ReviewModeSource();
    ~ReviewModeSource() override = default;
    
    // Collection management
    void addFrame(const cv::Mat& frame, const FrameMetadata& metadata);
    void clearFrames();
    void removeFrame(int index);
    int getFrameCount() const { return frames_.size(); }
    int getCurrentIndex() const { return current_index_; }
    
    // Navigation
    void nextFrame();
    void previousFrame();
    void jumpToFrame(int index);
    void setLoop(bool loop) { loop_ = loop; }
    void setAutoAdvance(bool enable, float fps = 1.0);
    
    // IFrameSource interface
    bool getNextFrame(cv::Mat& frame, FrameMetadata& metadata) override;
    bool isOpen() const override;      // Changed from isOpened()
    void close() override;
    int getWidth() const override;
    int getHeight() const override;
    float getFrameRate() const override;
    bool isColor() const override;
};

#endif // REVIEW_MODE_SOURCE_H
