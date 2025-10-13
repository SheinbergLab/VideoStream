#ifndef IFRAME_SOURCE_H
#define IFRAME_SOURCE_H

#include "opencv2/opencv.hpp"
#include <cstdint>

struct FrameMetadata {
    int64_t frameID;
    int64_t timestamp;
    bool lineStatus;
    std::chrono::high_resolution_clock::time_point systemTime;
};

class IFrameSource {
public:
  virtual ~IFrameSource() = default;
  
  virtual bool getNextFrame(cv::Mat& frame, FrameMetadata& metadata) = 0;
  virtual bool isOpen() const = 0;
  virtual float getFrameRate() const = 0;
  virtual int getWidth() const = 0;
  virtual int getHeight() const = 0;
  virtual bool isColor() const = 0;
  virtual void close() = 0;
  virtual bool isPlaybackMode() const { return false; }
  virtual bool isLooping() const { return true; }  
};

#endif
