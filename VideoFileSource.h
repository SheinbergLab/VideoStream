#ifndef VIDEO_FILE_SOURCE_H
#define VIDEO_FILE_SOURCE_H

#include "IFrameSource.h"
#include <df.h>
#include <string>
#include <memory>

class VideoFileSource : public IFrameSource {
private:
  cv::VideoCapture cap;
  std::unique_ptr<DYN_GROUP, void(*)(DYN_GROUP*)> metadata_dg;
  
  // Metadata arrays
  int64_t* stored_frameIDs;
  int64_t* stored_timestamps;
  unsigned char* stored_linestatus;
  int metadata_length;
  
  int current_idx;
  float fps;
  int width, height;
  bool color;
  bool has_metadata;
  
  // Playback control
  std::chrono::high_resolution_clock::time_point playback_start;
  float playback_speed;
  bool rate_limit;
  bool loop_playback;
  bool paused;
  
  int64_t default_frameID;
  
  bool loadMetadata(const std::string& dgzFile);
  
public:
  VideoFileSource(const std::string& videoFile, 
		  const std::string& dgzFile = "",
		  float playbackSpeed = 1.0,
		  bool rateLimited = true,
		  bool loopPlayback = true);
  
  ~VideoFileSource();
  
  bool getNextFrame(cv::Mat& frame, FrameMetadata& metadata) override;
  bool isOpen() const override { return cap.isOpened(); }
  float getFrameRate() const override { return fps; }
  int getWidth() const override { return width; }
  int getHeight() const override { return height; }
  bool isColor() const override { return color; }
  void close() override { cap.release(); }

  void setPaused(bool status) { paused = status; }
  bool isPaused() const { return paused; }
  
  void setPlaybackSpeed(float speed) { playback_speed = speed; }
  void setRateLimiting(bool enable) { rate_limit = enable; }
  void setLooping(bool enable) { loop_playback = enable; }
  void rewind();
  bool hasMetadata() const { return has_metadata; }
  void seekToFrame(int frame_number);
  int getCurrentFrameIndex() const { return current_idx; }
  int getTotalFrames() const;
  void stepFrame(int delta);  // Step forward (+1) or backward (-1)  
  
  bool isPlaybackMode() const override { return true; }
  bool isLooping() const override { return loop_playback; }
};

#endif
