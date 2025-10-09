#ifndef SOURCE_MANAGER_H
#define SOURCE_MANAGER_H

#include <memory>
#include <string>
#include <map>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "IFrameSource.h"
#include "Widget.h"
#include "WidgetManager.h"

class ReviewModeSource;

enum SourceState {
    SOURCE_IDLE,
    SOURCE_RUNNING,
    SOURCE_PAUSED,
    SOURCE_STOPPING,
    SOURCE_ERROR
};

class SourceManager {
private:
  WidgetManager* widget_manager_ = nullptr;

  // Track previous source properties
  int last_width_ = -1;
  int last_height_ = -1;
  bool last_is_color_ = true;
  
public:
  SourceManager();
  ~SourceManager();

  // Basic lifecycle
  bool startSource(const std::string& type, const std::map<std::string, std::string>& params);
  bool stopSource();

  // Check if current source is compatible with review frames
  bool isCompatibleWithReview() const;

  // Provide access to current widget manager to allow clearing
  void setWidgetManager(WidgetManager* wm) { widget_manager_ = wm; }
    
  // Status
  SourceState getState() const { return state_; }
  std::string getSourceType() const { return current_source_type_; }
  IFrameSource* getCurrentSource() { return current_source_.get(); }
  
  ReviewModeSource* getReviewSource();
  void ensureReviewSource();
  
  bool sampleCurrentFrame(const cv::Mat& frame, const FrameMetadata& metadata);  

  // For main loop compatibility
  bool isRunning() const { return state_ == SOURCE_RUNNING; }
  
private:
  std::unique_ptr<IFrameSource> current_source_;
  std::shared_ptr<ReviewModeSource> review_source_;   
  SourceState state_;
  mutable std::mutex state_mutex_;
  
  std::string current_source_type_;
  std::map<std::string, std::string> current_params_;
  
  std::unique_ptr<IFrameSource> createSourceFromParams(
						       const std::string& type,
						       const std::map<std::string, std::string>& params);
};

#endif // SOURCE_MANAGER_H
