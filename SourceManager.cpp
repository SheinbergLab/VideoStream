#include "SourceManager.h"
#include "WebcamSource.h"
#include "VideoFileSource.h"
#ifdef USE_FLIR
#include "FlirCameraSource.h"
#endif
#include "ReviewModeSource.h"
#include "SamplingManager.h"
#include "FrameBufferManager.h"
#include "VideoStream.h"

#include <iostream>

SourceManager::SourceManager()
    : state_(SOURCE_IDLE)
{
}

SourceManager::~SourceManager() {
    stopSource();
}

std::unique_ptr<IFrameSource> SourceManager::createSourceFromParams(
    const std::string& type,
    const std::map<std::string, std::string>& params)
{
    if (type == "webcam") {
        int camera_id = 0;
        if (params.count("id")) {
            camera_id = std::stoi(params.at("id"));
        }
        return std::make_unique<WebcamSource>(camera_id);
    }
    else if (type == "playback") {
        std::string file = params.at("file");
        std::string metadata = params.count("metadata") ? params.at("metadata") : "";
        float speed = params.count("speed") ? std::stof(params.at("speed")) : 1.0f;
        bool rate_limited = params.count("rate_limited") ? (params.at("rate_limited") == "1") : false;

        bool loop = params.count("loop") ? (params.at("loop") == "1") : true;
        return std::make_unique<VideoFileSource>(file, metadata, speed, rate_limited, loop);
    }
    else if (type == "review") {
      ensureReviewSource();
      IFrameSource* raw = review_source_.get();
      return std::unique_ptr<IFrameSource>(raw);
    }
#ifdef USE_FLIR
    else if (type == "flir") {
        int camera_id = 0;
        bool flip = false;
        int flip_code = -2;
        
        if (params.count("id")) {
            camera_id = std::stoi(params.at("id"));
        }
        if (params.count("flip")) {
            flip = (params.at("flip") == "1" || params.at("flip") == "true");
        }
        if (params.count("flip_code")) {
            flip_code = std::stoi(params.at("flip_code"));
        }

        if (params.count("width") || params.count("height") || 
            params.count("offset_x") || params.count("offset_y")) {
	  
	  int w = params.count("width") ? std::stoi(params["width"]) : 1440;
	  int h = params.count("height") ? std::stoi(params["height"]) : 1080;
	  int ox = params.count("offset_x") ? std::stoi(params["offset_x"]) : 0;
	  int oy = params.count("offset_y") ? std::stoi(params["offset_y"]) : 0;
          
	  FlirCameraSource* flirSource = dynamic_cast<FlirCameraSource*>(*fs);
	  if (flirSource) {
	    flirSource->configureROI(w, h, ox, oy);
	  }
        }
	
        return std::make_unique<FlirCameraSource>(camera_id, flip, flip_code);
    }
#endif
    
    throw std::runtime_error("Unknown source type: " + type);
}

void SourceManager::clearSampleFrames()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // If review source is currently active, stop it
  if (current_source_.get() == review_source_.get() && 
      (state_ == SOURCE_RUNNING || state_ == SOURCE_PAUSED)) {
    
    std::cout << "Stopping review source before clearing frames" << std::endl;
    
    // Don't call current_source_->close() since we're using shared_ptr
    current_source_.release();
    state_ = SOURCE_IDLE;
  }
  
  // Now safe to clear the frames
  if (review_source_) {
    review_source_->clearFrames();
  }
}

bool SourceManager::startSource(const std::string& type, 
                                const std::map<std::string, std::string>& params)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  if (state_ == SOURCE_RUNNING) {
    std::cerr << "Source already running" << std::endl;
    return false;
  }

  if (type == "review") {
    ensureReviewSource();
    if (review_source_->getFrameCount() == 0) {
        return false;
    }
  }
  
  try {
    current_source_ = createSourceFromParams(type, params);
    current_source_type_ = type;
    current_params_ = params;
    
    state_ = SOURCE_RUNNING;
    
    int new_width = current_source_->getWidth();
    int new_height = current_source_->getHeight();
    bool new_is_color = current_source_->isColor();
    
    bool properties_changed = (last_width_ != -1 && 
			       (last_width_ != new_width || 
				last_height_ != new_height ||
				last_is_color_ != new_is_color));
    
    if (properties_changed) {
      if (review_source_ && review_source_->getFrameCount()) {
	review_source_->clearFrames();
      }
    }
    
    // Update tracked properties
    last_width_ = new_width;
    last_height_ = new_height;
    last_is_color_ = new_is_color;
    
    if (widget_manager_) {
      widget_manager_->clearAll();
    }
    
    std::string data = "type " + type;
    fireEvent("source_started", data);
    
    return true;
    
  } catch (const std::exception& e) {
    std::cerr << "Failed to start source: " << e.what() << std::endl;
    state_ = SOURCE_ERROR;
    return false;
  }
}

bool SourceManager::isCompatibleWithReview() const
{
  if (!review_source_ || review_source_->getFrameCount() == 0) {
    return true;  // Empty review buffer is compatible with anything
  }
  
  if (!current_source_) {
    return false;
  }
  
  return (current_source_->getWidth() == review_source_->getWidth() &&
	  current_source_->getHeight() == review_source_->getHeight() &&
	  current_source_->isColor() == review_source_->isColor());
}

bool SourceManager::stopSource()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (state_ != SOURCE_RUNNING && state_ != SOURCE_PAUSED) {
        return false;
    }
    
    if (current_source_) {
      if (current_source_.get() != review_source_.get()) {
        current_source_->close();
        current_source_.reset();
      }
      current_source_.release();
    }

    // source_stopped callback can react...
    std::string data = "type " + getSourceType();
    fireEvent("source_stopped", data);    
    
    state_ = SOURCE_IDLE;
    return true;
}

bool SourceManager::sampleCurrentFrame(const cv::Mat& frame, const FrameMetadata& metadata)
{
    if (frame.empty()) {
        return false;
    }
    
    ensureReviewSource();
    review_source_->addFrame(frame, metadata);
    return true;
}

ReviewModeSource* SourceManager::getReviewSource() {
    if (!review_source_) {
        review_source_ = std::make_shared<ReviewModeSource>();
    }
    return review_source_.get();
}

void SourceManager::ensureReviewSource() {
    if (!review_source_) {
        review_source_ = std::make_shared<ReviewModeSource>();
    }
}
