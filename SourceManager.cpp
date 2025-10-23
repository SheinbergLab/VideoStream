#include "SourceManager.h"
#include "WebcamSource.h"
#include "VideoFileSource.h"
#ifdef USE_FLIR
#include "FlirCameraSource.h"
#endif
#include "ReviewModeSource.h"
#include "SamplingManager.h"
#include "FrameBufferManager.h"
#include "VstreamEvent.h"
#include "DservSocket.h"
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
        int width = 1920;
	int height = 1200;
	
        if (params.count("id")) {
            camera_id = std::stoi(params.at("id"));
        }
        if (params.count("width")) {
            width = std::stoi(params.at("width"));
        }
        if (params.count("height")) {
            height = std::stoi(params.at("height"));
        }
        
        return std::make_unique<FlirCameraSource>(camera_id, width, height);
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
      std::cout << "Source properties changed - clearing buffers" << std::endl;
      std::cout << "  Old: " << last_width_ << "x" << last_height_ 
		<< " (" << (last_is_color_ ? "color" : "grayscale") << ")" << std::endl;
      std::cout << "  New: " << new_width << "x" << new_height 
		<< " (" << (new_is_color ? "color" : "grayscale") << ")" << std::endl;

  
      if (frame_buffer_) {
	frame_buffer_->clearAll();
      }
      
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
    
    // Build comprehensive status data
    std::map<std::string, std::string> status_data;
    status_data["type"] = type;
    status_data["width"] = std::to_string(new_width);
    status_data["height"] = std::to_string(new_height);
    status_data["fps"] = std::to_string(current_source_->getFrameRate());
    status_data["is_color"] = new_is_color ? "1" : "0";
    
    // Add type-specific params
    if (type == "playback") {
      status_data["file"] = current_params_["file"];
      status_data["speed"] = current_params_.count("speed") ? current_params_["speed"] : "1.0";
      status_data["loop"] = current_params_.count("loop") ? current_params_["loop"] : "1";
    } else if (type == "flir" || type == "webcam") {
      status_data["id"] = current_params_.count("id") ? current_params_["id"] : "0";
      if (type == "flir") {
        status_data["width"] = current_params_.count("width") ? current_params_["width"] : std::to_string(new_width);
        status_data["height"] = current_params_.count("height") ? current_params_["height"] : std::to_string(new_height);
      }
    }
    
    fireEvent(VstreamEvent("vstream/source_started", 
			   VstreamEventData::makeKeyValue(status_data)));
    
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

    std::string source_type = current_source_type_;
    
    if (current_source_) {
      if (current_source_.get() != review_source_.get()) {
        current_source_->close();
        current_source_.reset();
      }
      current_source_.release();
    }

    std::string data = "type " + source_type;
    fireEvent(VstreamEvent("vstream/source_stopped", data));
    
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
