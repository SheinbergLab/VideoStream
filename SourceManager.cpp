#include "SourceManager.h"
#include "WebcamSource.h"
#include "VideoFileSource.h"
#ifdef USE_FLIR
#include "FlirCameraSource.h"
#endif
#include "ReviewModeSource.h"

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
        bool loop = params.count("loop") ? (params.at("loop") == "1") : true;
        return std::make_unique<VideoFileSource>(file, metadata, speed, loop);
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
        
        return std::make_unique<FlirCameraSource>(camera_id, flip, flip_code);
    }
#endif
    
    throw std::runtime_error("Unknown source type: " + type);
}

bool SourceManager::startSource(const std::string& type, 
                                const std::map<std::string, std::string>& params)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (state_ == SOURCE_RUNNING) {
        std::cerr << "Source already running" << std::endl;
        return false;
    }
    
    try {
        current_source_ = createSourceFromParams(type, params);
        current_source_type_ = type;
        current_params_ = params;
        
        state_ = SOURCE_RUNNING;
        
        std::cout << "Source started: " << type << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to start source: " << e.what() << std::endl;
        state_ = SOURCE_ERROR;
        return false;
    }
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
