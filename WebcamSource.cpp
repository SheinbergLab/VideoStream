#include "WebcamSource.h"
#include <iostream>

WebcamSource::WebcamSource(int cameraId)
    : camera_id(cameraId)
    , simulated_frameID(0)
{
    cap.open(camera_id);
    
    if (!cap.isOpened()) {
        throw std::runtime_error("Error opening webcam " + std::to_string(camera_id));
    }
    
    width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    fps = cap.get(cv::CAP_PROP_FPS);
    
    // Some webcams don't report FPS properly
    if (fps == 0.0 || fps > 1000.0) {
        fps = 30.0;
        std::cout << "Warning: Webcam did not report valid FPS, defaulting to 30.0" << std::endl;
    }
    
    // Determine if color by reading first frame
    cv::Mat test_frame;
    cap >> test_frame;
    color = (test_frame.channels() > 1);
    
    // Note: We don't rewind webcam since it's live
    // The first frame is lost but that's acceptable
    
    start_time = std::chrono::high_resolution_clock::now();
    
    std::cout << "Webcam initialized: " << width << "x" << height 
              << " @ " << fps << " fps, " 
              << (color ? "color" : "grayscale") << std::endl;
}

bool WebcamSource::getNextFrame(cv::Mat& frame, FrameMetadata& metadata) {
    if (!cap.isOpened()) {
        return false;
    }
    
   // Always grab to keep buffer fresh
    cap.grab();
    
    // If paused, don't retrieve the frame
    if (paused_) {
        return false;
    }
    
    // Retrieve the grabbed frame
    cap.retrieve(frame);
    
    if (frame.empty()) {
        return false;
    }
    
    // Update actual dimensions in case they changed
    width = frame.cols;
    height = frame.rows;
    
    // Generate metadata
    metadata.systemTime = std::chrono::high_resolution_clock::now();
    metadata.frameID = simulated_frameID++;
    
    // Calculate timestamp based on frame count and FPS
    // This simulates camera timestamp behavior
    metadata.timestamp = static_cast<int64_t>(
        (simulated_frameID * 1000000000LL) / fps  // nanoseconds
    );
    
    // Webcams don't have hardware line status
    metadata.lineStatus = false;
    
    return true;
}

bool WebcamSource::setResolution(int w, int h) {
    if (!cap.isOpened()) {
        return false;
    }
    
    bool success = true;
    success &= cap.set(cv::CAP_PROP_FRAME_WIDTH, w);
    success &= cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);
    
    if (success) {
        // Read back actual values (camera may not support exact resolution)
        width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        
        std::cout << "Resolution set to: " << width << "x" << height << std::endl;
    } else {
        std::cerr << "Failed to set resolution to " << w << "x" << h << std::endl;
    }
    
    return success;
}

bool WebcamSource::setFrameRate(float targetFps) {
    if (!cap.isOpened()) {
        return false;
    }
    
    bool success = cap.set(cv::CAP_PROP_FPS, targetFps);
    
    if (success) {
        // Read back actual value
        fps = cap.get(cv::CAP_PROP_FPS);
        
        // Validate the FPS
        if (fps == 0.0 || fps > 1000.0) {
            fps = targetFps;  // Use requested value if readback is invalid
        }
        
        std::cout << "Frame rate set to: " << fps << " fps" << std::endl;
    } else {
        std::cerr << "Failed to set frame rate to " << targetFps << " fps" << std::endl;
    }
    
    return success;
}

WebcamSource::~WebcamSource() {
    close();
}
