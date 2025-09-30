#ifndef WEBCAM_SOURCE_H
#define WEBCAM_SOURCE_H

#include "IFrameSource.h"
#include "opencv2/opencv.hpp"

class WebcamSource : public IFrameSource {
private:
    cv::VideoCapture cap;
    int camera_id;
    float fps;
    int width, height;
    bool color;
    
    // Frame tracking for metadata generation
    int64_t simulated_frameID;
    std::chrono::high_resolution_clock::time_point start_time;
    
public:
    WebcamSource(int cameraId = 0);
    ~WebcamSource();
    
    bool getNextFrame(cv::Mat& frame, FrameMetadata& metadata) override;
    bool isOpen() const override { return cap.isOpened(); }
    float getFrameRate() const override { return fps; }
    int getWidth() const override { return width; }
    int getHeight() const override { return height; }
    bool isColor() const override { return color; }
    void close() override { cap.release(); }
    
    // Optional: Allow runtime property changes
    bool setResolution(int w, int h);
    bool setFrameRate(float targetFps);
};

#endif // WEBCAM_SOURCE_H
