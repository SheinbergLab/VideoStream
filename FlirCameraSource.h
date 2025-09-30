#ifndef FLIR_CAMERA_SOURCE_H
#define FLIR_CAMERA_SOURCE_H

#include "IFrameSource.h"

#ifdef USE_FLIR
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

class FlirCameraSource : public IFrameSource {
private:
    Spinnaker::SystemPtr system;
    Spinnaker::CameraList camList;
    Spinnaker::CameraPtr pCam;
    Spinnaker::GenApi::INodeMap* nodeMapPtr;
    Spinnaker::ImageProcessor processor;
    
    int camera_id;
    float fps;
    int width, height;
    bool color;
    bool flip_view;
    int flip_code;
    
    bool initializeCamera();
    void configureCameraDefaults();
    
public:
    FlirCameraSource(int cameraId = 0, bool flipView = true, int flipCode = -2);
    ~FlirCameraSource();
    
    bool getNextFrame(cv::Mat& frame, FrameMetadata& metadata) override;
    bool isOpen() const override;
    float getFrameRate() const override { return fps; }
    int getWidth() const override { return width; }
    int getHeight() const override { return height; }
    bool isColor() const override { return color; }
    void close() override;
    
    // FLIR-specific configuration methods
    bool configureExposure(float exposureTime);
    bool configureGain(float gain);
    bool configureFrameRate(float frameRate);
    bool configureROI(int w, int h, int offsetX, int offsetY);
    bool configureChunkData(bool enable, bool verbose = false);
    bool getLineStatus();
    
    Spinnaker::GenApi::INodeMap* getNodeMap() { return nodeMapPtr; }
};

#endif // USE_FLIR
#endif // FLIR_CAMERA_SOURCE_H
