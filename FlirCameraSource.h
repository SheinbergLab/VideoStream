#ifndef FLIR_CAMERA_SOURCE_H
#define FLIR_CAMERA_SOURCE_H

#include "IFrameSource.h"
#include "CameraControl.h"

#ifdef USE_FLIR
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

class FlirCameraSource : public IFrameSource, public ICameraControl {
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
  int offset_x, offset_y;
  int binning_h;  // horizontal binning
  int binning_v;  // vertical binning

  // I/O line whose state stamps each frame's metadata.lineStatus
  // (bit index into ExposureEndLineStatusAll; -1 = unresolved -> polled fallback)
  int ttl_line_;
  bool chunk_line_status_ok_;  // ExposureEndLineStatusAll chunk enabled on this camera

  // Cache for pause
  cv::Mat last_frame_;
  FrameMetadata last_metadata_;
  bool has_last_frame_;

  bool initializeCamera();
  void configureCameraDefaults();
  void resolveTTLLine();
  bool readFrameLineStatus(Spinnaker::ChunkData& chunkData);

public:
  FlirCameraSource(int cameraId = 0, int width = 1920, int height = 1200);
  ~FlirCameraSource();

  // IFrameSource
  bool getNextFrame(cv::Mat& frame, FrameMetadata& metadata) override;
  bool isOpen() const override;
  int getWidth() const override { return width; }
  int getHeight() const override { return height; }
  bool isColor() const override { return color; }
  float getFrameRate() const override;  // reads from camera, falls back to cache
  void close() override;
  bool supportsPause() const override { return true; }

  // ICameraControl (Tcl commands: camera::* / flir::*, see CameraCommands.cpp)
  const char* vendorName() const override { return "flir"; }
  bool startAcquisition() override;
  bool stopAcquisition() override;
  bool isStreaming() const override { return pCam && pCam->IsStreaming(); }

  int getOffsetX() const override { return offset_x; }
  int getOffsetY() const override { return offset_y; }
  int getBinningH() const override { return binning_h; }
  int getBinningV() const override { return binning_v; }
  float getGain() const override { return settings_.gain; }
  float getExposureTime() const override { return settings_.exposure_time; }

  bool getROIConstraints(ROIConstraints& constraints) override;
  bool configureImageOrientation(bool reverseX, bool reverseY) override;
  bool configureExposure(float exposureTime) override;
  bool configureGain(float gain) override;
  bool configureFrameRate(float frameRate, float* actualRate = nullptr) override;
  bool getFrameRateRange(float& min, float& max) override;
  bool configureBinning(int horizontal, int vertical) override;
  bool configureROI(int w, int h, int offsetX, int offsetY) override;
  bool setROIOffset(int offsetX, int offsetY) override;
  bool setTTLLine(int line) override;
  int getTTLLine() const override { return ttl_line_; }
  int64_t getLineStatusAll() override;
  bool getNodeInfo(const std::string& name, NodeInfo& info, std::string& error) override;
  bool setNodeValue(const std::string& name, const std::string& value, std::string& error) override;
  void listNodes(std::vector<std::string>& names) override;

  // FLIR-specific
  bool configureChunkData(bool enable, bool verbose = false);
  bool getLineStatus();
  Spinnaker::GenApi::INodeMap* getNodeMap() { return nodeMapPtr; }
};

#endif // USE_FLIR
#endif // FLIR_CAMERA_SOURCE_H
