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
  
  bool startAcquisition();
  bool stopAcquisition();
  bool isStreaming() const { return pCam && pCam->IsStreaming(); }
  
  bool getNextFrame(cv::Mat& frame, FrameMetadata& metadata) override;
  bool isOpen() const override;
  int getWidth() const override { return width; }
  int getHeight() const override { return height; }
  bool isColor() const override { return color; }
  void close() override;

  struct FlirSettings {
    float exposure_time = 10000.0f;
    float gain = 0.0f;
    float frame_rate = 100.0f;
    bool acquisition_running = false;
  } settings_;
  
  void fireSettingChanged(const std::string& setting_name, const std::string& value);
  void fireAllSettings();
  
  // FLIR-specific configuration methods
  struct ROIConstraints {
    int width_min, width_max, width_inc;
    int height_min, height_max, height_inc;
    int offset_x_min, offset_x_max, offset_x_inc;
    int offset_y_min, offset_y_max, offset_y_inc;
  };

  bool supportsPause() const override { return true; }

  int getOffsetX() const { return offset_x; }
  int getOffsetY() const { return offset_y; }
  int getBinningH() const { return binning_h; }
  int getBinningV() const { return binning_v; }
  float getGain() const { return settings_.gain; }
  float getExposureTime() const { return settings_.exposure_time; }
  
  bool getROIConstraints(ROIConstraints& constraints);
 
  bool configureImageOrientation(bool reverseX, bool reverseY);
  bool configureExposure(float exposureTime);
  bool configureGain(float gain);
  bool configureFrameRate(float frameRate, float* actualRate = nullptr);
  float getFrameRate() const override;  // Already exists, but ensure it reads from camera  
  bool getFrameRateRange(float& min, float& max);  
  bool configureBinning(int horizontal, int vertical);
  bool configureROI(int w, int h, int offsetX, int offsetY);
  bool setROIOffset(int offsetX, int offsetY);  
  bool configureChunkData(bool enable, bool verbose = false);
  bool getLineStatus();
  bool setTTLLine(int line);
  int getTTLLine() const { return ttl_line_; }
  int64_t getLineStatusAll();
  
  Spinnaker::GenApi::INodeMap* getNodeMap() { return nodeMapPtr; }
};

int add_flir_commands(Tcl_Interp *);

#endif // USE_FLIR
#endif // FLIR_CAMERA_SOURCE_H
