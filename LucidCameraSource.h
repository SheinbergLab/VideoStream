#ifndef LUCID_CAMERA_SOURCE_H
#define LUCID_CAMERA_SOURCE_H

#include "IFrameSource.h"
#include "CameraControl.h"

#ifdef USE_LUCID
#include <memory>
#include <string>

// Lucid Vision Labs GigE camera (Arena SDK). Mirrors FlirCameraSource: the
// GenICam node names (ExposureTime, Gain, OffsetX, BinningHorizontal, ...)
// are the same, only the system/device/stream lifecycle differs.
//
// The Arena SDK headers are confined to LucidCameraSource.cpp (pimpl) so
// this header can be included next to the Spinnaker headers without the
// two GenICam distributions colliding.
class LucidCameraSource : public IFrameSource, public ICameraControl {
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  int camera_id;
  std::string serial_;   // optional: select device by serial instead of index
  float fps;
  int width, height;
  bool color;
  int offset_x, offset_y;
  int binning_h, binning_v;

  // I/O line whose state stamps each frame's metadata.lineStatus
  // (bit index into ChunkLineStatusAll; -1 = unresolved -> polled fallback)
  int ttl_line_;
  bool chunk_line_status_ok_;  // LineStatusAll chunk enabled on this camera

  unsigned int image_timeout_ms_;
  bool packet_size_negotiated_;  // auto-negotiation done on the first StartStream

  // Cache for pause
  cv::Mat last_frame_;
  FrameMetadata last_metadata_;
  bool has_last_frame_;

  bool initializeCamera();
  bool configureStreamDefaults();
  bool configureChunkData(bool enable, bool verbose = false);
  void resolveTTLLine();
  bool getLineStatus();
  void refreshGeometry();

public:
  LucidCameraSource(int cameraId = 0, const std::string& serial = "",
                    int width = 0, int height = 0);
  ~LucidCameraSource();

  // IFrameSource
  bool getNextFrame(cv::Mat& frame, FrameMetadata& metadata) override;
  bool isOpen() const override;
  int getWidth() const override { return width; }
  int getHeight() const override { return height; }
  bool isColor() const override { return color; }
  float getFrameRate() const override;
  void close() override;
  bool supportsPause() const override { return true; }

  // ICameraControl
  const char* vendorName() const override { return "lucid"; }
  bool startAcquisition() override;
  bool stopAcquisition() override;
  bool isStreaming() const override;

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
};

#endif // USE_LUCID
#endif // LUCID_CAMERA_SOURCE_H
