#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_H

#include <string>
#include <vector>
#include <cstdint>
#include <tcl.h>

// Vendor-neutral control surface shared by the GenICam camera sources
// (FlirCameraSource, LucidCameraSource). The camera::/flir::/lucid:: Tcl
// commands in CameraCommands.cpp reach the active source through this
// interface, so each vendor only implements the node-level plumbing.
//
// getWidth/getHeight/getFrameRate are declared here as well as in
// IFrameSource; a source that derives from both satisfies both with a
// single override.
class ICameraControl {
public:
  virtual ~ICameraControl() = default;

  struct Settings {
    float exposure_time = 10000.0f;
    float gain = 0.0f;
    float frame_rate = 100.0f;
    bool acquisition_running = false;
  } settings_;

  struct ROIConstraints {
    int width_min, width_max, width_inc;
    int height_min, height_max, height_inc;
    int offset_x_min, offset_x_max, offset_x_inc;
    int offset_y_min, offset_y_max, offset_y_inc;
  };

  // "flir" / "lucid": prefixes the "<vendor>/settings" events
  virtual const char* vendorName() const = 0;

  virtual bool startAcquisition() = 0;
  virtual bool stopAcquisition() = 0;
  virtual bool isStreaming() const = 0;

  virtual int getWidth() const = 0;
  virtual int getHeight() const = 0;
  virtual int getOffsetX() const = 0;
  virtual int getOffsetY() const = 0;
  virtual int getBinningH() const = 0;
  virtual int getBinningV() const = 0;
  virtual float getGain() const = 0;
  virtual float getExposureTime() const = 0;
  virtual float getFrameRate() const = 0;

  virtual bool getROIConstraints(ROIConstraints& constraints) = 0;
  virtual bool configureImageOrientation(bool reverseX, bool reverseY) = 0;
  virtual bool configureExposure(float exposureTime) = 0;
  virtual bool configureGain(float gain) = 0;
  virtual bool configureFrameRate(float frameRate, float* actualRate = nullptr) = 0;
  virtual bool getFrameRateRange(float& min, float& max) = 0;
  virtual bool configureBinning(int horizontal, int vertical) = 0;
  virtual bool configureROI(int w, int h, int offsetX, int offsetY) = 0;
  virtual bool setROIOffset(int offsetX, int offsetY) = 0;

  // I/O line whose state stamps each frame's metadata.lineStatus
  virtual bool setTTLLine(int line) = 0;
  virtual int getTTLLine() const = 0;
  // live bitfield of all I/O lines (bit N = LineN); -1 if unavailable
  virtual int64_t getLineStatusAll() = 0;

  // Generic GenICam feature access by node name (LineSelector, LineSource,
  // AcquisitionFrameTime, ...): camera::node / nodeInfo / nodes / configureLine
  // build on these. Values are strings: integers and floats as numbers,
  // booleans as 1/0, enumerations by entry name, commands execute on set.
  struct NodeInfo {
    std::string name;
    std::string type;        // integer float boolean enumeration string command other
    std::string access;      // NI NA WO RO RW
    std::string value;       // empty if not readable
    std::string unit;
    std::string description;
    bool has_range = false;
    double min = 0, max = 0, inc = 0;
    std::vector<std::string> entries;  // enumeration: available entry names
  };
  virtual bool getNodeInfo(const std::string& name, NodeInfo& info, std::string& error) = 0;
  // Writes the node; if the camera locks it while streaming, the stream is
  // paused and resumed around the write.
  virtual bool setNodeValue(const std::string& name, const std::string& value, std::string& error) = 0;
  virtual void listNodes(std::vector<std::string>& names) = 0;

  // "<vendor>/settings" events (implemented in CameraCommands.cpp)
  void fireSettingChanged(const std::string& setting_name, const std::string& value);
  void fireAllSettings();
};

// Register the camera command set under namespace `ns` (e.g. "camera",
// "flir", "lucid"). `available` is what <ns>::isAvailable reports, i.e.
// whether that backend was compiled in.
int add_camera_commands(Tcl_Interp* interp, const char* ns, bool available);

#endif // CAMERA_CONTROL_H
