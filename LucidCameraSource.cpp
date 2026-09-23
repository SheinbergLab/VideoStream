#ifdef USE_LUCID

#include <iostream>
#include <atomic>
#include <thread>
#include <algorithm>
#include <cstring>
#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#include "ArenaApi.h"
#include "LucidCameraSource.h"

using namespace cv;

extern std::atomic<int> frame_width, frame_height;

// Arena SDK handles (kept out of the header; see LucidCameraSource.h)
struct LucidCameraSource::Impl {
  Arena::ISystem* system = nullptr;
  Arena::IDevice* device = nullptr;
  GenApi::INodeMap* nodeMap = nullptr;    // device features
  GenApi::INodeMap* streamMap = nullptr;  // transport-layer stream settings
  bool streaming = false;
  size_t num_buffers = 20;
};

// Switch an "*Auto" enumeration (ExposureAuto, GainAuto) to Off. On the
// Triton the node reports read-only for a while after a stream stops even
// though it already reads Off, so only write it when it actually needs
// changing; if it still refuses, refresh the node cache and retry once.
static void setAutoOff(GenApi::INodeMap* nm, const char* name) {
  GenApi::CEnumerationPtr p = nm->GetNode(name);
  if (!p.IsValid() || !GenApi::IsReadable(p)) return;
  GenApi::CEnumEntryPtr cur = p->GetCurrentEntry();
  if (cur.IsValid() && cur->GetSymbolic() == "Off") return;
  if (!GenApi::IsWritable(p)) nm->InvalidateNodes();
  Arena::SetNodeValue<GenICam::gcstring>(nm, name, "Off");
}

// Lucid cameras throw OutOfRange if a float is set past the node's
// limits (Spinnaker does too, but the FLIR scripts stay in range). Clamp
// and report so a too-fast frame rate degrades to "as fast as possible".
static double clampToNode(GenApi::INodeMap* nm, const char* name, double value) {
  GenApi::CFloatPtr p = nm->GetNode(name);
  if (!p.IsValid() || !GenApi::IsReadable(p)) return value;
  double lo = p->GetMin(), hi = p->GetMax();
  if (value < lo || value > hi) {
    double clamped = std::max(lo, std::min(value, hi));
    std::cerr << name << " " << value << " outside [" << lo << ", " << hi
              << "], using " << clamped << std::endl;
    return clamped;
  }
  return value;
}

LucidCameraSource::LucidCameraSource(int cameraId, const std::string& serial,
                                     int w, int h)
    : impl_(new Impl)
    , camera_id(cameraId)
    , serial_(serial)
    , fps(100.0f)
    , width(w)
    , height(h)
    , color(false)
    , offset_x(0)
    , offset_y(0)
    , binning_h(1)
    , binning_v(1)
    , ttl_line_(-1)
    , chunk_line_status_ok_(false)
    , image_timeout_ms_(2000)
    , packet_size_negotiated_(false)
    , has_last_frame_(false)
{
  if (!initializeCamera()) {
    close();
    throw std::runtime_error("Failed to initialize Lucid camera");
  }

  // Optional startup ROI (top-left anchored); scripts normally use
  // camera::configureROI instead
  if (w > 0 && h > 0) {
    configureROI(w, h, 0, 0);
  }
}

LucidCameraSource::~LucidCameraSource() {
  close();
}

bool LucidCameraSource::initializeCamera() {
  try {
    impl_->system = Arena::OpenSystem();
    impl_->system->UpdateDevices(1000);
    std::vector<Arena::DeviceInfo> devices = impl_->system->GetDevices();

    int index = -1;
    if (!serial_.empty()) {
      for (size_t i = 0; i < devices.size(); i++) {
        if (serial_ == devices[i].SerialNumber().c_str()) { index = (int)i; break; }
      }
      if (index < 0) {
        std::cerr << "Lucid camera with serial " << serial_ << " not found ("
                  << devices.size() << " cameras available)" << std::endl;
        return false;
      }
    } else {
      if (camera_id < 0 || (size_t)camera_id >= devices.size()) {
        std::cerr << "Camera " << camera_id << " not found (only "
                  << devices.size() << " cameras available)" << std::endl;
        return false;
      }
      index = camera_id;
    }

    std::cout << "Lucid camera: " << devices[index].ModelName()
              << " sn " << devices[index].SerialNumber()
              << " ip " << devices[index].IpAddressStr() << std::endl;

    impl_->device = impl_->system->CreateDevice(devices[index]);
    impl_->nodeMap = impl_->device->GetNodeMap();
    impl_->streamMap = impl_->device->GetTLStreamNodeMap();
    GenApi::INodeMap* nm = impl_->nodeMap;

    // Ask the camera for Mono8 so no per-frame conversion is needed
    try {
      Arena::SetNodeValue<GenICam::gcstring>(nm, "PixelFormat", "Mono8");
    } catch (GenICam::GenericException& ge) {
      std::cerr << "Could not set PixelFormat Mono8 (" << ge.what()
                << "); frames will be converted" << std::endl;
    }
    color = false;

    Arena::SetNodeValue<GenICam::gcstring>(nm, "AcquisitionMode", "Continuous");

    configureStreamDefaults();

    // Per-frame timestamp / line-status chunks
    configureChunkData(true, false);

    // Default the TTL line to whatever the camera's LineSelector points at;
    // camera::ttlLine overrides.
    resolveTTLLine();
    std::cout << "TTL input line: "
              << (ttl_line_ >= 0 ? "Line" + std::to_string(ttl_line_)
                                 : std::string("unresolved"))
              << (chunk_line_status_ok_ && ttl_line_ >= 0
                      ? " (chunk LineStatusAll)" : " (polled)")
              << std::endl;

    try {
      fps = (float)Arena::GetNodeValue<double>(nm, "AcquisitionFrameRate");
    } catch (...) {}

    // The camera can retain ROI/binning from a previous session; cache its
    // actual state so recorded settings are right
    refreshGeometry();

    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error initializing Lucid camera: " << ge.what() << std::endl;
    return false;
  }
}

// GigE stream-engine settings (transport-layer node map); must be set
// before StartStream
bool LucidCameraSource::configureStreamDefaults() {
  GenApi::INodeMap* tl = impl_->streamMap;
  if (!tl) return false;
  try {
    // Deliver frames in order like Spinnaker's default; the acquisition
    // loop drains as fast as it can
    Arena::SetNodeValue<GenICam::gcstring>(tl, "StreamBufferHandlingMode", "OldestFirst");
    // Largest packet the NIC path allows (jumbo frames if the interface
    // MTU permits) and resend of dropped UDP packets. Negotiation costs
    // ~600 ms per StartStream on the Triton and the result persists in
    // DeviceStreamChannelPacketSize, so startAcquisition turns it off after
    // the first start; ROI/binning changes restart the stream and would
    // otherwise pay it every time.
    Arena::SetNodeValue<bool>(tl, "StreamAutoNegotiatePacketSize", true);
    packet_size_negotiated_ = false;
    Arena::SetNodeValue<bool>(tl, "StreamPacketResendEnable", true);
    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error configuring stream: " << ge.what() << std::endl;
    return false;
  }
}

// Pull Width/Height/Offset/Binning from the camera into the cached members
// and the global frame dimensions
void LucidCameraSource::refreshGeometry() {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return;
  try { width = (int)Arena::GetNodeValue<int64_t>(nm, "Width"); } catch (...) {}
  try { height = (int)Arena::GetNodeValue<int64_t>(nm, "Height"); } catch (...) {}
  try { offset_x = (int)Arena::GetNodeValue<int64_t>(nm, "OffsetX"); } catch (...) {}
  try { offset_y = (int)Arena::GetNodeValue<int64_t>(nm, "OffsetY"); } catch (...) {}
  try { binning_h = (int)Arena::GetNodeValue<int64_t>(nm, "BinningHorizontal"); } catch (...) {}
  try { binning_v = (int)Arena::GetNodeValue<int64_t>(nm, "BinningVertical"); } catch (...) {}
  frame_width = width;
  frame_height = height;
}

bool LucidCameraSource::startAcquisition() {
  if (!impl_->device) return false;
  if (impl_->streaming) return true;

  try {
    if (packet_size_negotiated_ && impl_->streamMap) {
      try {
        Arena::SetNodeValue<bool>(impl_->streamMap, "StreamAutoNegotiatePacketSize", false);
      } catch (GenICam::GenericException&) {}
    }
    impl_->device->StartStream(impl_->num_buffers);
    impl_->streaming = true;
    packet_size_negotiated_ = true;
    std::cout << "Lucid camera acquisition started" << std::endl;

    settings_.acquisition_running = true;
    fireSettingChanged("acquisition_running", "1");
    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error starting acquisition: " << ge.what() << std::endl;
    return false;
  }
}

bool LucidCameraSource::stopAcquisition() {
  if (!impl_->device || !impl_->streaming) return true;

  try {
    impl_->device->StopStream();
    impl_->streaming = false;
    std::cout << "Lucid camera acquisition stopped" << std::endl;

    settings_.acquisition_running = false;
    fireSettingChanged("acquisition_running", "0");
    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error stopping acquisition: " << ge.what() << std::endl;
    impl_->streaming = false;
    return false;
  }
}

bool LucidCameraSource::isStreaming() const {
  return impl_ && impl_->device && impl_->streaming;
}

bool LucidCameraSource::isOpen() const {
  return isStreaming();
}

bool LucidCameraSource::getNextFrame(cv::Mat& frame, FrameMetadata& metadata) {
  if (!impl_->device || !impl_->streaming) {
    return false;
  }

  Arena::IImage* image = nullptr;
  Arena::IImage* converted = nullptr;
  try {
    image = impl_->device->GetImage(image_timeout_ms_);

    if (image->IsIncomplete()) {
      std::cerr << "Image incomplete (frame " << image->GetFrameId()
                << ", " << image->GetSizeFilled() << " bytes)" << std::endl;
      impl_->device->RequeueBuffer(image);
      return false;
    }

    // If paused, release this frame and return cached frame
    if (paused_) {
      impl_->device->RequeueBuffer(image);

      if (has_last_frame_) {
        frame = last_frame_.clone();
        metadata = last_metadata_;
        return true;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return false;
      }
    }

    metadata.frameID = (int64_t)image->GetFrameId();
    metadata.timestamp = (int64_t)image->GetTimestampNs();
    metadata.systemTime = std::chrono::high_resolution_clock::now();

    // TTL status for this frame from the LineStatusAll chunk the camera
    // appends to each image; fall back to a live poll (sampled at dequeue
    // time, up to several frame periods late)
    bool line_status = false;
    bool have_chunk = false;
    if (chunk_line_status_ok_ && ttl_line_ >= 0 && image->HasChunkData()) {
      try {
        GenApi::CIntegerPtr all = image->AsChunkData()->GetChunk("ChunkLineStatusAll");
        if (all.IsValid() && GenApi::IsReadable(all)) {
          line_status = ((all->GetValue() >> ttl_line_) & 1) != 0;
          have_chunk = true;
        }
      } catch (GenICam::GenericException& ge) {
        chunk_line_status_ok_ = false;
        std::cerr << "ChunkLineStatusAll read failed (" << ge.what()
                  << "); reverting to polled LineStatus" << std::endl;
      }
    }
    metadata.lineStatus = have_chunk ? line_status : getLineStatus();

    // Convert to OpenCV Mat (Mono8; convert only if the camera is not
    // already delivering it)
    Arena::IImage* src = image;
    if (image->GetPixelFormat() != Mono8) {
      converted = Arena::ImageFactory::Convert(image, Mono8);
      src = converted;
    }

    unsigned int rowsize = (unsigned int)src->GetWidth();
    unsigned int colsize = (unsigned int)src->GetHeight();
    unsigned int XPadding = (unsigned int)src->GetPaddingX();

    width = rowsize;
    height = colsize;

    Mat cvimg(colsize, rowsize, CV_8UC1, (void*)src->GetData(),
              (size_t)rowsize + XPadding);
    frame = cvimg.clone();

    // Cache this frame for potential pause
    last_frame_ = frame.clone();
    last_metadata_ = metadata;
    has_last_frame_ = true;

    if (converted) Arena::ImageFactory::Destroy(converted);
    impl_->device->RequeueBuffer(image);
    return true;

  } catch (GenICam::TimeoutException&) {
    // no frame within image_timeout_ms_ (e.g. trigger mode, link stall)
    if (converted) Arena::ImageFactory::Destroy(converted);
    if (image) { try { impl_->device->RequeueBuffer(image); } catch (...) {} }
    return false;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error: " << ge.what() << std::endl;
    if (converted) Arena::ImageFactory::Destroy(converted);
    if (image) { try { impl_->device->RequeueBuffer(image); } catch (...) {} }
    return false;
  }
}

void LucidCameraSource::close() {
  if (!impl_) return;
  stopAcquisition();

  try {
    if (impl_->system && impl_->device) {
      impl_->system->DestroyDevice(impl_->device);
    }
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error destroying device: " << ge.what() << std::endl;
  }
  impl_->device = nullptr;
  impl_->nodeMap = nullptr;
  impl_->streamMap = nullptr;

  try {
    if (impl_->system) {
      Arena::CloseSystem(impl_->system);
    }
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error closing system: " << ge.what() << std::endl;
  }
  impl_->system = nullptr;
}

bool LucidCameraSource::getLineStatus() {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return false;
  try {
    GenApi::CBooleanPtr lineStatus = nm->GetNode("LineStatus");
    if (lineStatus.IsValid() && GenApi::IsReadable(lineStatus)) {
      return lineStatus->GetValue();
    }
  } catch (...) {}
  return false;
}

// Default the TTL bit index to the camera's currently selected line -- unless
// that line is an output (a strobe left LineSelector on it): then the first
// input line. camera::ttlLine / the ttl_line setting override either way.
void LucidCameraSource::resolveTTLLine() {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return;
  try {
    GenApi::CEnumerationPtr lineSelector = nm->GetNode("LineSelector");
    if (!lineSelector.IsValid() || !GenApi::IsReadable(lineSelector)) return;

    auto lineNumber = [](GenApi::CEnumEntryPtr e) -> int {
      const char* sym = e->GetSymbolic().c_str();  // e.g. "Line0"
      return (strncmp(sym, "Line", 4) == 0 && isdigit((unsigned char)sym[4])) ? atoi(sym + 4) : -1;
    };
    auto isInput = [&]() -> bool {
      GenApi::CEnumerationPtr mode = nm->GetNode("LineMode");
      if (!mode.IsValid() || !GenApi::IsReadable(mode)) return true;  // unknown: accept
      GenApi::CEnumEntryPtr cur = mode->GetCurrentEntry();
      return !cur.IsValid() || cur->GetSymbolic() != "Output";
    };

    GenApi::CEnumEntryPtr current = lineSelector->GetCurrentEntry();
    if (current.IsValid() && lineNumber(current) >= 0 && isInput()) {
      ttl_line_ = lineNumber(current);
      return;
    }

    // Selected line is an output: take the first input line instead
    GenApi::NodeList_t entries;
    lineSelector->GetEntries(entries);
    for (size_t i = 0; i < entries.size(); i++) {
      GenApi::CEnumEntryPtr e = entries[i];
      if (!e.IsValid() || !GenApi::IsAvailable(e) || lineNumber(e) < 0) continue;
      lineSelector->SetIntValue(e->GetValue());
      if (isInput()) {
        ttl_line_ = lineNumber(e);
        return;
      }
    }
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error resolving LineSelector: " << ge.what() << std::endl;
  }
}

bool LucidCameraSource::setTTLLine(int line) {
  if (line < 0 || line > 7) return false;
  GenApi::INodeMap* nm = impl_->nodeMap;

  // Point LineSelector at the same line so the polled fallback stays
  // consistent with the chunk bit we mask.
  if (nm) {
    try {
      GenApi::CEnumerationPtr lineSelector = nm->GetNode("LineSelector");
      if (lineSelector.IsValid() && GenApi::IsWritable(lineSelector)) {
        std::string name = "Line" + std::to_string(line);
        GenApi::CEnumEntryPtr entry = lineSelector->GetEntryByName(name.c_str());
        if (!entry.IsValid() || !GenApi::IsAvailable(entry)) {
          std::cerr << name << " not present on this camera" << std::endl;
          return false;
        }
        lineSelector->SetIntValue(entry->GetValue());
      }
    } catch (GenICam::GenericException& ge) {
      std::cerr << "Error selecting line " << line << ": " << ge.what() << std::endl;
      return false;
    }
  }

  ttl_line_ = line;
  fireSettingChanged("ttl_line", std::to_string(line));
  return true;
}

int64_t LucidCameraSource::getLineStatusAll() {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return -1;
  try {
    GenApi::CIntegerPtr all = nm->GetNode("LineStatusAll");
    if (all.IsValid() && GenApi::IsReadable(all)) {
      return all->GetValue();
    }
  } catch (...) {}
  return -1;
}

bool LucidCameraSource::configureImageOrientation(bool reverseX, bool reverseY) {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return false;
  try {
    GenApi::CBooleanPtr rx = nm->GetNode("ReverseX");
    if (rx.IsValid() && GenApi::IsWritable(rx)) rx->SetValue(reverseX);
    GenApi::CBooleanPtr ry = nm->GetNode("ReverseY");
    if (ry.IsValid() && GenApi::IsWritable(ry)) ry->SetValue(reverseY);
    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error configuring image orientation: " << ge.what() << std::endl;
    return false;
  }
}

bool LucidCameraSource::configureExposure(float exposureTime) {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return false;
  try {
    setAutoOff(nm, "ExposureAuto");

    double value = clampToNode(nm, "ExposureTime", exposureTime);
    Arena::SetNodeValue<double>(nm, "ExposureTime", value);

    settings_.exposure_time = (float)Arena::GetNodeValue<double>(nm, "ExposureTime");
    fireSettingChanged("exposure_time", std::to_string(settings_.exposure_time));
    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error configuring exposure: " << ge.what() << std::endl;
    return false;
  }
}

bool LucidCameraSource::configureGain(float gain) {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return false;
  try {
    setAutoOff(nm, "GainAuto");

    double value = clampToNode(nm, "Gain", gain);
    Arena::SetNodeValue<double>(nm, "Gain", value);

    settings_.gain = (float)Arena::GetNodeValue<double>(nm, "Gain");
    fireSettingChanged("gain", std::to_string(settings_.gain));
    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error configuring gain: " << ge.what() << std::endl;
    return false;
  }
}

bool LucidCameraSource::getFrameRateRange(float& min, float& max) {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return false;
  try {
    GenApi::CFloatPtr p = nm->GetNode("AcquisitionFrameRate");
    if (p.IsValid() && GenApi::IsReadable(p)) {
      min = (float)p->GetMin();
      max = (float)p->GetMax();
      return true;
    }
  } catch (...) {}
  return false;
}

bool LucidCameraSource::configureFrameRate(float frameRate, float* actualRate) {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return false;
  try {
    Arena::SetNodeValue<bool>(nm, "AcquisitionFrameRateEnable", true);

    double value = clampToNode(nm, "AcquisitionFrameRate", frameRate);
    Arena::SetNodeValue<double>(nm, "AcquisitionFrameRate", value);

    // Read back actual value
    fps = (float)Arena::GetNodeValue<double>(nm, "AcquisitionFrameRate");
    if (actualRate) *actualRate = fps;

    settings_.frame_rate = fps;
    fireSettingChanged("frame_rate", std::to_string(fps));

    std::cout << "Frame rate - Requested: " << frameRate
              << " Hz, Actual: " << fps << " Hz" << std::endl;
    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error configuring frame rate: " << ge.what() << std::endl;
    return false;
  }
}

float LucidCameraSource::getFrameRate() const {
  if (!impl_ || !impl_->nodeMap) return fps;
  try {
    GenApi::CFloatPtr p = impl_->nodeMap->GetNode("AcquisitionFrameRate");
    if (p.IsValid() && GenApi::IsReadable(p)) {
      return (float)p->GetValue();
    }
  } catch (...) {}
  return fps;
}

bool LucidCameraSource::getROIConstraints(ROIConstraints& c) {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return false;
  try {
    struct { const char* name; int* mn; int* mx; int* inc; } nodes[] = {
      {"Width",   &c.width_min,    &c.width_max,    &c.width_inc},
      {"Height",  &c.height_min,   &c.height_max,   &c.height_inc},
      {"OffsetX", &c.offset_x_min, &c.offset_x_max, &c.offset_x_inc},
      {"OffsetY", &c.offset_y_min, &c.offset_y_max, &c.offset_y_inc},
    };
    for (auto& n : nodes) {
      GenApi::CIntegerPtr p = nm->GetNode(n.name);
      if (p.IsValid() && GenApi::IsReadable(p)) {
        *n.mn = (int)p->GetMin();
        *n.mx = (int)p->GetMax();
        *n.inc = (int)p->GetInc();
      }
    }
    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error getting ROI constraints: " << ge.what() << std::endl;
    return false;
  }
}

bool LucidCameraSource::setROIOffset(int offsetX, int offsetY) {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return false;
  try {
    ROIConstraints constraints;
    if (!getROIConstraints(constraints)) {
      std::cerr << "Failed to get ROI constraints" << std::endl;
      return false;
    }

    // Align to increment
    offsetX = (offsetX / constraints.offset_x_inc) * constraints.offset_x_inc;
    offsetY = (offsetY / constraints.offset_y_inc) * constraints.offset_y_inc;

    // Clamp to the sensor (at the current binning)
    int sensor_width = width, sensor_height = height;
    try { sensor_width = (int)Arena::GetNodeValue<int64_t>(nm, "WidthMax"); } catch (...) {}
    try { sensor_height = (int)Arena::GetNodeValue<int64_t>(nm, "HeightMax"); } catch (...) {}

    offsetX = std::max(0, std::min(offsetX, sensor_width - width));
    offsetY = std::max(0, std::min(offsetY, sensor_height - height));

    // Re-align after clamping
    offsetX = (offsetX / constraints.offset_x_inc) * constraints.offset_x_inc;
    offsetY = (offsetY / constraints.offset_y_inc) * constraints.offset_y_inc;

    // Set offsets ONLY (width/height unchanged). Spinnaker lets offsets
    // move while streaming; the Triton locks them, so pause and resume the
    // stream only if the camera refuses a live write.
    GenApi::CIntegerPtr ptrOffsetX = nm->GetNode("OffsetX");
    bool restart = impl_->streaming &&
                   !(ptrOffsetX.IsValid() && GenApi::IsWritable(ptrOffsetX));
    if (restart) stopAcquisition();

    Arena::SetNodeValue<int64_t>(nm, "OffsetX", offsetX);
    Arena::SetNodeValue<int64_t>(nm, "OffsetY", offsetY);

    offset_x = offsetX;
    offset_y = offsetY;

    if (restart) startAcquisition();
    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error setting ROI offset: " << ge.what() << std::endl;
    return false;
  }
}

bool LucidCameraSource::configureBinning(int horizontal, int vertical)
{
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return false;
  try {
    // Width/Height/Binning are locked while streaming
    bool wasAcquiring = impl_->streaming;
    if (wasAcquiring) stopAcquisition();

    // Reset ROI to maximum before changing binning
    GenApi::CIntegerPtr ptrWidth = nm->GetNode("Width");
    GenApi::CIntegerPtr ptrHeight = nm->GetNode("Height");
    GenApi::CIntegerPtr ptrOffsetX = nm->GetNode("OffsetX");
    GenApi::CIntegerPtr ptrOffsetY = nm->GetNode("OffsetY");

    if (GenApi::IsWritable(ptrOffsetX)) ptrOffsetX->SetValue(0);
    if (GenApi::IsWritable(ptrOffsetY)) ptrOffsetY->SetValue(0);
    if (GenApi::IsWritable(ptrWidth)) ptrWidth->SetValue(ptrWidth->GetMax());
    if (GenApi::IsWritable(ptrHeight)) ptrHeight->SetValue(ptrHeight->GetMax());

    // Prefer on-sensor binning (raises the achievable frame rate); fall
    // back to digital binning on models without it. Binning 1x1 is set
    // through whichever selector is current.
    if (horizontal > 1 || vertical > 1) {
      GenApi::CEnumerationPtr sel = nm->GetNode("BinningSelector");
      if (sel.IsValid() && GenApi::IsWritable(sel)) {
        GenApi::CEnumEntryPtr sensor = sel->GetEntryByName("Sensor");
        GenApi::CEnumEntryPtr digital = sel->GetEntryByName("Digital");
        if (sensor.IsValid() && GenApi::IsAvailable(sensor)) {
          sel->SetIntValue(sensor->GetValue());
        } else if (digital.IsValid() && GenApi::IsAvailable(digital)) {
          sel->SetIntValue(digital->GetValue());
        }
      }
    }

    GenApi::CIntegerPtr ptrBinH = nm->GetNode("BinningHorizontal");
    GenApi::CIntegerPtr ptrBinV = nm->GetNode("BinningVertical");

    if (GenApi::IsWritable(ptrBinH)) {
      int64_t v = std::max(ptrBinH->GetMin(), std::min((int64_t)horizontal, ptrBinH->GetMax()));
      ptrBinH->SetValue(v);
      std::cout << "Set horizontal binning to " << v << std::endl;
    }
    if (GenApi::IsWritable(ptrBinV)) {
      int64_t v = std::max(ptrBinV->GetMin(), std::min((int64_t)vertical, ptrBinV->GetMax()));
      ptrBinV->SetValue(v);
      std::cout << "Set vertical binning to " << v << std::endl;
    }

    // Binning changes the image size: cache what the camera actually
    // accepted (binning, Width, Height) before restarting
    refreshGeometry();

    if (wasAcquiring) startAcquisition();
    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error setting binning: " << ge.what() << std::endl;
    return false;
  }
}

bool LucidCameraSource::configureROI(int w, int h, int offsetX, int offsetY) {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return false;
  try {
    ROIConstraints constraints;
    if (!getROIConstraints(constraints)) {
      std::cerr << "Failed to get ROI constraints" << std::endl;
      return false;
    }

    // Validate and adjust values to meet increment requirements
    if (w % constraints.width_inc != 0) {
      int adjusted = (w / constraints.width_inc) * constraints.width_inc;
      std::cerr << "Warning: Width " << w << " not divisible by "
                << constraints.width_inc << ", adjusting to " << adjusted << std::endl;
      w = adjusted;
    }
    if (h % constraints.height_inc != 0) {
      int adjusted = (h / constraints.height_inc) * constraints.height_inc;
      std::cerr << "Warning: Height " << h << " not divisible by "
                << constraints.height_inc << ", adjusting to " << adjusted << std::endl;
      h = adjusted;
    }
    if (offsetX % constraints.offset_x_inc != 0) {
      int adjusted = (offsetX / constraints.offset_x_inc) * constraints.offset_x_inc;
      std::cerr << "Warning: OffsetX " << offsetX << " not divisible by "
                << constraints.offset_x_inc << ", adjusting to " << adjusted << std::endl;
      offsetX = adjusted;
    }
    if (offsetY % constraints.offset_y_inc != 0) {
      int adjusted = (offsetY / constraints.offset_y_inc) * constraints.offset_y_inc;
      std::cerr << "Warning: OffsetY " << offsetY << " not divisible by "
                << constraints.offset_y_inc << ", adjusting to " << adjusted << std::endl;
      offsetY = adjusted;
    }

    // Width/Height are locked while streaming
    bool wasAcquiring = impl_->streaming;
    if (wasAcquiring) stopAcquisition();

    // Offsets first to 0 so the new size always fits, then size, then
    // the requested offsets (clamped by setROIOffset)
    Arena::SetNodeValue<int64_t>(nm, "OffsetX", 0);
    Arena::SetNodeValue<int64_t>(nm, "OffsetY", 0);
    Arena::SetNodeValue<int64_t>(nm, "Width", w);
    Arena::SetNodeValue<int64_t>(nm, "Height", h);

    width = w;
    height = h;
    frame_width = w;
    frame_height = h;

    bool ok = setROIOffset(offsetX, offsetY);
    if (ok && (offset_x != offsetX || offset_y != offsetY)) {
      std::cerr << "Warning: ROI offset clamped to (" << offset_x << ", "
                << offset_y << ")" << std::endl;
    }

    if (wasAcquiring) startAcquisition();
    return ok;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error configuring ROI: " << ge.what() << std::endl;
    return false;
  }
}

bool LucidCameraSource::configureChunkData(bool enable, bool verbose) {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) return false;
  try {
    if (!enable) {
      Arena::SetNodeValue<bool>(nm, "ChunkModeActive", false);
      chunk_line_status_ok_ = false;
      return true;
    }

    Arena::SetNodeValue<bool>(nm, "ChunkModeActive", true);
    if (verbose) std::cout << "Chunk mode activated..." << std::endl;

    // Only the chunks we read; CRC/HistogramStats cost camera time
    const char* wanted[] = {"Timestamp", "LineStatusAll", "ExposureTime", "Gain"};
    GenApi::CEnumerationPtr sel = nm->GetNode("ChunkSelector");
    if (!sel.IsValid() || !GenApi::IsWritable(sel)) {
      std::cerr << "Unable to retrieve chunk selector" << std::endl;
      return false;
    }
    for (const char* name : wanted) {
      GenApi::CEnumEntryPtr entry = sel->GetEntryByName(name);
      if (!entry.IsValid() || !GenApi::IsAvailable(entry)) {
        if (verbose) std::cout << "\t" << name << ": not available" << std::endl;
        continue;
      }
      sel->SetIntValue(entry->GetValue());
      GenApi::CBooleanPtr en = nm->GetNode("ChunkEnable");
      bool enabled = false;
      if (en.IsValid() && GenApi::IsReadable(en) && en->GetValue()) {
        enabled = true;
      } else if (en.IsValid() && GenApi::IsWritable(en)) {
        en->SetValue(true);
        enabled = true;
      }
      if (verbose) std::cout << "\t" << name << ": " << (enabled ? "Enabled" : "Not writable") << std::endl;
      if (enabled && strcmp(name, "LineStatusAll") == 0) {
        chunk_line_status_ok_ = true;
      }
    }
    return true;
  } catch (GenICam::GenericException& ge) {
    std::cerr << "Error configuring chunk data: " << ge.what() << std::endl;
    return false;
  }
}

/*********************************************************************/
/*                 Generic GenICam node access                       */
/*********************************************************************/

namespace {
using namespace GenApi;
using namespace GenICam;
#include "GenICamNodeOps.inl"
}

bool LucidCameraSource::getNodeInfo(const std::string& name, NodeInfo& info,
                                    std::string& error) {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) { error = "camera not open"; return false; }
  return genicamNodeInfo(nm, name, info, error);
}

bool LucidCameraSource::setNodeValue(const std::string& name, const std::string& value,
                                     std::string& error) {
  GenApi::INodeMap* nm = impl_->nodeMap;
  if (!nm) { error = "camera not open"; return false; }

  // The Triton reports stale access modes after a stream stop (see
  // setAutoOff); refresh before deciding whether the node is locked
  try { nm->InvalidateNodes(); } catch (...) {}

  // Features locked while streaming (Width, OffsetX, PixelFormat, ...):
  // pause the stream around the write
  bool restart = false;
  try {
    GenApi::INode* node = nm->GetNode(name.c_str());
    restart = impl_->streaming && node && !GenApi::IsWritable(node);
  } catch (...) {}
  if (restart) stopAcquisition();

  bool ok = genicamSetNode(nm, name, value, error);

  if (restart) startAcquisition();
  return ok;
}

void LucidCameraSource::listNodes(std::vector<std::string>& names) {
  if (impl_->nodeMap) genicamListNodes(impl_->nodeMap, names);
}

#endif // USE_LUCID
