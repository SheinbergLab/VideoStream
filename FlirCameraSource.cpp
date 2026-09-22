#ifdef USE_FLIR

#include <iostream>
#include <atomic>
#include <thread>
#include <algorithm>   // std::max/min (GCC 14 / Debian Trixie: no transitive include)
#include <cstring>
#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>
#include "VstreamEvent.h"
#include <tcl.h>
#include "FlirCameraSource.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;


extern std::atomic<int> frame_width, frame_height;

FlirCameraSource::FlirCameraSource(int cameraId, int width, int height)
    : camera_id(cameraId)
    , width(width)
    , height(height)
    , nodeMapPtr(nullptr)
    , fps(100.0)
    , offset_x(0)
    , offset_y(0)
    , binning_h(1)
    , binning_v(1)
    , color(false)
    , ttl_line_(-1)
    , chunk_line_status_ok_(false)
    , has_last_frame_(false)
{
    processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
    
    if (!initializeCamera()) {
        throw std::runtime_error("Failed to initialize FLIR camera");
    }
}

bool FlirCameraSource::initializeCamera() {
    // Retrieve singleton reference to system object
    system = System::GetInstance();
    
    // Retrieve list of cameras from the system
    camList = system->GetCameras();
    unsigned int numCameras = camList.GetSize();
    
    // Check if camera exists
    if (numCameras <= camera_id) {
        std::cerr << "Camera " << camera_id << " not found (only " 
                  << numCameras << " cameras available)" << std::endl;
        camList.Clear();
        system->ReleaseInstance();
        return false;
    }
    
    // Select camera
    pCam = camList.GetByIndex(camera_id);
    
    // Initialize camera
    pCam->Init();
    
    // Retrieve GenICam nodemap
    INodeMap& nodeMap = pCam->GetNodeMap();
    nodeMapPtr = &nodeMap;
    
    // Configure chunk data by default
    configureChunkData(true, false);

    // Default the TTL line to whatever the camera's LineSelector points at
    // (the same line the legacy polled path read); flir::ttlLine overrides.
    resolveTTLLine();
    std::cout << "TTL input line: "
              << (ttl_line_ >= 0 ? "Line" + std::to_string(ttl_line_)
                                 : std::string("unresolved"))
              << (chunk_line_status_ok_ && ttl_line_ >= 0
                      ? " (exposure-end latched)" : " (polled)")
              << std::endl;

    // Set acquisition mode to continuous
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) {
        std::cerr << "Unable to set acquisition mode to continuous" << std::endl;
        return false;
    }
    
    CEnumEntryPtr ptrAcquisitionModeContinuous = 
        ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || 
        !IsReadable(ptrAcquisitionModeContinuous)) {
        std::cerr << "Unable to get continuous acquisition mode entry" << std::endl;
        return false;
    }
    
    ptrAcquisitionMode->SetIntValue(ptrAcquisitionModeContinuous->GetValue());
    
    // Get frame rate
    CFloatPtr ptrAcquisitionFrameRate = nodeMap.GetNode("AcquisitionFrameRate");
    if (IsAvailable(ptrAcquisitionFrameRate) && IsReadable(ptrAcquisitionFrameRate)) {
        fps = static_cast<float>(ptrAcquisitionFrameRate->GetValue());
    }

    // Get initial frame dimensions
    CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
    if (IsAvailable(ptrWidth) && IsReadable(ptrWidth)) {
        width = static_cast<int>(ptrWidth->GetValue());
    }
    
    CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
    if (IsAvailable(ptrHeight) && IsReadable(ptrHeight)) {
        height = static_cast<int>(ptrHeight->GetValue());
    }

    // The camera can retain binning from a previous session; cache its
    // actual state so recorded settings are right even if configureBinning
    // is never called
    CIntegerPtr ptrBinH = nodeMap.GetNode("BinningHorizontal");
    if (IsAvailable(ptrBinH) && IsReadable(ptrBinH)) {
        binning_h = static_cast<int>(ptrBinH->GetValue());
    }

    CIntegerPtr ptrBinV = nodeMap.GetNode("BinningVertical");
    if (IsAvailable(ptrBinV) && IsReadable(ptrBinV)) {
        binning_v = static_cast<int>(ptrBinV->GetValue());
    }
    
    // Update global dimensions immediately
    frame_width = width;
    frame_height = height;    
    
    return true;
}

bool FlirCameraSource::startAcquisition() {
    if (!pCam || !pCam->IsValid()) {
        return false;
    }
    
    if (pCam->IsStreaming()) {
        return true; // Already streaming
    }
    
    try {
        pCam->BeginAcquisition();
        std::cout << "FLIR camera acquisition started" << std::endl;

        settings_.acquisition_running = true;
        fireSettingChanged("acquisition_running", "1");
	
        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error starting acquisition: " << e.what() << std::endl;
        return false;
    }
}

bool FlirCameraSource::stopAcquisition() {
    if (!pCam || !pCam->IsStreaming()) {
        return true;
    }
    
    try {
        pCam->EndAcquisition();
        std::cout << "FLIR camera acquisition stopped" << std::endl;

	settings_.acquisition_running = false;
        fireSettingChanged("acquisition_running", "0");
	
	
        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error stopping acquisition: " << e.what() << std::endl;
        return false;
    }
}

bool FlirCameraSource::getNextFrame(cv::Mat& frame, FrameMetadata& metadata) {
    if (!pCam || !pCam->IsStreaming()) {
        return false;
    }
    
    try {
        ImagePtr pResultImage = pCam->GetNextImage();
        
        // Check if image is incomplete
        if (pResultImage->IsIncomplete()) {
            std::cerr << "Image incomplete with status " 
                      << pResultImage->GetImageStatus() << std::endl;
            pResultImage->Release();
            return false;
        }
        
        // If paused, release this frame and return cached frame
        if (paused_) {
            pResultImage->Release();
            
            if (has_last_frame_) {
                frame = last_frame_.clone();
                metadata = last_metadata_;
                return true;  // Return true with cached frame
            } else {
                // No cached frame yet, sleep and return false
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                return false;
            }
        }
        
        // Get chunk data
        ChunkData chunkData = pResultImage->GetChunkData();
        metadata.frameID = chunkData.GetFrameID();
        metadata.timestamp = chunkData.GetTimestamp();
        metadata.systemTime = std::chrono::high_resolution_clock::now();

        // TTL status for this frame: the camera latches all line states at
        // exposure end into the chunk; a live poll would instead sample at
        // dequeue time, up to several frame periods late.
        metadata.lineStatus = readFrameLineStatus(chunkData);
        
        // Convert to OpenCV Mat
        ImagePtr convertedImage = 
            processor.Convert(pResultImage, PixelFormat_Mono8);
            
        unsigned int XPadding = static_cast<unsigned int>(convertedImage->GetXPadding());
        unsigned int YPadding = static_cast<unsigned int>(convertedImage->GetYPadding());
        unsigned int rowsize = static_cast<unsigned int>(convertedImage->GetWidth());
        unsigned int colsize = static_cast<unsigned int>(convertedImage->GetHeight());
        
        width = rowsize;
        height = colsize;
        
        // Create Mat with padding accounted for
        Mat cvimg = Mat(colsize + YPadding, rowsize + XPadding,
                       CV_8UC1, convertedImage->GetData(),
                       convertedImage->GetStride());
        
        frame = cvimg.clone();
        
        // Cache this frame for potential pause
        last_frame_ = frame.clone();
        last_metadata_ = metadata;
        has_last_frame_ = true;
        
        pResultImage->Release();
        return true;
        
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}


bool FlirCameraSource::isOpen() const {
    return pCam && pCam->IsValid() && pCam->IsStreaming();
}

void FlirCameraSource::close() {
  stopAcquisition();
  
  if (pCam && pCam->IsInitialized()) {
    pCam->DeInit();
  }
  
  pCam = nullptr;
  camList.Clear();
  
  if (system) {
    system->ReleaseInstance();
    system = nullptr;
  }
}

bool FlirCameraSource::getLineStatus() {
    if (!nodeMapPtr) return false;

    try {
        CBooleanPtr lineStatus = nodeMapPtr->GetNode("LineStatus");
        if (IsAvailable(lineStatus) && IsReadable(lineStatus)) {
            return lineStatus->GetValue();
        }
    } catch (...) {
        return false;
    }
    return false;
}

bool FlirCameraSource::readFrameLineStatus(ChunkData& chunkData) {
    if (chunk_line_status_ok_ && ttl_line_ >= 0) {
        try {
            return ((chunkData.GetExposureEndLineStatusAll() >> ttl_line_) & 1) != 0;
        } catch (Spinnaker::Exception &e) {
            chunk_line_status_ok_ = false;
            std::cerr << "ExposureEndLineStatusAll chunk read failed ("
                      << e.what() << "); reverting to polled LineStatus"
                      << std::endl;
        }
    }
    return getLineStatus();
}

// Default the TTL bit index to the camera's currently selected line, so the
// latched chunk path samples the same pin the legacy poll did.
void FlirCameraSource::resolveTTLLine() {
    if (!nodeMapPtr) return;

    try {
        CEnumerationPtr lineSelector = nodeMapPtr->GetNode("LineSelector");
        if (IsAvailable(lineSelector) && IsReadable(lineSelector)) {
            CEnumEntryPtr current = lineSelector->GetCurrentEntry();
            if (current) {
                const char* sym = current->GetSymbolic().c_str();  // e.g. "Line0"
                if (strncmp(sym, "Line", 4) == 0 &&
                    isdigit((unsigned char)sym[4])) {
                    ttl_line_ = atoi(sym + 4);
                }
            }
        }
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error resolving LineSelector: " << e.what() << std::endl;
    }
}

bool FlirCameraSource::setTTLLine(int line) {
    if (line < 0 || line > 7) return false;

    // Point LineSelector at the same line so the polled fallback stays
    // consistent with the chunk bit we mask.
    if (nodeMapPtr) {
        try {
            CEnumerationPtr lineSelector = nodeMapPtr->GetNode("LineSelector");
            if (IsAvailable(lineSelector) && IsWritable(lineSelector)) {
                std::string name = "Line" + std::to_string(line);
                CEnumEntryPtr entry = lineSelector->GetEntryByName(name.c_str());
                if (!IsAvailable(entry) || !IsReadable(entry)) {
                    std::cerr << name << " not present on this camera" << std::endl;
                    return false;
                }
                lineSelector->SetIntValue(entry->GetValue());
            }
        } catch (Spinnaker::Exception &e) {
            std::cerr << "Error selecting line " << line << ": "
                      << e.what() << std::endl;
            return false;
        }
    }

    ttl_line_ = line;
    fireSettingChanged("ttl_line", std::to_string(line));
    return true;
}

// Live poll of every line's state (bit N = LineN); toggle the TTL while
// watching this to identify which line it's wired to. -1 if unavailable.
int64_t FlirCameraSource::getLineStatusAll() {
    if (!nodeMapPtr) return -1;

    try {
        CIntegerPtr all = nodeMapPtr->GetNode("LineStatusAll");
        if (IsAvailable(all) && IsReadable(all)) {
            return all->GetValue();
        }
    } catch (...) {}
    return -1;
}

bool FlirCameraSource::configureImageOrientation(bool reverseX, bool reverseY) {
    if (!nodeMapPtr) return false;
    
    try {
        CBooleanPtr ptrReverseX = nodeMapPtr->GetNode("ReverseX");
        if (IsAvailable(ptrReverseX) && IsWritable(ptrReverseX)) {
            ptrReverseX->SetValue(reverseX);
        }
        
        CBooleanPtr ptrReverseY = nodeMapPtr->GetNode("ReverseY");
        if (IsAvailable(ptrReverseY) && IsWritable(ptrReverseY)) {
            ptrReverseY->SetValue(reverseY);
        }
        
        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error configuring image orientation: " << e.what() << std::endl;
        return false;
    }
}

bool FlirCameraSource::configureExposure(float exposureTime) {
    if (!nodeMapPtr) return false;
    
    try {
        CEnumerationPtr ptrExposureAuto = nodeMapPtr->GetNode("ExposureAuto");
        if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
            return false;
        
        CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
        if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
            return false;
        ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
        
        CFloatPtr ptrExposureTime = nodeMapPtr->GetNode("ExposureTime");
        if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
            return false;
        
        ptrExposureTime->SetValue(exposureTime);

        settings_.exposure_time = exposureTime;
        fireSettingChanged("exposure_time", std::to_string(exposureTime));
	
        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error configuring exposure: " << e.what() << std::endl;
        return false;
    }
}

bool FlirCameraSource::configureGain(float gain) {
    if (!nodeMapPtr) return false;
    
    try {
        CEnumerationPtr ptrGainAuto = nodeMapPtr->GetNode("GainAuto");
        if (!IsAvailable(ptrGainAuto) || !IsWritable(ptrGainAuto))
            return false;
        
        CEnumEntryPtr ptrGainAutoOff = ptrGainAuto->GetEntryByName("Off");
        if (!IsAvailable(ptrGainAutoOff) || !IsReadable(ptrGainAutoOff))
            return false;
        ptrGainAuto->SetIntValue(ptrGainAutoOff->GetValue());
        
        CFloatPtr ptrGain = nodeMapPtr->GetNode("Gain");
        if (!IsAvailable(ptrGain) || !IsWritable(ptrGain))
            return false;
        
        ptrGain->SetValue(gain);

        settings_.gain = gain;
        fireSettingChanged("gain", std::to_string(gain));
	
        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error configuring gain: " << e.what() << std::endl;
        return false;
    }
}

bool FlirCameraSource::getFrameRateRange(float& min, float& max) {
    if (!nodeMapPtr) return false;
    
    try {
        CFloatPtr ptrFrameRate = nodeMapPtr->GetNode("AcquisitionFrameRate");
        if (IsAvailable(ptrFrameRate) && IsReadable(ptrFrameRate)) {
            min = static_cast<float>(ptrFrameRate->GetMin());
            max = static_cast<float>(ptrFrameRate->GetMax());
            return true;
        }
    } catch (...) {}
    return false;
}

bool FlirCameraSource::configureFrameRate(float frameRate, float* actualRate) {
  if (!nodeMapPtr) return false;
  
  try {
    CBooleanPtr ptrFrameRateEnable = nodeMapPtr->GetNode("AcquisitionFrameRateEnable");
    if (!IsAvailable(ptrFrameRateEnable) || !IsWritable(ptrFrameRateEnable))
      return false;
    ptrFrameRateEnable->SetValue(true);
    
    CFloatPtr ptrFrameRate = nodeMapPtr->GetNode("AcquisitionFrameRate");
    if (!IsAvailable(ptrFrameRate) || !IsWritable(ptrFrameRate))
      return false;
    
    ptrFrameRate->SetValue(frameRate);
    
    // Read back actual value
    fps = static_cast<float>(ptrFrameRate->GetValue());
    if (actualRate) *actualRate = fps;


    settings_.frame_rate = fps;
    fireSettingChanged("frame_rate", std::to_string(fps));
	
    std::cout << "Frame rate - Requested: " << frameRate 
	      << " Hz, Actual: " << fps << " Hz" << std::endl;
    
    return true;
  } catch (Spinnaker::Exception &e) {
    std::cerr << "Error configuring frame rate: " << e.what() << std::endl;
    return false;
  }
}

float FlirCameraSource::getFrameRate() const {
  if (!nodeMapPtr) return fps;  // Return cached value
  
  try {
    CFloatPtr ptrFrameRate = nodeMapPtr->GetNode("AcquisitionFrameRate");
    if (IsAvailable(ptrFrameRate) && IsReadable(ptrFrameRate)) {
      return static_cast<float>(ptrFrameRate->GetValue());
    }
  } catch (...) {}
  
  return fps;  // Fallback to cached value
}

bool FlirCameraSource::getROIConstraints(ROIConstraints& constraints) {
    if (!nodeMapPtr) return false;
    
    try {
        CIntegerPtr ptrWidth = nodeMapPtr->GetNode("Width");
        if (IsAvailable(ptrWidth) && IsReadable(ptrWidth)) {
            constraints.width_min = ptrWidth->GetMin();
            constraints.width_max = ptrWidth->GetMax();
            constraints.width_inc = ptrWidth->GetInc();
        }
        
        CIntegerPtr ptrHeight = nodeMapPtr->GetNode("Height");
        if (IsAvailable(ptrHeight) && IsReadable(ptrHeight)) {
            constraints.height_min = ptrHeight->GetMin();
            constraints.height_max = ptrHeight->GetMax();
            constraints.height_inc = ptrHeight->GetInc();
        }
        
        CIntegerPtr ptrOffsetX = nodeMapPtr->GetNode("OffsetX");
        if (IsAvailable(ptrOffsetX) && IsReadable(ptrOffsetX)) {
            constraints.offset_x_min = ptrOffsetX->GetMin();
            constraints.offset_x_max = ptrOffsetX->GetMax();
            constraints.offset_x_inc = ptrOffsetX->GetInc();
        }
        
        CIntegerPtr ptrOffsetY = nodeMapPtr->GetNode("OffsetY");
        if (IsAvailable(ptrOffsetY) && IsReadable(ptrOffsetY)) {
            constraints.offset_y_min = ptrOffsetY->GetMin();
            constraints.offset_y_max = ptrOffsetY->GetMax();
            constraints.offset_y_inc = ptrOffsetY->GetInc();
        }
        
        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error getting ROI constraints: " << e.what() << std::endl;
        return false;
    }
}

bool FlirCameraSource::setROIOffset(int offsetX, int offsetY) {
    if (!nodeMapPtr) return false;
    
    try {
        // Get constraints
        ROIConstraints constraints;
        if (!getROIConstraints(constraints)) {
            std::cerr << "Failed to get ROI constraints" << std::endl;
            return false;
        }
        
        // Align to increment
        offsetX = (offsetX / constraints.offset_x_inc) * constraints.offset_x_inc;
        offsetY = (offsetY / constraints.offset_y_inc) * constraints.offset_y_inc;
        
        // Get sensor and current ROI dimensions
        CIntegerPtr ptrWidthMax = nodeMapPtr->GetNode("WidthMax");
        CIntegerPtr ptrHeightMax = nodeMapPtr->GetNode("HeightMax");
        int sensor_width = (IsAvailable(ptrWidthMax) && IsReadable(ptrWidthMax)) ? 
                          ptrWidthMax->GetValue() : 1920;
        int sensor_height = (IsAvailable(ptrHeightMax) && IsReadable(ptrHeightMax)) ? 
                           ptrHeightMax->GetValue() : 1200;
        
        // Clamp to valid range
        int max_offset_x = sensor_width - width;
        int max_offset_y = sensor_height - height;
        
        offsetX = std::max(0, std::min(offsetX, max_offset_x));
        offsetY = std::max(0, std::min(offsetY, max_offset_y));
        
        // Re-align after clamping
        offsetX = (offsetX / constraints.offset_x_inc) * constraints.offset_x_inc;
        offsetY = (offsetY / constraints.offset_y_inc) * constraints.offset_y_inc;
        
        // Set offsets ONLY (width/height unchanged, safe during streaming)
        CIntegerPtr ptrOffsetX = nodeMapPtr->GetNode("OffsetX");
        if (!IsAvailable(ptrOffsetX) || !IsWritable(ptrOffsetX))
            return false;
        ptrOffsetX->SetValue(offsetX);
        
        CIntegerPtr ptrOffsetY = nodeMapPtr->GetNode("OffsetY");
        if (!IsAvailable(ptrOffsetY) || !IsWritable(ptrOffsetY))
            return false;
        ptrOffsetY->SetValue(offsetY);
        
        // Update cached values
        offset_x = offsetX;
        offset_y = offsetY;
        
        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error setting ROI offset: " << e.what() << std::endl;
        return false;
    }
}

bool FlirCameraSource::configureBinning(int horizontal, int vertical)
{
  try {
    Spinnaker::GenApi::INodeMap& nodeMap = pCam->GetNodeMap();
    
    // Stop acquisition if running
    bool wasAcquiring = pCam->IsStreaming();
    if (wasAcquiring) {
      pCam->EndAcquisition();
    }
    
    // Reset ROI to maximum before changing binning
    Spinnaker::GenApi::CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
    Spinnaker::GenApi::CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
    Spinnaker::GenApi::CIntegerPtr ptrOffsetX = nodeMap.GetNode("OffsetX");
    Spinnaker::GenApi::CIntegerPtr ptrOffsetY = nodeMap.GetNode("OffsetY");
    
    if (IsWritable(ptrOffsetX)) ptrOffsetX->SetValue(0);
    if (IsWritable(ptrOffsetY)) ptrOffsetY->SetValue(0);
    if (IsWritable(ptrWidth)) ptrWidth->SetValue(ptrWidth->GetMax());
    if (IsWritable(ptrHeight)) ptrHeight->SetValue(ptrHeight->GetMax());
    
    // Now set binning selector and binning amount
    CEnumerationPtr ptrBinningSelector = nodeMap.GetNode("BinningSelector");
    if (IsWritable(ptrBinningSelector)) {
      CEnumEntryPtr ptrBinningSelectorAll = ptrBinningSelector->GetEntryByName("All");
      if (IsReadable(ptrBinningSelectorAll)) {
        ptrBinningSelector->SetIntValue(ptrBinningSelectorAll->GetValue());
      }
    }
    
    Spinnaker::GenApi::CIntegerPtr ptrBinningHorizontal = nodeMap.GetNode("BinningHorizontal");
    Spinnaker::GenApi::CIntegerPtr ptrBinningVertical = nodeMap.GetNode("BinningVertical");
    
    if (IsWritable(ptrBinningHorizontal)) {
      ptrBinningHorizontal->SetValue(horizontal);
      std::cout << "Set horizontal binning to " << horizontal << std::endl;
    }
    
    if (IsWritable(ptrBinningVertical)) {
      ptrBinningVertical->SetValue(vertical);
      std::cout << "Set vertical binning to " << vertical << std::endl;
    }

    // Update cached values with what the camera actually accepted
    if (IsReadable(ptrBinningHorizontal)) {
      binning_h = static_cast<int>(ptrBinningHorizontal->GetValue());
    }
    if (IsReadable(ptrBinningVertical)) {
      binning_v = static_cast<int>(ptrBinningVertical->GetValue());
    }

    // Restart acquisition if it was running
    if (wasAcquiring) {
      pCam->BeginAcquisition();
    }
    return true;
  } catch (Spinnaker::Exception& e) {
    std::cerr << "Error setting binning: " << e.what() << std::endl;
    return false;
  }
}

bool FlirCameraSource::configureROI(int w, int h, int offsetX, int offsetY) {
    if (!nodeMapPtr) return false;
    
    try {
        // Get constraints first
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
        
        // Now set the values
        CIntegerPtr ptrWidth = nodeMapPtr->GetNode("Width");
        if (!IsAvailable(ptrWidth) || !IsWritable(ptrWidth))
            return false;
        ptrWidth->SetValue(w);
        
        CIntegerPtr ptrHeight = nodeMapPtr->GetNode("Height");
        if (!IsAvailable(ptrHeight) || !IsWritable(ptrHeight))
            return false;
        ptrHeight->SetValue(h);
        
        CIntegerPtr ptrOffsetX = nodeMapPtr->GetNode("OffsetX");
        if (!IsAvailable(ptrOffsetX) || !IsWritable(ptrOffsetX))
            return false;
        ptrOffsetX->SetValue(offsetX);
        
        CIntegerPtr ptrOffsetY = nodeMapPtr->GetNode("OffsetY");
        if (!IsAvailable(ptrOffsetY) || !IsWritable(ptrOffsetY))
            return false;
        ptrOffsetY->SetValue(offsetY);
        
        width = w;
        height = h;
	offset_x = offsetX;
	offset_y = offsetY;
	
        // Update global frame dimensions
        frame_width = w;
        frame_height = h;
        
        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error configuring ROI: " << e.what() << std::endl;
        return false;
    }
}

bool FlirCameraSource::configureChunkData(bool enable, bool verbose) {
    if (!nodeMapPtr) return false;
    
    try {
        if (!enable) {
            CBooleanPtr ptrChunkModeActive = nodeMapPtr->GetNode("ChunkModeActive");
            if (IsAvailable(ptrChunkModeActive) && IsWritable(ptrChunkModeActive)) {
                ptrChunkModeActive->SetValue(false);
            }
            chunk_line_status_ok_ = false;
            return true;
        }
        
        // Activate chunk mode
        CBooleanPtr ptrChunkModeActive = nodeMapPtr->GetNode("ChunkModeActive");
        if (!IsAvailable(ptrChunkModeActive) || !IsWritable(ptrChunkModeActive)) {
            std::cerr << "Unable to activate chunk mode" << std::endl;
            return false;
        }
        ptrChunkModeActive->SetValue(true);
        if (verbose) std::cout << "Chunk mode activated..." << std::endl;
        
        // Enable all types of chunk data
        NodeList_t entries;
        CEnumerationPtr ptrChunkSelector = nodeMapPtr->GetNode("ChunkSelector");
        if (!IsAvailable(ptrChunkSelector) || !IsReadable(ptrChunkSelector)) {
            std::cerr << "Unable to retrieve chunk selector" << std::endl;
            return false;
        }
        
        ptrChunkSelector->GetEntries(entries);
        if (verbose) std::cout << "Enabling chunk entries..." << std::endl;
        
        for (size_t i = 0; i < entries.size(); i++) {
            CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);
            if (!IsAvailable(ptrChunkSelectorEntry) || !IsReadable(ptrChunkSelectorEntry))
                continue;

            ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());
            if (verbose) std::cout << "\t" << ptrChunkSelectorEntry->GetSymbolic() << ": ";

            bool enabled = false;
            CBooleanPtr ptrChunkEnable = nodeMapPtr->GetNode("ChunkEnable");
            if (!IsAvailable(ptrChunkEnable)) {
                if (verbose) std::cout << "Node not available" << std::endl;
            } else if (ptrChunkEnable->GetValue()) {
                enabled = true;
                if (verbose) std::cout << "Enabled" << std::endl;
            } else if (IsWritable(ptrChunkEnable)) {
                ptrChunkEnable->SetValue(true);
                enabled = true;
                if (verbose) std::cout << "Enabled" << std::endl;
            } else {
                if (verbose) std::cout << "Node not writable" << std::endl;
            }
            if (enabled &&
                ptrChunkSelectorEntry->GetSymbolic() == "ExposureEndLineStatusAll") {
                chunk_line_status_ok_ = true;
            }
        }

        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error configuring chunk data: " << e.what() << std::endl;
        return false;
    }
}

FlirCameraSource::~FlirCameraSource() {
    close();
}

/*********************************************************************/
/*                 Generic GenICam node access                       */
/*********************************************************************/

namespace {
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
#include "GenICamNodeOps.inl"
}

bool FlirCameraSource::getNodeInfo(const std::string& name, NodeInfo& info,
                                   std::string& error) {
    if (!nodeMapPtr) { error = "camera not open"; return false; }
    return genicamNodeInfo(nodeMapPtr, name, info, error);
}

bool FlirCameraSource::setNodeValue(const std::string& name, const std::string& value,
                                    std::string& error) {
    if (!nodeMapPtr) { error = "camera not open"; return false; }

    // Some features are locked while streaming; pause the stream for those
    bool restart = false;
    try {
        INode* node = nodeMapPtr->GetNode(name.c_str());
        restart = pCam && pCam->IsStreaming() && node && !IsWritable(node);
    } catch (...) {}
    if (restart) stopAcquisition();

    bool ok = genicamSetNode(nodeMapPtr, name, value, error);

    if (restart) startAcquisition();
    return ok;
}

void FlirCameraSource::listNodes(std::vector<std::string>& names) {
    if (nodeMapPtr) genicamListNodes(nodeMapPtr, names);
}


#endif // USE_FLIR
