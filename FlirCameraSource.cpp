#ifdef USE_FLIR

#include <iostream>
#include <atomic>
#include <thread>
#include "VstreamEvent.h"
#include <tcl.h>
#include "FlirCameraSource.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;


extern std::atomic<int> frame_width, frame_height;

void FlirCameraSource::fireSettingChanged(const std::string& setting_name, 
                                          const std::string& value) {
    std::map<std::string, std::string> data;
    data["name"] = setting_name;
    data["value"] = value;
    
    VstreamEvent evt("flir/settings", VstreamEventData::makeKeyValue(data));
    evt.rate_limit_exempt = true;
    fireEvent(evt);
}

void FlirCameraSource::fireAllSettings() {
    fireSettingChanged("exposure_time", std::to_string(settings_.exposure_time));
    fireSettingChanged("gain", std::to_string(settings_.gain));
    fireSettingChanged("frame_rate", std::to_string(settings_.frame_rate));
    fireSettingChanged("acquisition_running", 
                      settings_.acquisition_running ? "1" : "0");
}

FlirCameraSource::FlirCameraSource(int cameraId, int width, int height)
    : camera_id(cameraId)
    , width(width)
    , height(height)
    , nodeMapPtr(nullptr)
    , fps(100.0)
    , offset_x(0)
    , offset_y(0)      
    , color(false)
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
        
        // Get line status
        metadata.lineStatus = getLineStatus();
        
        // Get chunk data
        ChunkData chunkData = pResultImage->GetChunkData();
        metadata.frameID = chunkData.GetFrameID();
        metadata.timestamp = chunkData.GetTimestamp();
        metadata.systemTime = std::chrono::high_resolution_clock::now();
        
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

bool FlirCameraSource::configureBinning(int horizontal, int vertical) {
    if (!nodeMapPtr) return false;
    
    try {
        bool wasStreaming = pCam->IsStreaming();
        if (wasStreaming) {
            pCam->EndAcquisition();
        }
        
        // Set horizontal binning
        CIntegerPtr ptrBinningHorizontal = nodeMapPtr->GetNode("BinningHorizontal");
        if (IsAvailable(ptrBinningHorizontal) && IsWritable(ptrBinningHorizontal)) {
            ptrBinningHorizontal->SetValue(horizontal);
            binning_h = static_cast<int>(ptrBinningHorizontal->GetValue());
            std::cout << "Horizontal binning set to: " << binning_h << std::endl;
        } else {
            std::cerr << "BinningHorizontal not available" << std::endl;
            if (wasStreaming) pCam->BeginAcquisition();
            return false;
        }
        
        // Set vertical binning
        CIntegerPtr ptrBinningVertical = nodeMapPtr->GetNode("BinningVertical");
        if (IsAvailable(ptrBinningVertical) && IsWritable(ptrBinningVertical)) {
            ptrBinningVertical->SetValue(vertical);
            binning_v = static_cast<int>(ptrBinningVertical->GetValue());
            std::cout << "Vertical binning set to: " << binning_v << std::endl;
        } else {
            std::cerr << "BinningVertical not available" << std::endl;
            if (wasStreaming) pCam->BeginAcquisition();
            return false;
        }
        
        // Update width/height after binning change
        CIntegerPtr ptrWidth = nodeMapPtr->GetNode("Width");
        if (IsAvailable(ptrWidth) && IsReadable(ptrWidth)) {
            width = static_cast<int>(ptrWidth->GetValue());
        }
        
        CIntegerPtr ptrHeight = nodeMapPtr->GetNode("Height");
        if (IsAvailable(ptrHeight) && IsReadable(ptrHeight)) {
            height = static_cast<int>(ptrHeight->GetValue());
        }
        
        frame_width = width;
        frame_height = height;
        
        if (wasStreaming) {
            pCam->BeginAcquisition();
        }
        
        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error configuring binning: " << e.what() << std::endl;
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
            
            CBooleanPtr ptrChunkEnable = nodeMapPtr->GetNode("ChunkEnable");
            if (!IsAvailable(ptrChunkEnable)) {
                if (verbose) std::cout << "Node not available" << std::endl;
            } else if (ptrChunkEnable->GetValue()) {
                if (verbose) std::cout << "Enabled" << std::endl;
            } else if (IsWritable(ptrChunkEnable)) {
                ptrChunkEnable->SetValue(true);
                if (verbose) std::cout << "Enabled" << std::endl;
            } else {
                if (verbose) std::cout << "Node not writable" << std::endl;
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
/*                  FLIR Configure Commands                          */
/*********************************************************************/

// Check if FLIR support is compiled in
static int flirIsAvailableCmd(ClientData clientData, Tcl_Interp *interp,
			      int objc, Tcl_Obj *const objv[]) {
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, "");
    return TCL_ERROR;
  }
  
#ifdef USE_FLIR
  Tcl_SetObjResult(interp, Tcl_NewBooleanObj(1));
#else
  Tcl_SetObjResult(interp, Tcl_NewBooleanObj(0));
#endif
  return TCL_OK;
}

static int flirGetSettingsCmd(ClientData clientData, Tcl_Interp *interp,
                              int objc, Tcl_Obj *const objv[]) {
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* source = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
    
  if (source) {
    
    Tcl_Obj* settingsDict = Tcl_NewDictObj();
    
    Tcl_DictObjPut(interp, settingsDict,
                   Tcl_NewStringObj("exposure_time", -1),
                   Tcl_NewDoubleObj(source->settings_.exposure_time));
    
    Tcl_DictObjPut(interp, settingsDict,
                   Tcl_NewStringObj("gain", -1),
                   Tcl_NewDoubleObj(source->settings_.gain));
    
    Tcl_DictObjPut(interp, settingsDict,
                   Tcl_NewStringObj("frame_rate", -1),
                   Tcl_NewDoubleObj(source->settings_.frame_rate));
    
    Tcl_DictObjPut(interp, settingsDict,
                   Tcl_NewStringObj("acquisition_running", -1),
                   Tcl_NewBooleanObj(source->settings_.acquisition_running));
    
    Tcl_SetObjResult(interp, settingsDict);
  }
  else {
    Tcl_SetResult(interp, "No FLIR camera active", TCL_STATIC);
    return TCL_ERROR;
  }
#endif
  return TCL_OK;
}

static int flirRefreshSettingsCmd(ClientData clientData, Tcl_Interp *interp,
                                  int objc, Tcl_Obj *const objv[]) {
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* flirSource = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
    
  if (!flirSource) {
    Tcl_SetResult(interp, "No FLIR camera active", TCL_STATIC);
    return TCL_ERROR;
  }
  
  // Fire events for all current settings
  flirSource->fireAllSettings();
#endif
  
  return TCL_OK;
}

static int startAcquisitionCmd(ClientData clientData, Tcl_Interp *interp,
                int argc, char *argv[])
{
  int res = 0;
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* flirSource = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
  if (flirSource) {
    res = flirSource->startAcquisition() ? 1 : -1;
  } else {
    Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
    return TCL_ERROR;
  }
#else
  Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
  return TCL_ERROR;
#endif
  
  if (res < 0) {
    Tcl_AppendResult(interp, argv[0], ": error starting acquisition", NULL);
    return TCL_ERROR;
  }
  return TCL_OK;
}

static int stopAcquisitionCmd(ClientData clientData, Tcl_Interp *interp,
                int argc, char *argv[])
{
  int res = 0;
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* flirSource = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
  if (flirSource) {
    res = flirSource->stopAcquisition() ? 1 : -1;
  } else {
    Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
    return TCL_ERROR;
  }
#else
  Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
  return TCL_ERROR;
#endif
  
  if (res < 0) {
    Tcl_AppendResult(interp, argv[0], ": error stopping acquisition", NULL);
    return TCL_ERROR;
  }
  return TCL_OK;
}

static int isStreamingCmd(ClientData clientData, Tcl_Interp *interp,
                int argc, char *argv[])
{
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* flirSource = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
  if (flirSource) {
    Tcl_SetResult(interp, (char*)(flirSource->isStreaming() ? "1" : "0"), TCL_STATIC);
  } else {
    Tcl_SetResult(interp, "0", TCL_STATIC);
  }
#else
  Tcl_SetResult(interp, "0", TCL_STATIC);
#endif
  return TCL_OK;
}

static int configureImageOrientationCmd(ClientData clientData, Tcl_Interp *interp,
                                        int argc, char *argv[])
{
#ifdef USE_FLIR
    extern IFrameSource* g_frameSource;
    FlirCameraSource* flirSource = 
        dynamic_cast<FlirCameraSource*>(g_frameSource);
    
    if (!flirSource) {
        Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
        return TCL_ERROR;
    }

    // Query mode - return current orientation (note: you'd need to add getters to FlirCameraSource)
    if (argc == 1) {
        // For now, just return success since we don't track this state
        // TODO: Add reverseX/reverseY state tracking to FlirCameraSource
        Tcl_SetResult(interp, "Query not implemented - use: flir::configureImageOrientation reverseX reverseY", TCL_STATIC);
        return TCL_ERROR;
    }
    
    if (argc != 3) {
        Tcl_AppendResult(interp, "usage: ", argv[0], " reverseX reverseY", NULL);
        return TCL_ERROR;
    }
    
    int reverseX, reverseY;
    if (Tcl_GetInt(interp, argv[1], &reverseX) != TCL_OK ||
        Tcl_GetInt(interp, argv[2], &reverseY) != TCL_OK) {
        return TCL_ERROR;
    }
    
    if (flirSource->configureImageOrientation(reverseX, reverseY)) {
        return TCL_OK;
    } else {
        Tcl_AppendResult(interp, argv[0], ": failed to configure orientation", NULL);
        return TCL_ERROR;
    }
#else
    Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
    return TCL_ERROR;
#endif
}

static int configureExposureCmd(ClientData clientData, Tcl_Interp *interp,
                int argc, char *argv[])
{
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* flirSource = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
    
  if (!flirSource) {
    Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
    return TCL_ERROR;
  }
  
  // Query mode - return current exposure
  if (argc == 1) {
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(flirSource->settings_.exposure_time));
    return TCL_OK;
  }
  
  // Set mode
  if (argc != 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " ?exposure?", NULL);
    return TCL_ERROR;
  }
  
  double exposure;
  if (Tcl_GetDouble(interp, argv[1], &exposure) != TCL_OK) {
    return TCL_ERROR;
  }
  
  if (flirSource->configureExposure(exposure)) {
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(flirSource->settings_.exposure_time));
    return TCL_OK;
  } else {
    Tcl_AppendResult(interp, argv[0], ": error configuring exposure", NULL);
    return TCL_ERROR;
  }
#else
  Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
  return TCL_ERROR;
#endif
}

static int configureGainCmd(ClientData clientData, Tcl_Interp *interp,
                int argc, char *argv[])
{
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* flirSource = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
    
  if (!flirSource) {
    Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
    return TCL_ERROR;
  }
  
  // Query mode - return current gain
  if (argc == 1) {
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(flirSource->settings_.gain));
    return TCL_OK;
  }
  
  // Set mode
  if (argc != 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " ?gain?", NULL);
    return TCL_ERROR;
  }
  
  double gain;
  if (Tcl_GetDouble(interp, argv[1], &gain) != TCL_OK) {
    return TCL_ERROR;
  }
  
  if (flirSource->configureGain(gain)) {
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(flirSource->settings_.gain));
    return TCL_OK;
  } else {
    Tcl_AppendResult(interp, argv[0], ": error configuring gain", NULL);
    return TCL_ERROR;
  }
#else
  Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
  return TCL_ERROR;
#endif
}

static int configureFrameRateCmd(ClientData clientData, Tcl_Interp *interp,
                                 int argc, char *argv[])
{
#ifdef USE_FLIR
    extern IFrameSource* g_frameSource;
    FlirCameraSource* flirSource = 
        dynamic_cast<FlirCameraSource*>(g_frameSource);
    
    if (!flirSource) {
        Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
        return TCL_ERROR;
    }
    
    // Query mode - return current frame rate
    if (argc == 1) {
        float current_fps = flirSource->getFrameRate();
        Tcl_SetObjResult(interp, Tcl_NewDoubleObj(current_fps));
        return TCL_OK;
    }
    
    // Set mode
    if (argc != 2) {
        Tcl_AppendResult(interp, "usage: ", argv[0], " ?frameRate?", NULL);
        return TCL_ERROR;
    }
    
    double requested_fr;
    if (Tcl_GetDouble(interp, argv[1], &requested_fr) != TCL_OK) {
        return TCL_ERROR;
    }
    
    float actual_fr;
    if (flirSource->configureFrameRate(requested_fr, &actual_fr)) {
        Tcl_SetObjResult(interp, Tcl_NewDoubleObj(actual_fr));
        return TCL_OK;
    } else {
        Tcl_AppendResult(interp, argv[0], ": error configuring frame rate", NULL);
        return TCL_ERROR;
    }
#else
    Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
    return TCL_ERROR;
#endif
}

static int getFrameRateRangeCmd(ClientData clientData, Tcl_Interp *interp,
                                int argc, char *argv[])
{
#ifdef USE_FLIR
    extern IFrameSource* g_frameSource;
    FlirCameraSource* flirSource = 
        dynamic_cast<FlirCameraSource*>(g_frameSource);
    
    if (!flirSource) {
        Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
        return TCL_ERROR;
    }
    
    float min_fps, max_fps;
    if (!flirSource->getFrameRateRange(min_fps, max_fps)) {
        Tcl_AppendResult(interp, argv[0], ": failed to get frame rate range", NULL);
        return TCL_ERROR;
    }
    
    // Return as a dict
    Tcl_Obj* resultDict = Tcl_NewDictObj();
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("min", -1),
                   Tcl_NewDoubleObj(min_fps));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("max", -1),
                   Tcl_NewDoubleObj(max_fps));
    
    Tcl_SetObjResult(interp, resultDict);
    return TCL_OK;
#else
    Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
    return TCL_ERROR;
#endif
}

static int getROIConstraintsCmd(ClientData clientData, Tcl_Interp *interp,
                                int argc, char *argv[])
{
#ifdef USE_FLIR
    extern IFrameSource* g_frameSource;
    FlirCameraSource* flirSource = 
        dynamic_cast<FlirCameraSource*>(g_frameSource);
    
    if (!flirSource) {
        Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
        return TCL_ERROR;
    }
    
    FlirCameraSource::ROIConstraints constraints;
    if (!flirSource->getROIConstraints(constraints)) {
        Tcl_AppendResult(interp, argv[0], ": failed to get constraints", NULL);
        return TCL_ERROR;
    }
    
    // Return as a dict
    Tcl_Obj* resultDict = Tcl_NewDictObj();
    
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("width_min", -1),
                   Tcl_NewIntObj(constraints.width_min));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("width_max", -1),
                   Tcl_NewIntObj(constraints.width_max));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("width_inc", -1),
                   Tcl_NewIntObj(constraints.width_inc));
    
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("height_min", -1),
                   Tcl_NewIntObj(constraints.height_min));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("height_max", -1),
                   Tcl_NewIntObj(constraints.height_max));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("height_inc", -1),
                   Tcl_NewIntObj(constraints.height_inc));
    
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_x_min", -1),
                   Tcl_NewIntObj(constraints.offset_x_min));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_x_max", -1),
                   Tcl_NewIntObj(constraints.offset_x_max));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_x_inc", -1),
                   Tcl_NewIntObj(constraints.offset_x_inc));
    
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_y_min", -1),
                   Tcl_NewIntObj(constraints.offset_y_min));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_y_max", -1),
                   Tcl_NewIntObj(constraints.offset_y_max));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_y_inc", -1),
                   Tcl_NewIntObj(constraints.offset_y_inc));
    
    Tcl_SetObjResult(interp, resultDict);
    return TCL_OK;
#else
    Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
    return TCL_ERROR;
#endif
}

static int configureBinningCmd(ClientData clientData, Tcl_Interp *interp,
                               int argc, char *argv[])
{
#ifdef USE_FLIR
    extern IFrameSource* g_frameSource;
    FlirCameraSource* flirSource = 
        dynamic_cast<FlirCameraSource*>(g_frameSource);
    
    if (!flirSource) {
        Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
        return TCL_ERROR;
    }
    
    // Query mode - return current binning
    if (argc == 1) {
        Tcl_Obj* resultDict = Tcl_NewDictObj();
        Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("horizontal", -1),
                       Tcl_NewIntObj(flirSource->getBinningH()));
        Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("vertical", -1),
                       Tcl_NewIntObj(flirSource->getBinningV()));
        Tcl_SetObjResult(interp, resultDict);
        return TCL_OK;
    }
    
    // Set mode
    if (argc != 3) {
        Tcl_AppendResult(interp, "usage: ", argv[0], " ?horizontal vertical?", NULL);
        return TCL_ERROR;
    }
    
    int h, v;
    if (Tcl_GetInt(interp, argv[1], &h) != TCL_OK ||
        Tcl_GetInt(interp, argv[2], &v) != TCL_OK) {
        return TCL_ERROR;
    }
    
    if (flirSource->configureBinning(h, v)) {
        Tcl_Obj* resultDict = Tcl_NewDictObj();
        Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("horizontal", -1),
                       Tcl_NewIntObj(flirSource->getBinningH()));
        Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("vertical", -1),
                       Tcl_NewIntObj(flirSource->getBinningV()));
        Tcl_SetObjResult(interp, resultDict);
        return TCL_OK;
    } else {
        Tcl_AppendResult(interp, argv[0], ": failed to configure binning", NULL);
        return TCL_ERROR;
    }
#else
    Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
    return TCL_ERROR;
#endif
}

static int configureROICmd(ClientData clientData, Tcl_Interp *interp,
               int argc, char *argv[])
{
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* flirSource = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
    
  if (!flirSource) {
    Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
    return TCL_ERROR;
  }
  
  // Query mode - return current ROI
  if (argc == 1) {
    Tcl_Obj* resultDict = Tcl_NewDictObj();
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("width", -1),
                   Tcl_NewIntObj(flirSource->getWidth()));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("height", -1),
                   Tcl_NewIntObj(flirSource->getHeight()));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_x", -1),
                   Tcl_NewIntObj(flirSource->getOffsetX()));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_y", -1),
                   Tcl_NewIntObj(flirSource->getOffsetY()));
    Tcl_SetObjResult(interp, resultDict);
    return TCL_OK;
  }
  
  // Set mode
  if (argc != 5) {
    Tcl_AppendResult(interp, "usage: ", argv[0],
             " ?width height offsetX offsetY?", NULL);
    return TCL_ERROR;
  }
  
  int w, h, x, y;
  if (Tcl_GetInt(interp, argv[1], &w) != TCL_OK) return TCL_ERROR;
  if (Tcl_GetInt(interp, argv[2], &h) != TCL_OK) return TCL_ERROR;
  if (Tcl_GetInt(interp, argv[3], &x) != TCL_OK) return TCL_ERROR;
  if (Tcl_GetInt(interp, argv[4], &y) != TCL_OK) return TCL_ERROR;
  
  if (flirSource->configureROI(w, h, x, y)) {
    // Return actual values set
    Tcl_Obj* resultDict = Tcl_NewDictObj();
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("width", -1),
                   Tcl_NewIntObj(flirSource->getWidth()));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("height", -1),
                   Tcl_NewIntObj(flirSource->getHeight()));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_x", -1),
                   Tcl_NewIntObj(flirSource->getOffsetX()));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_y", -1),
                   Tcl_NewIntObj(flirSource->getOffsetY()));
    Tcl_SetObjResult(interp, resultDict);
    return TCL_OK;
  } else {
    Tcl_AppendResult(interp, argv[0], ": error configuring ROI", NULL);
    return TCL_ERROR;
  }
#else
  Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
  return TCL_ERROR;
#endif
}

static int getROICmd(ClientData clientData, Tcl_Interp *interp,
                     int argc, char *argv[])
{
#ifdef USE_FLIR
    extern IFrameSource* g_frameSource;
    FlirCameraSource* flirSource = 
        dynamic_cast<FlirCameraSource*>(g_frameSource);
    
    if (!flirSource) {
        Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
        return TCL_ERROR;
    }
    
    Tcl_Obj* resultDict = Tcl_NewDictObj();
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("width", -1),
                   Tcl_NewIntObj(flirSource->getWidth()));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("height", -1),
                   Tcl_NewIntObj(flirSource->getHeight()));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_x", -1),
                   Tcl_NewIntObj(flirSource->getOffsetX()));
    Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_y", -1),
                   Tcl_NewIntObj(flirSource->getOffsetY()));
    
    Tcl_SetObjResult(interp, resultDict);
    return TCL_OK;
#else
    Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
    return TCL_ERROR;
#endif
}

static int setROIOffsetCmd(ClientData clientData, Tcl_Interp *interp,
                           int argc, char *argv[])
{
#ifdef USE_FLIR
    extern IFrameSource* g_frameSource;
    FlirCameraSource* flirSource = 
        dynamic_cast<FlirCameraSource*>(g_frameSource);
    
    if (!flirSource) {
        Tcl_AppendResult(interp, argv[0], ": FLIR camera not active", NULL);
        return TCL_ERROR;
    }
    
    // Query mode - return current offset
    if (argc == 1) {
        Tcl_Obj* resultDict = Tcl_NewDictObj();
        Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_x", -1),
                       Tcl_NewIntObj(flirSource->getOffsetX()));
        Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_y", -1),
                       Tcl_NewIntObj(flirSource->getOffsetY()));
        Tcl_SetObjResult(interp, resultDict);
        return TCL_OK;
    }
    
    // Set mode
    if (argc != 3) {
        Tcl_AppendResult(interp, "usage: ", argv[0], " ?offsetX offsetY?", NULL);
        return TCL_ERROR;
    }
    
    int offset_x, offset_y;
    if (Tcl_GetInt(interp, argv[1], &offset_x) != TCL_OK ||
        Tcl_GetInt(interp, argv[2], &offset_y) != TCL_OK) {
        return TCL_ERROR;
    }
    
    if (flirSource->setROIOffset(offset_x, offset_y)) {
        // Return actual values set
        Tcl_Obj* resultDict = Tcl_NewDictObj();
        Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_x", -1),
                       Tcl_NewIntObj(flirSource->getOffsetX()));
        Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("offset_y", -1),
                       Tcl_NewIntObj(flirSource->getOffsetY()));
        Tcl_SetObjResult(interp, resultDict);
        return TCL_OK;
    } else {
        Tcl_AppendResult(interp, argv[0], ": failed to set offset", NULL);
        return TCL_ERROR;
    }
#else
    Tcl_AppendResult(interp, argv[0], ": FLIR support not compiled", NULL);
    return TCL_ERROR;
#endif
}

int add_flir_commands(Tcl_Interp *interp)
{
  Tcl_CreateObjCommand(interp, "flir::isAvailable", 
		       (Tcl_ObjCmdProc *)flirIsAvailableCmd, 
		       (ClientData)NULL, NULL);
  
  Tcl_CreateObjCommand(interp, "flir::getSettings", 
		       flirGetSettingsCmd, (ClientData)NULL, NULL);
  Tcl_CreateObjCommand(interp, "flir::refreshSettings", 
		       flirRefreshSettingsCmd, (ClientData)NULL, NULL);
  
  Tcl_CreateCommand(interp, "flir::startAcquisition",
		    (Tcl_CmdProc *) startAcquisitionCmd, 
		    (ClientData)NULL, (Tcl_CmdDeleteProc *)NULL);
  Tcl_CreateCommand(interp, "flir::stopAcquisition",
		    (Tcl_CmdProc *) stopAcquisitionCmd, 
		    (ClientData)NULL, (Tcl_CmdDeleteProc *)NULL);
  Tcl_CreateCommand(interp, "flir::isStreaming",
		    (Tcl_CmdProc *) isStreamingCmd, 
		    (ClientData)NULL, (Tcl_CmdDeleteProc *)NULL);
  Tcl_CreateCommand(interp, "flir::configureImageOrientation",
		    (Tcl_CmdProc *) configureImageOrientationCmd, 
		    (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "flir::configureExposure",
		    (Tcl_CmdProc *) configureExposureCmd, 
		    (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "flir::configureGain",
		    (Tcl_CmdProc *) configureGainCmd, 
		    (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "flir::configureFrameRate",
		    (Tcl_CmdProc *) configureFrameRateCmd, 
		    (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "flir::getFrameRateRange",
		    (Tcl_CmdProc *) getFrameRateRangeCmd, 
		    (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

  Tcl_CreateCommand(interp, "flir::configureBinning",
		    (Tcl_CmdProc *) configureBinningCmd, 
		    (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
 
  Tcl_CreateCommand(interp, "flir::getROIConstraints",
		    (Tcl_CmdProc *) getROIConstraintsCmd, 
		    (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);  
  Tcl_CreateCommand(interp, "flir::configureROI",
		    (Tcl_CmdProc *) configureROICmd, 
		    (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "flir::getROI",
		    (Tcl_CmdProc *) getROICmd, 
		    (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "flir::setROIOffset",
		    (Tcl_CmdProc *) setROIOffsetCmd, 
		    (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);  
  return TCL_OK;
}

#endif // USE_FLIR
