#include "FlirCameraSource.h"

#ifdef USE_FLIR

#include <iostream>
#include <atomic>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;


extern std::atomic<int> frame_width, frame_height;

FlirCameraSource::FlirCameraSource(int cameraId, bool flipView, int flipCode, int width, int height)
    : camera_id(cameraId)
    , flip_view(flipView)
    , flip_code(flipCode)
    , width(width)
    , height(height)
    , nodeMapPtr(nullptr)
    , fps(100.0)
    , offset_x(0)
    , offset_y(0)      
    , color(false)
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
        
        // Apply flip if requested
        if (flip_view) {
            Mat flipped = Mat(cvimg.rows, cvimg.cols, CV_8UC1);
            cv::flip(cvimg, flipped, flip_code);
            frame = flipped.clone();
        } else {
            frame = cvimg.clone();
        }
        
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
        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error configuring gain: " << e.what() << std::endl;
        return false;
    }
}

bool FlirCameraSource::configureFrameRate(float frameRate) {
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
        fps = frameRate;
        return true;
    } catch (Spinnaker::Exception &e) {
        std::cerr << "Error configuring frame rate: " << e.what() << std::endl;
        return false;
    }
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

#endif // USE_FLIR
