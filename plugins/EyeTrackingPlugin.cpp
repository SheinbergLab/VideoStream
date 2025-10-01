#include <tcl.h>
#include "opencv2/opencv.hpp"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <cmath>

#include "SharedQueue.hpp"
#include "IAnalysisPlugin.h"
#include "AnalysisPluginRegistry.h"

// Forward declare the registry
extern AnalysisPluginRegistry g_pluginRegistry;

// Analysis result structures
struct PupilData {
    cv::Point2f center;
    float radius;
    bool detected;
};

struct PurkinjeData {
    cv::Point2f p1_center;
    cv::Point2f p4_center;
    bool p1_detected, p4_detected;
};

struct AnalysisResults {
    int frame_idx;
    PupilData pupil;
    PurkinjeData purkinje;
    bool valid;
};

class EyeTrackingPlugin : public IAnalysisPlugin {
private:
    // Results storage
    std::mutex results_mutex_;
    AnalysisResults latest_results_;
    
    // ROI management
    cv::Rect current_roi_;
    bool roi_enabled_;
    
    // Processing control
    std::atomic<bool> running_;
    std::thread analysis_thread_;

    struct FrameData {
        cv::Mat frame;
        int frame_idx;
        FrameMetadata metadata;
    };
    
    SharedQueue<FrameData> frame_queue_;
    
    // Pre-allocated working buffers
    cv::Mat gray_buffer_;
    cv::Mat binary_buffer_;
    cv::Mat bright_spots_buffer_;
    cv::Mat dim_spots_buffer_;
    cv::Size frame_size_;
    bool buffers_initialized_;
    
    // Pupil detection parameters
    int pupil_threshold_;
    int pupil_min_area_;
    int pupil_max_area_;
    float pupil_min_circularity_;
    
    // Purkinje detection parameters
    int purkinje_threshold_;
    
    // Blob detector
    cv::SimpleBlobDetector::Params blob_params_;
    cv::Ptr<cv::SimpleBlobDetector> detector_;
    
    int frame_count_;
    
    void initializeBlobDetector() {
        blob_params_.filterByArea = true;
        blob_params_.minArea = 5;
        blob_params_.maxArea = 200;
        
        blob_params_.filterByCircularity = true;
        blob_params_.minCircularity = 0.8f;
        
        blob_params_.filterByConvexity = true;
        blob_params_.minConvexity = 0.8f;
        
        blob_params_.filterByInertia = true;
        blob_params_.minInertiaRatio = 0.5f;
        
        detector_ = cv::SimpleBlobDetector::create(blob_params_);
    }
    
    void ensureBuffersAllocated(const cv::Mat& frame) {
        cv::Size current_size(frame.cols, frame.rows);
        
        // Only allocate/reallocate if size changed or not yet initialized
        if (!buffers_initialized_ || current_size != frame_size_) {
            frame_size_ = current_size;
            
            gray_buffer_ = cv::Mat(frame_size_, CV_8UC1);
            binary_buffer_ = cv::Mat(frame_size_, CV_8UC1);
            bright_spots_buffer_ = cv::Mat(frame_size_, CV_8UC1);
            dim_spots_buffer_ = cv::Mat(frame_size_, CV_8UC1);
            
            buffers_initialized_ = true;
            
            std::cout << "Allocated eye tracking buffers: " 
                      << frame_size_.width << "x" << frame_size_.height << std::endl;
        }
    }
    
    PupilData detectPupil(const cv::Mat& frame) {
        PupilData result = {cv::Point2f(-1,-1), -1, false};
        
        cv::Mat roi_frame = roi_enabled_ ? frame(current_roi_) : frame;
        
        // Reuse pre-allocated buffers instead of creating new ones
        cv::Mat gray_roi, binary_roi;
        if (roi_enabled_) {
            // For ROI, use a subset of the buffer
            gray_roi = gray_buffer_(current_roi_);
            binary_roi = binary_buffer_(current_roi_);
        } else {
            gray_roi = gray_buffer_;
            binary_roi = binary_buffer_;
        }
        
        if (roi_frame.channels() > 1) {
            cv::cvtColor(roi_frame, gray_roi, cv::COLOR_BGR2GRAY);
        } else {
            roi_frame.copyTo(gray_roi);
        }
        
        cv::threshold(gray_roi, binary_roi, pupil_threshold_, 255, 
                     cv::THRESH_BINARY_INV);
        
        // Calculate moments for center of mass
        cv::Moments m = cv::moments(binary_roi, true);
        
        if (m.m00 > 0) {
            cv::Point2f center;
            center.x = m.m10 / m.m00;
            center.y = m.m01 / m.m00;
            
            float area = m.m00;
            float radius = sqrt(area / M_PI);
            
            if (roi_enabled_) {
                center.x += current_roi_.x;
                center.y += current_roi_.y;
            }
            
            result = {center, radius, true};
        }
        
        return result;
    }
    
    PurkinjeData detectPurkinje(const cv::Mat& frame, const PupilData& pupil) {
        PurkinjeData result = {{-1,-1}, {-1,-1}, false, false};
        
        // If no pupil detected, can't use spatial prior
        if (!pupil.detected) {
            return result;
        }
        
        cv::Mat roi_frame = roi_enabled_ ? frame(current_roi_) : frame;
        
        cv::Mat gray_roi;
        if (roi_enabled_) {
            gray_roi = gray_buffer_(current_roi_);
        } else {
            gray_roi = gray_buffer_;
        }
        
        if (roi_frame.channels() > 1) {
            cv::cvtColor(roi_frame, gray_roi, cv::COLOR_BGR2GRAY);
        } else {
            roi_frame.copyTo(gray_roi);
        }
        
        // Define search regions based on pupil location
        // Adjust pupil coordinates if ROI is enabled
        cv::Point2f pupil_center_local = pupil.center;
        if (roi_enabled_) {
            pupil_center_local.x -= current_roi_.x;
            pupil_center_local.y -= current_roi_.y;
        }
        
        // P1: search within pupil radius around pupil center
        cv::Rect p1_search_region(
            std::max(0, (int)(pupil_center_local.x - pupil.radius)),
            std::max(0, (int)(pupil_center_local.y - pupil.radius)),
            (int)(pupil.radius * 2),
            (int)(pupil.radius * 2)
        );
        
        // P4: search only inside pupil (slightly smaller to avoid edge effects)
        float p4_radius = pupil.radius * 0.8f;
        cv::Rect p4_search_region(
            std::max(0, (int)(pupil_center_local.x - p4_radius)),
            std::max(0, (int)(pupil_center_local.y - p4_radius)),
            (int)(p4_radius * 2),
            (int)(p4_radius * 2)
        );
        
        // Clamp to gray_roi boundaries
        p1_search_region = p1_search_region & cv::Rect(0, 0, gray_roi.cols, gray_roi.rows);
        p4_search_region = p4_search_region & cv::Rect(0, 0, gray_roi.cols, gray_roi.rows);
        
        // Search for P1 in the smaller region around pupil
        cv::Mat p1_region = gray_roi(p1_search_region);
        cv::Mat p1_bright = bright_spots_buffer_(p1_search_region);
        cv::threshold(p1_region, p1_bright, 240, 255, cv::THRESH_BINARY);
        
        std::vector<cv::KeyPoint> p1_keypoints;
        detector_->detect(p1_bright, p1_keypoints);
        
        // Take brightest spot as P1
        if (!p1_keypoints.empty()) {
            std::sort(p1_keypoints.begin(), p1_keypoints.end(), 
                      [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                          return a.response > b.response;
                      });
            
            result.p1_center = p1_keypoints[0].pt;
            result.p1_center.x += p1_search_region.x;
            result.p1_center.y += p1_search_region.y;
            
            if (roi_enabled_) {
                result.p1_center.x += current_roi_.x;
                result.p1_center.y += current_roi_.y;
            }
            result.p1_detected = true;
        }
        
        // Search for P4 only inside pupil
        cv::Mat p4_region = gray_roi(p4_search_region);
        cv::Mat p4_dim = dim_spots_buffer_(p4_search_region);
        cv::threshold(p4_region, p4_dim, 180, 255, cv::THRESH_BINARY);
        
        std::vector<cv::KeyPoint> p4_keypoints;
        detector_->detect(p4_dim, p4_keypoints);
        
        if (!p4_keypoints.empty()) {
            std::sort(p4_keypoints.begin(), p4_keypoints.end(), 
                      [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                          return a.response > b.response;
                      });
            
            // Take brightest inside pupil as P4
            result.p4_center = p4_keypoints[0].pt;
            result.p4_center.x += p4_search_region.x;
            result.p4_center.y += p4_search_region.y;
            
            if (roi_enabled_) {
                result.p4_center.x += current_roi_.x;
                result.p4_center.y += current_roi_.y;
            }
            result.p4_detected = true;
        }
        
        return result;
    }
    
    void analysisThreadFunc() {
        std::cout << "Eye tracking analysis thread started" << std::endl;
        
        while (running_) {
            try {
                FrameData frame_data = frame_queue_.front();
                frame_queue_.pop_front();

                // Check for shutdown sentinel
                if (frame_data.frame_idx < 0) {
                    break;
                }
                
                if (frame_data.frame.empty()) {
                    continue;
                }
                
                // Ensure buffers are allocated before first use
                ensureBuffersAllocated(frame_data.frame);
                
                auto start = std::chrono::high_resolution_clock::now();
                
                // Detect pupil first
                PupilData pupil = detectPupil(frame_data.frame);
                
                // Use pupil location to constrain Purkinje search
                PurkinjeData purkinje = detectPurkinje(frame_data.frame, pupil);
                
                {
                    std::lock_guard<std::mutex> lock(results_mutex_);
                    latest_results_.frame_idx = frame_data.frame_idx;
                    latest_results_.pupil = pupil;
                    latest_results_.purkinje = purkinje;
                    latest_results_.valid = true;
                }
                
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                
                frame_count_++;
                
                if (frame_count_ % 100 == 0) {
                    std::cout << "Analysis time: " << duration.count() << "µs" << std::endl;
                    if (pupil.detected) {
                        std::cout << "Pupil: (" << pupil.center.x << "," << pupil.center.y 
                                  << ") r=" << pupil.radius << std::endl;
                    }
                }
            } catch (...) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
    
    // Tcl command callbacks
    static int setPupilThresholdCmd(ClientData clientData, Tcl_Interp *interp,
                                    int objc, Tcl_Obj *const objv[]) {
        if (objc != 2) {
            Tcl_WrongNumArgs(interp, 1, objv, "threshold");
            return TCL_ERROR;
        }
        
        int threshold;
        if (Tcl_GetIntFromObj(interp, objv[1], &threshold) != TCL_OK) {
            return TCL_ERROR;
        }
        
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        plugin->pupil_threshold_ = threshold;
        return TCL_OK;
    }
    
    static int setROICmd(ClientData clientData, Tcl_Interp *interp,
                        int objc, Tcl_Obj *const objv[]) {
        if (objc != 5) {
            Tcl_WrongNumArgs(interp, 1, objv, "x y width height");
            return TCL_ERROR;
        }
        
        int x, y, width, height;
        if (Tcl_GetIntFromObj(interp, objv[1], &x) != TCL_OK ||
            Tcl_GetIntFromObj(interp, objv[2], &y) != TCL_OK ||
            Tcl_GetIntFromObj(interp, objv[3], &width) != TCL_OK ||
            Tcl_GetIntFromObj(interp, objv[4], &height) != TCL_OK) {
            return TCL_ERROR;
        }
        
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        plugin->current_roi_ = cv::Rect(x, y, width, height);
        plugin->roi_enabled_ = true;
        return TCL_OK;
    }
    
    static int disableROICmd(ClientData clientData, Tcl_Interp *interp,
                            int objc, Tcl_Obj *const objv[]) {
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        plugin->roi_enabled_ = false;
        return TCL_OK;
    }

    static int getResultsCmd(ClientData clientData, Tcl_Interp *interp,
                             int objc, Tcl_Obj *const objv[]) {
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        
        std::lock_guard<std::mutex> lock(plugin->results_mutex_);
        
        if (!plugin->latest_results_.valid) {
            Tcl_SetObjResult(interp, Tcl_NewStringObj("no results", -1));
            return TCL_OK;
        }
        
        // Build result dict
        Tcl_Obj* resultDict = Tcl_NewDictObj();
        
        // Pupil data
        if (plugin->latest_results_.pupil.detected) {
            Tcl_Obj* pupilDict = Tcl_NewDictObj();
            Tcl_DictObjPut(interp, pupilDict, Tcl_NewStringObj("x", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.pupil.center.x));
            Tcl_DictObjPut(interp, pupilDict, Tcl_NewStringObj("y", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.pupil.center.y));
            Tcl_DictObjPut(interp, pupilDict, Tcl_NewStringObj("radius", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.pupil.radius));
            Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("pupil", -1), pupilDict);
        }
        
        // Purkinje data
        if (plugin->latest_results_.purkinje.p1_detected) {
            Tcl_Obj* p1Dict = Tcl_NewDictObj();
            Tcl_DictObjPut(interp, p1Dict, Tcl_NewStringObj("x", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.purkinje.p1_center.x));
            Tcl_DictObjPut(interp, p1Dict, Tcl_NewStringObj("y", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.purkinje.p1_center.y));
            Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("p1", -1), p1Dict);
        }
        
        if (plugin->latest_results_.purkinje.p4_detected) {
            Tcl_Obj* p4Dict = Tcl_NewDictObj();
            Tcl_DictObjPut(interp, p4Dict, Tcl_NewStringObj("x", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.purkinje.p4_center.x));
            Tcl_DictObjPut(interp, p4Dict, Tcl_NewStringObj("y", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.purkinje.p4_center.y));
            Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("p4", -1), p4Dict);
        }
        
        Tcl_SetObjResult(interp, resultDict);
        return TCL_OK;
    }  
    
public:
    EyeTrackingPlugin() 
        : roi_enabled_(false),
          running_(false),
          pupil_threshold_(65),
          pupil_min_area_(5000),
          pupil_max_area_(100000),
          pupil_min_circularity_(0.65),
          purkinje_threshold_(240),
          frame_count_(0),
          buffers_initialized_(false) {
        
        latest_results_.valid = false;
    }
    
    ~EyeTrackingPlugin() {
        shutdown();
    }
    
    bool initialize(Tcl_Interp* interp) override {
        initializeBlobDetector();
        running_ = true;
        analysis_thread_ = std::thread(&EyeTrackingPlugin::analysisThreadFunc, this);
        return true;
    }
    
    void shutdown() override {
        running_ = false;

        // Push sentinel to unblock thread
        FrameData sentinel;
        sentinel.frame_idx = -1;  // Signal to exit
        frame_queue_.push_back(sentinel);
        
        if (analysis_thread_.joinable()) {
            analysis_thread_.join();
        }
    }
    
    void analyzeFrame(const cv::Mat& frame, int frameIdx, 
                     const FrameMetadata& metadata) override {
        FrameData frame_data;
        frame_data.frame = frame.clone();
        frame_data.frame_idx = frameIdx;
        frame_data.metadata = metadata;
        
        frame_queue_.push_back(frame_data);
    }
    
    void registerTclCommands(Tcl_Interp* interp) override {
        Tcl_Eval(interp, "namespace eval ::eyetracking {}");

        Tcl_CreateObjCommand(interp, "::eyetracking::setPupilThreshold", 
                            setPupilThresholdCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setROI", 
                            setROICmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::disableROI", 
                            disableROICmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::getResults",
                            getResultsCmd, this, NULL);      
    }
    
    const char* getName() override { return "eye_tracking"; }
    const char* getVersion() override { return "1.0"; }
    const char* getDescription() override { 
        return "Pupil and Purkinje reflection tracking"; 
    }
    
    bool drawOverlay(cv::Mat& frame, int frame_idx) override {
        std::lock_guard<std::mutex> lock(results_mutex_);
        
        if (!latest_results_.valid) return false;
        
        // Draw ROI if enabled
        if (roi_enabled_) {
            cv::rectangle(frame, current_roi_, cv::Scalar(255, 255, 0), 2);
            cv::putText(frame, "ROI", 
                       cv::Point(current_roi_.x + 5, current_roi_.y + 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                       cv::Scalar(255, 255, 0), 1);
        }
        
        // Draw pupil
        if (latest_results_.pupil.detected) {
            cv::circle(frame, latest_results_.pupil.center, 
                      (int)latest_results_.pupil.radius, 
                      cv::Scalar(0, 255, 0), 2);
            cv::drawMarker(frame, latest_results_.pupil.center, 
                          cv::Scalar(0, 255, 0), 
                          cv::MARKER_CROSS, 10, 2);
            cv::putText(frame, "Pupil", 
                       cv::Point(latest_results_.pupil.center.x + 
                                latest_results_.pupil.radius + 5, 
                                latest_results_.pupil.center.y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                       cv::Scalar(0, 255, 0), 1);
        }
        
        // Draw Purkinje reflections
        if (latest_results_.purkinje.p1_detected) {
            cv::circle(frame, latest_results_.purkinje.p1_center, 
                      5, cv::Scalar(255, 0, 0), 2);
            cv::putText(frame, "P1", 
                       cv::Point(latest_results_.purkinje.p1_center.x + 8, 
                                latest_results_.purkinje.p1_center.y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, 
                       cv::Scalar(255, 0, 0), 1);
        }
        
        if (latest_results_.purkinje.p4_detected) {
            cv::circle(frame, latest_results_.purkinje.p4_center, 
                      5, cv::Scalar(0, 0, 255), 2);
            cv::putText(frame, "P4", 
                       cv::Point(latest_results_.purkinje.p4_center.x + 8, 
                                latest_results_.purkinje.p4_center.y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, 
                       cv::Scalar(0, 0, 255), 1);
        }
        
        return true;
    }
};

// Standard Tcl package entry point
extern "C" {
    int Eyetracking_Init(Tcl_Interp *interp) {
        EyeTrackingPlugin* plugin = new EyeTrackingPlugin();
        
        if (!plugin->initialize(interp)) {
            delete plugin;
            return TCL_ERROR;
        }
        
        // Register with the global registry
        g_pluginRegistry.registerPlugin(plugin);
        
        // Register Tcl commands
        plugin->registerTclCommands(interp);
        
        // Declare package
        Tcl_PkgProvide(interp, "Eyetracking", "1.0");
        
        return TCL_OK;
    }
}
