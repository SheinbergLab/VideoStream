#ifndef IANALYSIS_PLUGIN_H
#define IANALYSIS_PLUGIN_H

#include <tcl.h>
#include "opencv2/opencv.hpp"
#include "IFrameSource.h"

// Base interface for all analysis plugins
class IAnalysisPlugin {
public:
    virtual ~IAnalysisPlugin() = default;
    
    // Plugin lifecycle
    virtual bool initialize(Tcl_Interp* interp) = 0;
    virtual void shutdown() = 0;
    
    // Analysis hook - called for each frame
    // Plugins should be non-blocking or manage their own threading
    virtual void analyzeFrame(const cv::Mat& frame, int frameIdx, 
                             const FrameMetadata& metadata) = 0;
    
    // Tcl command registration for plugin-specific commands
    virtual void registerTclCommands(Tcl_Interp* interp) = 0;
    
    // Plugin identification
    virtual const char* getName() = 0;
    virtual const char* getVersion() = 0;
    virtual const char* getDescription() = 0;
    
    // Optional: plugin draws its own overlay on the frame
    // Return true if drawing was performed, false if no visualization
    virtual bool drawOverlay(cv::Mat& frame, int frame_idx) { 
        return false; 
    }
    
    // Optional: plugin can provide results for display overlay
    // Returns nullptr if no visualization data available
    virtual void* getDisplayData() { return nullptr; }
};

#endif // IANALYSIS_PLUGIN_H
