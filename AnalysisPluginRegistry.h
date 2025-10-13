#ifndef ANALYSIS_PLUGIN_REGISTRY_H
#define ANALYSIS_PLUGIN_REGISTRY_H

#include <string>
#include <vector>
#include <map>
#include <mutex>

#include "IAnalysisPlugin.h"
#include "opencv2/opencv.hpp"

// Simple registry for analysis plugins
// Plugins self-register during their Tcl XX_Init() call
class AnalysisPluginRegistry {
private:
    std::map<std::string, IAnalysisPlugin*> plugins_;
    mutable std::mutex plugins_mutex_;
    
public:
    AnalysisPluginRegistry() = default;
    ~AnalysisPluginRegistry();
    
    // Plugins call this during their XX_Init() function
    void registerPlugin(IAnalysisPlugin* plugin);
    
    // Unregister and cleanup plugin
    void unregisterPlugin(const std::string& plugin_name);
    
    // Process frame through all registered plugins
    void processFrame(const cv::Mat& frame, int frameIdx, 
                     const FrameMetadata& metadata);
    
    // Query functions
    bool hasPlugins() const;
    bool isPluginRegistered(const std::string& plugin_name) const;
    std::vector<std::string> listPlugins() const;
    IAnalysisPlugin* getPlugin(const std::string& plugin_name);
    
    // Shutdown all plugins
    void shutdownAll();
};

#endif // ANALYSIS_PLUGIN_REGISTRY_H
