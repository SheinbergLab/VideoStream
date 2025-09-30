#include "AnalysisPluginRegistry.h"
#include <iostream>

AnalysisPluginRegistry::~AnalysisPluginRegistry() {
    shutdownAll();
}

void AnalysisPluginRegistry::registerPlugin(IAnalysisPlugin* plugin) {
    if (!plugin) {
        std::cerr << "Attempted to register null plugin" << std::endl;
        return;
    }
    
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    
    std::string plugin_name = plugin->getName();
    
    // Check if plugin already registered
    if (plugins_.find(plugin_name) != plugins_.end()) {
        std::cerr << "Plugin " << plugin_name << " already registered" << std::endl;
        return;
    }
    
    plugins_[plugin_name] = plugin;
    
    std::cout << "Registered plugin: " << plugin_name 
              << " v" << plugin->getVersion() << std::endl;
    std::cout << "  " << plugin->getDescription() << std::endl;
}

void AnalysisPluginRegistry::unregisterPlugin(const std::string& plugin_name) {
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    
    auto it = plugins_.find(plugin_name);
    if (it == plugins_.end()) {
        std::cerr << "Plugin " << plugin_name << " not registered" << std::endl;
        return;
    }
    
    // Shutdown and delete the plugin
    it->second->shutdown();
    delete it->second;
    
    plugins_.erase(it);
    
    std::cout << "Unregistered plugin: " << plugin_name << std::endl;
}

void AnalysisPluginRegistry::processFrame(const cv::Mat& frame, int frameIdx,
                                          const FrameMetadata& metadata) {
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    
    for (auto& pair : plugins_) {
        try {
            pair.second->analyzeFrame(frame, frameIdx, metadata);
        } catch (const std::exception& e) {
            std::cerr << "Plugin " << pair.first 
                      << " threw exception: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Plugin " << pair.first 
                      << " threw unknown exception" << std::endl;
        }
    }
}

bool AnalysisPluginRegistry::hasPlugins() const {
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    return !plugins_.empty();
}

bool AnalysisPluginRegistry::isPluginRegistered(const std::string& plugin_name) const {
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    return plugins_.find(plugin_name) != plugins_.end();
}

std::vector<std::string> AnalysisPluginRegistry::listPlugins() const {
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    
    std::vector<std::string> plugin_list;
    for (const auto& pair : plugins_) {
        plugin_list.push_back(pair.first);
    }
    return plugin_list;
}

IAnalysisPlugin* AnalysisPluginRegistry::getPlugin(const std::string& plugin_name) {
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    
    auto it = plugins_.find(plugin_name);
    if (it != plugins_.end()) {
        return it->second;
    }
    return nullptr;
}

void* AnalysisPluginRegistry::getPluginDisplayData(const std::string& plugin_name) {
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    
    auto it = plugins_.find(plugin_name);
    if (it != plugins_.end()) {
        return it->second->getDisplayData();
    }
    return nullptr;
}

void AnalysisPluginRegistry::shutdownAll() {
    std::lock_guard<std::mutex> lock(plugins_mutex_);
    
    for (auto& pair : plugins_) {
        pair.second->shutdown();
        delete pair.second;
    }
    
    plugins_.clear();
}
