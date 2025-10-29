#pragma once

#include <string>
#include <tcl.h>
#include "opencv2/opencv.hpp"
#include "IFrameSource.h"
#include <sqlite3.h>

// Base interface for all analysis plugins
class IAnalysisPlugin {
public:
    virtual ~IAnalysisPlugin() = default;
    
    // Plugin identification
    virtual const char* getName() = 0;
    virtual const char* getVersion() = 0;
    virtual const char* getDescription() = 0;
    
    // Lifecycle
    virtual bool initialize(Tcl_Interp* interp) = 0;
    virtual void shutdown() = 0;
    
    // Frame processing
    virtual void analyzeFrame(const cv::Mat& frame, int frameIdx, 
                             const FrameMetadata& metadata) = 0;
    
    // Visualization
    virtual bool drawOverlay(cv::Mat& frame, int frame_idx) { return false; }
    
    // TCL command registration
    virtual void registerTclCommands(Tcl_Interp* interp) {}
    
    // ========================================================================
    // STORAGE INTERFACE
    // ========================================================================
    
    /**
     * Legacy JSON serialization (for backward compatibility or simple plugins)
     * Returns JSON string of results for the given frame index.
     * Return "{}" if no data available.
     */
    virtual std::string serializeResults(int frame_idx) { return "{}"; }
    
    /**
     * Does this plugin use structured storage (its own database table)?
     * If true, the plugin will use storeFrameData() instead of serializeResults()
     */
    virtual bool usesStructuredStorage() const { return false; }
    
    /**
     * Get SQL schema for plugin's table(s)
     * Should include CREATE TABLE IF NOT EXISTS and any indexes
     * Can include multiple statements separated by semicolons
     */
    virtual std::string getTableSchema() const { return ""; }
    
    /**
     * Store frame data directly to database
     * Called once per frame during recording
     * @param db Open SQLite database handle
     * @param frame_number Sequential frame number in output video
     * @return true if data was stored successfully
     */
    virtual bool storeFrameData(sqlite3* db, int frame_number, int obs_id) { 
    	return false; 
    }
    
    /**
     * Optional: Called before a batch of frames will be stored
     * Useful for beginning transactions, preparing statements, etc.
     */
    virtual void beginStorageBatch(sqlite3* db) {}
    
    /**
     * Optional: Called after a batch of frames has been stored
     * Useful for committing transactions, finalizing statements, etc.
     */
    virtual void endStorageBatch(sqlite3* db) {}
    
    // ========================================================================
    // WEB UI INTERFACE
    // ========================================================================
    
    /**
     * Does this plugin provide a web UI?
     */
    virtual bool hasWebUI() const { return false; }
    
    /**
     * Get plugin's HTML content (will be injected into a control section)
     * Return empty string if no UI provided
     * 
     * NOTE: Use unique raw string delimiters to avoid conflicts with )" sequences:
     *   R"PLUGINNAME_HTML(...)PLUGINNAME_HTML"
     *   R"PLUGINNAME_JS(...)PLUGINNAME_JS"
     *   R"PLUGINNAME_CSS(...)PLUGINNAME_CSS"
     */
    virtual std::string getUIHTML() const { return ""; }
    
    /**
     * Get plugin's JavaScript code
     * Will be wrapped in an IIFE: (function() { YOUR_CODE })();
     * Has access to global functions: sendCommand(), showInfo(), showError()
     * Return empty string if no script needed
     */
    virtual std::string getUIScript() const { return ""; }
    
    /**
     * Get plugin's CSS styles
     * Return empty string if no custom styling needed
     */
    virtual std::string getUIStyle() const { return ""; }
};
