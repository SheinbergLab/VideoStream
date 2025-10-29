#ifndef STORAGE_MANAGER_H
#define STORAGE_MANAGER_H

#include <string>
#include <vector>
#include <cstdint>
#include "sqlite3.h"

// Forward declarations
namespace cv {
    class Mat;
}

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct RecordingMetadata {
    std::string filename;
    int64_t start_time;      // Unix timestamp (microseconds)
    float frame_rate;
    int width;
    int height;
    bool is_color;
    std::string codec;       // e.g., "XVID"
};

struct FrameData {
    int frame_number;
    int relative_frame_id;   // Offset from recording start frame ID
    int64_t timestamp_us;    // Microseconds from start
    int64_t system_time_us;  // System time microseconds from start
    uint8_t line_status;
};

struct CameraSettings {
    int binning_horizontal;
    int binning_vertical;
    int roi_offset_x;
    int roi_offset_y;
    int roi_width;
    int roi_height;
    float exposure_time;
    float gain;
    float frame_rate;
    std::string pixel_format;
};

struct ObservationRange {
    int obs_id;
    int start_frame;
    int stop_frame;
};

// ============================================================================
// STORAGE MANAGER CLASS
// ============================================================================

/**
 * StorageManager handles SQLite database storage for video recordings.
 * 
 * Design: One database per recording
 * - Each recording gets its own .db file alongside the video file
 * - No recording_id needed - all data in DB belongs to single recording
 * - Plugins manage their own tables through the structured storage interface
 */
class StorageManager {
private:
    sqlite3* db_;
    std::string current_db_path_;
    bool recording_open_;
    bool recording_active_;  // Controls whether frames are written

    bool in_obs_ = false;
    int current_obs_id_ = -1;
    int next_obs_id_ = 0;
      
    // Prepared statements for base tables only
    sqlite3_stmt* stmt_insert_frame_;
    sqlite3_stmt* stmt_insert_obs_;
    
    // Batch transaction management
    int frames_since_commit_;
    static constexpr int BATCH_SIZE = 100;
    
    // Plugin state
    bool plugins_initialized_;
    
    // Helper methods
    bool executeSQL(const char* sql);
    bool createTables();
    bool prepareStatements();
    void finalizeStatements();
    bool beginTransaction();
    bool commitTransaction();
    void checkBatchCommit();
    bool openDatabase(const std::string& db_path, const RecordingMetadata& metadata);
    
public:
    StorageManager();
    ~StorageManager();
    
    // Recording management
    bool openRecording(const std::string& video_filename, 
                      const RecordingMetadata& metadata);
    bool openMetadataOnly(const std::string& base_filename,
                         const std::string& source_video_filename,
                         const RecordingMetadata& metadata);
    bool closeRecording();
    
    bool isRecordingOpen() const { return recording_open_; }
    std::string getCurrentDBPath() const { return current_db_path_; }

    // Recording control (pause/resume frame storage)
    void startRecording();
    void stopRecording();
    bool isRecordingActive() const { return recording_active_; }
    
    void resetObsState();
    void setObsState(bool in_obs);
    
    // Base table data storage
    bool storeFrame(const FrameData& frame);
    bool storeObservationStart(int frame_number);
    bool storeObservationEnd(int frame_number);
    bool storeCameraSettings(const CameraSettings& settings);
    
    // Plugin storage interface
    bool initializePluginStorage();
    bool storeFrameWithPlugins(int frame_number, int buffer_index);
    void beginPluginStorageBatch();
    void endPluginStorageBatch();
    
    // Provide database handle for plugins
    sqlite3* getDatabase() { return db_; }
    
    // Query interface
    bool getRecordingMetadata(RecordingMetadata& metadata);
    std::vector<FrameData> getFrameRange(int start_frame, int end_frame);
    std::vector<ObservationRange> getObservations();
    
    // Error handling
    std::string getLastError() const;
};

#endif // STORAGE_MANAGER_H
