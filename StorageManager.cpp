#include <iostream>
#include <sstream>
#include <chrono>
#include <cstring>

#include "StorageManager.h"
#include "AnalysisPluginRegistry.h"
#include "VstreamEvent.h"
#ifdef USE_FLIR
#include "FlirCameraSource.h"
#endif

extern AnalysisPluginRegistry g_pluginRegistry;

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

StorageManager::StorageManager() 
    : db_(nullptr),
      current_recording_id_(-1),
      recording_open_(false),
      recording_active_(false),        
      stmt_insert_frame_(nullptr),
      stmt_insert_obs_(nullptr),
      stmt_insert_plugin_(nullptr),
      plugins_initialized_(false),      
      frames_since_commit_(0) {
}

StorageManager::~StorageManager() {
    if (recording_open_) {
        closeRecording();
    }
}

// ============================================================================
// SQL EXECUTION HELPERS
// ============================================================================

bool StorageManager::executeSQL(const char* sql) {
    char* err_msg = nullptr;
    int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &err_msg);
    
    if (rc != SQLITE_OK) {
        std::cerr << "SQL error: " << (err_msg ? err_msg : "unknown") << std::endl;
        std::cerr << "SQL: " << sql << std::endl;
        if (err_msg) sqlite3_free(err_msg);
        return false;
    }
    
    return true;
}

bool StorageManager::createTables() {
    const char* sql = R"(
        CREATE TABLE IF NOT EXISTS recordings (
            recording_id INTEGER PRIMARY KEY AUTOINCREMENT,
            filename TEXT NOT NULL,
            start_time INTEGER,
            frame_rate REAL,
            width INTEGER,
            height INTEGER,
            is_color INTEGER,
            codec TEXT
        );

        CREATE TABLE IF NOT EXISTS camera_settings (
            recording_id INTEGER PRIMARY KEY,
            binning_horizontal INTEGER DEFAULT 1,
            binning_vertical INTEGER DEFAULT 1,
            roi_offset_x INTEGER DEFAULT 0,
            roi_offset_y INTEGER DEFAULT 0,
            roi_width INTEGER,
            roi_height INTEGER,
            exposure_time REAL,
            gain REAL,
            pixel_format TEXT,
            FOREIGN KEY (recording_id) REFERENCES recordings(recording_id)
        );

        CREATE TABLE IF NOT EXISTS frames (
            frame_id INTEGER PRIMARY KEY AUTOINCREMENT,
            recording_id INTEGER NOT NULL,
            frame_number INTEGER NOT NULL,
            relative_frame_id INTEGER,
            timestamp_us INTEGER,
            system_time_us INTEGER,
            line_status INTEGER,
            FOREIGN KEY (recording_id) REFERENCES recordings(recording_id)
        );
        
        CREATE TABLE IF NOT EXISTS observations (
            obs_id INTEGER PRIMARY KEY AUTOINCREMENT,
            recording_id INTEGER NOT NULL,
            start_frame INTEGER,
            stop_frame INTEGER,
            FOREIGN KEY (recording_id) REFERENCES recordings(recording_id)
        );
        
        CREATE TABLE IF NOT EXISTS plugin_data (
            data_id INTEGER PRIMARY KEY AUTOINCREMENT,
            recording_id INTEGER NOT NULL,
            frame_number INTEGER NOT NULL,
            plugin_name TEXT NOT NULL,
            data_json TEXT,
            FOREIGN KEY (recording_id) REFERENCES recordings(recording_id)
        );
        
        CREATE INDEX IF NOT EXISTS idx_frames_recording 
            ON frames(recording_id, frame_number);
        CREATE INDEX IF NOT EXISTS idx_plugin_data_recording 
            ON plugin_data(recording_id, frame_number);
        CREATE INDEX IF NOT EXISTS idx_plugin_data_plugin 
            ON plugin_data(plugin_name, recording_id);
    )";
    
    return executeSQL(sql);
}

bool StorageManager::prepareStatements() {
    const char* sql_frame = 
        "INSERT INTO frames (recording_id, frame_number, relative_frame_id, "
        "timestamp_us, system_time_us, line_status) "
        "VALUES (?, ?, ?, ?, ?, ?)";
    
    if (sqlite3_prepare_v2(db_, sql_frame, -1, &stmt_insert_frame_, nullptr) != SQLITE_OK) {
        std::cerr << "Failed to prepare frame insert statement" << std::endl;
        return false;
    }
    
    const char* sql_obs = 
        "INSERT INTO observations (recording_id, start_frame, stop_frame) "
        "VALUES (?, ?, ?)";
    
    if (sqlite3_prepare_v2(db_, sql_obs, -1, &stmt_insert_obs_, nullptr) != SQLITE_OK) {
        std::cerr << "Failed to prepare observation insert statement" << std::endl;
        return false;
    }
    
    const char* sql_plugin = 
        "INSERT INTO plugin_data (recording_id, frame_number, plugin_name, data_json) "
        "VALUES (?, ?, ?, ?)";
    
    if (sqlite3_prepare_v2(db_, sql_plugin, -1, &stmt_insert_plugin_, nullptr) != SQLITE_OK) {
        std::cerr << "Failed to prepare plugin insert statement" << std::endl;
        return false;
    }
    
    return true;
}

void StorageManager::finalizeStatements() {
    if (stmt_insert_frame_) {
        sqlite3_finalize(stmt_insert_frame_);
        stmt_insert_frame_ = nullptr;
    }
    if (stmt_insert_obs_) {
        sqlite3_finalize(stmt_insert_obs_);
        stmt_insert_obs_ = nullptr;
    }
    if (stmt_insert_plugin_) {
        sqlite3_finalize(stmt_insert_plugin_);
        stmt_insert_plugin_ = nullptr;
    }
}

bool StorageManager::beginTransaction() {
    return executeSQL("BEGIN TRANSACTION");
}

bool StorageManager::commitTransaction() {
    frames_since_commit_ = 0;
    return executeSQL("COMMIT");
}

void StorageManager::checkBatchCommit() {
    if (++frames_since_commit_ >= BATCH_SIZE) {
        commitTransaction();
        beginTransaction();
    }
}

// ============================================================================
// RECORDING MANAGEMENT
// ============================================================================

bool StorageManager::openRecording(const std::string& video_filename, 
                                   const RecordingMetadata& metadata) {
    if (recording_open_) {
        std::cerr << "Recording already open. Close current recording first." << std::endl;
        return false;
    }
    
    // Create database filename: video.avi -> video.db
    std::string db_path = video_filename;
    size_t dot_pos = db_path.find_last_of('.');
    if (dot_pos != std::string::npos) {
        db_path = db_path.substr(0, dot_pos);
    }
    db_path += ".db";
    
    return openDatabase(db_path, metadata);
}

bool StorageManager::openMetadataOnly(const std::string& base_filename,
                                      const std::string& source_video_filename,
                                      const RecordingMetadata& metadata) {
    if (recording_open_) {
        std::cerr << "Recording already open. Close current recording first." << std::endl;
        return false;
    }
    
    // Create unique database filename with source reference
    // e.g., video_reprocess_001.db, video_openiris_001.db
    std::string db_path = base_filename + ".db";
    
    // Create modified metadata that references source video
    RecordingMetadata meta_copy = metadata;
    meta_copy.filename = source_video_filename;  // Store reference to original video
    
    return openDatabase(db_path, meta_copy);
}

bool StorageManager::openDatabase(const std::string& db_path, 
                                  const RecordingMetadata& metadata) {
    current_db_path_ = db_path;
    
    // Open database
    int rc = sqlite3_open(db_path.c_str(), &db_);
    if (rc != SQLITE_OK) {
        std::cerr << "Cannot open database: " << sqlite3_errmsg(db_) << std::endl;
        sqlite3_close(db_);
        db_ = nullptr;
        
        // Fire error event
        fireEvent(VstreamEvent("vstream/storage_open_failed",
			"file " + db_path + " error " + sqlite3_errmsg(db_)));
        return false;
    }
    
    // Enable WAL mode for better concurrency
    executeSQL("PRAGMA journal_mode=WAL");
    
    // Create tables
    if (!createTables()) {
        sqlite3_close(db_);
        db_ = nullptr;
        fireEvent(VstreamEvent("vstream/storage_open_failed",
			"file " + db_path + " error table_creation_failed"));
        return false;
    }
    
    // Prepare statements
    if (!prepareStatements()) {
        sqlite3_close(db_);
        db_ = nullptr;
        fireEvent(VstreamEvent("vstream/storage_open_failed",
			"file " + db_path + " error statement_preparation_failed"));
        return false;
    }
    
    // Insert recording metadata
    std::ostringstream sql;
    sql << "INSERT INTO recordings (filename, start_time, frame_rate, width, height, "
        << "is_color, codec) VALUES ('"
        << metadata.filename << "', "
        << metadata.start_time << ", "
        << metadata.frame_rate << ", "
        << metadata.width << ", "
        << metadata.height << ", "
        << (metadata.is_color ? 1 : 0) << ", '"
        << metadata.codec << "')";
    
    if (!executeSQL(sql.str().c_str())) {
        finalizeStatements();
        sqlite3_close(db_);
        db_ = nullptr;
        fireEvent(VstreamEvent("vstream/storage_open_failed",
			"file " + db_path + " error metadata_insert_failed"));
        return false;
    }
    
    current_recording_id_ = sqlite3_last_insert_rowid(db_);

// Store camera settings if using FLIR
#ifdef USE_FLIR
extern IFrameSource* g_frameSource;
FlirCameraSource* flirSource = dynamic_cast<FlirCameraSource*>(g_frameSource);
if (flirSource) {
    CameraSettings cam_settings;
    cam_settings.binning_horizontal = flirSource->getBinningH();
    cam_settings.binning_vertical = flirSource->getBinningV();
    cam_settings.roi_offset_x = flirSource->getOffsetX();
    cam_settings.roi_offset_y = flirSource->getOffsetY();
    cam_settings.roi_width = flirSource->getWidth();
    cam_settings.roi_height = flirSource->getHeight();
    cam_settings.exposure_time = flirSource->getExposureTime();
    cam_settings.gain = flirSource->getGain();
    cam_settings.pixel_format = "Mono8"; // or get from camera
    
    storeCameraSettings(cam_settings);
}
#endif
    
    recording_open_ = true;
    recording_active_ = false;  // Open but not recording yet
    
    // Start first transaction
    beginTransaction();
    
    std::cout << "Opened recording database: " << db_path << std::endl;
    std::cout << "Recording ID: " << current_recording_id_ << std::endl;
    
    // Fire success event

    std::string data = "file " + db_path + " id " +
      std::to_string(current_recording_id_);
    fireEvent(VstreamEvent("vstream/storage_opened", data));
    
    return true;
}

bool StorageManager::closeRecording() {
    if (!recording_open_) {
        return false;
    }
    
    // Commit any pending frames
    if (frames_since_commit_ > 0) {
        commitTransaction();
    }
    
    finalizeStatements();
    
    if (db_) {
      // Checkpoint the WAL and truncates to zero bytes
      sqlite3_wal_checkpoint_v2(db_, nullptr,
				SQLITE_CHECKPOINT_TRUNCATE, nullptr, nullptr);
      
      sqlite3_close(db_);
      db_ = nullptr;
    }
    
    recording_open_ = false;
    current_recording_id_ = -1;
    recording_active_ = false;
    
    // Fire close event
    fireEvent(VstreamEvent("vstream/storage_closed",
		    "file " + current_db_path_));
    
    return true;
}

void StorageManager::startRecording() {
    if (recording_open_ && !recording_active_) {
        recording_active_ = true;
        std::cout << "Recording ACTIVE" << std::endl;
        
        fireEvent(VstreamEvent("vstream/storage_recording_started",
			"file " + current_db_path_));
    }
}

void StorageManager::stopRecording() {
    if (recording_active_) {
        recording_active_ = false;
        
        // Commit any pending frames immediately
        if (frames_since_commit_ > 0) {
            commitTransaction();
            beginTransaction();
        }
        
        std::cout << "Recording STOPPED" << std::endl;
        
        fireEvent(VstreamEvent("vstream/storage_recording_stopped",
			"file " + current_db_path_));
    }
}


// ============================================================================
// DATA STORAGE
// ============================================================================

bool StorageManager::storeCameraSettings(const CameraSettings& settings) {
    if (!db_ || current_recording_id_ < 0) {
        return false;
    }
    
    std::ostringstream sql;
    sql << "INSERT INTO camera_settings (recording_id, binning_horizontal, "
        << "binning_vertical, roi_offset_x, roi_offset_y, roi_width, roi_height, "
        << "exposure_time, gain, pixel_format) VALUES ("
        << current_recording_id_ << ", "
        << settings.binning_horizontal << ", "
        << settings.binning_vertical << ", "
        << settings.roi_offset_x << ", "
        << settings.roi_offset_y << ", "
        << settings.roi_width << ", "
        << settings.roi_height << ", "
        << settings.exposure_time << ", "
        << settings.gain << ", '"
        << settings.pixel_format << "')";
    
    return executeSQL(sql.str().c_str());
}

bool StorageManager::storeFrame(const FrameData& frame) {
    if (!recording_open_ || !stmt_insert_frame_) {
        return false;
    }
    
    sqlite3_reset(stmt_insert_frame_);
    
    sqlite3_bind_int64(stmt_insert_frame_, 1, current_recording_id_);
    sqlite3_bind_int(stmt_insert_frame_, 2, frame.frame_number);
    sqlite3_bind_int(stmt_insert_frame_, 3, frame.relative_frame_id);
    sqlite3_bind_int64(stmt_insert_frame_, 4, frame.timestamp_us);
    sqlite3_bind_int64(stmt_insert_frame_, 5, frame.system_time_us);
    sqlite3_bind_int(stmt_insert_frame_, 6, frame.line_status);
    
    int rc = sqlite3_step(stmt_insert_frame_);
    if (rc != SQLITE_DONE) {
        std::cerr << "Failed to insert frame: " << sqlite3_errmsg(db_) << std::endl;
        return false;
    }
    
    checkBatchCommit();
    
    return true;
}

bool StorageManager::storeObservationStart(int frame_number) {
    if (!recording_open_) {
        return false;
    }
    
    // Insert with only start_frame, stop_frame will be updated later
    std::ostringstream sql;
    sql << "INSERT INTO observations (recording_id, start_frame, stop_frame) VALUES ("
        << current_recording_id_ << ", " << frame_number << ", NULL)";
    
    return executeSQL(sql.str().c_str());
}

bool StorageManager::storeObservationEnd(int frame_number) {
    if (!recording_open_) {
        return false;
    }
    
    // Update the most recent observation with NULL stop_frame
    std::ostringstream sql;
    sql << "UPDATE observations SET stop_frame = " << frame_number
        << " WHERE recording_id = " << current_recording_id_
        << " AND stop_frame IS NULL "
        << " ORDER BY obs_id DESC LIMIT 1";
    
    return executeSQL(sql.str().c_str());
}

bool StorageManager::storePluginData(int frame_number, 
                                     const std::string& plugin_name,
                                     const std::string& json_data) {
    if (!recording_open_ || !stmt_insert_plugin_) {
        return false;
    }
    
    sqlite3_reset(stmt_insert_plugin_);
    
    sqlite3_bind_int64(stmt_insert_plugin_, 1, current_recording_id_);
    sqlite3_bind_int(stmt_insert_plugin_, 2, frame_number);
    sqlite3_bind_text(stmt_insert_plugin_, 3, plugin_name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt_insert_plugin_, 4, json_data.c_str(), -1, SQLITE_TRANSIENT);
    
    int rc = sqlite3_step(stmt_insert_plugin_);
    if (rc != SQLITE_DONE) {
        std::cerr << "Failed to insert plugin data: " << sqlite3_errmsg(db_) << std::endl;
        return false;
    }
    
    return true;
}

// ============================================================================
// QUERY INTERFACE
// ============================================================================

bool StorageManager::getRecordingMetadata(RecordingMetadata& metadata) {
    if (!recording_open_) {
        return false;
    }
    
    std::ostringstream sql;
    sql << "SELECT filename, start_time, frame_rate, width, height, is_color, codec "
        << "FROM recordings WHERE recording_id = " << current_recording_id_;
    
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql.str().c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        return false;
    }
    
    bool success = false;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        metadata.filename = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
        metadata.start_time = sqlite3_column_int64(stmt, 1);
        metadata.frame_rate = sqlite3_column_double(stmt, 2);
        metadata.width = sqlite3_column_int(stmt, 3);
        metadata.height = sqlite3_column_int(stmt, 4);
        metadata.is_color = sqlite3_column_int(stmt, 5) != 0;
        metadata.codec = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6));
        success = true;
    }
    
    sqlite3_finalize(stmt);
    return success;
}

std::vector<FrameData> StorageManager::getFrameRange(int start_frame, int end_frame) {
    std::vector<FrameData> frames;
    
    if (!recording_open_) {
        return frames;
    }
    
    std::ostringstream sql;
    sql << "SELECT frame_number, relative_frame_id, timestamp_us, system_time_us, line_status "
        << "FROM frames WHERE recording_id = " << current_recording_id_
        << " AND frame_number >= " << start_frame
        << " AND frame_number <= " << end_frame
        << " ORDER BY frame_number";
    
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql.str().c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        return frames;
    }
    
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        FrameData frame;
        frame.frame_number = sqlite3_column_int(stmt, 0);
        frame.relative_frame_id = sqlite3_column_int(stmt, 1);
        frame.timestamp_us = sqlite3_column_int64(stmt, 2);
        frame.system_time_us = sqlite3_column_int64(stmt, 3);
        frame.line_status = sqlite3_column_int(stmt, 4);
        frames.push_back(frame);
    }
    
    sqlite3_finalize(stmt);
    return frames;
}

std::vector<ObservationRange> StorageManager::getObservations() {
    std::vector<ObservationRange> observations;
    
    if (!recording_open_) {
        return observations;
    }
    
    std::ostringstream sql;
    sql << "SELECT obs_id, start_frame, stop_frame FROM observations "
        << "WHERE recording_id = " << current_recording_id_
        << " ORDER BY start_frame";
    
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql.str().c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        return observations;
    }
    
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        ObservationRange obs;
        obs.obs_id = sqlite3_column_int(stmt, 0);
        obs.start_frame = sqlite3_column_int(stmt, 1);
        obs.stop_frame = sqlite3_column_int(stmt, 2);
        observations.push_back(obs);
    }
    
    sqlite3_finalize(stmt);
    return observations;
}

// ============================================================================
// ERROR HANDLING
// ============================================================================

std::string StorageManager::getLastError() const {
    if (db_) {
        return std::string(sqlite3_errmsg(db_));
    }
    return "No database connection";
}


// ============================================================================
// PLUGIN STORAGE METHODS
// ============================================================================

bool StorageManager::initializePluginStorage() {
    if (!db_) {
        std::cerr << "Database not open" << std::endl;
        return false;
    }
    
    if (plugins_initialized_) {
        return true; // Already initialized
    }
    
    auto plugin_names = g_pluginRegistry.listPlugins();
    for (const auto& name : plugin_names) {
        auto* plugin = g_pluginRegistry.getPlugin(name);
        if (plugin && plugin->usesStructuredStorage()) {
            std::string schema = plugin->getTableSchema();
            
            if (schema.empty()) {
                std::cerr << "Plugin " << name 
                          << " claims to use structured storage but has no schema" 
                          << std::endl;
                continue;
            }
            
            char* err_msg = nullptr;
            int rc = sqlite3_exec(db_, schema.c_str(), nullptr, nullptr, &err_msg);
            
            if (rc != SQLITE_OK) {
                std::cerr << "Failed to create tables for plugin " << name 
                          << ": " << err_msg << std::endl;
                sqlite3_free(err_msg);
                return false;
            }
            
            std::cout << "Initialized storage for plugin: " << name << std::endl;
        }
    }
    
    plugins_initialized_ = true;
    return true;
}

bool StorageManager::storeFrameWithPlugins(int frame_number, int buffer_index) {
    if (!recording_open_) {
        return false;
    }
    
    auto plugin_names = g_pluginRegistry.listPlugins();
    for (const auto& name : plugin_names) {
        auto* plugin = g_pluginRegistry.getPlugin(name);
        if (!plugin) continue;
        
        if (plugin->usesStructuredStorage()) {
            // Use structured storage
            plugin->storeFrameData(db_, frame_number);
            // Note: Don't treat as error if plugin has no data for this frame
        } else {
            // Fallback to JSON serialization
            std::string json_data = plugin->serializeResults(buffer_index);
            if (json_data != "{}") {
                storePluginData(frame_number, name, json_data);
            }
        }
    }
    
    return true;
}

void StorageManager::beginPluginStorageBatch() {
    if (!db_) return;
    
    auto plugin_names = g_pluginRegistry.listPlugins();
    for (const auto& name : plugin_names) {
        auto* plugin = g_pluginRegistry.getPlugin(name);
        if (plugin && plugin->usesStructuredStorage()) {
            plugin->beginStorageBatch(db_);
        }
    }
}

void StorageManager::endPluginStorageBatch() {
    if (!db_) return;
    
    auto plugin_names = g_pluginRegistry.listPlugins();
    for (const auto& name : plugin_names) {
        auto* plugin = g_pluginRegistry.getPlugin(name);
        if (plugin && plugin->usesStructuredStorage()) {
            plugin->endStorageBatch(db_);
        }
    }
}
