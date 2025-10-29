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
      recording_open_(false),
      recording_active_(false),        
      stmt_insert_frame_(nullptr),
      stmt_insert_obs_(nullptr),
      plugins_initialized_(false),      
      frames_since_commit_(0) {
}

StorageManager::~StorageManager() {
    if (recording_open_) {
        closeRecording();
    }
}

// ============================================================================
// OBS TRACKING
// ============================================================================


void StorageManager::setObsState(bool in_obs) {
  if (in_obs && !in_obs_) {
    // Entering obs period
    current_obs_id_ = next_obs_id_++;
  } else if (!in_obs && in_obs_) {
    // Leaving obs period
    current_obs_id_ = -1;
  }
  in_obs_ = in_obs;
}

void StorageManager::resetObsState() {
    in_obs_ = false;
    current_obs_id_ = -1;
    next_obs_id_ = 0;
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
    // Simplified schema: one database = one recording
    // No recording_id needed, no foreign keys
    const char* sql = R"(
        CREATE TABLE IF NOT EXISTS recording_metadata (
            filename TEXT NOT NULL,
            start_time INTEGER,
            frame_rate REAL,
            width INTEGER,
            height INTEGER,
            is_color INTEGER,
            codec TEXT
        );

        CREATE TABLE IF NOT EXISTS camera_settings (
            binning_horizontal INTEGER DEFAULT 1,
            binning_vertical INTEGER DEFAULT 1,
            roi_offset_x INTEGER DEFAULT 0,
            roi_offset_y INTEGER DEFAULT 0,
            roi_width INTEGER,
            roi_height INTEGER,
            exposure_time REAL,
            gain REAL,
            frame_rate REAL,
            pixel_format TEXT
        );

        CREATE TABLE IF NOT EXISTS frames (
            frame_number INTEGER PRIMARY KEY,
            obs_id INTEGER,
            relative_frame_id INTEGER,
            timestamp_us INTEGER,
            system_time_us INTEGER,
            line_status INTEGER
        );
        CREATE INDEX IF NOT EXISTS idx_frames_obs ON frames(obs_id);

        CREATE TABLE IF NOT EXISTS observations (
            obs_id INTEGER PRIMARY KEY AUTOINCREMENT,
            start_frame INTEGER NOT NULL,
            stop_frame INTEGER
        );
        
        CREATE INDEX IF NOT EXISTS idx_frames_number ON frames(frame_number);
        CREATE INDEX IF NOT EXISTS idx_obs_frames ON observations(start_frame, stop_frame);
    )";
    
    return executeSQL(sql);
}

bool StorageManager::prepareStatements() {
    // Frame insert statement
    const char* sql_frame = 
        "INSERT INTO frames (frame_number, obs_id, relative_frame_id, "
        "timestamp_us, system_time_us, line_status) "
        "VALUES (?, ?, ?, ?, ?, ?)";
    
    if (sqlite3_prepare_v2(db_, sql_frame, -1, &stmt_insert_frame_, nullptr) != SQLITE_OK) {
        std::cerr << "Failed to prepare frame insert statement: " 
                  << sqlite3_errmsg(db_) << std::endl;
        return false;
    }
    
    // Observation insert statement
    const char* sql_obs = 
        "INSERT INTO observations (start_frame, stop_frame) "
        "VALUES (?, ?)";
    
    if (sqlite3_prepare_v2(db_, sql_obs, -1, &stmt_insert_obs_, nullptr) != SQLITE_OK) {
        std::cerr << "Failed to prepare observation insert statement: "
                  << sqlite3_errmsg(db_) << std::endl;
        sqlite3_finalize(stmt_insert_frame_);
        stmt_insert_frame_ = nullptr;
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
        
        fireEvent(VstreamEvent("vstream/storage_open_failed",
                               "file " + db_path + " error " + sqlite3_errmsg(db_)));
        return false;
    }
    
    // Enable WAL mode for better concurrency
    executeSQL("PRAGMA journal_mode=WAL");
    
    // Create base tables
    if (!createTables()) {
        sqlite3_close(db_);
        db_ = nullptr;
        fireEvent(VstreamEvent("vstream/storage_open_failed",
                               "file " + db_path + " error table_creation_failed"));
        return false;
    }
    
    // Prepare base statements
    if (!prepareStatements()) {
        sqlite3_close(db_);
        db_ = nullptr;
        fireEvent(VstreamEvent("vstream/storage_open_failed",
                               "file " + db_path + " error statement_preparation_failed"));
        return false;
    }
    
    // Store recording metadata (single row)
    std::ostringstream sql;
    sql << "INSERT INTO recording_metadata "
        << "(filename, start_time, frame_rate, width, height, is_color, codec) "
        << "VALUES ("
        << "'" << metadata.filename << "', "
        << metadata.start_time << ", "
        << metadata.frame_rate << ", "
        << metadata.width << ", "
        << metadata.height << ", "
        << (metadata.is_color ? 1 : 0) << ", "
        << "'" << metadata.codec << "')";
    
    if (!executeSQL(sql.str().c_str())) {
        std::cerr << "Failed to store recording metadata" << std::endl;
        finalizeStatements();
        sqlite3_close(db_);
        db_ = nullptr;
        fireEvent(VstreamEvent("vstream/storage_open_failed",
                               "file " + db_path + " error metadata_insert_failed"));
        return false;
    }
        
    resetObsState();  
    recording_open_ = true;
    
    std::cout << "Opened recording database: " << db_path << std::endl;
    
    return true;
}

bool StorageManager::closeRecording() {
    if (!recording_open_) {
        return false;
    }
    
    // Commit any pending transaction
    commitTransaction();
    
    // Finalize prepared statements
    finalizeStatements();
    
    // Close database
    if (db_) {
        int rc = sqlite3_close(db_);
        if (rc != SQLITE_OK) {
            std::cerr << "Warning: sqlite3_close returned error: " 
                      << sqlite3_errmsg(db_) << " (code " << rc << ")" << std::endl;
            // Force close even with errors
            sqlite3_close_v2(db_);
        }
        db_ = nullptr;
    }
    
    recording_open_ = false;
    recording_active_ = false;
    plugins_initialized_ = false;
    current_db_path_.clear();
    
    std::cout << "Closed recording database" << std::endl;
    
    return true;
}

void StorageManager::startRecording() {
    recording_active_ = true;
}

void StorageManager::stopRecording() {
    recording_active_ = false;
}

// ============================================================================
// DATA STORAGE
// ============================================================================

bool StorageManager::storeFrame(const FrameData& frame) {
    if (!recording_open_ || !stmt_insert_frame_) {
        return false;
    }
    
    if (!recording_active_) {
        return true;  // Not an error, just not recording
    }
    
    sqlite3_reset(stmt_insert_frame_);

    sqlite3_bind_int(stmt_insert_frame_, 1, frame.frame_number);
    
    if (frame.obs_id >= 0) {
        sqlite3_bind_int(stmt_insert_frame_, 2, frame.obs_id);
    } else {
        sqlite3_bind_null(stmt_insert_frame_, 2);
    }
    
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
    if (!recording_open_ || !stmt_insert_obs_) {
        return false;
    }
    
    sqlite3_reset(stmt_insert_obs_);
    
    // Insert observation with start frame, NULL stop frame
    sqlite3_bind_int(stmt_insert_obs_, 1, frame_number);
    sqlite3_bind_null(stmt_insert_obs_, 2);  // stop_frame is NULL initially
    
    int rc = sqlite3_step(stmt_insert_obs_);
    if (rc != SQLITE_DONE) {
        std::cerr << "Failed to insert observation start: " << sqlite3_errmsg(db_) << std::endl;
        return false;
    }
    
    return true;
}

bool StorageManager::storeObservationEnd(int frame_number) {
    if (!recording_open_) {
        return false;
    }
    
    // Update the most recent observation with NULL stop_frame
    std::ostringstream sql;
    sql << "UPDATE observations SET stop_frame = " << frame_number
        << " WHERE obs_id = ("
        << "SELECT obs_id FROM observations "
        << "WHERE stop_frame IS NULL "
        << "ORDER BY obs_id DESC LIMIT 1)";
    
    return executeSQL(sql.str().c_str());
}

bool StorageManager::storeCameraSettings(const CameraSettings& settings) {
    if (!recording_open_) {
        return false;
    }
    
    // Single row table - delete any existing and insert new
    executeSQL("DELETE FROM camera_settings");
    
    std::ostringstream sql;
    sql << "INSERT INTO camera_settings ("
        << "binning_horizontal, binning_vertical, "
        << "roi_offset_x, roi_offset_y, roi_width, roi_height, "
        << "exposure_time, gain, frame_rate, pixel_format) VALUES ("
        << settings.binning_horizontal << ", "
        << settings.binning_vertical << ", "
        << settings.roi_offset_x << ", "
        << settings.roi_offset_y << ", "
        << settings.roi_width << ", "
        << settings.roi_height << ", "
        << settings.exposure_time << ", "
        << settings.gain << ", "
        << settings.frame_rate << ", "       
        << "'" << settings.pixel_format << "')";
    
    return executeSQL(sql.str().c_str());
}

// ============================================================================
// QUERY INTERFACE
// ============================================================================

bool StorageManager::getRecordingMetadata(RecordingMetadata& metadata) {
    if (!recording_open_) {
        return false;
    }
    
    const char* sql = "SELECT filename, start_time, frame_rate, width, height, is_color, codec "
                     "FROM recording_metadata LIMIT 1";
    
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
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
        << "FROM frames "
        << "WHERE frame_number >= " << start_frame
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
    
    const char* sql = "SELECT obs_id, start_frame, stop_frame FROM observations "
                     "ORDER BY start_frame";
    
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
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
        std::cerr << "initializePluginStorage: Database not open" << std::endl;
        return false;
    }
    
    if (plugins_initialized_) {
        std::cout << "Plugins already initialized for this recording" << std::endl;
        return true;
    }
    
    std::cout << "=== Initializing Plugin Storage ===" << std::endl;
    
    auto plugin_names = g_pluginRegistry.listPlugins();
    std::cout << "Found " << plugin_names.size() << " registered plugins" << std::endl;
    
    for (const auto& name : plugin_names) {
        auto* plugin = g_pluginRegistry.getPlugin(name);
        if (!plugin) {
            std::cerr << "WARNING: Plugin '" << name << "' returned null pointer" << std::endl;
            continue;
        }
        
        if (plugin->usesStructuredStorage()) {
            std::cout << "  Initializing plugin: " << name << std::endl;
            
            std::string schema = plugin->getTableSchema();
            
            if (schema.empty()) {
                std::cerr << "  ERROR: Plugin " << name 
                          << " claims to use structured storage but has no schema" 
                          << std::endl;
                continue;
            }
            
            // Execute the schema SQL to create tables
            char* err_msg = nullptr;
            int rc = sqlite3_exec(db_, schema.c_str(), nullptr, nullptr, &err_msg);
            
            if (rc != SQLITE_OK) {
                std::cerr << "  ERROR: Failed to create tables for plugin " << name 
                          << ": " << (err_msg ? err_msg : "unknown error") << std::endl;
                if (err_msg) sqlite3_free(err_msg);
                return false;
            }
            
            std::cout << "  ✓ Created tables for plugin: " << name << std::endl;
        } else {
            std::cout << "  Plugin " << name << " does not use structured storage" << std::endl;
        }
    }
    
    plugins_initialized_ = true;
    std::cout << "=== Plugin Storage Initialized ===" << std::endl;
    
    return true;
}

bool StorageManager::storeFrameWithPlugins(int frame_number, int buffer_index) {
    if (!recording_open_) {
        return false;
    }
    
    if (!recording_active_) {
        return true;  // Not an error, just not recording
    }
    
    auto plugin_names = g_pluginRegistry.listPlugins();
    for (const auto& name : plugin_names) {
        auto* plugin = g_pluginRegistry.getPlugin(name);
        if (!plugin) continue;
        
        if (plugin->usesStructuredStorage()) {
            // Plugin manages its own table inserts
            plugin->storeFrameData(db_, frame_number, current_obs_id_);
            // Note: Don't treat as error if plugin has no data for this frame
        }
        // Note: Removed fallback JSON serialization - plugins should use structured storage
    }
    
    return true;
}

void StorageManager::beginPluginStorageBatch() {
    if (!db_) return;
    
    // Start transaction for batch inserts
    beginTransaction();
    
    // Notify plugins that batching is starting
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
    
    // Notify plugins that batching is ending
    auto plugin_names = g_pluginRegistry.listPlugins();
    for (const auto& name : plugin_names) {
        auto* plugin = g_pluginRegistry.getPlugin(name);
        if (plugin && plugin->usesStructuredStorage()) {
            plugin->endStorageBatch(db_);
        }
    }
    
    // Commit transaction
    commitTransaction();
}
