#include <thread>
#include "VideoFileSource.h"
#include "VstreamEvent.h"
#include "VstreamVars.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <cstdint>
#include <dynio.h>

static std::string humanSize(std::uintmax_t bytes) {
    char buf[32];
    if (bytes < 1024) snprintf(buf, sizeof buf, "%ju bytes", bytes);
    else if (bytes < 1024 * 1024) snprintf(buf, sizeof buf, "%.1f KB", bytes / 1024.0);
    else snprintf(buf, sizeof buf, "%.1f MB", bytes / (1024.0 * 1024.0));
    return buf;
}

// Why OpenCV could not open (or read a frame from) a video file, for the
// error shown in the viewer. An MP4 is only playable once its index (the
// moov atom) is written when the recording closes, so a recording that was
// never closed has frame data but no index, or nothing at all.
static std::string describeUnplayable(const std::string& path) {
    namespace fs = std::filesystem;
    std::error_code ec;
    std::string name = fs::path(path).filename().string();
    if (!fs::exists(path, ec)) return name + ": file not found";
    std::uintmax_t size = fs::file_size(path, ec);
    if (ec) return name + ": cannot read file";
    if (size == 0) return name + " is empty (0 bytes)";

    std::ifstream f(path, std::ios::binary);
    if (!f) return name + ": cannot read file (permission denied?)";
    bool isMp4 = false, hasMoov = false;
    std::uintmax_t pos = 0;
    while (pos + 8 <= size) {
        unsigned char h[8];
        f.seekg((std::streamoff) pos);
        if (!f.read((char*) h, 8)) break;
        std::uintmax_t atom = ((std::uintmax_t) h[0] << 24) | (h[1] << 16) | (h[2] << 8) | h[3];
        std::string type((const char*) h + 4, 4);
        if (pos == 0) isMp4 = (type == "ftyp");
        if (!isMp4) break;
        if (type == "moov") hasMoov = true;
        if (atom == 1) {                       // 64-bit size follows
            unsigned char x[8];
            if (!f.read((char*) x, 8)) break;
            atom = 0;
            for (unsigned char b : x) atom = (atom << 8) | b;
        }
        if (atom < 8) break;                   // 0 = runs to end of file
        pos += atom;
    }
    if (isMp4 && !hasMoov) {
        if (size < 4096)
            return name + " has no video frames (" + humanSize(size) +
                   "): the recording stopped before any frames were written";
        return name + " is an unfinished recording (" + humanSize(size) +
               ", missing its index): it was not closed properly, so it cannot be played";
    }
    return name + " could not be opened as a video (" + humanSize(size) + ")";
}

VideoFileSource::VideoFileSource(const std::string& videoFile,
                                 const std::string& dgzFile,
                                 float playbackSpeed,
                                 bool rateLimited,
				 bool loopPlayback)
    : metadata_dg(nullptr, [](DYN_GROUP*){ dgCloseBuffer(); })
    , stored_frameIDs(nullptr)
    , stored_timestamps(nullptr)
    , stored_linestatus(nullptr)
    , metadata_length(0)
    , current_idx(0)
    , playback_speed(playbackSpeed)
    , rate_limit(rateLimited)
    , loop_playback(loopPlayback)
    , default_frameID(0)
    , has_metadata(false)
{
    cap.open(videoFile);
    if (!cap.isOpened()) {
        throw std::runtime_error(describeUnplayable(videoFile));
    }
    
    width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    fps = cap.get(cv::CAP_PROP_FPS);
    if (fps == 0.0) fps = 30.0;
    
    // Try to determine if color (read first frame and check)
    cv::Mat test_frame;
    cap >> test_frame;
    if (test_frame.empty()) {
        throw std::runtime_error(
            std::filesystem::path(videoFile).filename().string() +
            " contains no video frames");
    }
    color = (test_frame.channels() > 1);
    cap.set(cv::CAP_PROP_POS_FRAMES, 0); // Rewind
    
    if (!dgzFile.empty()) {
      //        has_metadata = loadMetadata(dgzFile);
    }
    
    playback_start = std::chrono::high_resolution_clock::now();
}

bool VideoFileSource::loadMetadata(const std::string& dgzFile) {
  return true;
}

void VideoFileSource::seekToFrame(int frame_number) {
    if (frame_number < 0) frame_number = 0;
    
    int total = getTotalFrames();
    if (total > 0 && frame_number >= total) {
        frame_number = total - 1;
    }
    
    cap.set(cv::CAP_PROP_POS_FRAMES, frame_number);
    reseek_on_resume_ = false;
    current_idx = frame_number;
    delivered_idx_ = frame_number;
    default_frameID = frame_number;  // Keep frameID in sync
    
    // Adjust playback timing
    if (rate_limit) {
        std::lock_guard<std::mutex> lock(timing_mutex);
        playback_start = std::chrono::high_resolution_clock::now() - 
            std::chrono::microseconds((int64_t)(current_idx * 1e6 / (fps * playback_speed)));
    }
}

void VideoFileSource::setPlaybackSpeed(float speed) {
    std::lock_guard<std::mutex> lock(timing_mutex);
    if (speed <= 0 || speed == playback_speed) return;
    // Re-anchor so the current frame stays due now; otherwise the whole
    // timeline rescales and playback stalls (slower) or bursts (faster).
    playback_speed = speed;
    playback_start = std::chrono::high_resolution_clock::now() -
        std::chrono::microseconds((int64_t)(current_idx * 1e6 / (fps * playback_speed)));
}

int VideoFileSource::getTotalFrames() const {
    return (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
}

void VideoFileSource::stepFrame(int delta) {
    seekToFrame(delivered_idx_ + delta);
}

void VideoFileSource::setPaused(bool status) {
    const bool was = paused_;
    paused_ = status;
    // While paused the pacing clock kept running; on resume every frame looked
    // late and decoded in a burst. Re-anchor like setPlaybackSpeed does.
    if (was && !status && rate_limit) {
        std::lock_guard<std::mutex> lock(timing_mutex);
        playback_start = std::chrono::high_resolution_clock::now() -
            std::chrono::microseconds(
                (int64_t)(current_idx * 1e6 / (fps * playback_speed)));
    }
}

bool VideoFileSource::getNextFrame(cv::Mat& frame, FrameMetadata& metadata) {
    // If paused, re-read the frame already on screen. current_idx has
    // already moved on to the next frame; using it here advances the picture
    // by one and leaves the overlay on the previous frame.
    int read_idx = paused_ ? delivered_idx_ : current_idx;
    if (paused_) {
        cap.set(cv::CAP_PROP_POS_FRAMES, read_idx);
        reseek_on_resume_ = true;
        if (rate_limit && fps > 0.f && playback_speed > 0.f) {
            const auto interval = std::chrono::microseconds(
                (int64_t)(1e6 / (fps * playback_speed)));
            std::this_thread::sleep_for(interval);
        }
    } else {
        if (reseek_on_resume_) {
            cap.set(cv::CAP_PROP_POS_FRAMES, read_idx);
            reseek_on_resume_ = false;
        }
        // Rate limiting for normal playback
        if (rate_limit && current_idx > 0) {
            std::chrono::high_resolution_clock::time_point target_time;
            {
                std::lock_guard<std::mutex> lock(timing_mutex);
                target_time = playback_start +
                    std::chrono::microseconds((int64_t)(current_idx * 1e6 / (fps * playback_speed)));
            }
            std::this_thread::sleep_until(target_time);
        }
    }
    
    cap >> frame;
    if (frame.empty()) {
        if (loop_playback && !paused_) {  // Don't auto-loop when paused
            rewind();
            cap >> frame;
            
            if (frame.empty()) {
                return false;
            }
            read_idx = 0;
        } else {
            return false;
        }    
    }
    
    metadata.systemTime = std::chrono::high_resolution_clock::now();
    
    if (has_metadata && read_idx >= 0 && read_idx < metadata_length) {
        metadata.frameID = stored_frameIDs[read_idx];
        metadata.timestamp = stored_timestamps[read_idx];
        metadata.lineStatus = stored_linestatus ?
            (bool)stored_linestatus[read_idx] : false;
    } else {
        metadata.frameID = read_idx;
        metadata.timestamp = (int64_t)(read_idx * 1e9 / fps);
        metadata.lineStatus = ds_in_obs;
    }

    delivered_idx_ = read_idx;
    // Only advance if not paused
    if (!paused_) {
        current_idx = read_idx + 1;
        default_frameID = read_idx + 1;
    }
    
    return true;
}

void VideoFileSource::rewind() {
  cap.set(cv::CAP_PROP_POS_FRAMES, 0);
  current_idx = 0;
  delivered_idx_ = 0;
  default_frameID = 0;
  {
    std::lock_guard<std::mutex> lock(timing_mutex);
    playback_start = std::chrono::high_resolution_clock::now();
  }
  
  // Fire event to notify plugins/UI
  fireEvent(VstreamEvent("vstream/video_source_rewind"));
}

VideoFileSource::~VideoFileSource() {
  close();
  // metadata_dg will be cleaned up by unique_ptr with custom deleter
}
