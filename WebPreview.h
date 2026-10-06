// WebPreview.h
//
// Browser preview stream: a rate-limited copy of the live frame, JPEG-encoded
// on its own thread and packed with the plugins' structured overlay data.
//
// Wire format of one preview message (WebSocket BINARY):
//   [uint32 little-endian header_len][header JSON, header_len bytes][JPEG]
// The header is {"type":"frame", seq, video_frame, frame_id, ring_index,
// ring_size, ts_us, width, height, channels, src_fps, in_obs, encode_ms,
// overlay:{<plugin name>:{...}}}.
//
// The capture loop only ever calls offer(), which returns immediately unless
// a client wants preview and the next frame is due; the encoder works from a
// single latest-wins slot, so a slow encode drops preview frames rather than
// ever holding up capture.

#pragma once

#include "opencv2/opencv.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

struct json_t;

struct PreviewFrameInfo {
  long long video_frame = -1;   // frame number within the video file (playback)
  long long frame_id = -1;      // source frame id (metadata.frameID)
  int ring_index = -1;          // host ring-buffer slot (matches plugin analysis_frame)
  int ring_size = 0;
  double src_fps = 0.0;
  long long incomplete_frames = 0;
  bool in_obs = false;
  std::string datafile;         // dataserver datafile currently open ("" = none)
};

class WebPreview {
public:
  // Number of clients wanting preview and the frame rate / JPEG quality they
  // asked for (highest over clients). Called on the capture thread: must be cheap.
  using DemandFn = std::function<int(int& fps, int& quality)>;
  using SinkFn = std::function<void(std::shared_ptr<const std::string>)>;
  // JSON object text {"<plugin>":{...},...}, built at encode time.
  using OverlayFn = std::function<std::string(const PreviewFrameInfo& info)>;
  // Preview header "source" object (type, label, tooltip).
  using SourceFn = std::function<json_t*(const PreviewFrameInfo&, int frame_w, int frame_h)>;

  WebPreview() = default;
  ~WebPreview() { stop(); }

  void start(DemandFn demand, SinkFn sink, OverlayFn overlay, SourceFn source = {});
  void stop();

  // Capture thread, every frame.
  void offer(const cv::Mat& frame, const PreviewFrameInfo& info);

  // Server-side cap on the encode rate, whatever clients ask for.
  void setMaxFps(int fps) { max_fps_ = std::max(1, fps); }
  int maxFps() const { return max_fps_; }

  // JPEG quality used when no client asked for one (demand quality <= 0).
  void setDefaultQuality(int q) { default_quality_ = std::min(100, std::max(10, q)); }
  int defaultQuality() const { return default_quality_; }

  struct Stats {
    long long frames_encoded;
    double last_encode_ms;
    size_t last_bytes;
  };
  Stats stats() const;

private:
  void run();

  DemandFn demand_;
  SinkFn sink_;
  OverlayFn overlay_;
  SourceFn source_;

  std::atomic<int> max_fps_{30};
  std::atomic<int> default_quality_{80};
  std::chrono::steady_clock::time_point last_offer_{};

  std::mutex mutex_;
  std::condition_variable cv_;
  bool pending_ = false;
  bool running_ = false;
  cv::Mat slot_;
  PreviewFrameInfo slot_info_;
  int slot_quality_ = 80;
  long long slot_ts_us_ = 0;
  std::thread thread_;

  std::atomic<long long> seq_{0};
  std::atomic<long long> frames_encoded_{0};
  std::atomic<double> last_encode_ms_{0.0};
  std::atomic<size_t> last_bytes_{0};
};
