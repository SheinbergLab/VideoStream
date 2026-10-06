// WebPreview.cpp

#include "WebPreview.h"

#include <jansson.h>
#include <unistd.h>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct HostLoad {
  bool cpu_ready = false;
  double proc_pct = 0;
  double host_pct = 0;
  long rss_kb = -1;
  long avail_kb = -1;
};

bool readProcTicks(unsigned long long& ticks) {
  std::ifstream in("/proc/self/stat");
  std::string line;
  if (!std::getline(in, line)) return false;
  auto r = line.rfind(')');
  if (r == std::string::npos) return false;
  // After comm: state ppid pgrp session tty tpgid flags minflt cminflt majflt cmajflt utime stime
  std::istringstream ss(line.substr(r + 1));
  std::string skip;
  unsigned long long utime = 0, stime = 0;
  for (int i = 0; i < 11; i++) ss >> skip;
  ss >> utime >> stime;
  if (!ss) return false;
  ticks = utime + stime;
  return true;
}

bool readHostCpu(unsigned long long& total, unsigned long long& idle) {
  std::ifstream in("/proc/stat");
  std::string label;
  unsigned long long user = 0, nice = 0, system = 0, idle_v = 0, iowait = 0, irq = 0, softirq = 0, steal = 0;
  if (!(in >> label >> user >> nice >> system >> idle_v >> iowait >> irq >> softirq >> steal)) return false;
  if (label != "cpu") return false;
  idle = idle_v + iowait;
  total = user + nice + system + idle_v + iowait + irq + softirq + steal;
  return true;
}

long readKeyedKb(const char* path, const char* key) {
  std::ifstream in(path);
  std::string label;
  while (in >> label) {
    if (label == key) {
      long kb = 0;
      in >> kb;
      return kb;
    }
    std::string rest;
    std::getline(in, rest);
  }
  return -1;
}

// Cached for a second. The first sample has no CPU rate yet.
HostLoad sampleHostLoad() {
  static auto last = std::chrono::steady_clock::time_point{};
  static HostLoad cached;
  static unsigned long long prev_proc = 0, prev_total = 0, prev_idle = 0;
  static bool have_prev = false;

  auto now = std::chrono::steady_clock::now();
  if (last.time_since_epoch().count() != 0 && now - last < std::chrono::seconds(1))
    return cached;

  unsigned long long proc = 0, total = 0, idle = 0;
  bool got_proc = readProcTicks(proc);
  bool got_host = readHostCpu(total, idle);
  cached.rss_kb = readKeyedKb("/proc/self/status", "VmRSS:");
  cached.avail_kb = readKeyedKb("/proc/meminfo", "MemAvailable:");

  if (have_prev && got_proc && got_host && last.time_since_epoch().count() != 0) {
    double dt = std::chrono::duration<double>(now - last).count();
    long clk = sysconf(_SC_CLK_TCK);
    if (dt > 0 && clk > 0 && proc >= prev_proc) {
      cached.proc_pct = (double)(proc - prev_proc) / (double)clk / dt * 100.0;
      cached.cpu_ready = true;
    }
    if (total > prev_total && idle >= prev_idle) {
      double busy = 1.0 - (double)(idle - prev_idle) / (double)(total - prev_total);
      if (busy < 0) busy = 0;
      if (busy > 1) busy = 1;
      cached.host_pct = busy * 100.0;
    }
  }
  if (got_proc) prev_proc = proc;
  if (got_host) {
    prev_total = total;
    prev_idle = idle;
  }
  have_prev = got_proc && got_host;
  last = now;
  return cached;
}

}  // namespace

void WebPreview::start(DemandFn demand, SinkFn sink, OverlayFn overlay, SourceFn source)
{
  if (running_) return;
  demand_ = std::move(demand);
  sink_ = std::move(sink);
  overlay_ = std::move(overlay);
  source_ = std::move(source);
  running_ = true;
  thread_ = std::thread(&WebPreview::run, this);
}

void WebPreview::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) return;
    running_ = false;
  }
  cv_.notify_one();
  if (thread_.joinable()) thread_.join();
}

void WebPreview::offer(const cv::Mat& frame, const PreviewFrameInfo& info)
{
  if (!running_ || frame.empty()) return;

  int fps = 0, quality = 0;
  if (demand_(fps, quality) <= 0) return;
  fps = std::min(std::max(fps, 1), max_fps_.load());
  if (quality <= 0) quality = default_quality_;

  // Fixed schedule rather than "interval since the last frame": source frames
  // arrive every 4 ms at 250 Hz, and measuring from the last send would round
  // every period up and settle well under the requested rate.
  auto now = std::chrono::steady_clock::now();
  auto interval = std::chrono::microseconds(1000000 / fps);
  if (now < last_offer_) return;
  last_offer_ += interval;
  if (now - last_offer_ > interval) last_offer_ = now + interval;

  long long ts_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frame.copyTo(slot_);
    slot_info_ = info;
    slot_quality_ = quality;
    slot_ts_us_ = ts_us;
    pending_ = true;
  }
  cv_.notify_one();
}

WebPreview::Stats WebPreview::stats() const
{
  return { frames_encoded_.load(), last_encode_ms_.load(), last_bytes_.load() };
}

void WebPreview::run()
{
  cv::Mat frame;
  std::vector<uchar> jpeg;

  while (true) {
    PreviewFrameInfo info;
    int quality;
    long long ts_us;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return pending_ || !running_; });
      if (!running_) break;
      // Swap rather than copy: the slot keeps this buffer for the next offer.
      cv::swap(frame, slot_);
      info = slot_info_;
      quality = slot_quality_;
      ts_us = slot_ts_us_;
      pending_ = false;
    }

    auto t0 = std::chrono::steady_clock::now();
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, quality };
    if (!cv::imencode(".jpg", frame, jpeg, params)) continue;
    std::string overlay;
    if (overlay_) {
      // The JPEG was captured a moment ago. Analysis of that exact frame may
      // still be in flight, or already overwritten by a newer one. Wait until
      // every plugin overlay that reports a frame_id is for this frame, so
      // stored and live markers are never drawn on the neighboring picture.
      // Overlays without a frame_id can't be matched and pass through. On
      // timeout, only the overlays still on another frame are dropped.
      for (int attempt = 0; attempt < 40; ++attempt) {
        overlay = overlay_(info);
        json_t* root = json_loads(overlay.c_str(), 0, nullptr);
        if (!root) break;
        std::vector<std::string> stale;
        const char* name;
        json_t* plugin;
        json_object_foreach(root, name, plugin) {
          json_t* fid = json_object_get(plugin, "frame_id");
          if (fid && json_is_integer(fid) && json_integer_value(fid) != info.frame_id)
            stale.push_back(name);
        }
        if (!stale.empty() && attempt == 39) {
          for (const auto& n : stale) json_object_del(root, n.c_str());
          char* out = json_dumps(root, JSON_COMPACT);
          overlay = out ? out : "{}";
          free(out);
        }
        json_decref(root);
        if (stale.empty() || attempt == 39) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    double encode_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    json_t* header = json_object();
    json_object_set_new(header, "type", json_string("frame"));
    json_object_set_new(header, "seq", json_integer(++seq_));
    json_object_set_new(header, "video_frame", json_integer(info.video_frame));
    json_object_set_new(header, "frame_id", json_integer(info.frame_id));
    json_object_set_new(header, "ring_index", json_integer(info.ring_index));
    json_object_set_new(header, "ring_size", json_integer(info.ring_size));
    json_object_set_new(header, "ts_us", json_integer(ts_us));
    json_object_set_new(header, "width", json_integer(frame.cols));
    json_object_set_new(header, "height", json_integer(frame.rows));
    json_object_set_new(header, "channels", json_integer(frame.channels()));
    json_object_set_new(header, "src_fps", json_real(info.src_fps));
    json_object_set_new(header, "incomplete_frames", json_integer(info.incomplete_frames));
    HostLoad load = sampleHostLoad();
    if (load.cpu_ready) {
      json_object_set_new(header, "proc_cpu", json_real(load.proc_pct));
      json_object_set_new(header, "host_cpu", json_real(load.host_pct));
    }
    if (load.rss_kb >= 0)
      json_object_set_new(header, "rss_kb", json_integer(load.rss_kb));
    if (load.avail_kb >= 0)
      json_object_set_new(header, "mem_avail_kb", json_integer(load.avail_kb));
    json_object_set_new(header, "in_obs", json_boolean(info.in_obs));
    if (!info.datafile.empty())
      json_object_set_new(header, "datafile", json_string(info.datafile.c_str()));
    json_object_set_new(header, "encode_ms", json_real(encode_ms));
    json_t* overlay_obj = overlay.empty() ? nullptr : json_loads(overlay.c_str(), 0, nullptr);
    json_object_set_new(header, "overlay", overlay_obj ? overlay_obj : json_object());
    if (source_) {
      json_t* src = source_(info, frame.cols, frame.rows);
      if (src) json_object_set_new(header, "source", src);
    }

    char* header_str = json_dumps(header, JSON_COMPACT);
    size_t header_len = strlen(header_str);

    auto message = std::make_shared<std::string>();
    message->resize(4 + header_len + jpeg.size());
    char* p = message->data();
    uint32_t len32 = (uint32_t) header_len;
    p[0] = (char) (len32 & 0xff);
    p[1] = (char) ((len32 >> 8) & 0xff);
    p[2] = (char) ((len32 >> 16) & 0xff);
    p[3] = (char) ((len32 >> 24) & 0xff);
    memcpy(p + 4, header_str, header_len);
    memcpy(p + 4 + header_len, jpeg.data(), jpeg.size());
    free(header_str);
    json_decref(header);

    frames_encoded_++;
    last_encode_ms_ = encode_ms;
    last_bytes_ = message->size();

    if (sink_) sink_(message);
  }
}
