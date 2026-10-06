#ifndef CONSOLE_CAPTURE_H
#define CONSOLE_CAPTURE_H

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct ConsoleLine {
  uint64_t seq;   // 1, 2, 3, ... in arrival order
  bool err;       // came from stderr
  std::string text;
};

// Copies everything the process writes to stdout and stderr, including Tcl's
// puts and library warnings that bypass std::cout, into a bounded ring the
// browser viewer can read. Output still reaches the original stdout/stderr
// unchanged. POSIX only; start() does nothing elsewhere.
class ConsoleCapture {
public:
  static ConsoleCapture& instance();
  ~ConsoleCapture();

  // Redirect fd 1 and 2 through pipes. Call once, before the output to keep.
  bool start();
  // Put fd 1 and 2 back and stop the reader once it has drained the pipes.
  void stop();

  // Up to `max` lines newer than `since`, appended to `out`.
  void linesSince(uint64_t since, size_t max, std::vector<ConsoleLine>& out);

  // Number of the newest line so far (0 before any output).
  uint64_t latestSeq();

  // Called (from the reader thread) after new lines arrive. Pass nullptr to
  // clear; clearing waits for a call in progress, so the callback's captures
  // may be destroyed afterwards.
  void setListener(std::function<void()> listener);

private:
  ConsoleCapture() = default;
  void run();
  void addLine(bool err, const std::string& raw);

  static constexpr size_t kMaxLines = 2000;
  static constexpr size_t kMaxLineBytes = 4000;

  bool started_ = false;
  int saved_[2] = {-1, -1};  // original stdout, stderr
  int read_[2] = {-1, -1};   // pipe read ends
  std::thread thread_;
  std::atomic<bool> stop_{false};

  std::mutex ring_mutex_;
  std::deque<ConsoleLine> ring_;
  uint64_t last_seq_ = 0;

  std::mutex listener_mutex_;
  std::function<void()> listener_;
};

#endif
