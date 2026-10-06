#include "ConsoleCapture.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <iostream>

#ifndef _WIN32
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#endif

ConsoleCapture& ConsoleCapture::instance() {
  static ConsoleCapture capture;
  return capture;
}

ConsoleCapture::~ConsoleCapture() {
  stop();
}

// Text the browser can show: no terminal colour codes or control characters,
// and valid UTF-8 (JSON text frames must be), invalid bytes becoming '?'.
static std::string sanitize(const std::string& in, size_t max_bytes) {
  std::string out;
  out.reserve(in.size());
  const size_t n = in.size();
  size_t i = 0;
  while (i < n) {
    const unsigned char c = in[i];
    if (c == 0x1b) {  // skip an ANSI CSI sequence such as ESC[31m
      i++;
      if (i < n && in[i] == '[') {
        i++;
        while (i < n && ((unsigned char) in[i] < 0x40 || (unsigned char) in[i] > 0x7e)) i++;
        if (i < n) i++;
      }
      continue;
    }
    if (c < 0x20 && c != '\t') { i++; continue; }
    if (c == 0x7f) { i++; continue; }
    if (c < 0x80) { out += (char) c; i++; continue; }

    int len = 0;
    if (c >= 0xC2 && c <= 0xDF) len = 2;
    else if (c >= 0xE0 && c <= 0xEF) len = 3;
    else if (c >= 0xF0 && c <= 0xF4) len = 4;
    bool ok = len > 0 && i + len <= n;
    for (int k = 1; ok && k < len; k++) ok = (((unsigned char) in[i + k]) & 0xC0) == 0x80;
    if (ok && len >= 3) {  // no overlong forms, surrogates or values past U+10FFFF
      const unsigned char c1 = in[i + 1];
      if ((c == 0xE0 && c1 < 0xA0) || (c == 0xED && c1 > 0x9F) ||
          (c == 0xF0 && c1 < 0x90) || (c == 0xF4 && c1 > 0x8F))
        ok = false;
    }
    if (ok) {
      out.append(in, i, len);
      i += len;
    } else {
      out += '?';
      i++;
    }
  }
  if (out.size() > max_bytes) {
    size_t cut = max_bytes;
    while (cut > 0 && (((unsigned char) out[cut]) & 0xC0) == 0x80) cut--;
    out.resize(cut);
    out += "\xE2\x80\xA6";  // ellipsis
  }
  return out;
}

void ConsoleCapture::addLine(bool err, const std::string& raw) {
  std::string text = sanitize(raw, kMaxLineBytes);
  std::lock_guard<std::mutex> lock(ring_mutex_);
  ring_.push_back({++last_seq_, err, std::move(text)});
  if (ring_.size() > kMaxLines) ring_.pop_front();
}

void ConsoleCapture::linesSince(uint64_t since, size_t max, std::vector<ConsoleLine>& out) {
  std::lock_guard<std::mutex> lock(ring_mutex_);
  if (ring_.empty()) return;
  const uint64_t first = ring_.front().seq;  // seqs are contiguous
  size_t idx = since + 1 > first ? (size_t)(since + 1 - first) : 0;
  for (; idx < ring_.size() && out.size() < max; idx++) out.push_back(ring_[idx]);
}

uint64_t ConsoleCapture::latestSeq() {
  std::lock_guard<std::mutex> lock(ring_mutex_);
  return last_seq_;
}

void ConsoleCapture::setListener(std::function<void()> listener) {
  std::lock_guard<std::mutex> lock(listener_mutex_);
  listener_ = std::move(listener);
}

#ifdef _WIN32

bool ConsoleCapture::start() { return false; }
void ConsoleCapture::stop() {}
void ConsoleCapture::run() {}

#else

bool ConsoleCapture::start() {
  if (started_) return true;
  int out_pipe[2], err_pipe[2];
  if (pipe(out_pipe) != 0) return false;
  if (pipe(err_pipe) != 0) {
    close(out_pipe[0]);
    close(out_pipe[1]);
    return false;
  }
  // F_DUPFD_CLOEXEC keeps these out of child processes (Tcl exec); the child
  // still inherits fd 1 and 2, which now lead into the pipes.
  saved_[0] = fcntl(STDOUT_FILENO, F_DUPFD_CLOEXEC, 3);
  saved_[1] = fcntl(STDERR_FILENO, F_DUPFD_CLOEXEC, 3);
  if (saved_[0] < 0 || saved_[1] < 0) {
    for (int fd : {saved_[0], saved_[1], out_pipe[0], out_pipe[1], err_pipe[0], err_pipe[1]})
      if (fd >= 0) close(fd);
    saved_[0] = saved_[1] = -1;
    return false;
  }
  fcntl(out_pipe[0], F_SETFD, FD_CLOEXEC);
  fcntl(err_pipe[0], F_SETFD, FD_CLOEXEC);

  std::cout.flush();
  std::cerr.flush();
  fflush(stdout);
  fflush(stderr);
  dup2(out_pipe[1], STDOUT_FILENO);
  dup2(err_pipe[1], STDERR_FILENO);
  close(out_pipe[1]);
  close(err_pipe[1]);
  read_[0] = out_pipe[0];
  read_[1] = err_pipe[0];

  // A pipe is not a terminal, so stdio would switch from line to full
  // buffering and the log would arrive in delayed clumps. (std::cout follows
  // stdio; Tcl's stdout channel is set to line buffering in setupTcl.)
  setvbuf(stdout, nullptr, _IOLBF, 0);

  stop_ = false;
  started_ = true;
  thread_ = std::thread(&ConsoleCapture::run, this);
  return true;
}

void ConsoleCapture::stop() {
  if (!started_) return;
  std::cout.flush();
  std::cerr.flush();
  fflush(stdout);
  fflush(stderr);
  // Back to the real stdout/stderr. That closes our write ends of the pipes
  // (unless a child process still holds one), so the reader sees end of file;
  // stop_ makes it quit even if it does not.
  dup2(saved_[0], STDOUT_FILENO);
  dup2(saved_[1], STDERR_FILENO);
  stop_ = true;
  if (thread_.joinable()) thread_.join();
  for (int& fd : read_) {
    if (fd >= 0) close(fd);
    fd = -1;
  }
  for (int& fd : saved_) {
    if (fd >= 0) close(fd);
    fd = -1;
  }
  started_ = false;
}

static void writeAll(int fd, const char* data, size_t len) {
  while (len > 0) {
    ssize_t w = write(fd, data, len);
    if (w < 0) {
      if (errno == EINTR) continue;
      return;  // the terminal went away; nothing useful left to do
    }
    data += w;
    len -= (size_t) w;
  }
}

void ConsoleCapture::run() {
  struct Stream {
    bool open = true;
    std::string partial;  // text after the last newline
    std::chrono::steady_clock::time_point partial_since;
  } streams[2];
  constexpr int kPollMs = 100;
  constexpr auto kPartialFlush = std::chrono::milliseconds(250);

  char buf[8192];
  while (true) {
    pollfd fds[2];
    int n = 0;
    int which[2];
    for (int s = 0; s < 2; s++) {
      if (!streams[s].open) continue;
      fds[n] = {read_[s], POLLIN, 0};
      which[n++] = s;
    }
    if (n == 0) break;

    int ready = poll(fds, n, kPollMs);
    bool added = false;
    for (int i = 0; ready > 0 && i < n; i++) {
      if (!(fds[i].revents & (POLLIN | POLLHUP | POLLERR))) continue;
      const int s = which[i];
      Stream& st = streams[s];
      ssize_t got = read(read_[s], buf, sizeof buf);
      if (got < 0 && errno == EINTR) continue;
      if (got <= 0) {
        st.open = false;
        if (!st.partial.empty()) {
          addLine(s == 1, st.partial);
          st.partial.clear();
          added = true;
        }
        continue;
      }
      writeAll(saved_[s], buf, (size_t) got);  // the terminal sees it as before
      if (st.partial.empty()) st.partial_since = std::chrono::steady_clock::now();
      st.partial.append(buf, (size_t) got);
      size_t nl;
      while ((nl = st.partial.find('\n')) != std::string::npos) {
        addLine(s == 1, st.partial.substr(0, nl));
        st.partial.erase(0, nl + 1);
        added = true;
      }
      if (!st.partial.empty()) st.partial_since = std::chrono::steady_clock::now();
    }

    // Text with no newline yet (a prompt, progress output): show it once it
    // has been quiet for a moment instead of holding it back.
    const auto now = std::chrono::steady_clock::now();
    for (int s = 0; s < 2; s++) {
      Stream& st = streams[s];
      if (!st.partial.empty() && now - st.partial_since > kPartialFlush) {
        addLine(s == 1, st.partial);
        st.partial.clear();
        added = true;
      }
    }

    if (added) {
      std::lock_guard<std::mutex> lock(listener_mutex_);
      if (listener_) listener_();
    }
    if (ready == 0 && stop_) break;  // restored fds, nothing left to read
  }
}

#endif
