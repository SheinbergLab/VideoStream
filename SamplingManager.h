#pragma once

typedef struct _proginfo_t proginfo_t;

class SamplingManager
{
private:
    std::atomic<bool> m_bDone{false};
    std::atomic<bool> m_bActive{false};
    std::thread m_thread;
    
    int n_frames_;
    int interval_ms_;
    bool random_intervals_;

    proginfo_t* prog_info_;

    std::mutex m_mutex;   
    std::string completion_callback_;
 
public:
    SamplingManager(proginfo_t* p) : prog_info_(p) {}
    ~SamplingManager() {
      stop();
    }

    void setCompletionCallback(const std::string& callback) {
      std::lock_guard<std::mutex> lock(m_mutex);
      completion_callback_ = callback;
    }
     
    void start(int n_frames, int interval_ms, bool random = false);
    void stop();
    bool isActive();
    
private:
    void samplingLoop();
};
