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
    
public:
    SamplingManager(proginfo_t* p) : prog_info_(p) {}
    ~SamplingManager() {
      stop();
    }
    
    void start(int n_frames, int interval_ms, bool random = false);
    void stop();
    bool isActive();
    
private:
    void samplingLoop();
};
