// Declarations for VideoStream.cpp and tclproc.cpp

typedef struct _proginfo_t {
  char *name;
  Tcl_Interp *interp;
  int display;

  SourceManager* sourceManager;
  SamplingManager *samplingManager;
  ReviewModeSource* reviewSource;
  FrameBufferManager* frameBuffer;
  WidgetManager* widgetManager;
  
  std::atomic<int> *curFrame;
  std::atomic<int> *displayFrame;
  
  IFrameSource** frameSource;  // Pointer to pointer so we can update it
  
  // Frame properties
  float* frame_rate;
  std::atomic<int>* frame_width;
  std::atomic<int>* frame_height;
  bool* is_color;
  DservSocket *dservSocket;
  const char *ds_host;
  int ds_port;
} proginfo_t;

class WebSocketThread;
extern WebSocketThread* g_wsServer;

// thread safe tcl command evals
int tcl_eval(const std::string& cmd);
int tcl_eval(const std::string& cmd, std::string& response);

// send events to event queue
void fireEvent(const std::string& type, const std::string& data);
std::string jsonToTclDict(const std::string& json);

// Add uWebSockets support
#include <App.h>

// Add WebSocket per-socket data structure
struct WSPerSocketData {
  SharedQueue<std::string> *rqueue;
  std::string client_name;
  std::vector<std::string> subscriptions;

  std::map<std::string, std::chrono::steady_clock::time_point> last_sent;
  std::map<std::string, int> event_counts;
  std::chrono::steady_clock::time_point rate_window_start;
};

// To help manage large WebSocket messages (stimdg -> ess/stiminfo)
struct ChunkedMessage {
    std::string messageId;
    size_t chunkIndex;
    size_t totalChunks;
    std::string data;
    bool isLastChunk;
};


#ifdef __cplusplus
extern "C" {
#endif
  void addTclCommands(Tcl_Interp *interp, proginfo_t *p);
  int open_videoFile(char *filename);
  int close_videoFile(void);

  void start_recording();
  void stop_recording();
  
  int open_metadataFile(const char *base_name, const char *source_video);
  int is_metadataOnly(void);
  void set_useSQLite(int enable);
  int get_useSQLite(void);
  
  int open_domainSocket(char *socket_path);
  int close_domainSocket(void);
  int sendn_domainSocket(int n);

  int set_inObs(int status);
  int set_onlySaveInObs(int status);
  int set_fourCC(char *str);

  void add_shutdown_command(char *str);
  
  int show_display(proginfo_t *p);
  int hide_display(proginfo_t *p);

  int configure_exposure(float exposure);
  int configure_ROI(int w, int h, int offsetx, int offsety);
  int configure_gain(float gain);
  int configure_framerate(float framerate);

  int do_shutdown();

  /* Shared with tcl */
  extern int dsPort;
  extern int useWebcam;
  extern int displayEvery;   // Determines how often to update display

  extern int ShowChunk;

#ifdef __cplusplus
}


extern std::atomic<int> frame_width;
extern std::atomic<int> frame_height;

#endif

