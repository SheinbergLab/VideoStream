// Declarations for VideoStream.cpp and tclproc.cpp

#include "VstreamVars.h"

typedef struct _proginfo_t {
  char *name;
  Tcl_Interp *interp;
  int display;
  char **argv;
  int argc;
  const char *script_file;
  SourceManager* sourceManager;
  SamplingManager *samplingManager;
  ReviewModeSource* reviewSource;
  FrameBufferManager* frameBuffer;
  WidgetManager* widgetManager;
  
  std::atomic<int> *curFrame;
  std::atomic<int> *displayFrame;
  
  IFrameSource** frameSource;  // Pointer to pointer so we can update it
  
  // Frame properties
  std::atomic<int>* frame_width;
  std::atomic<int>* frame_height;
  std::atomic<float>* frame_rate;
  std::atomic<bool>* is_color;
  DservSocket *dservSocket;
  const char *ds_host;
  int ds_port;
} proginfo_t;

class WebSocketThread;
extern WebSocketThread* g_wsServer;

class WebPreview;
extern WebPreview g_webPreview;
int web_preview_clients(void);

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

  // Browser preview stream (opted into with {"cmd":"preview"})
  bool preview_enabled = false;
  int preview_fps = 30;
  int preview_quality = 0;   // 0 = server default (vstream::webPreview quality)
  std::chrono::steady_clock::time_point preview_next_due{};
  long long preview_dropped = 0;

  // Server console stream (opted into with {"cmd":"logs"}); logs_sent is the
  // newest ConsoleCapture line already delivered to this client.
  bool logs_enabled = false;
  unsigned long long logs_sent = 0;
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
  int set_reprocessMode(int status);
  int set_fourCC(char *str);

  void add_shutdown_command(char *str);

  // Name of the datafile the dataserver has open ("" = none); the browser
  // viewer shows a FILE OPEN tag while it is set. Called from Tcl.
  void set_viewer_datafile(const char *name);
  
  int show_display(proginfo_t *p);
  int hide_display(proginfo_t *p);

  int configure_exposure(float exposure);
  int configure_ROI(int w, int h, int offsetx, int offsety);
  int configure_gain(float gain);
  int configure_framerate(float framerate);

  int do_shutdown();

#ifdef __cplusplus
}


extern std::atomic<int> frame_width;
extern std::atomic<int> frame_height;
extern std::atomic<float> frame_rate;
extern std::atomic<bool>  is_color;
extern std::atomic<bool>  g_reprocess_serial;

#endif

