#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "opencv2/opencv.hpp"

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>   // std::max/min, std::remove* (GCC 14 / Debian Trixie: no transitive include)
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <random>
#include <unordered_map>
#include <queue>
#include <atomic>
#include <csignal>
#include <cmath>

// Use dgz format to store metadata about frames
#include <df.h>
#include <dynio.h>
#include <tcl.h>

#include <jansson.h>
#include <sys/un.h>
#include <sys/socket.h>

#if !defined(_WIN32)
#include <unistd.h>
#endif

#include "cxxopts.hpp"
#include "SharedQueue.hpp"
#include "DservSocket.h"
#include "VstreamEvent.h"
#include "EventToTcl.h"
#include "KeyboardCallbackRegistry.h"
#include "SourceManager.h"
#include "StorageManager.h"
#include "FrameBufferManager.h"
#include "IFrameSource.h"
#include "WebcamSource.h"
#include "VideoFileSource.h"
#include "ReviewModeSource.h"
#include "CameraControl.h"
#include <deque>
#include <cstdlib>
#ifdef USE_FLIR
#include "FlirCameraSource.h"
#endif
#ifdef USE_LUCID
#include "LucidCameraSource.h"
#endif

#include "Widget.h"
#include "WidgetManager.h"
#include "SamplingManager.h"
#include "DataserverForwarder.h"

#include "VideoStream.h"

// our minified html pages (in www/*.html)
#include "embedded_terminal.h"
#include "embedded_interface.h"

using namespace std;
using namespace cv;

SourceManager g_sourceManager;
WidgetManager g_widgetManager;

SharedQueue<DataPoint> ds_forward_queue;
DataserverForwarder* g_dataForwarder = nullptr;

// Global frame source pointer
IFrameSource* g_frameSource = nullptr;

// Analysis registry support for plugins
#include "AnalysisPluginRegistry.h"
AnalysisPluginRegistry g_pluginRegistry;

class WebSocketThread;
WebSocketThread* g_wsServer = nullptr;

std::thread processThreadID;
SharedQueue<int> process_queue;

std::thread displayThreadID;
SharedQueue<int> display_queue;

SharedQueue<std::string> mouse_queue;

SharedQueue<std::string> cqueue;
SharedQueue<std::string> rqueue;

SharedQueue<std::string> wd_cqueue;
SharedQueue<std::string> wd_rqueue;

SharedQueue<std::string> disp_cqueue;
SharedQueue<std::string> disp_rqueue;

SharedQueue<std::string> ds_queue;
SharedQueue<std::string> shutdown_queue;

SharedQueue<VstreamEvent> event_queue;

std::atomic<bool> events_ready{false};

Tcl_Interp *interp = NULL;

/* Shared with tcl */
int dsPort;
int displayEvery = 1;   // Determines how often to update display

int ShowChunk = 0;

namespace
{
  volatile std::sig_atomic_t done;
}

void signal_handler(int signal)
{
  done = true;
  do_shutdown();
}

const int FRAME_SENTINEL_OPEN = -3;   // Signal to wakeup and process
const int FRAME_SENTINEL_CLOSE = -2;  // Signal to close file
const int FRAME_SENTINEL_END = -1;    // Signal end of stream

/* Buffer to hold video frames */
int nFrames = 500;
FrameBufferManager frameBufferManager(nFrames);

std::atomic<int> displayFrame{0};
std::atomic<int> processFrame{0};

std::atomic<int> frame_width, frame_height;

std::atomic<bool> is_color = false;
std::atomic<float> frame_rate;

std::string output_file;
bool overwrite = false;

bool in_obs = false;		
bool ds_in_obs = false;		// dataserver obs status (no line_status)

// Where a frame's obs state comes from (vstream::obsSource):
//   line       the camera's TTL input latched per frame (hardware)
//   dserv      the last ess/in_obs datapoint, applied on arrival
//   timestamp  ess/in_obs datapoints matched to frames by timestamp -- exact
//              when the camera clock is PTP-synced with the dserv host
enum ObsSource { OBS_SOURCE_LINE = 0, OBS_SOURCE_DSERV = 1, OBS_SOURCE_TIMESTAMP = 2 };
std::atomic<int> obs_source{OBS_SOURCE_LINE};
// ess/in_obs transitions (dserv timestamp in us, state) waiting for the frame
// they belong to. Only the acquisition thread touches these (processDSCommands
// runs there too).
struct ObsTransition { int64_t time_us; bool state; };
static std::deque<ObsTransition> pending_obs_transitions;
static bool timestamp_obs_state = false;
static bool timestamp_obs_warned = false;
// camera clock minus dserv (UTC) clock, in us. PTP runs on TAI, 37 s ahead
// of UTC (2026), so a PTP-synced camera stamps frames 37 s "late" relative
// to dserv datapoints; the offset is detected from the frames' own camera
// vs host timestamps (rounded to whole seconds, so host NTP error does not
// matter) unless set explicitly with vstream::obsClockOffset.
static int64_t obs_clock_offset_us = 0;
static bool obs_clock_offset_known = false;
static bool obs_clock_offset_manual = false;
const char* obsSourceName();
int set_obsSource(const char* name);
bool only_save_in_obs = true;
int obs_count = -1;

// Deterministic offline reprocess: when true, the capture loop processes one
// frame fully (analyze inline + store) before capturing the next, so the
// recorded result for frame N is exactly frame N's analysis. Live = false.
std::atomic<bool> g_reprocess_serial{false};

#ifdef _WIN32
bool WSA_initialized = false;   // Windows Socket startup needs to be called once
bool WSA_shutdown = false;  // Windows Socket cleanup only once
#endif

class WatchdogThread
{
  public:
  bool m_bDone;
  int interval;
  
  WatchdogThread()
  {
    m_bDone = false;
    interval = 1000;       // 1 second wakeup
  }

  void startWatchdog(void) {
    while (!m_bDone) {
      // Check if proc exists before calling it
      wd_cqueue.push_back(std::string("if {[info commands onWatchdog] ne \"\"} { onWatchdog }"));
      /* rqueue will be available after command has been processed */
      std::string s(wd_rqueue.front());
      wd_rqueue.pop_front();
      // The "reply" may be the shutdown unblocker pushed by cleanup (the main
      // loop that services wd_cqueue has exited by then) — check before
      // sleeping, and sleep in slices so shutdown never waits a full interval.
      if (m_bDone) break;
      for (int slept = 0; slept < interval && !m_bDone; slept += 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }
};

class TcpipThread
{
private:
  // Connection limiting
  static constexpr int MAX_TOTAL_CONNECTIONS = 50;
  static constexpr int MAX_CONNECTIONS_PER_IP = 5;
  std::atomic<int> active_connections{0};
  std::mutex connection_mutex;
  std::unordered_map<std::string, int> ip_connection_count;
  std::unordered_map<int, std::string> socket_to_ip;
  
  std::mutex m_sig_mutex;
  
  public:
  bool m_bDone;

  std::condition_variable m_sig_cv;
  int port;

  int listening_socket_fd;  // Store this in startTcpServer
  
  // Add a public shutdown method to both thread classes:
  void shutdown() {
    m_bDone = true;
    if (listening_socket_fd >= 0) {
      ::shutdown(listening_socket_fd, SHUT_RDWR);
      close(listening_socket_fd);
      listening_socket_fd = -1;
    }
}
  
  TcpipThread()
  {
    m_bDone = false;
    port = 4630;
    done = false;
  }

  ~TcpipThread()
  {
  }
  
  std::string get_client_ip(const struct sockaddr& client_address) {
    char ip_str[INET_ADDRSTRLEN];
    const struct sockaddr_in* addr_in = (const struct sockaddr_in*)&client_address;
    inet_ntop(AF_INET, &(addr_in->sin_addr), ip_str, INET_ADDRSTRLEN);
    return std::string(ip_str);
  }
  
  bool accept_new_connection(const std::string& client_ip) {
    std::lock_guard<std::mutex> lock(connection_mutex);
    
    if (active_connections.load() >= MAX_TOTAL_CONNECTIONS) {
      return false;
    }
    
    auto ip_count_it = ip_connection_count.find(client_ip);
    int current_ip_connections = (ip_count_it != ip_connection_count.end()) ? ip_count_it->second : 0;
    
    return current_ip_connections < MAX_CONNECTIONS_PER_IP;
  }
  
  void register_connection(int socket_fd, const std::string& client_ip) {
    std::lock_guard<std::mutex> lock(connection_mutex);
    active_connections++;
    ip_connection_count[client_ip]++;
    socket_to_ip[socket_fd] = client_ip;
  }
  
  void unregister_connection(int socket_fd) {
    std::lock_guard<std::mutex> lock(connection_mutex);
    
    auto it = socket_to_ip.find(socket_fd);
    if (it != socket_to_ip.end()) {
      const std::string& client_ip = it->second;
      
      active_connections--;
      ip_connection_count[client_ip]--;
      if (ip_connection_count[client_ip] <= 0) {
	ip_connection_count.erase(client_ip);
      }
      
      socket_to_ip.erase(it);
    }
    
    close(socket_fd);
  }
  
  static void tcpClientProcess(TcpipThread* server, int sockfd) {
    ssize_t n;
    static char buf[1024];
    
    while ((n = recv(sockfd, buf, sizeof(buf), 0)) > 0) {
      cqueue.push_back(std::string(buf, n));
      
      std::string s(rqueue.front());
      rqueue.pop_front();
      s += "\n";
      
      if (send(sockfd, s.data(), s.size(), 0) != (ssize_t)s.size()) {
	break;
      }
    }
    
    server->unregister_connection(sockfd);
  }
  
  void startTcpServer(void) {
#ifdef _WIN32
    if (!WSA_initialized) {
      WORD version = MAKEWORD(2, 2);
      WSADATA data;
      if (WSAStartup(version, &data) != 0) {
	std::cerr << "WSAStartup() failure" << std::endl;
	return;
      }
      WSA_initialized = true;
    }
#endif
    
    struct sockaddr_in address;
    struct sockaddr client_address;
    socklen_t client_address_len = sizeof(client_address);
    int socket_fd;
    int new_socket_fd;
    int on = 1;
    
    memset(&address, 0, sizeof(struct sockaddr_in));
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = INADDR_ANY;
    
    if ((socket_fd = ::socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        return;
    }

    listening_socket_fd = socket_fd;
    
    setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    
    if (::bind(socket_fd, (const struct sockaddr*)&address, sizeof(struct sockaddr)) == -1) {
        perror("bind");
        close(socket_fd);
        return;
    }
    
    if (::listen(socket_fd, 20) == -1) {
        perror("listen");
        close(socket_fd);
        return;
    }

    while (!m_bDone) {
        new_socket_fd = ::accept(socket_fd, &client_address, &client_address_len);
        if (new_socket_fd == -1) {
            if (!m_bDone) perror("accept");
            continue;
        }
	
        std::string client_ip = get_client_ip(client_address);
        
        if (!accept_new_connection(client_ip)) {
	  std::cout << "Connection limit reached, rejecting client from " << client_ip << std::endl;
	  close(new_socket_fd);
	  continue;
        }
        
        register_connection(new_socket_fd, client_ip);
        setsockopt(new_socket_fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
        
        std::thread thr(tcpClientProcess, this, new_socket_fd);
        thr.detach();
    }
    
    close(socket_fd);
    
#ifdef _WIN32
    if (!WSA_shutdown) {
      WSACleanup();
      WSA_shutdown = true;
    }
#endif
  } 
};


class WebSocketThread
{
private:
  std::mutex ws_connections_mutex;
  std::map<std::string, uWS::WebSocket<false, true, WSPerSocketData>*> ws_connections;
  uWS::Loop *ws_loop = nullptr;

  std::vector<std::string> default_subscriptions_ = {
    "vstream/*",             // Plugin, source, recording status
  };

  std::map<std::string, int> rate_limits_ = {
    {"*/results", 10},       // High-frequency results: 10 Hz max
    {"*/frame_data", 10},    // Frame data: 10 Hz max
  };

  bool shouldSendToClient(WSPerSocketData* userData, const VstreamEvent& event) {
        // Critical events bypass all checks
        if (event.rate_limit_exempt) {
            return true;
        }
        
        // Check subscriptions
        bool subscribed = false;
        for (const auto& pattern : userData->subscriptions) {
            if (event.matchesPattern(pattern)) {
                subscribed = true;
                break;
            }
        }
        
        if (!subscribed) {
            return false;
        }
        
        // Check rate limits
        auto now = std::chrono::steady_clock::now();
        
        // Reset rate window every second
        if (std::chrono::duration_cast<std::chrono::seconds>(
                now - userData->rate_window_start).count() >= 1) {
            userData->event_counts.clear();
            userData->rate_window_start = now;
        }
        
        // Find matching rate limit
        int rate_limit = rate_limits_["*"];  // Default
        for (const auto& [pattern, limit] : rate_limits_) {
            if (event.matchesPattern(pattern)) {
                rate_limit = limit;
                break;
            }
        }
        
        // Check if we've exceeded the rate limit
        int& count = userData->event_counts[event.type];
        if (count >= rate_limit) {
            return false;  // Drop this event
        }
        
        count++;
        return true;
    }
  
public:
  bool m_bDone;
  int port;
  int listening_socket_fd;
  
  WebSocketThread() : m_bDone(false), port(8080), listening_socket_fd(-1) {}
  
  void shutdown() {
    m_bDone = true;
    // uWebsockets handles cleanup internally
  }

void broadcastEvent(const VstreamEvent& event) {
    std::lock_guard<std::mutex> lock(ws_connections_mutex);
    
    if (ws_connections.empty()) {
        return;
    }
    
    // Build JSON event message once
    json_t* event_msg = json_object();
    json_object_set_new(event_msg, "type", json_string("event"));
    json_object_set_new(event_msg, "event", json_string(event.type.c_str()));
    
    // Convert EventData to JSON
    json_t* data_json = nullptr;
    
    switch (event.data.type) {
        case VstreamEventDataType::NONE:
            data_json = json_null();
            break;
            
        case VstreamEventDataType::STRING:
            data_json = json_string(event.data.asString().c_str());
            break;
            
        case VstreamEventDataType::INTEGER:
            data_json = json_integer(event.data.asInt());
            break;
            
        case VstreamEventDataType::FLOAT:
            data_json = json_real(event.data.asFloat());
            break;
            
        case VstreamEventDataType::INT_ARRAY: {
            data_json = json_array();
            for (int64_t val : event.data.asIntArray()) {
                json_array_append_new(data_json, json_integer(val));
            }
            break;
        }
        
        case VstreamEventDataType::FLOAT_ARRAY: {
            data_json = json_array();
            for (double val : event.data.asFloatArray()) {
                json_array_append_new(data_json, json_real(val));
            }
            break;
        }
        
        case VstreamEventDataType::KEY_VALUE: {
            data_json = json_object();
            for (const auto& [key, value] : event.data.asKeyValue()) {
                json_object_set_new(data_json, key.c_str(), json_string(value.c_str()));
            }
            break;
        }
        
        case VstreamEventDataType::BINARY:
            // For now, skip binary in broadcast (could add base64 later)
            data_json = json_string("[binary data]");
            break;
    }
    
    json_object_set_new(event_msg, "data", data_json);
    json_object_set_new(event_msg, "data_type", 
                       json_string(eventDataTypeToString(event.data.type)));
    
    if (!event.source.empty()) {
        json_object_set_new(event_msg, "source", json_string(event.source.c_str()));
    }
    
    char* msg_str = json_dumps(event_msg, 0);
    std::string message(msg_str);
    free(msg_str);
    json_decref(event_msg);
    
    // Send to subscribed clients only
    for (auto& [name, conn] : ws_connections) {
        WSPerSocketData* userData = (WSPerSocketData*)conn->getUserData();
        
        if (!userData || !shouldSendToClient(userData, event)) {
            continue;
        }
        
        try {
            conn->send(message, uWS::OpCode::TEXT);
        } catch (...) {
            // Client may have disconnected
        }
    }
}
  
  void startWebSocketServer(void) {
    ws_loop = uWS::Loop::get();
    
    auto app = uWS::App();
    
    // Serve your embedded HTML
    app.get("/", [](auto *res, auto *req) {
      res->writeHeader("Content-Type", "text/html; charset=utf-8")
         ->writeHeader("Cache-Control", "no-cache")
         ->end(embedded::interface_html);
    });
    
    app.get("/terminal", [](auto *res, auto *req) {
      res->writeHeader("Content-Type", "text/html; charset=utf-8")
         ->writeHeader("Cache-Control", "no-cache")
         ->end(embedded::terminal_html);
    });
    
    // Health check
    app.get("/health", [](auto *res, auto *req) {
      res->writeHeader("Content-Type", "application/json")
         ->end("{\"status\":\"ok\",\"service\":\"videostream\"}");
    });

    app.get("/api/plugins", [](auto *res, auto *req) {
      json_t* plugins_array = json_array();
      
      auto plugin_names = g_pluginRegistry.listPlugins();
      for (const auto& name : plugin_names) {
        auto* plugin = g_pluginRegistry.getPlugin(name);
        if (plugin && plugin->hasWebUI()) {
          json_t* plugin_obj = json_object();
          json_object_set_new(plugin_obj, "name", json_string(name.c_str()));
          json_object_set_new(plugin_obj, "version", json_string(plugin->getVersion()));
          json_object_set_new(plugin_obj, "description", json_string(plugin->getDescription()));
          json_object_set_new(plugin_obj, "html", json_string(plugin->getUIHTML().c_str()));
          json_object_set_new(plugin_obj, "script", json_string(plugin->getUIScript().c_str()));
          json_object_set_new(plugin_obj, "style", json_string(plugin->getUIStyle().c_str()));
          json_array_append_new(plugins_array, plugin_obj);
        }
      }
      
      char* json_str = json_dumps(plugins_array, JSON_COMPACT);
      std::string response(json_str);
      free(json_str);
      json_decref(plugins_array);
      
      res->writeHeader("Content-Type", "application/json")
         ->writeHeader("Cache-Control", "no-cache")
         ->end(response);
    });
    
    // WebSocket endpoint
    app.ws<WSPerSocketData>("/ws", {
      .compression = uWS::SHARED_COMPRESSOR,
      .maxPayloadLength = 16 * 1024 * 1024,
      .idleTimeout = 120,
      .maxBackpressure = 16 * 1024 * 1024,
      
      .upgrade = [](auto *res, auto *req, auto *context) {
        res->template upgrade<WSPerSocketData>({
          .rqueue = new SharedQueue<std::string>(),
          .client_name = "",
          .subscriptions = std::vector<std::string>()
        }, 
        req->getHeader("sec-websocket-key"),
        req->getHeader("sec-websocket-protocol"),
        req->getHeader("sec-websocket-extensions"),
        context);
      },

.open = [this](auto *ws) {
    WSPerSocketData *userData = (WSPerSocketData *) ws->getUserData();
    
    if (!userData || !userData->rqueue) {
        std::cerr << "ERROR: Invalid userData in WebSocket open" << std::endl;
        ws->close();
        return;
    }
    
    // Create unique client name
    char client_id[32];
    snprintf(client_id, sizeof(client_id), "ws_%p", (void*)ws);
    userData->client_name = std::string(client_id);
    
    // Set default subscriptions
    userData->subscriptions = this->default_subscriptions_;
    userData->rate_window_start = std::chrono::steady_clock::now();
    
    // Store connection for broadcasting
    {
        std::lock_guard<std::mutex> lock(this->ws_connections_mutex);
        this->ws_connections[userData->client_name] = ws;
    }
    
    std::cout << "WebSocket client connected: " << userData->client_name << std::endl;
    
    // Send welcome with subscription info
    json_t *welcome = json_object();
    json_object_set_new(welcome, "type", json_string("welcome"));
    json_object_set_new(welcome, "client_id", json_string(userData->client_name.c_str()));
    
    json_t *subs_array = json_array();
    for (const auto& sub : userData->subscriptions) {
        json_array_append_new(subs_array, json_string(sub.c_str()));
    }
    json_object_set_new(welcome, "subscriptions", subs_array);
    
    char *welcome_str = json_dumps(welcome, 0);
    ws->send(welcome_str, uWS::OpCode::TEXT);
    free(welcome_str);
    json_decref(welcome);
    
    // Fire event for Tcl
    fireEvent(VstreamEvent("vstream/client_connected",userData->client_name));
 },	
	
	.message = [this](auto *ws, std::string_view message, uWS::OpCode opCode) {
        WSPerSocketData *userData = (WSPerSocketData *) ws->getUserData();
        
        if (!userData || !userData->rqueue) {
          std::cerr << "ERROR: Invalid userData in message handler" << std::endl;
          ws->close();
          return;
        }
        
        // Handle JSON protocol for web clients
        if (message.length() > 0 && message[0] == '{') {
          std::string json_str(message.data(), message.length());
          
          json_error_t error;
          json_t *root = json_loads(json_str.c_str(), 0, &error);
          
          if (!root) {
            json_t *error_response = json_object();
            json_object_set_new(error_response, "error", json_string("Invalid JSON"));
            char *error_str = json_dumps(error_response, 0);
            ws->send(error_str, uWS::OpCode::TEXT);
            free(error_str);
            json_decref(error_response);
            return;
          }
          
          json_t *cmd_obj = json_object_get(root, "cmd");
          if (!cmd_obj || !json_is_string(cmd_obj)) {
            json_t *error_response = json_object();
            json_object_set_new(error_response, "error",
				json_string("Missing 'cmd' field"));
            char *error_str = json_dumps(error_response, 0);
            ws->send(error_str, uWS::OpCode::TEXT);
            free(error_str);
            json_decref(error_response);
            json_decref(root);
            return;
          }
          
          const char *cmd = json_string_value(cmd_obj);

        if (strcmp(cmd, "subscribe") == 0) {
            json_t *topics_obj = json_object_get(root, "topics");
            if (topics_obj && json_is_array(topics_obj)) {
                size_t index;
                json_t *value;
                
                json_array_foreach(topics_obj, index, value) {
                    if (json_is_string(value)) {
                        std::string topic = json_string_value(value);
                        
                        // Add if not already subscribed
                        if (std::find(userData->subscriptions.begin(), 
                                     userData->subscriptions.end(), 
                                     topic) == userData->subscriptions.end()) {
                            userData->subscriptions.push_back(topic);
                        }
                    }
                }
                
                json_t *response = json_object();
                json_object_set_new(response, "status", json_string("ok"));
                json_object_set_new(response, "subscriptions", 
                                   json_integer(userData->subscriptions.size()));
                char *response_str = json_dumps(response, 0);
                ws->send(response_str, uWS::OpCode::TEXT);
                free(response_str);
                json_decref(response);
            }
        }
        else if (strcmp(cmd, "unsubscribe") == 0) {
            json_t *topics_obj = json_object_get(root, "topics");
            if (topics_obj && json_is_array(topics_obj)) {
                size_t index;
                json_t *value;
                
                json_array_foreach(topics_obj, index, value) {
                    if (json_is_string(value)) {
                        std::string topic = json_string_value(value);
                        userData->subscriptions.erase(
                            std::remove(userData->subscriptions.begin(),
                                       userData->subscriptions.end(),
                                       topic),
                            userData->subscriptions.end());
                    }
                }
                
                json_t *response = json_object();
                json_object_set_new(response, "status", json_string("ok"));
                char *response_str = json_dumps(response, 0);
                ws->send(response_str, uWS::OpCode::TEXT);
                free(response_str);
                json_decref(response);
            }
        }
        else if (strcmp(cmd, "list_subscriptions") == 0) {
            json_t *subs_array = json_array();
            for (const auto& sub : userData->subscriptions) {
                json_array_append_new(subs_array, json_string(sub.c_str()));
            }
            
            json_t *response = json_object();
            json_object_set_new(response, "status", json_string("ok"));
            json_object_set_new(response, "subscriptions", subs_array);
            char *response_str = json_dumps(response, 0);
            ws->send(response_str, uWS::OpCode::TEXT);
            free(response_str);
            json_decref(response);
        }	  
	else if (strcmp(cmd, "eval") == 0) {
            // Handle Tcl script evaluation
            json_t *script_obj = json_object_get(root, "script");
            json_t *requestId_obj = json_object_get(root, "requestId");
            
            if (script_obj && json_is_string(script_obj)) {
              const char *script = json_string_value(script_obj);
              
              // Use thread-safe tcl_eval function
              std::string result;
              int retcode = tcl_eval(std::string(script), result);
              
              // Create JSON response
              json_t *response = json_object();
              if (retcode != TCL_OK) {
                json_object_set_new(response, "status", json_string("error"));
                json_object_set_new(response, "error", json_string(result.c_str()));
              } else {
                json_object_set_new(response, "status", json_string("ok"));
                json_object_set_new(response, "result", json_string(result.c_str()));
              }
              
              // If a requestId was provided, include it in the response
              if (requestId_obj && json_is_string(requestId_obj)) {
                json_object_set(response, "requestId", requestId_obj);
              }
              
              char *response_str = json_dumps(response, 0);
              ws->send(response_str, uWS::OpCode::TEXT);
              free(response_str);
              json_decref(response);
            }
          }
          
          json_decref(root);
        }
        else {
          // Handle legacy text protocol (newline-terminated commands)
          std::string script(message);
          
          // Remove trailing newline if present
          if (!script.empty() && script.back() == '\n') {
            script.pop_back();
          }
          
          // Use thread-safe tcl_eval
          std::string result;
          tcl_eval(script, result);
          
          // For text protocol, send plain response
          ws->send(result, uWS::OpCode::TEXT);
        }
      },
      
      .drain = [](auto *ws) {
        if (ws->getBufferedAmount() > 1024 * 1024) {
          ws->close();
        }
      },
      
      .close = [this](auto *ws, int code, std::string_view message) {
        WSPerSocketData *userData = (WSPerSocketData *) ws->getUserData();
        
        if (!userData) return;
        
        // Remove from connections map
        {
          std::lock_guard<std::mutex> lock(this->ws_connections_mutex);
          this->ws_connections.erase(userData->client_name);
        }
        
        // Cleanup
        delete userData->rqueue;
        userData->rqueue = nullptr;
        
        std::cout << "WebSocket client disconnected: " << userData->client_name << std::endl;
      }
    })
    .listen(port, [this](auto *listen_socket) {
      if (listen_socket) {
        std::cout << "WebSocket server listening on port " << port << std::endl;
        std::cout << "Open http://localhost:" << port << "/ in your browser" << std::endl;
      } else {
        std::cerr << "Failed to start WebSocket server on port " << port << std::endl;
      }
    }).run();
  }
};

void fireEvent(const VstreamEvent& event) {
    if (events_ready.load()) {
        event_queue.push_back(event);
        
        // Broadcast to WebSocket clients
        if (g_wsServer) {
            g_wsServer->broadcastEvent(event);
        }
    }
}

class ProcessThread
{
  int overwrite;
  int fourcc;
  std::string output_file;

  std::atomic<bool> openfile;
  std::atomic<bool> closefile;

  bool just_opened;
  bool annotate;
  int frame_count;
  int prev_fr;
  int start_obs;
  int end_obs;
  bool frame_in_obs;

  int sfd = -1;
  struct sockaddr_un svaddr;

  int push_next_frame = -1;

  VideoWriter video;
  DYN_GROUP *dg;
  DYN_LIST *ids, *obs_starts, *obs_stops;
  DYN_LIST *frame_ids, *frame_timestamps, *frame_systemtimes, *frame_linestatus;

  int64_t on_frameID;
  int64_t on_frameTimestamp;
  int64_t last_stored_cam_us = 0;  // camera clock (us) of the last stored frame
  std::chrono::high_resolution_clock::time_point on_systemTimestamp;

  StorageManager storage_manager_;
  bool use_sqlite_;
  std::string metadata_base_name_;
  
  std::atomic<bool> metadata_only_;
  std::atomic<bool> file_opened_;
  std::atomic<bool> recording_active_;
  
  bool exists (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
      fclose(file);
      return true;
    } else {
      return false;
    }
  }

public:
  // Monotonic count of real frames fully processed+stored (excludes sentinels).
  // Used by the capture loop's serial-reprocess barrier.
  std::atomic<long> frames_done_{0};

  ProcessThread()
  {
    int j;
    openfile = false;
    closefile = false;
    overwrite = false;
    just_opened = false;
    only_save_in_obs = true;
    annotate = true;
    file_opened_ = false;
    recording_active_ = false;
    use_sqlite_ = true;  // Set to false to use DYN_GROUP format
    metadata_only_ = false;
    
    dg = dfuCreateDynGroup(6);
    
    j = dfuAddDynGroupNewList(dg, (char *) "ids", DF_LONG, 500);
    ids = DYN_GROUP_LIST(dg, j);
    
    j = dfuAddDynGroupNewList(dg, (char *) "frame_ids", DF_LONG, 500);
    frame_ids = DYN_GROUP_LIST(dg, j);

    j = dfuAddDynGroupNewList(dg, (char *) "frame_times", DF_LONG, 500);
    frame_timestamps = DYN_GROUP_LIST(dg, j);

    j = dfuAddDynGroupNewList(dg, (char *) "frame_systemtimes", DF_LONG, 500);
    frame_systemtimes = DYN_GROUP_LIST(dg, j);
    
    j = dfuAddDynGroupNewList(dg, (char *) "frame_linestatus", DF_CHAR, 500);
    frame_linestatus = DYN_GROUP_LIST(dg, j);

    j = dfuAddDynGroupNewList(dg, (char *) "obs_starts", DF_LONG, 50);
    obs_starts = DYN_GROUP_LIST(dg, j);

    j = dfuAddDynGroupNewList(dg, (char *) "obs_stops", DF_LONG, 50);
    obs_stops = DYN_GROUP_LIST(dg, j);

    //    fourcc = cv::VideoWriter::fourcc('F','F','V','1');
    //    fourcc = cv::VideoWriter::fourcc('X','V','I','D');
    fourcc = cv::VideoWriter::fourcc('a','v','c','1');    
  }

  int getFrameCount(void) {
    return frame_count;
  }

  bool getUseSQLite() const { return use_sqlite_; }

  // this is for storage to video file
  void annotate_process_frame(Mat frame, bool in_obs)
  {
    // Draw obs sync indicator square
    int square_size = 15;
    cv::Point square_top_left(5, 5);
    cv::Point square_bottom_right(5 + square_size, 5 + square_size);
    
    cv::Scalar square_color = in_obs ? cv::Scalar(255, 255, 255) : cv::Scalar(20, 20, 20);
    cv::rectangle(frame, square_top_left, square_bottom_right, square_color, cv::FILLED);
    
    // Draw frame number
    cv::putText(frame,
		std::to_string(frame_count),
		cv::Point(25 + square_size, 20),
		cv::FONT_HERSHEY_SIMPLEX,
		0.7,
		cv::Scalar(20, 20, 20));
  }

  bool fileIsOpen(void) {
    return openfile;
  }
  
  std::string currentFile(void) {
    return output_file;
  }

  int setFourCC(char *s)
  {
    int oldfourcc = fourcc;
    if (strlen(s) != 4)
      return -1;
    fourcc = cv::VideoWriter::fourcc(s[0], s[1], s[2], s[3]);
    return oldfourcc;
  }
  
  bool closeFile(void)
  {
    if (!openfile.load(std::memory_order_acquire)) return false;
    
    // Push sentinel to wake up the process thread
    process_queue.push_back(FRAME_SENTINEL_CLOSE);
    
    return true;
  }
  
  bool openFile(char *filename)
  {
    if (openfile) return false;
    obs_count = -1;
    frame_count = 0;
    output_file = std::string(filename);
    metadata_only_ = false;    

    // Push sentinel to the process thread to signal open
    process_queue.push_back(FRAME_SENTINEL_OPEN);
       
    return true;
  }

  void startRecording() { storage_manager_.startRecording(); }
  void stopRecording() { storage_manager_.stopRecording(); }

  bool openMetadataFile(const char *base_name, const char *source_video)
  {
    if (openfile.load()) return false;

    output_file = std::string(source_video);  // Reference to source
    metadata_base_name_ = std::string(base_name);
    metadata_only_ = true;

    obs_count = -1;
    frame_count = 0;

    process_queue.push_back(FRAME_SENTINEL_OPEN);
    
    return true;
  }

  bool isMetadataOnly() const { 
    return metadata_only_; 
  }

  void setUseSQLite(bool enable) { 
    use_sqlite_ = enable; 
  }

 bool doOpenFile(void)
  {
    if (openfile) return 0;
    
    if (!metadata_only_) {
      // Check if video parameters are valid
      int fw = frame_width.load();
      int fh = frame_height.load();
      
      if (fw <= 0 || fh <= 0 || frame_rate <= 0) {
	std::cerr << "Cannot open video file: invalid dimensions or frame rate ("
		  << fw << "x" << fh << " @ " << frame_rate << " fps)" << std::endl;
	return false;
      }

      std::cout << "opening video file" << output_file << std::endl;
      // Check for overwrite
      if (exists(output_file) && overwrite) {
	std::remove(output_file.c_str());
      }
      
      // FFmpeg explicitly: with the default backend order a failed open
      // (bad path, unsupported codec) falls back to GStreamer, whose plugins
      // may drag Qt into the recording thread -- fatal on builds whose
      // highgui is Qt-based (Debian), where the display thread owns Qt.
      video = VideoWriter(output_file,
			  cv::CAP_FFMPEG,
			  fourcc,
			  frame_rate,
			  Size(fw, fh),
			  is_color);

      if (!video.isOpened()) {
	std::cerr << "Failed to open video file for writing: " << output_file
		  << " (" << fw << "x" << fh << " @ " << frame_rate
		  << " fps; check that the folder exists and the codec is available)"
		  << std::endl;
	return false;
      }
    }
    
    // Open storage
    if (use_sqlite_) {
      RecordingMetadata metadata;
      metadata.filename = output_file;
      metadata.start_time = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      metadata.frame_rate = frame_rate;
      metadata.width = frame_width;
      metadata.height = frame_height;
      metadata.is_color = is_color;
      metadata.obs_source = obsSourceName();
      metadata.camera_clock_offset_us = obs_clock_offset_known ? obs_clock_offset_us : 0;

      // Convert fourcc to string
      char codec_str[5];
      codec_str[0] = (fourcc & 0xFF);
      codec_str[1] = ((fourcc >> 8) & 0xFF);
      codec_str[2] = ((fourcc >> 16) & 0xFF);
      codec_str[3] = ((fourcc >> 24) & 0xFF);
      codec_str[4] = '\0';
      metadata.codec = std::string(codec_str);

      bool storage_ok;
      if (metadata_only_) {
	storage_ok = storage_manager_.openMetadataOnly(
						       metadata_base_name_, output_file, metadata);
	
	if (storage_ok) {
	  std::cout << "Opened metadata-only recording: " << metadata_base_name_ << ".db" << std::endl;
	  std::cout << "Source video: " << output_file << std::endl;
	  
	  // Fire event with data
	  std::string data = "file " + metadata_base_name_ + ".db source " + output_file;
	  fireEvent(VstreamEvent("vstream/metadata_recording_started", data));
	}
      } else {
	storage_ok = storage_manager_.openRecording(output_file, metadata);

	if (storage_ok) {
	  // Fire event for normal recording
	  fireEvent(VstreamEvent("vstream/recording_started", "file " + output_file));
	}
      }
      
      if (!storage_ok) {
	std::cerr << "Failed to open SQLite storage" << std::endl;
	if (!metadata_only_) video.release();
	return false;
      }


      // Record the camera configuration (FLIR or Lucid) with the session
      if (g_frameSource) {
	ICameraControl* cam = dynamic_cast<ICameraControl*>(g_frameSource);
	if (cam) {
	  CameraSettings cam_settings;
	  cam_settings.binning_horizontal = cam->getBinningH();
	  cam_settings.binning_vertical = cam->getBinningV();
	  cam_settings.roi_offset_x = cam->getOffsetX();
	  cam_settings.roi_offset_y = cam->getOffsetY();
	  cam_settings.roi_width = cam->getWidth();
	  cam_settings.roi_height = cam->getHeight();
	  cam_settings.exposure_time = cam->getExposureTime();
	  cam_settings.frame_rate = cam->getFrameRate();
	  cam_settings.gain = cam->getGain();
	  cam_settings.pixel_format = "Mono8";  // both sources deliver Mono8

	  if (!storage_manager_.storeCameraSettings(cam_settings)) {
	    std::cerr << "Warning: Failed to store camera settings" << std::endl;
	  }
	}
      }
      
      // allow plugins to store data in our db
      storage_manager_.initializePluginStorage();
      storage_manager_.beginPluginStorageBatch();
      storage_manager_.startRecording();      
    }
    else {
      // Old DYN_GROUP format
      dfuResetDynList(ids);
      dfuResetDynList(obs_starts);
      dfuResetDynList(obs_stops);
      dfuResetDynList(frame_ids);
      dfuResetDynList(frame_timestamps);
      dfuResetDynList(frame_systemtimes);
      dfuResetDynList(frame_linestatus);
    }
    
    openfile = true;
    return true;
  }  

  string getFileName(const string& s) {
    
    char sep = '/';
   
#ifdef _WIN32
    sep = '\\';
#endif
   
    size_t i = s.rfind(sep, s.length());
    if (i != string::npos) {
      return(s.substr(i+1, s.length() - i));
    }
   
    return(s);
  }
  
  bool writeFrameDG(DYN_GROUP *dg, std::string filename)
  {
    dgInitBuffer();
    strncpy(DYN_GROUP_NAME(dg),
	    getFileName(filename).c_str(),
	    DYN_GROUP_NAME_SIZE); 
    dgRecordDynGroup(dg);
    if (!dgWriteBufferCompressed((char *) (output_file+".dgz").c_str())) {
      return false;
    }
    dgCloseBuffer();
    return true;
  }

  bool openDomainSocket(char *path)
  {
    if (sfd >= 0) close(sfd);

    sfd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if(sfd == -1) return false;

    memset(&svaddr,0,sizeof(struct sockaddr_un));
    svaddr.sun_family = AF_UNIX;
    strncpy(svaddr.sun_path, path, strlen(path));

    if (::connect(sfd, (struct sockaddr *) &svaddr,
		  sizeof(struct sockaddr_un)) == -1) {
      close(sfd);
      sfd = -1;
      return false;
    }
    return true;
  }
  
  int sendToDomainSocket(Mat *m)
  {
    if (sfd >= 0) {
      char buf[32];
      uint32_t header[4];
      header[0] = m->total() * m->elemSize();
      header[1] = m->rows;
      header[2] = m->cols;
      header[3] = m->type();
      int msgLen = sizeof(header);

      if (send(sfd, header, msgLen, 0) != msgLen) {
	fprintf(stderr, "error writing header to domain socket\n");
	return -1;
      }

      if (send(sfd, m->data, header[0], 0) != header[0]) {
        fprintf(stderr, "error writing frame to domain socket\n");
	return -1;
      }
      
      return 1;
    }
    return 0;
  }

  bool closeDomainSocket(void)
  {
    if (sfd >= 0) {
      close(sfd);
      sfd = -1;
      return true;
    }
    return false;
  }
  
  int setNSendDomainSocket(int n)
  {
    int last = push_next_frame;
    push_next_frame = n;
    return last;
  }

  void doCloseFile() {
    if (!openfile.load()) return;  // Already closed
    
    if (!metadata_only_) {
      video.release();
    }
    
    if (use_sqlite_) {
      storage_manager_.endPluginStorageBatch();
      storage_manager_.closeRecording();
      
      if (metadata_only_) {
	std::string data = "file " + metadata_base_name_ + ".db";
	fireEvent(VstreamEvent("vstream/metadata_recording_closed", data));
      } else {
	fireEvent(VstreamEvent("vstream/recording_closed", "file " + output_file));
      }
    } else {
      writeFrameDG(dg, output_file);
    }
    
    openfile.store(false, std::memory_order_release);
  }
  
  void startProcessThread(void)
  {
    while (1) {
      do {
	processFrame = process_queue.front();
	process_queue.pop_front();

	// Handle open sentinel - user requested 
	if (processFrame == FRAME_SENTINEL_OPEN) {
	    doOpenFile();
	    just_opened = true;
	  break;
	}

	// Handle close sentinel - user requested close
	if (processFrame == FRAME_SENTINEL_CLOSE) {
	  doCloseFile();
	  break;
	}

	if (processFrame == FRAME_SENTINEL_END || processFrame >= nFrames) {
	  doCloseFile();
	  return;  // Exit thread entirely
	}

    
	if (just_opened) {
	  just_opened = false;
	  // Get initial frame metadata
	  FrameMetadata init_metadata;
	  bool init_in_obs;
	  cv::Mat temp_frame;
	  if (frameBufferManager.copyFrame(processFrame, temp_frame, init_metadata, init_in_obs)) {
	    on_frameID = init_metadata.frameID;
	    on_frameTimestamp = init_metadata.timestamp;
	    on_systemTimestamp = init_metadata.systemTime;
	  }

	  if (g_pluginRegistry.hasPlugins()) {
	    g_pluginRegistry.fileOpenAll(output_file);
	  }	  
	}
	
	// Domain socket push (for external access to frames)
	if (push_next_frame != 0) {
	  auto access = frameBufferManager.accessFrame(processFrame);
	  if (access.isValid()) {
	    sendToDomainSocket(const_cast<Mat*>(access.frame));
	  }
	}
	if (push_next_frame > 0) push_next_frame--;
	
	if (openfile.load()) {
	  if (processFrame)
	    prev_fr = (processFrame-1);
	  else prev_fr = nFrames-1;

	  // Copy the frame first: its camera timestamp also stamps the obs
	  // boundaries below (absolute camera clock; PTP epoch when synced)
	  cv::Mat frame_copy;
	  FrameMetadata frame_metadata;
	  bool copy_in_obs = false;
	  bool have_frame = frameBufferManager.copyFrame(processFrame, frame_copy,
							 frame_metadata, copy_in_obs);
	  int64_t cam_us = have_frame ? frame_metadata.timestamp / 1000 : 0;

	  // Check obs period boundaries
	  auto obs =
	    frameBufferManager.getObservationPair(processFrame, prev_fr);

	  if (!obs.cur_valid || !obs.prev_valid) {
	    // Can't check boundaries without both frames
	    start_obs = false;
	    end_obs = false;
	    frame_in_obs = false;
	  } else {
	    if (obs.cur_valid) frame_in_obs = true; // used for annotation
	    // Obs period start
	    start_obs = (obs.cur_in_obs && !obs.prev_in_obs);
	    if (start_obs) {
	      if (use_sqlite_) {
		storage_manager_.storeObservationStart(frame_count, cam_us);
	      } else {
		dfuAddDynListLong(obs_starts, frame_count);
	      }
	    }
	    
	    // Obs period end
	    end_obs = (!obs.cur_in_obs && obs.prev_in_obs);
	    if (end_obs) {
	      if (use_sqlite_) {
		storage_manager_.storeObservationEnd(frame_count - 1, last_stored_cam_us);
	      } else {
		dfuAddDynListLong(obs_stops, frame_count - 1);
	      }
	    }
	  }	  
	  
	  // (frame_copy / frame_metadata were fetched above; this local shadows
	  // the annotation member set in the obs block, as it always did)
	  bool frame_in_obs = copy_in_obs;

	  if (have_frame) {
	    if (!only_save_in_obs || (only_save_in_obs && frame_in_obs)) {

	      // Write video frame (unless metadata-only mode)
	      if (!metadata_only_) {
		if (annotate) annotate_process_frame(frame_copy, frame_in_obs);
		video.write(frame_copy);
	      }
	      
	      // Store frame metadata
	      if (use_sqlite_) {
		storage_manager_.setObsState(frame_in_obs);

		FrameData fdata;
		fdata.frame_number = frame_count;
		fdata.obs_id = storage_manager_.getCurrentObsId();
		fdata.frame_id = frame_metadata.frameID;
		fdata.relative_frame_id = (int)(frame_metadata.frameID - on_frameID);
		fdata.timestamp_us = (frame_metadata.timestamp - on_frameTimestamp) / 1000;
		fdata.system_time_us =
		  std::chrono::duration_cast<std::chrono::microseconds>(
									frame_metadata.systemTime - on_systemTimestamp).count();
		fdata.line_status = frame_metadata.lineStatus;
		fdata.camera_time_us = cam_us;

		storage_manager_.storeFrame(fdata);
		storage_manager_.storeFrameWithPlugins(frame_count, prev_fr);
	      }
	      else {
		// DYN_GROUP format
		dfuAddDynListLong(ids, frame_count);
		dfuAddDynListLong(frame_ids,
				  (int) (frame_metadata.frameID - on_frameID));
		
		int64_t ns = frame_metadata.timestamp - on_frameTimestamp; 
		dfuAddDynListLong(frame_timestamps, (int) (ns/1000));
		
		int elapsed =
		  std::chrono::duration_cast<std::chrono::microseconds>(
									frame_metadata.systemTime - on_systemTimestamp).count();
		dfuAddDynListLong(frame_systemtimes, elapsed);
		
		dfuAddDynListChar(frame_linestatus,
				  (unsigned char) frame_metadata.lineStatus);
	      }

	      last_stored_cam_us = cam_us;
	      frame_count++;
	    }
	  }
	}
	// Signal the serial-reprocess barrier that this frame is fully handled.
	frames_done_.fetch_add(1, std::memory_order_release);
      } while (process_queue.size());
    }
  }
};
  
ProcessThread processThread;

int open_videoFile(char *filename)
{
  
  return processThread.openFile(filename);
}

int close_videoFile(void)
{
  return processThread.closeFile();
}

void start_recording()
{
   processThread.startRecording();
}

void stop_recording()
{
  processThread.stopRecording();
}

int open_metadataFile(const char *base_name, const char *source_video)
{
  return processThread.openMetadataFile(base_name, source_video);
}

int is_metadataOnly(void)
{
  return processThread.isMetadataOnly() ? 1 : 0;
}

void set_useSQLite(int enable)
{
  processThread.setUseSQLite(enable != 0);
}

int get_useSQLite(void)
{
  return processThread.getUseSQLite() ? 1 : 0;
}

int open_domainSocket(char *socket_path)
{
  return processThread.openDomainSocket(socket_path);
}

int close_domainSocket(void)
{
  return processThread.closeDomainSocket();
}

int sendn_domainSocket(int n)
{
  return processThread.setNSendDomainSocket(n);
}

int set_inObs(int status)
{
  int old = in_obs;
  if (status == 0 || status == 1)
    in_obs = status;
  if (!old && status) {
    fireEvent(VstreamEvent("vstream/begin_obs"));
    obs_count++;
  } else if (old && !status) {
    fireEvent(VstreamEvent("vstream/end_obs"));
  }
  return old;
}

const char* obsSourceName()
{
  switch (obs_source.load()) {
  case OBS_SOURCE_DSERV: return "dserv";
  case OBS_SOURCE_TIMESTAMP: return "timestamp";
  default: return "line";
  }
}

// -1 on an unknown name. Pending timestamp transitions are dropped on a change.
int set_obsSource(const char* name)
{
  int mode;
  if (!strcmp(name, "line")) mode = OBS_SOURCE_LINE;
  else if (!strcmp(name, "dserv")) mode = OBS_SOURCE_DSERV;
  else if (!strcmp(name, "timestamp")) mode = OBS_SOURCE_TIMESTAMP;
  else return -1;
  obs_source = mode;
  pending_obs_transitions.clear();
  timestamp_obs_state = false;
  timestamp_obs_warned = false;
  if (!obs_clock_offset_manual) obs_clock_offset_known = false;
  return mode;
}

// vstream::obsClockOffset: seconds (camera minus dserv clock), or auto
int64_t get_obsClockOffset_us() { return obs_clock_offset_us; }
bool obsClockOffsetKnown() { return obs_clock_offset_known; }
void set_obsClockOffset_auto()
{
  obs_clock_offset_manual = false;
  obs_clock_offset_known = false;
}
void set_obsClockOffset_us(int64_t us)
{
  obs_clock_offset_us = us;
  obs_clock_offset_known = true;
  obs_clock_offset_manual = true;
}

// Learn the camera-minus-host clock offset from a frame: a PTP-synced camera
// is an integer number of seconds (the TAI-UTC offset) from a host on UTC; a
// free-running camera clock (uptime) is nowhere near and leaves it unknown.
static void learnObsClockOffset(const FrameMetadata& m, int64_t frame_us)
{
  if (obs_clock_offset_manual) return;
  int64_t host_us = std::chrono::duration_cast<std::chrono::microseconds>(
      m.systemTime.time_since_epoch()).count();
  int64_t delta_us = frame_us - host_us;
  int64_t whole_s = (delta_us >= 0 ? delta_us + 500000 : delta_us - 500000) / 1000000;
  bool epoch_clock = std::llabs(whole_s) <= 120 &&
    std::llabs(delta_us - whole_s * 1000000) < 300000;   // host NTP slack
  if (epoch_clock) {
    int64_t off = whole_s * 1000000;
    if (!obs_clock_offset_known || off != obs_clock_offset_us) {
      obs_clock_offset_us = off;
      obs_clock_offset_known = true;
      std::cout << "obsSource timestamp: camera clock is " << (whole_s >= 0 ? "+" : "")
		<< whole_s << " s from host UTC (PTP TAI offset); dserv datapoint "
		<< "times are shifted by that much" << std::endl;
    }
  } else if (obs_clock_offset_known) {
    obs_clock_offset_known = false;
    std::cerr << "obsSource timestamp: camera clock no longer epoch-based "
	      << "(PTP lost?); comparing datapoint times unshifted" << std::endl;
  }
}

// Obs state to stamp on a frame, per vstream::obsSource
static int obsStateForFrame(const FrameMetadata& m)
{
  switch (obs_source.load()) {
  case OBS_SOURCE_DSERV:
    return ds_in_obs;
  case OBS_SOURCE_TIMESTAMP: {
    int64_t frame_us = m.timestamp / 1000;
    learnObsClockOffset(m, frame_us);
    int64_t shift = obs_clock_offset_known ? obs_clock_offset_us : 0;
    while (!pending_obs_transitions.empty()) {
      const ObsTransition& t = pending_obs_transitions.front();
      int64_t due_us = t.time_us + shift;   // datapoint time on the camera clock
      if (t.time_us > 0 && due_us > frame_us) {
	// Not yet: unless it is absurdly far ahead, which means the two
	// clocks are unrelated (camera not PTP-synced) -- then apply now
	if (due_us - frame_us < 10LL * 1000000LL) break;
	if (!timestamp_obs_warned) {
	  std::cerr << "obsSource timestamp: ess/in_obs datapoint is "
		    << (due_us - frame_us) / 1000000.0 << " s ahead of the camera "
		    << "clock (camera not PTP-synced with dserv?); applying on arrival"
		    << std::endl;
	  timestamp_obs_warned = true;
	}
      }
      // due (or stale: a datapoint older than the frame just applies)
      timestamp_obs_state = t.state;
      pending_obs_transitions.pop_front();
    }
    return timestamp_obs_state;
  }
  default:
    return m.lineStatus;
  }
}

int set_onlySaveInObs(int status)
{
  int old = only_save_in_obs;
  if (status == 0 || status == 1)
    only_save_in_obs = status;
  return old;
}

int set_reprocessMode(int status)
{
  int old = g_reprocess_serial.load();
  if (status == 0 || status == 1)
    g_reprocess_serial.store(status != 0);
  return old;
}

int set_fourCC(char *str)
{
  return processThread.setFourCC(str);
}


static void updateWidgetVariables(int frame_idx)
{
  g_widgetManager.setVariable("frame_count", 
			      std::to_string(processThread.getFrameCount()));
  g_widgetManager.setVariable("obs_count", 
			      std::to_string(obs_count));
  g_widgetManager.setVariable("file", 
			      processThread.currentFile());
  g_widgetManager.setVariable("cur_frame", 
			      std::to_string(frame_idx));
  g_widgetManager.setVariable("fps", 
			      std::to_string(static_cast<int>(frame_rate)));
  g_widgetManager.setVariable("resolution", 
			      std::to_string(frame_width) + "x" + 
			      std::to_string(frame_height));
  
  // Add more variables as needed
  if (processThread.fileIsOpen()) {
    g_widgetManager.setVariable("recording", "1");
  } else {
    g_widgetManager.setVariable("recording", "0");
  }
}


// Defined below, after DisplayThread; declared here because the non-Apple
// display path inside the class calls it.
void note_displayed_frame(const cv::Mat& raw, int video_frame, bool in_obs);

class DisplayThread
{
  int show_frames;
  float scale;

public:
  static void onMouse(int event, int x, int y, int flags, void* userdata)
  {
    float* scale_ptr = static_cast<float*>(userdata);
    float scale = scale_ptr ? *scale_ptr : 1.0f;
    
    // Convert display coordinates back to original frame coordinates
    int orig_x = x / scale;
    int orig_y = y / scale;
    
    // Build Tcl command string based on event
    std::string tcl_cmd;
    
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (g_widgetManager.handleMouseDown(orig_x, orig_y)) {
            return;  // Widget handled it
        }
	
        // Determine modifier
        std::string modifier = "none";
        if (flags & cv::EVENT_FLAG_CTRLKEY) {
            modifier = "ctrl";
        } else if (flags & cv::EVENT_FLAG_SHIFTKEY) {
            modifier = "shift";
        } else if (flags & cv::EVENT_FLAG_ALTKEY) {
            modifier = "alt";
        }
        
        // Call Tcl proc: onMouseClick x y modifier
        tcl_cmd = "if {[info procs onMouseClick] ne \"\"} { onMouseClick " 
                  + std::to_string(orig_x) + " " 
                  + std::to_string(orig_y) + " " 
                  + modifier + " }";
    }                  
    else if (event == cv::EVENT_MOUSEMOVE) {
      g_widgetManager.handleMouseDrag(orig_x, orig_y);
    }
    else if (event == cv::EVENT_LBUTTONUP) {
      g_widgetManager.handleMouseUp(orig_x, orig_y);
    }
    else if (event == cv::EVENT_RBUTTONDOWN) {
        tcl_cmd = "if {[info procs onMouseRightClick] ne \"\"} { onMouseRightClick " 
                  + std::to_string(orig_x) + " " 
                  + std::to_string(orig_y) + " }";
                  
    } else if (event == cv::EVENT_MBUTTONDOWN) {
      tcl_cmd = "if {[info procs onMouseMiddleClick] ne \"\"} { onMouseMiddleClick " 
	+ std::to_string(orig_x) + " " 
	+ std::to_string(orig_y) + " }";
    }

    g_widgetManager.processPendingCallbacks();
 
    if (!tcl_cmd.empty()) {
      mouse_queue.push_back(tcl_cmd);
    }
  }
  
  DisplayThread()
  {
    show_frames = false;
  }

  float setScale(float s)
  {
    float old = s;
    scale = s;
    return old;
  }
  
  bool show(void)
  {
    bool old = show_frames;
    if (!old) display_queue.push_back(-2);
    show_frames = true;
    return old;
  }
  
  bool hide(void)
  {
    bool old = show_frames;
    show_frames = false;
    if (old) display_queue.push_back(-3);
    return old;
  }

  static void draw_plugin_overlays(Mat& frame, int frame_idx)
  {
    // Let each plugin draw its own overlay
    extern AnalysisPluginRegistry g_pluginRegistry;
    auto plugin_names = g_pluginRegistry.listPlugins();
    
    for (const auto& name : plugin_names) {
      auto* plugin = g_pluginRegistry.getPlugin(name);
      if (plugin) {
	plugin->drawOverlay(frame, frame_idx);
      }
    }
  }
  
  static void annotate_frame(Mat& frame, bool inobs, int frame_idx)
  {
    // Convert to color if needed for annotation
    if (frame.channels() == 1) {
        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    }
        
    // Update widget variables that might be used in UI
    updateWidgetVariables(frame_idx);
    
    // draw widgets that have been added to widget manager
    g_widgetManager.drawAll(frame);
    
    draw_plugin_overlays(frame, frame_idx);    
    
#if 0
    cv::putText(frame,
        "Frame: " + std::to_string(processThread.getFrameCount()),
        cv::Point(50,50),
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        cv::Scalar(20,20,20));
    
    if (processThread.fileIsOpen()) {
      cv::putText(frame,
          "File: " + processThread.currentFile() + "[" +
          std::to_string(obs_count) + "]",
          cv::Point(50,85),
          cv::FONT_HERSHEY_SIMPLEX,
          0.75,
          cv::Scalar(20,20,20));
    }
    if (inobs) {
      cv::circle(frame, cv::Point(40,80), 6, cv::Scalar(255,255,255), -1);
    }
#endif    
  }

  void startDisplayThread(void)
  {
    while (1) {
      do {
	displayFrame = display_queue.front();
	display_queue.pop_front();
      }
      while (displayFrame >= 0 && display_queue.size());
      
      if (displayFrame == -1 || displayFrame >= nFrames) {
	destroyAllWindows();
	break;
      }
      else if (displayFrame == -2) {
	namedWindow("Frame", WINDOW_AUTOSIZE);
	static float display_scale = scale;    
	cv::setMouseCallback("Frame", onMouse, &display_scale);
      }
      else if (displayFrame == -3) {
	destroyWindow("Frame");
      }
      else if (show_frames) {
	if (getWindowProperty("Frame", WND_PROP_AUTOSIZE) >= 0) {
	  auto access = frameBufferManager.accessFrame(displayFrame);
	  if (access.isValid()) {
	    Mat dframe;
	    if (scale != 1.0) {
	      cv::resize(*access.frame, dframe,
			 cv::Size(access.frame->cols * scale,
				  access.frame->rows * scale),
			 0, 0, cv::INTER_LINEAR);
	    }
	    else {
	      dframe = access.frame->clone();
	    }
	    // Lock released here when access goes out of scope
	    
	    annotate_frame(dframe, access.in_obs, displayFrame);
	    
	    imshow( "Frame", dframe );
	  }

	  int key = waitKeyEx(5);
	  if (key != -1) {
	    std::string callback = g_keyboardCallbacks.getCallback(key);
	    if (!callback.empty()) {
	      std::ostringstream cmd;
	      cmd << callback << " " << key;
	      mouse_queue.push_back(cmd.str());
	    } else {
	      // Handle built-in keys
	      switch (key) {
	      case 27:  // ESC
	      case 1048603:
		do_shutdown();
		break;
	      default:
		std::string generic = g_keyboardCallbacks.getCallback(-1);
		if (!generic.empty()) {
		  std::ostringstream cmd;
		  cmd << generic << " " << key;
		  mouse_queue.push_back(cmd.str());
		}
		break;
	      }
	    }
	  }	  
	}
	else {
	  show_frames = false;
	}
      }
    }
  }
};

DisplayThread displayThread;

// The frame the display last put on screen, kept unannotated so that a
// snapshot taken later - while paused, typically - composites the overlay onto
// exactly those pixels.
//
// Re-reading the ring buffer at curFrame instead does NOT work: curFrame is a
// slot index, and by the time a Tcl command runs, that slot holds the frame
// from a full lap earlier (nFrames back), while the overlay always reflects
// the most recent analysis. The two then disagree by the buffer depth, which
// on a moving eye shows up as an overlay that misses the pupil.
static std::mutex last_display_mutex;
static cv::Mat    last_display_frame;
static int        last_display_video_frame = -1;
static bool       last_display_in_obs = false;

void note_displayed_frame(const cv::Mat& raw, int video_frame, bool in_obs)
{
  std::lock_guard<std::mutex> lock(last_display_mutex);
  raw.copyTo(last_display_frame);
  last_display_video_frame = video_frame;
  last_display_in_obs = in_obs;
}

// Write one frame to an image file. For figures: the window has no export path
// of its own, and a screen grab gives you window scale and whatever the desktop
// does to colors rather than the pixels the detector saw.
//
// mode 0 (raw)   source pixels alone
// mode 1 (clean) detector overlay only - no sliders, buttons or status text,
//                which is what a figure panel wants
// mode 2 (full)  exactly what the display window shows
//
// Returns false if the frame is no longer in the ring buffer or the write
// fails (unwritable path, or an extension OpenCV has no encoder for).
bool save_frame_image(const std::string& path, int mode, int *frame_out)
{
  cv::Mat frame_copy;
  bool in_obs = false;
  int frame_idx = -1;

  {
    std::lock_guard<std::mutex> lock(last_display_mutex);
    if (last_display_frame.empty()) return false;
    last_display_frame.copyTo(frame_copy);
    frame_idx = last_display_video_frame;
    in_obs = last_display_in_obs;
  }
  if (frame_copy.empty()) return false;
  if (frame_out) *frame_out = frame_idx;

  if (mode == 2) {
    updateWidgetVariables(frame_idx);
    DisplayThread::annotate_frame(frame_copy, in_obs, frame_idx);
  } else if (mode == 1) {
    // The plugin half of annotate_frame, without g_widgetManager.drawAll.
    if (frame_copy.channels() == 1) {
      cv::cvtColor(frame_copy, frame_copy, cv::COLOR_GRAY2BGR);
    }
    DisplayThread::draw_plugin_overlays(frame_copy, frame_idx);
  }

  try {
    return cv::imwrite(path, frame_copy);
  } catch (const cv::Exception& e) {
    std::cerr << "saveFrame: " << e.what() << std::endl;
    return false;
  }
}

int show_display(proginfo_t *p)
{
  bool old = p->display;
#ifndef __APPLE__
  return (int) displayThread.show();
#endif
  p->display = 1;
  return old;
}

int hide_display(proginfo_t *p)
{
  bool old = p->display;
#ifndef __APPLE__
  return (int) displayThread.hide();
#endif
  p->display = 0;
  return old;
}

void add_shutdown_command(char *cmd)
{
  shutdown_queue.push_back(std::string(cmd));
  return;
}

int do_shutdown(void)
{
  done = true;
  return 1;
}

int Tcl_AppInit(Tcl_Interp *interp)
{
  if (Tcl_Init(interp) == TCL_ERROR) return TCL_ERROR;
  return TCL_OK;
}

int setupTcl(proginfo_t *p)
{
  int exitCode = 0;
  
  Tcl_FindExecutable(p->name);
  interp = Tcl_CreateInterp();
  
  if (!interp) {
    std::cerr << "Error initialializing tcl interpreter" << std::endl;
  }
  
  if (Tcl_AppInit(interp) != TCL_OK) {
    std::cerr << "application-specific initialization failed: ";
    std::cerr << Tcl_GetStringResult(interp) << std::endl;
  }
  else {
    // core commands
    addTclCommands(interp, p);

    // camera commands: camera::* works with whichever camera backend is
    // active; flir::* / lucid::* are the same commands under the vendor
    // name (flir::* is what the existing scripts call)
    bool flir_available = false, lucid_available = false;
#ifdef USE_FLIR
    flir_available = true;
    add_camera_commands(interp, "flir", true);
#endif
#ifdef USE_LUCID
    lucid_available = true;
    add_camera_commands(interp, "lucid", true);
#endif
    add_camera_commands(interp, "camera", flir_available || lucid_available);
    
    Tcl_SourceRCFile(interp);
  }
  return TCL_OK;
}

int processShutdownCommands(void) {
  int n = 0;
  while (shutdown_queue.size()) {
    n++;
    std::string s(shutdown_queue.front());
    shutdown_queue.pop_front();
    const char *script = s.c_str();
    int retcode = Tcl_Eval(interp, script);
    if (retcode == TCL_OK) {
      const char *rcstr = Tcl_GetStringResult(interp);
    }
    else {
      const char *rcstr = Tcl_GetStringResult(interp);
    }
  }
  return n;
}


int processMouseEvents(void)
{
  int count = 0;
  while (mouse_queue.size()) {
    std::string cmd = mouse_queue.front();
    mouse_queue.pop_front();
    
    int retcode = Tcl_Eval(interp, cmd.c_str());
    if (retcode != TCL_OK) {
      const char *err = Tcl_GetStringResult(interp);
      if (err) {
	std::cerr << "Mouse event Tcl error: " << err << std::endl;
      }
    }
    count++;
  }

  g_widgetManager.processPendingCallbacks();
 
  return count;
}

int processEvents(void)
{
    int count = 0;
    while (event_queue.size()) {
        VstreamEvent event = event_queue.front();
        event_queue.pop_front();
        
        // Call Tcl handler with native objects (much more efficient!)
        int retcode = invokeTclEventHandler(interp, event);
        
        if (retcode != TCL_OK) {
            const char *err = Tcl_GetStringResult(interp);
            if (err && strlen(err) > 0) {
                std::cerr << "Event Tcl error (" << event.type << "): " << err << std::endl;
            }
        }
        
        count++;
    }
    return count;
}

static void processTclQueues(SharedQueue<std::string> *cmdqueue,
			     SharedQueue<std::string> *respqueue)
{
  while (cmdqueue->size())
  {
    std::string s(cmdqueue->front());
    cmdqueue->pop_front();
    const char *script = s.c_str();
    int retcode = Tcl_Eval(interp, script);
    if (retcode == TCL_OK)
    {
      const char *rcstr = Tcl_GetStringResult(interp);
      if (rcstr)
      {
        respqueue->push_back(std::string(rcstr));
      }
      else
      {
        respqueue->push_back(std::string(""));
      }
    }
    else
    {
      const char *rcstr = Tcl_GetStringResult(interp);
      if (rcstr)
      {
        respqueue->push_back(std::string("!TCL_ERROR ").append(rcstr));
      }
      else
      {
        respqueue->push_back(std::string("Error:"));
      }
    }
  }
}

int processTclCommands(void)
{
  processTclQueues(&cqueue, &rqueue);
  processTclQueues(&wd_cqueue, &wd_rqueue);
  processTclQueues(&disp_cqueue, &disp_rqueue);
  while (Tcl_DoOneEvent(TCL_DONT_WAIT))
    ;
  return 0;
}

int sourceFile(const char *filename)
{
  if (!interp) {
    std::cerr << "no tcl interpreter" << std::endl;
    return TCL_ERROR;
  }
  
  return Tcl_EvalFile(interp, filename);
}

// thread safe command eval using "disp" shared queues
int tcl_eval(const std::string& cmd, std::string &response)
{
  disp_cqueue.push_back(cmd);
  std::string s(disp_rqueue.front());
  disp_rqueue.pop_front();

  const std::string error_prefix = "!TCL_ERROR ";
  if (s.compare(0, error_prefix.size(), error_prefix) == 0) {
    response = s.substr(error_prefix.size());
    return TCL_ERROR;
  }
  else
  {
    response = s;
    return TCL_OK;
  }
}

int tcl_eval(const std::string& cmd)
{
  disp_cqueue.push_back(cmd);
  std::string s(disp_rqueue.front());
  disp_rqueue.pop_front();

  const std::string error_prefix = "!TCL_ERROR ";
  if (s.compare(0, error_prefix.size(), error_prefix) == 0) {
    return TCL_ERROR;
  }
  else
  {
    return TCL_OK;
  }
}

static int docmd(const char *dscmd)
{
  char *p;
  char varname[128];
  const char *cmd;
  
  p = strchr((char *) dscmd, ' ');
  if (!p) return -1;
  
  memcpy(varname, dscmd, p-dscmd);
  varname[p-dscmd] = '\0';
  Tcl_SetVar2(interp, "dsVals", varname, dscmd, TCL_GLOBAL_ONLY);
  
  if ((cmd = Tcl_GetVar2(interp, "dsCmds", varname, TCL_GLOBAL_ONLY))) {
    Tcl_VarEval(interp, cmd, " ", varname, (char *) NULL);
    Tcl_ResetResult(interp);
  }
  return 0;
}

// =============================================================================
// DATASERVER EVENT INTEGRATION
// =============================================================================

// Helper function: Convert JSON datapoint to Tcl dictionary format
std::string jsonToTclDict(const std::string& json) {
  std::string dict;
  json_t* root = json_loads(json.c_str(), 0, NULL);
  
  if (root && json_is_object(root)) {
    // Iterate through all keys in the JSON object
    const char* key;
    json_t* value;
    
    json_object_foreach(root, key, value) {
      dict += key;
      dict += " ";
      
      if (json_is_string(value)) {
	// String values - wrap in braces if they contain spaces
	const char* str_val = json_string_value(value);
	if (!strlen(str_val) || strchr(str_val, ' ')) {
	  dict += "{";
	  dict += str_val;
	  dict += "}";
	} else {
	  dict += str_val;
	}
      } else if (json_is_integer(value)) {
	dict += std::to_string(json_integer_value(value));
      } else if (json_is_real(value)) {
	dict += std::to_string(json_real_value(value));
      } else if (json_is_boolean(value)) {
	dict += json_is_true(value) ? "1" : "0";
      } else if (json_is_null(value)) {
	dict += "{}";
      } else {
	// For objects/arrays, convert back to JSON string
	char* json_str = json_dumps(value, JSON_COMPACT);
	if (json_str) {
	  dict += "{";
	  dict += json_str;
	  dict += "}";
	  free(json_str);
	}
      }
      dict += " ";
    }
    
    // Remove trailing space
    if (!dict.empty() && dict.back() == ' ') {
      dict.pop_back();
    }
    
    json_decref(root);
  }
  
  return dict;
}

// Updated processDSCommands() function
void processDSCommands(void) {
  while (ds_queue.size()) {
    std::string dpoint = ds_queue.front();
    ds_queue.pop_front();
    
    // Parse JSON to extract the name field
    json_t* root = json_loads(dpoint.c_str(), 0, NULL);
    if (root) {
      json_t* name_obj = json_object_get(root, "name");
      if (name_obj && json_is_string(name_obj)) {
	const char* name = json_string_value(name_obj);

	// ess/in_obs feeds the dserv/timestamp obs sources directly (the
	// datapoint's timestamp is dserv's clock in microseconds)
	if (!strcmp(name, "ess/in_obs")) {
	  json_t* d = json_object_get(root, "data");
	  json_t* t = json_object_get(root, "timestamp");
	  int state = -1;
	  if (json_is_integer(d)) state = json_integer_value(d) != 0;
	  else if (json_is_string(d) && json_string_value(d)[0])
	    state = atoi(json_string_value(d)) != 0;
	  if (state >= 0) {
	    ds_in_obs = state;
	    if (obs_source.load() == OBS_SOURCE_TIMESTAMP) {
	      int64_t ts = json_is_integer(t) ? json_integer_value(t) : 0;
	      pending_obs_transitions.push_back({ts, state != 0});
	    }
	  }
	}

	// Fire event with namespaced name, JSON data
	// WebSocket clients get JSON directly
	// Tcl handlers can call jsonToTclDict() if they want dict format
	fireEvent(VstreamEvent("ds/" + std::string(name), dpoint));
      } else {
	// No name field - fire generic event
	fireEvent(VstreamEvent("ds/unknown", dpoint));
      }
      json_decref(root);
    } else {
      // JSON parse failed - might be old text format or malformed
      // Fire as generic event with the raw data
      fireEvent(VstreamEvent("ds/parse_error", dpoint));
    }
  }
}

#ifdef __APPLE__

void handle_keyboard(int key)
{
  std::string callback = g_keyboardCallbacks.getCallback(key);
  if (!callback.empty()) {
    std::ostringstream cmd;
    cmd << callback << " " << key;
    mouse_queue.push_back(cmd.str());
    return;
  }
  
  switch (key) {
  case 27:  // ESC
    do_shutdown();
    break;
  default:
    std::string generic = g_keyboardCallbacks.getCallback(-1);
    if (!generic.empty()) {
      std::ostringstream cmd;
      cmd << generic << " " << key;
      mouse_queue.push_back(cmd.str());
    }
    break;
  }  
}

void processMacOSEvents(proginfo_t* p, float scale = 1.0, Mat* frame_to_display = nullptr) {
  static bool macos_window_initialized = false;
 
  // Create window on first frame (when we know the size)
  if (!macos_window_initialized) {
    if (frame_to_display && !frame_to_display->empty()) {
      // We have a real frame with known dimensions
      namedWindow("Frame", WINDOW_AUTOSIZE);
      static float display_scale = scale;
      cv::setMouseCallback("Frame", DisplayThread::onMouse, &display_scale);
      macos_window_initialized = true;
    } else if (frame_width > 0 && frame_height > 0) {
      // We know dimensions but don't have a frame yet - create with idle screen
      namedWindow("Frame", WINDOW_AUTOSIZE);
      static float display_scale = scale;
      cv::setMouseCallback("Frame", DisplayThread::onMouse, &display_scale);
      macos_window_initialized = true;
    } else {
      return;
    }
  }
  
  if (frame_to_display) {
    imshow("Frame", *frame_to_display);
  } else {
    // Display idle screen with status
    static Mat idle_frame;

    int fw = frame_width.load();
    int fh = frame_height.load();
    int width = (fw > 0) ? fw : 640;
    int height = (fh > 0) ? fh : 480;
    
    // Recreate idle frame each time to show current status
    idle_frame = Mat(height, width, CV_8UC3, Scalar(40, 40, 40));
    
    // Title
    cv::putText(idle_frame, "No Source Active",
		cv::Point(width/4, height/2 - 80),
		cv::FONT_HERSHEY_SIMPLEX,
		1.0,
		cv::Scalar(200, 200, 200), 2);
    
    // Show source state
    std::string state_msg = "State: ";
    switch (g_sourceManager.getState()) {
    case SOURCE_IDLE: state_msg += "Idle"; break;
    case SOURCE_ERROR: state_msg += "Error"; break;
    case SOURCE_STOPPING: state_msg += "Stopping..."; break;
    default: state_msg += "Unknown"; break;
    }
    
    cv::putText(idle_frame, state_msg,
		cv::Point(width/4, height/2 - 20),
		cv::FONT_HERSHEY_SIMPLEX,
		0.7,
		cv::Scalar(150, 150, 150), 1);
    
    // Instructions
    cv::putText(idle_frame, "Commands:",
		cv::Point(width/4, height/2 + 30),
		cv::FONT_HERSHEY_SIMPLEX,
		0.6,
		cv::Scalar(100, 200, 100), 1);
    
    cv::putText(idle_frame, "startSource webcam id 0",
		cv::Point(width/4, height/2 + 60),
		cv::FONT_HERSHEY_SIMPLEX,
		0.5,
		cv::Scalar(100, 150, 100), 1);
    
    cv::putText(idle_frame, "startSource playback file \"video.avi\"",
		cv::Point(width/4, height/2 + 85),
		cv::FONT_HERSHEY_SIMPLEX,
		0.5,
		cv::Scalar(100, 150, 100), 1);
    
    cv::putText(idle_frame, "[ESC] Quit",
		cv::Point(width/4, height/2 + 120),
		cv::FONT_HERSHEY_SIMPLEX,
		0.5,
		cv::Scalar(200, 100, 100), 1);
    
    imshow("Frame", idle_frame);
  }
  
  int key = waitKeyEx(10);
  if (key != -1) {
    handle_keyboard(key);
  }
}
#endif

int main(int argc, char **argv)
{
  int camera_id = 0;
  bool verbose = false;
  bool use_webcam = false;
  bool use_flir = false;
  bool use_lucid = false;
  int display_every = 1;
  bool help = false;
  bool init_display = false;
  bool no_source = true;
  bool flip_view = true;
  int flip_code = -2;
  float scale = 1.0;
  const char *startup_file = NULL;
  int port = 4630;

  // Playback mode options
  std::string playback_file = "";
  std::string metadata_file = "";
  float playback_speed = 1.0;
  bool playback_mode = false;
  bool playback_loop = true;

  int ws_port = 8080;

  std::string ds_host = "";  // Empty = disabled
  int ds_port = 4620;
  
  cxxopts::Options options("videostream","video streaming example program");


  WatchdogThread watchdogTimer;
  std::thread watchdog_thread;
  std::thread process_thread;
#if !defined(__APPLE__)
  std::thread display_thread;
#endif
  
  
  options.add_options()
    ("v,verbose", "Verbose mode", cxxopts::value<bool>(verbose))
    ("ws-port", "WebSocket server port", cxxopts::value<int>()->default_value("8080"))    
    ("w,webcam", "Use webcam", cxxopts::value<bool>(use_webcam))
    ("flir", "Use flir", cxxopts::value<bool>(use_flir))
    ("lucid", "Use Lucid (Arena SDK) camera", cxxopts::value<bool>(use_lucid))
    ("d,display", "Start with display", cxxopts::value<bool>(init_display))
    ("p,port", "TCP/IP server port", cxxopts::value<int>(port))
    ("c,camera_id", "Camera ID", cxxopts::value<int>(camera_id))
    ("o,overwrite", "Overwrite file", cxxopts::value<bool>(overwrite))
    ("s,scale", "Scale factor", cxxopts::value<float>(scale))
    ("n,showevery", "Show every n frames", cxxopts::value<int>(display_every))
    ("f,file", "Startup file name", cxxopts::value<std::string>())
    ("e,flipcode", "Flip code (OpenCV)", cxxopts::value<int>(flip_code))
    ("l,flip", "Flip video(OpenCV)", cxxopts::value<bool>(flip_view))
    ("playback", "Playback mode (video file)", cxxopts::value<std::string>())
    ("metadata", "Metadata file (.dgz) for playback", cxxopts::value<std::string>())
    ("speed", "Playback speed multiplier", cxxopts::value<float>()->default_value("1.0"))
    ("noloop", "Disable playback looping", cxxopts::value<bool>())
    ("ds-host", "Dataserver host address", cxxopts::value<std::string>())
    ("ds-port", "Dataserver port", cxxopts::value<int>()->default_value("4620"))
    
    ("help", "Print help", cxxopts::value<bool>(help))
    ;

  try {
    auto result = options.parse(argc, argv);
    
    if (result.count("file")) {
      startup_file = strdup((result["file"].as<std::string>()).c_str());
    }

    if (result.count("ws-port")) {
      ws_port = result["ws-port"].as<int>();
    }

    if (result.count("ds-host")) {
      ds_host = result["ds-host"].as<std::string>();
    }
    
    if (result.count("ds-port")) {
      ds_port = result["ds-port"].as<int>();
    }
    
    if (result.count("playback")) {
      playback_mode = true;
      playback_file = result["playback"].as<std::string>();
      
      if (result.count("metadata")) {
        metadata_file = result["metadata"].as<std::string>();
      }
      
      playback_speed = result["speed"].as<float>();

      if (result.count("noloop")) {
        playback_loop = false;  
      }
    }
  }
  catch (const std::exception& e) {
    std::cout << "error parsing options: " << e.what() << std::endl;
    exit(1);
  }

  if (help) {
    std::cout << options.help({"", "Group"}) << std::endl;
    exit(0);
  }

  // index of current frame
  std::atomic<int> curFrame{0};

  proginfo_t programInfo;
  ReviewModeSource reviewModeSource;
  
  displayEvery = display_every;
  programInfo.name = argv[0];
  programInfo.argv = argv;
  programInfo.argc = argc;
  programInfo.script_file = startup_file;
  programInfo.display = init_display;
  programInfo.sourceManager = &g_sourceManager;
  programInfo.widgetManager = &g_widgetManager;
  programInfo.frameBuffer = &frameBufferManager;
  programInfo.curFrame = &curFrame;
  programInfo.displayFrame = &displayFrame;
  programInfo.frameSource = &g_frameSource;
  programInfo.frame_rate = &frame_rate;
  programInfo.frame_width = &frame_width;
  programInfo.frame_height = &frame_height;
  programInfo.is_color = &is_color;
  programInfo.ds_host = ds_host.c_str();
  programInfo.ds_port = ds_port;
  
  std::string source_type;
  std::map<std::string, std::string> source_params;
  
  if (playback_mode) {
    source_type = "playback";
    source_params["file"] = playback_file;
    if (!metadata_file.empty()) {
      source_params["metadata"] = metadata_file;
    }
    source_params["speed"] = std::to_string(playback_speed);
    source_params["loop"] = playback_loop ? "1" : "0";
    use_webcam = false;
    use_flir = false;
    no_source = false;
  } else if (use_webcam) {
    source_type = "webcam";
    source_params["id"] = std::to_string(camera_id);
    use_webcam = true;
    use_flir = false;
    no_source = false;
  } else if (use_flir) {
#ifdef USE_FLIR
    source_type = "flir";
    source_params["id"] = std::to_string(camera_id);
    source_params["flip"] = flip_view ? "1" : "0";
    source_params["flip_code"] = std::to_string(flip_code);
    use_webcam = false;
    use_flir = true;
    no_source = false;
#else
    std::cerr << "FLIR support not compiled. Use --webcam, or rebuild with -DWITH_FLIR=ON (requires the Spinnaker SDK)." << std::endl;
    return -1;
#endif
  } else if (use_lucid) {
#ifdef USE_LUCID
    source_type = "lucid";
    source_params["id"] = std::to_string(camera_id);
    use_webcam = false;
    use_flir = false;
    no_source = false;
#else
    std::cerr << "Lucid support not compiled. Use --webcam, or rebuild with -DWITH_LUCID=ON (requires the Arena SDK)." << std::endl;
    return -1;
#endif
  }

  if (!no_source) {
    if (!g_sourceManager.startSource(source_type, source_params)) {
      std::cerr << "Failed to start source" << std::endl;
      return -1;
    }
    
    g_frameSource = g_sourceManager.getCurrentSource();

    // give sourceManager access to clear widgets and frame buffer clear
    g_sourceManager.setWidgetManager(&g_widgetManager);
    g_sourceManager.setFrameBuffer(&frameBufferManager);
    
    if (!g_frameSource) {
      std::cerr << "No frame source available" << std::endl;
      return -1;
    }
    
    // Get properties from source 
    frame_rate = g_frameSource->getFrameRate();
    frame_width = g_frameSource->getWidth();
    frame_height = g_frameSource->getHeight();
    is_color = g_frameSource->isColor();
    
    std::cout << "Frame source: " << frame_width << "x" << frame_height 
	      << " @ " << frame_rate << " fps, " 
	      << (is_color ? "color" : "grayscale") << std::endl;
  }

  done = false;

  // Used to take video samples and store in review source
  programInfo.samplingManager = new SamplingManager(&programInfo);

  if (!ds_host.empty()) {
    g_dataForwarder = new DataserverForwarder(ds_host, ds_port);
    g_dataForwarder->start();
    std::cout << "Started dataserver forwarder to " << ds_host << ":" << ds_port << std::endl;
  } else {
    // Start the queue draining thread even without connection
    g_dataForwarder = new DataserverForwarder("", 0);  // Dummy values
    g_dataForwarder->startDrainOnly();
    std::cout << "Dataserver forwarding disabled (queue draining only)" << std::endl;
  }
  
  std::signal(SIGINT, signal_handler);
  
  TcpipThread tcpServer;
  tcpServer.port = port;
  cout << "Starting cmd server on port " << tcpServer.port << std::endl;
  std::thread net_thread(&TcpipThread::startTcpServer, &tcpServer);

  DservSocket dservSocket;
  dservSocket.dsport = port+1;  // Set the desired port
  dservSocket.set_queue(&ds_queue);  // Connect to the global ds_queue
  dsPort = dservSocket.dsport;
  programInfo.dservSocket = &dservSocket;

  WebSocketThread wsServer;
  wsServer.port = ws_port;
  g_wsServer = &wsServer;
 
  if (verbose)
    cout << "Starting WebSocket server on port " << wsServer.port << std::endl;
  std::thread ws_thread(&WebSocketThread::startWebSocketServer, &wsServer);
  
  if (verbose)
    cout << "Starting ds server on port " << dservSocket.dsport << std::endl;
  std::thread ds_thread = dservSocket.start_server();

  setupTcl(&programInfo);

  // --flir / --lucid also pick the camera backend for the tracker scripts,
  // which read ::camera_type (default flir) before going live
  if (source_type == "flir" || source_type == "lucid") {
    Tcl_SetVar2(interp, "camera_type", NULL, source_type.c_str(), TCL_GLOBAL_ONLY);
  }

  if (startup_file) {
    if (sourceFile(startup_file) != TCL_OK) {
      std::cerr << Tcl_GetStringResult(interp) << std::endl;
    }
    
    // Check if startup script requested shutdown
    if (done) {
      wsServer.shutdown();
      tcpServer.shutdown();
      dservSocket.shutdown();
      
      if (ws_thread.joinable()) ws_thread.detach();
      if (net_thread.joinable()) net_thread.join();
      if (ds_thread.joinable()) ds_thread.join();
      
      if (g_dataForwarder) {
	g_dataForwarder->stop();
	delete g_dataForwarder;
      }
      
      processShutdownCommands();
      return 0;
    }
  }
  
  process_thread = std::thread(&ProcessThread::startProcessThread, &processThread);

#if !defined(__APPLE__)
  display_thread = std::thread(&DisplayThread::startDisplayThread, &displayThread);
  if (init_display) displayThread.show();
  displayThread.setScale(scale);
#endif

  watchdog_thread = std::thread(&WatchdogThread::startWatchdog, &watchdogTimer);
  
  // Ready to accept "fired" events
  events_ready = true;
  
  while(!done)
    {
      SourceState state = g_sourceManager.getState();
      switch (state) {
      case SOURCE_IDLE:
      case SOURCE_ERROR:
	
        processTclCommands();
        processMouseEvents();
	processEvents();
        processDSCommands();

#ifdef __APPLE__
        processMacOSEvents(&programInfo);  // Process events even when idle
#else
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
#endif	
      
        break;
	
      case SOURCE_RUNNING:
	if (g_frameSource) {
	  FrameMetadata metadata;
	  Mat frame;

#if !defined(__APPLE__)
	  static double last_known_fps = 0.0;
	  double current_fps = g_frameSource->getFrameRate();
	  
	  if (current_fps != last_known_fps && current_fps > 0) {
	    // Target max 60 fps display
	    displayEvery = std::max(1, static_cast<int>(std::round(current_fps / 60.0)));
	    last_known_fps = current_fps;
	    
	    if (verbose) {
	      std::cout << "Display throttle: showing every " << displayEvery 
			<< " frame(s) for " << current_fps << " fps source" << std::endl;
	    }
	  }
#endif
    
	  if (!g_frameSource->getNextFrame(frame, metadata)) {
	    if (g_frameSource->isPlaybackMode() && !g_frameSource->isLooping()) {

	      fireEvent(VstreamEvent("vstream/video_source_eof"));
 
	      // Don't shutdown, just stop the source
	      g_sourceManager.stopSource();
	      g_frameSource = nullptr;
	      continue;
	    }
	    if (verbose) {
	      std::cerr << "Frame acquisition error, retrying..." << std::endl;
	    }

	    // Frame acquisition failed 
	    // Process commands so can react
	    processTclCommands();
	    processMouseEvents();
	    processEvents();
	    
	    // Sleep to avoid busy-waiting
	    std::this_thread::sleep_for(std::chrono::milliseconds(10));
	    
	    continue;
	  }
	  
	  processDSCommands();

	  // Update global color state from current source
	  is_color = g_frameSource->isColor();
	  
	  frame_width = frame.cols;
	  frame_height = frame.rows;

	  // TTL line, dserv datapoint, or datapoint-by-timestamp (vstream::obsSource)
	  set_inObs(obsStateForFrame(metadata));
	  
	  // Store in circular buffer (thread-safe)
	  frameBufferManager.storeFrame(curFrame, frame, metadata, in_obs);	  
	  
	  if (g_pluginRegistry.hasPlugins()) {
	    auto access = frameBufferManager.accessFrame(curFrame);
	    if (access.isValid()) {
	      // ensure plugin gets accurate metadata for this frame
	      FrameMetadata plugin_metadata;
	      plugin_metadata.frameID = access.frameID;
	      plugin_metadata.timestamp = access.timestamp;
	      plugin_metadata.systemTime = access.systemTime;
	      plugin_metadata.lineStatus = access.linestatus;
	      
	      g_pluginRegistry.processFrame(const_cast<cv::Mat&>(*access.frame), 
					    curFrame, 
					    plugin_metadata);	      
	    }	    
	  }

	  if (processThread.fileIsOpen()) {
	    process_queue.push_back(curFrame);

	    // Serial reprocess barrier: wait until the process thread has fully
	    // stored this frame before capturing the next. Combined with the
	    // plugin's synchronous mode this makes the recorded .db deterministic
	    // and frame-complete. No effect on the live path (flag default false).
	    if (g_reprocess_serial.load()) {
	      static long serial_pushed = 0;
	      ++serial_pushed;
	      while (processThread.frames_done_.load(std::memory_order_acquire)
		     < serial_pushed) {
		std::this_thread::yield();
	      }
	    }
	  }

#define MONITOR_PROCESS_QUEUE
#ifdef MONITOR_PROCESS_QUEUE
	  static int max_queue_depth = 0;
	  int current_depth = process_queue.size();
	  if (current_depth > max_queue_depth) {
	    max_queue_depth = current_depth;
	    std::cout << "New max queue depth: " << max_queue_depth 
		      << " (buffer size: " << nFrames << ")" << std::endl;
	  }
	  
	  // Warn if getting too close to buffer size
	  if (current_depth > nFrames * 0.8) {
	    std::cerr << "WARNING: Queue depth " << current_depth 
		      << " approaching buffer size " << nFrames << std::endl;
	  }	  
#endif	  
	  if (curFrame % displayEvery == 0) {
	    // Stash the unscaled, unannotated frame with the video frame number
	    // these pixels actually are, for vstream::saveFrame. Done whether or
	    // not a window is open, so panels can be scripted headless.
	    {
	      int vf = curFrame;
	      IFrameSource *src = g_sourceManager.getCurrentSource();
	      if (src && src->isPlaybackMode()) {
		VideoFileSource *fs = dynamic_cast<VideoFileSource *>(src);
		if (fs) vf = fs->getCurrentFrameIndex();
	      }
	      note_displayed_frame(frame, vf, in_obs);
	    }
#if !defined(__APPLE__)
	    display_queue.push_back(curFrame);
#else
	    if (programInfo.display) {
	      Mat dframe;
	      if (scale != 1.0) {
		cv::resize(frame, dframe,
			   cv::Size(frame.cols * scale,frame.rows * scale),
			   0, 0, cv::INTER_LINEAR);
	      }
	      else {
		dframe = frame.clone();
	      }

	      updateWidgetVariables(curFrame);
	      DisplayThread::annotate_frame(dframe, in_obs, curFrame);
	      processMacOSEvents(&programInfo, scale, &dframe);
	    }
#endif
	  }
	  
	  if (!frameBufferManager.hasFrame(curFrame))
	    break;
	  
	  curFrame = (curFrame+1)%nFrames;
	  
	  processTclCommands();
	  processMouseEvents();
	  processEvents();
	}
	break;
      case SOURCE_PAUSED:
	processTclCommands();
	processMouseEvents();
	processEvents();
	processDSCommands();
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	break;
	
      case SOURCE_STOPPING:
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	break;
      }
    }

cleanup:
  if (programInfo.samplingManager) {
    programInfo.samplingManager->stop();
    delete programInfo.samplingManager;
  }
  
  g_pluginRegistry.shutdownAll();
  
  // Only cleanup threads that were actually started
  if (process_thread.joinable()) {
    process_queue.push_back(-1);
    process_thread.join();
  }

#if !defined(__APPLE__)
  if (display_thread.joinable()) {
    display_queue.push_back(-1);
    display_thread.join();
  }
#endif

  if (watchdog_thread.joinable()) {
    watchdogTimer.m_bDone = true;
    // The watchdog round-trips through wd_cqueue/wd_rqueue, which only the
    // main loop services — and the main loop has exited. If the watchdog
    // fired into the teardown window it is blocked in wd_rqueue.front()
    // awaiting a reply that will never come (the ~50%-of-runs exit hang):
    // feed it a dummy reply so the join always completes.
    wd_rqueue.push_back(std::string());
    watchdog_thread.join();
  }

  g_wsServer = nullptr;

  
  if (g_dataForwarder) {
    g_dataForwarder->stop();
    delete g_dataForwarder;
  }
  
  g_sourceManager.stopSource();
  
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  wsServer.shutdown();
  if (ws_thread.joinable()) {
    ws_thread.detach();
  }
  
  tcpServer.shutdown();
  dservSocket.shutdown();

  if (net_thread.joinable()) net_thread.join();
  if (ds_thread.joinable()) ds_thread.join();

  if (verbose) std::cout << "Shutting down" << std::endl;

  processShutdownCommands();  
  
  return 0;
}
