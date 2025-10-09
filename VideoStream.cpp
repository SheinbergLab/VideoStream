#include <tcl.h>

// Use dgz format to store metadata about frames
#include <df.h>
#include <dynio.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "opencv2/opencv.hpp"

#include <iostream>
#include <sstream> 
#include <fstream>
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

#include <sys/un.h>
#include <sys/socket.h>

#if !defined(_WIN32)
#include <unistd.h>
#endif

#include "cxxopts.hpp"
#include "SharedQueue.hpp"
#include "KeyboardCallbackRegistry.h"
#include "SourceManager.h"
#include "FrameBufferManager.h"
#include "IFrameSource.h"
#include "WebcamSource.h"
#include "VideoFileSource.h"
#include "ReviewModeSource.h"
#ifdef USE_FLIR
#include "FlirCameraSource.h"
#endif

#include "Widget.h"
#include "WidgetManager.h"
#include "SamplingManager.h"

#include "VideoStream.h"

using namespace std;
using namespace cv;

SourceManager g_sourceManager;
WidgetManager g_widgetManager;

// Global frame source pointer
IFrameSource* g_frameSource = nullptr;

// Analysis registry support for plugins
#include "AnalysisPluginRegistry.h"
AnalysisPluginRegistry g_pluginRegistry;

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

/* Buffer to hold video frames */
int nFrames = 50;
FrameBufferManager frameBufferManager(nFrames);

int processFrame, displayFrame;
int frame_width, frame_height;
bool is_color = true;
float frame_rate;
std::string output_file;
bool overwrite = false;
bool in_obs = false;
bool only_save_in_obs = true;
int obs_count = -1;

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
      wd_cqueue.push_back(std::string("onWatchdog"));
      /* rqueue will be available after command has been processed */
      std::string s(wd_rqueue.front());
      wd_rqueue.pop_front();

      // sleep 
      std::this_thread::sleep_for(std::chrono::milliseconds(interval));
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

class DSTcpipThread
{
private:
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

  int listening_socket_fd;

  DSTcpipThread()
  {
    m_bDone = false;
    done = false;
  }

  void shutdown() {
    m_bDone = true;
    if (listening_socket_fd >= 0) {
      ::shutdown(listening_socket_fd, SHUT_RDWR);
      close(listening_socket_fd);
      listening_socket_fd = -1;
    }
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
  
  static void dstcpClientProcess(DSTcpipThread* server, int sockfd) {
    ssize_t n;
    char buf[1024];
    
    while ((n = recv(sockfd, buf, sizeof(buf), 0)) > 0) {
      ds_queue.push_back(std::string(buf, n));
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
      
      std::thread thr(dstcpClientProcess, this, new_socket_fd);
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
class ProcessThread
{
  int overwrite;
  int fourcc;
  std::string output_file;
  bool openfile;
  bool closefile;
  bool do_open;
  bool just_opened;
  bool annotate;
  int frame_count;
  int prev_fr;
  int start_obs;
  int end_obs;

  int sfd = -1;
  struct sockaddr_un svaddr;

  int push_next_frame = -1;

  VideoWriter video;
  DYN_GROUP *dg;
  DYN_LIST *ids, *obs_starts, *obs_stops;
  DYN_LIST *frame_ids, *frame_timestamps, *frame_systemtimes, *frame_linestatus;

  int64_t on_frameID;
  int64_t on_frameTimestamp;
  std::chrono::high_resolution_clock::time_point on_systemTimestamp;
  
  bool exists (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
      fclose(file);
      return true;
    } else {
      return false;
    }
  }

public:
  ProcessThread()
  {
    int j;
    openfile = false;
    closefile = false;
    overwrite = false;
    do_open = false;
    just_opened = false;
    only_save_in_obs = true;
    annotate = true;
    
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

    fourcc = cv::VideoWriter::fourcc('X','V','I','D');
  }

  int getFrameCount(void) {
    return frame_count;
  }
  
  void annotate_frame(Mat frame)
  {
    cv::putText(frame,
		std::to_string(frame_count),
		cv::Point(20,20),
		cv::FONT_HERSHEY_SIMPLEX,
		0.7,
		cv::Scalar(20,20,20));
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
    if (!openfile) return false;
    closefile = true;
    return true;
  }
  
  bool openFile(char *filename)
  {
    if (openfile) return false;
    do_open = true;
    obs_count = -1;
    frame_count = 0;
    output_file = std::string(filename);
    return true;
  }

  bool doOpenFile(void)
  {
    if (openfile) return 0;

    if (exists(output_file) && overwrite) {
      std::remove(output_file.c_str());
    }
    
    video = VideoWriter(output_file,
			fourcc,
			frame_rate,
			Size(frame_width,frame_height), is_color);

    dfuResetDynList(ids);
    dfuResetDynList(obs_starts);
    dfuResetDynList(obs_stops);
    dfuResetDynList(frame_ids);
    dfuResetDynList(frame_timestamps);
    dfuResetDynList(frame_systemtimes);
    dfuResetDynList(frame_linestatus);
    
    openfile = true;
    return 1;
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

  void startProcessThread(void)
  {
    while (1) {
      if (do_open) {
	doOpenFile();
	do_open = false;
	just_opened = true;
      }
      
      do {
	processFrame = process_queue.front();
	process_queue.pop_front();
	
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
	}
	
	if (push_next_frame != 0) {
	  auto access = frameBufferManager.accessFrame(processFrame);
	  if (access.isValid()) {
	    sendToDomainSocket(const_cast<Mat*>(access.frame));
	  }
	}
	if (push_next_frame > 0) push_next_frame--;
	
	if (openfile) {
	  if (processFrame)
	    prev_fr = (processFrame-1);
	  else prev_fr = nFrames-1;
	  
	  // Check observation boundaries
	  auto curr_access = frameBufferManager.accessFrame(processFrame);
	  auto prev_access = frameBufferManager.accessFrame(prev_fr);
	  
	  if (curr_access.isValid() && prev_access.isValid()) {
	    if (curr_access.in_obs && !prev_access.in_obs) {
	      start_obs = true;
	      dfuAddDynListLong(obs_starts, frame_count);
	    }
	    else start_obs = false;
	    
	    if (!curr_access.in_obs && prev_access.in_obs) {
	      end_obs = true;
	      dfuAddDynListLong(obs_stops, frame_count-1);
	    }
	    else end_obs = false;
	  }
	  
	  // Copy frame for processing
	  cv::Mat frame_copy;
	  FrameMetadata frame_metadata;
	  bool frame_in_obs;
	  
	  if (frameBufferManager.copyFrame(processFrame, frame_copy, frame_metadata, frame_in_obs)) {
	    if (!only_save_in_obs || (only_save_in_obs && frame_in_obs)) {
	      
	      if (annotate) annotate_frame(frame_copy);
	      
	      video.write(frame_copy);
	      
	      dfuAddDynListLong(ids, frame_count);
	      dfuAddDynListLong(frame_ids,
				(int) (frame_metadata.frameID - on_frameID));
	      
	      int64_t ns = frame_metadata.timestamp - on_frameTimestamp; 
	      dfuAddDynListLong(frame_timestamps, (int) (ns/1000));
	      
	      int elapsed = chrono::duration_cast<chrono::microseconds>(
									frame_metadata.systemTime - on_systemTimestamp).count();
	      dfuAddDynListLong(frame_systemtimes, elapsed);
	      
	      dfuAddDynListChar(frame_linestatus,
				(unsigned char) frame_metadata.lineStatus);
	      
	      frame_count++;
	    }
	  }
	}
      } while (processFrame >= 0 && process_queue.size());
      
      if (processFrame < 0 || processFrame >= nFrames) {
	if (openfile) {
	  video.release();
	  writeFrameDG(dg, output_file);
	}
	break;
      }
      
      if (closefile) {
	video.release();
	openfile = false;
	closefile = false;
	writeFrameDG(dg, output_file);
      }
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
    obs_count++;
  }
  return old;
}

int set_onlySaveInObs(int status)
{
  int old = only_save_in_obs;
  if (status == 0 || status == 1)
    only_save_in_obs = status;
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
    // Convert to color if grayscale (plugins may want color)
#if 0
    if (frame.channels() == 1) {
      cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    }
#endif
    
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
  
  static void annotate_frame(Mat frame, bool inobs, int frame_idx)
  {
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
		do_shutdown();
		break;
	      case 'h':
		hide();
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
    addTclCommands(interp, p);
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

static void processTclQueues(SharedQueue<std::string> *cmdqueue, SharedQueue<std::string> *respqueue)
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

int processDSCommand(const char *dscmd)
{
  static char buf[1024];
  const char *begin, *end;

  if (!strchr((char *) dscmd, '\n')) {
    return docmd(dscmd);
  }
  else {
    begin = dscmd;
    while ((end = strchr(begin, '\n'))) {
      memset(buf, '\0', sizeof(buf));
      memcpy(buf, begin, end-begin);
      docmd(buf);
      begin = end+1;
    }
    if (begin[0]) {
      docmd(begin);
    }
  }
  
  return 1;
}

int processDSCommands(void) {
  int n = 0;
  while (ds_queue.size()) {
    n++;
    std::string s(ds_queue.front());
    ds_queue.pop_front();
    processDSCommand(s.c_str());
  }
  return n;
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
    
    int width = (frame_width > 0) ? frame_width : 640;
    int height = (frame_height > 0) ? frame_height : 480;
    
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
  
  int key = waitKeyEx(1);
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
  
  cxxopts::Options options("videostream","video streaming example program");

  options.add_options()
    ("v,verbose", "Verbose mode", cxxopts::value<bool>(verbose))
    ("w,webcam", "Use webcam", cxxopts::value<bool>(use_webcam))
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
    ("help", "Print help", cxxopts::value<bool>(help))
    ;

  try {
    auto result = options.parse(argc, argv);
    
    if (result.count("file")) {
      startup_file = strdup((result["file"].as<std::string>()).c_str());
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

  proginfo_t programInfo;
  ReviewModeSource reviewModeSource;
  
  displayEvery = display_every;
  programInfo.name = argv[0];
  programInfo.display = init_display;
  programInfo.sourceManager = &g_sourceManager;
  programInfo.widgetManager = &g_widgetManager;
  programInfo.frameBuffer = &frameBufferManager;
  programInfo.displayFrame = &displayFrame;
  programInfo.frameSource = &g_frameSource;
  programInfo.frame_rate = &frame_rate;
  programInfo.frame_width = &frame_width;
  programInfo.frame_height = &frame_height;
  programInfo.is_color = &is_color;
 
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
    std::cerr << "FLIR support not compiled. Use --webcam or recompile with USE_FLIR" << std::endl;
    return -1;
#endif
  }

  if (!no_source) {
    if (!g_sourceManager.startSource(source_type, source_params)) {
      std::cerr << "Failed to start source" << std::endl;
      return -1;
    }
    
    g_frameSource = g_sourceManager.getCurrentSource();

    // give sourceManager access to clear widgets
    g_sourceManager.setWidgetManager(&g_widgetManager);
    
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
  
  int curFrame = 0;
  done = false;

  // Used to take video samples and store in review source
  programInfo.samplingManager = new SamplingManager(&programInfo);
  
  std::signal(SIGINT, signal_handler);
  
  setupTcl(&programInfo);
  
  WatchdogThread watchdogTimer;
  std::thread watchdog_thread(&WatchdogThread::startWatchdog, &watchdogTimer);

  TcpipThread tcpServer;
  tcpServer.port = port;
  if (verbose)
    cout << "Starting cmd server on port " << tcpServer.port << std::endl;
  std::thread net_thread(&TcpipThread::startTcpServer, &tcpServer);

  DSTcpipThread dstcpServer;
  dstcpServer.port = port+1;
  dsPort = dstcpServer.port;
  
  if (verbose)
    cout << "Starting ds server on port " << dstcpServer.port << std::endl;
  std::thread ds_thread(&DSTcpipThread::startTcpServer, &dstcpServer);

  if (startup_file) {
    if (sourceFile(startup_file) != TCL_OK) {
      std::cerr << Tcl_GetStringResult(interp) << std::endl;
    }
  }
  
  std::thread process_thread(&ProcessThread::startProcessThread, &processThread);

#if !defined(__APPLE__)
  std::thread display_thread(&DisplayThread::startDisplayThread, &displayThread);
#endif
  
#if !defined(__APPLE__)
  if (init_display) displayThread.show();
  displayThread.setScale(scale);
#endif
  
  while(!done)
    {
      SourceState state = g_sourceManager.getState();
      switch (state) {
      case SOURCE_IDLE:
      case SOURCE_ERROR:
	
        processTclCommands();
        processMouseEvents();
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
	  
	  if (!g_frameSource->getNextFrame(frame, metadata)) {
	    if (playback_mode && !playback_loop) {
	      std::cout << "End of playback file" << std::endl;
	      // Don't shutdown, just stop the source
	      g_sourceManager.stopSource();
	      g_frameSource = nullptr;
	      continue;
	    }
	    if (verbose) {
	      std::cerr << "Frame acquisition error, retrying..." << std::endl;
	    }
	    continue;
	  }
	  
	  processDSCommands();
	  
	  frame_width = frame.cols;
	  frame_height = frame.rows;
	  
	  // Update observation status based on line status
	  if (use_flir) {
	    set_inObs(metadata.lineStatus);
	  }
	  
	  // Store in circular buffer (thread-safe)
	  frameBufferManager.storeFrame(curFrame, frame, metadata, in_obs);	  
	  
	  if (g_pluginRegistry.hasPlugins()) {
	    auto access = frameBufferManager.accessFrame(curFrame);
	    if (access.isValid()) {
	      g_pluginRegistry.processFrame(const_cast<cv::Mat&>(*access.frame), curFrame, metadata);
	    }	    
	  }

	  process_queue.push_back(curFrame);
	  
	  if (curFrame % displayEvery == 0) {
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
	}
	break;
      case SOURCE_PAUSED:
	processTclCommands();
	processMouseEvents();
	processDSCommands();
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	break;
	
      case SOURCE_STOPPING:
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	break;
      }
    }

  // stop and cleanup sampling thread manager
  if (programInfo.samplingManager) {
    programInfo.samplingManager->stop();
    delete programInfo.samplingManager;
    programInfo.samplingManager = nullptr;
  }
  
  g_pluginRegistry.shutdownAll();
  
  process_queue.push_back(-1);
  process_thread.join();
  
#if !defined(__APPLE__)
  display_queue.push_back(-1);
  display_thread.join();
#endif

  watchdogTimer.m_bDone = true;
  watchdog_thread.join();

  
  // Cleanup
  g_sourceManager.stopSource();
  g_frameSource = nullptr;
  
  // Give CoreMediaIO time to settle
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  tcpServer.shutdown();
  dstcpServer.shutdown();

  // Join instead of detach
  if (net_thread.joinable()) {
    net_thread.join();
  }
  if (ds_thread.joinable()) {
    ds_thread.join();
  }
  
  if (verbose) std::cout << "Shutting down" << std::endl;

  processShutdownCommands();
  
  return 0;
}
