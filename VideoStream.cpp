
#include <tcl.h>

// Use dgz format to store metadata about frames
#include <df.h>
#include <dynio.h>

#include <sockpp/tcp_acceptor.h>

#include "opencv2/opencv.hpp"

#ifdef USE_FLIR
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
#endif

#include <iostream>
#include <sstream> 
#include <fstream>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <queue>
#include <csignal>
#include <sys/un.h>
#include <sys/socket.h>

#if !defined(_WIN32)
#include <unistd.h>
#endif

#include "cxxopts.hpp"
#include "SharedQueue.hpp"

using namespace std;
using namespace cv;

std::thread processThreadID;
SharedQueue<int> process_queue;

std::thread displayThreadID;
SharedQueue<int> display_queue;

SharedQueue<std::string> cqueue;
SharedQueue<std::string> rqueue;

SharedQueue<std::string> ds_queue;
SharedQueue<std::string> shutdown_queue;

Tcl_Interp *interp = NULL;

#ifdef __cplusplus
extern "C" {
#endif
  void addTclCommands(Tcl_Interp *interp);
  int open_videoFile(char *filename);
  int close_videoFile(void);

  int open_domainSocket(char *socket_path);
  int close_domainSocket(void);

  int set_inObs(int status);
  int set_fourCC(char *str);

  void add_shutdown_command(char *str);
  
  int show_display();
  int hide_display();

  int configure_exposure(float exposure);
  int configure_ROI(int w, int h, int offsetx, int offsety);
  int configure_gain(float gain);
  int configure_framerate(float framerate);

  int do_shutdown();

  /* Shared with tcl */
  int dsPort;
  int useWebcam = 1;
  int displayEvery = 1;   // Determines how often to update display

  int ShowChunk = 0;

#ifdef __cplusplus
}
#endif


namespace
{
  volatile std::sig_atomic_t done;
}

void signal_handler(int signal)
{
  do_shutdown();
}

/* Buffer to hold video frames */
int nFrames = 50;
std::vector<Mat> Frames(nFrames);
std::vector<bool> FrameInObs(nFrames);
std::vector<bool> FrameLinestatus(nFrames);
std::vector<int64_t> FrameIDs(nFrames); // supplied by camera
std::vector<int64_t> FrameTimestamps(nFrames);
std::vector<std::chrono::high_resolution_clock::time_point> SystemTimestamps(nFrames);

int processFrame, displayFrame;
int frame_width, frame_height;
bool is_color = true;
float frame_rate;
std::string output_file;
bool overwrite = false;
bool in_obs = false;
int obs_count = -1;


#ifdef _WIN32
bool WSA_initialized = false;   // Windows Socket startup needs to be called once
bool WSA_shutdown = false;  // Windows Socket cleanup only once
#endif


#ifdef USE_FLIR
INodeMap *nodeMapPtr = NULL;

/* Camera parameters */
float fps = 100;
int hpix = 1000;
int vpix = 500;
int hoffset = 140;
int voffset = 264;
float exposure = 9500.0;
float gain = 4.0;

#endif
  


class WatchdogThread
{
  public:
  bool m_bDone;
  int interval;
  
  WatchdogThread()
  {
    m_bDone = false;
    interval = 1;       // 1 second wakeup
  }
  
  void startWatchdog(void) {
    while (!m_bDone) {
      cqueue.push_back("onWatchdog");
      /* rqueue will be available after command has been processed */
      std::string s(rqueue.front());
      rqueue.pop_front();
#ifndef _WIN32
      sleep(interval);
#else
      _sleep(interval);
#endif      
    }
  }
};



class TcpipThread
{
  std::mutex m_sig_mutex;
  bool m_bDone;
  
  public:

  std::condition_variable m_sig_cv;
  int port;

  TcpipThread()
  {
    m_bDone = false;
    port = 4610;
    done = false;
  }

  ~TcpipThread()
  {
  }

  static void tcpClientProcess(sockpp::tcp_socket sock) {
    ssize_t n;
    char buf[1024];
    
    while ((n = sock.read(buf, sizeof(buf))) > 0) {
      /*
       * Queue up message 
       */

      /* push command onto Tcl queue */
      cqueue.push_back(std::string(buf, n));

      /* rqueue will be available after command has been processed */
      std::string s(rqueue.front());
      rqueue.pop_front();
      s += "\n";
      sock.write_n(s.data(), s.size()+1);
    }
  }
  
  void
  startTcpServer(void) {
    
#ifdef _WIN32
    if (!WSA_initialized ) {
      //! Windows netword DLL init
      WORD version = MAKEWORD(2, 2);
      WSADATA data;
      
      if (WSAStartup(version, &data) != 0) {
    std::cerr << "WSAStartup() failure" << std::endl;
    return;
      }
      WSA_initialized = true;
    }
#endif /* _WIN32 */


    sockpp::socket_initializer sockInit;
    
    sockpp::tcp_acceptor acc(port);
    
    if (!acc) {
      std::cerr << "Error creating the acceptor: " << acc.last_error_str() << std::endl;
      return;
    }
    //cout << "Acceptor bound to address: " << acc.address() << std::endl;
    //        std::cout << "Awaiting connections on port " << port << "..." << std::endl;
    
    while (!m_bDone) {
      sockpp::inet_address peer;
      // Accept a new client connection
      sockpp::tcp_socket sock = acc.accept(&peer);
      //            std::cout << "Received a connection request from " << peer << std::endl;
      
      if (!sock) {
    std::cerr << "Error accepting incoming connection: "
          << acc.last_error_str() << std::endl;
      }
      else {
    // Create a thread and transfer the new stream to it.
    std::thread thr(tcpClientProcess, std::move(sock));
    thr.detach();
      }
    }

    std::unique_lock<std::mutex> lock(m_sig_mutex);
    m_sig_cv.wait(lock);
    
#ifdef _WIN32
    if (!WSA_shutdown) {
      WSACleanup();
      WSA_shutdown = true;
    }
#endif /* _WIN32 */
  }
  
};



class DSTcpipThread
{
  std::mutex m_sig_mutex;
  bool m_bDone;
  
  public:

  std::condition_variable m_sig_cv;
  int port;

  DSTcpipThread()
  {
    m_bDone = false;
    done = false;
  }

  static void dstcpClientProcess(sockpp::tcp_socket sock) {
    ssize_t n;
    char buf[1024];
    
    while ((n = sock.read(buf, sizeof(buf))) > 0) {
      ds_queue.push_back(std::string(buf, n));
    }
  }
  
  
  void
  startTcpServer(void) {
    
#ifdef _WIN32
    if (!WSA_initialized ) {
      //! Windows netword DLL init
      WORD version = MAKEWORD(2, 2);
      WSADATA data;
      
      if (WSAStartup(version, &data) != 0) {
    std::cerr << "WSAStartup() failure" << std::endl;
    return;
      }
      WSA_initialized = true;
    }
#endif /* _WIN32 */


    sockpp::socket_initializer sockInit;
    
    sockpp::tcp_acceptor acc(port);
    
    if (!acc) {
      std::cerr << "Error creating the acceptor: " << acc.last_error_str() << std::endl;
      return;
    }

    while (!m_bDone) {
      sockpp::inet_address peer;
      // Accept a new client connection
      sockpp::tcp_socket sock = acc.accept(&peer);
      
      if (!sock) {
    std::cerr << "Error accepting incoming connection: "
          << acc.last_error_str() << std::endl;
      }
      else {
    // Create a thread and transfer the new stream to it.
    std::thread thr(dstcpClientProcess, std::move(sock));
    thr.detach();
      }
    }
    
    std::unique_lock<std::mutex> lock(m_sig_mutex);
    m_sig_cv.wait(lock);

#ifdef _WIN32
    if (!WSA_shutdown) {
      WSACleanup();
      WSA_shutdown = true;
    }
#endif /* _WIN32 */
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
  bool only_save_in_obs;
  bool annotate;
  int frame_count;      // keep track of frames output
  int prev_fr;          // track previous frame id
  int start_obs;            
  int end_obs;          

  int sfd = -1;         // socket file descriptor 
  struct sockaddr_un svaddr;

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
    annotate = true;    // add info to each frame
    
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
    //    ('x','2','6','4');
    //    ('a','v','c','1');
  }


  int getFrameCount(void) {
    return frame_count;
  }

  void annotate_frame(Mat frame)
  {
    cv::putText(frame,
        std::to_string(frame_count),
        cv::Point(20,20), // Coordinates
        cv::FONT_HERSHEY_SIMPLEX, // Font
        0.7, // Scale. 2.0 = 2x bigger
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
    obs_count = -1;     // reset
    frame_count = 0;        // reset
    output_file = std::string(filename);
    return true;
  }

  bool doOpenFile(void)
  {
    if (openfile) return 0;

    if (exists(output_file) && overwrite) {
      std::remove(output_file.c_str());
    }
    
    // Define the codec and create VideoWriter object
    video = VideoWriter(output_file,
            fourcc,
            frame_rate,
            Size(frame_width,frame_height), is_color);
    

    /*
      cout << "Opened " + output_file + " (" +
      std::to_string(frame_width) + "x" + std::to_string(frame_height) +
      "@" + std::to_string(frame_rate) + "hz)" << std::endl;
    */

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

    sfd = socket(AF_UNIX, SOCK_STREAM, 0);
    if(sfd == -1) return false;

    memset(&svaddr,0,sizeof(struct sockaddr_un));
    svaddr.sun_family = AF_UNIX;
    strncpy(svaddr.sun_path, path, strlen(path));

  if (connect(sfd, (struct sockaddr *) &svaddr,
                sizeof(struct sockaddr_un)) == -1) {
      close(sfd);
      sfd = -1;
      return false;
    }
   return true;
  }
  
  int sendToDomainSocket(Mat &m)
  {
    if (sfd >= 0) {
      char buf[32];
      uint32_t header[4];
      header[0] = m.total() * m.elemSize();
      header[1] = m.rows;
      header[2] = m.cols;
      header[3] = m.type();
      int msgLen = sizeof(header);

      // send the header info
      if (sendto(sfd, header, msgLen, 0, (struct sockaddr*) &svaddr, sizeof(struct sockaddr_un)) != msgLen) {
           fprintf(stderr, "error writing to domain socket\n");
           return -1;
      }

      // now send the data
      if (sendto(sfd, m.data, header[0], 0, (struct sockaddr*) &svaddr, sizeof(struct sockaddr_un)) != header[0]) {
        fprintf(stderr, "error writing to domain socket\n");
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

    // If we just opened a file, stash away id and timestamp
    if (just_opened) {
      just_opened = false;
      on_frameID = FrameIDs[processFrame];
      on_frameTimestamp = FrameTimestamps[processFrame];
      on_systemTimestamp = SystemTimestamps[processFrame];
    }
    
    sendToDomainSocket(Frames[processFrame]);

    // Write the frame into the file
    if (openfile) {
      // Add metadata to frame info file
      if (processFrame)
        prev_fr = (processFrame-1);
      else prev_fr = nFrames-1;
      
      if (FrameInObs[processFrame] && !FrameInObs[prev_fr]) {
        start_obs = true;
        dfuAddDynListLong(obs_starts, frame_count);
      }
      else start_obs = false;
      
      if (!FrameInObs[processFrame] && FrameInObs[prev_fr]) {
        end_obs = true;
        dfuAddDynListLong(obs_stops, frame_count-1);
      }
      else end_obs = false;

      if (!only_save_in_obs ||
          (only_save_in_obs && FrameInObs[processFrame])) {

        if (annotate) annotate_frame(Frames[processFrame]);
        
        video.write(Frames[processFrame]);

        dfuAddDynListLong(ids, frame_count);
        dfuAddDynListLong(frame_ids,
                  (int) (FrameIDs[processFrame]-on_frameID));

        int64_t ns = FrameTimestamps[processFrame]-on_frameTimestamp; 
        dfuAddDynListLong(frame_timestamps, (int) (ns/1000));

        int elapsed = chrono::duration_cast<chrono::microseconds>(SystemTimestamps[processFrame] - on_systemTimestamp).count();
        dfuAddDynListLong(frame_systemtimes, elapsed);

        dfuAddDynListChar(frame_linestatus,
                  (unsigned char) FrameLinestatus[processFrame]);
        
        frame_count++;
      }
    }
      } while (processFrame >= 0 && process_queue.size());

      /* Message to shut down - if file open, then close */
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

int set_fourCC(char *str)
{
  return processThread.setFourCC(str);
}


class DisplayThread
{

  int show_frames;
  float scale;

  static void onMouse(int event, int x, int y, int flags, void* userdata)
  {
    return;
  }
  
  
public:
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


  static void annotate_frame(Mat frame, bool inobs)
  {
    cv::putText(frame,
        "Frame: " + std::to_string(processThread.getFrameCount()),
        cv::Point(50,50), // Coordinates
        cv::FONT_HERSHEY_SIMPLEX, // Font
        1.0, // Scale. 2.0 = 2x bigger
        cv::Scalar(20,20,20));
    
    if (processThread.fileIsOpen()) {
      cv::putText(frame,
          "File: " + processThread.currentFile() + "[" +
          std::to_string(obs_count) + "]",
          cv::Point(50,85), // Coordinates
          cv::FONT_HERSHEY_SIMPLEX, // Font
          0.75, // Scale. 2.0 = 2x bigger
          cv::Scalar(20,20,20));
    }
    if (inobs) {
      cv::circle(frame, cv::Point(40,80), 6, cv::Scalar(10,10,200));
    }
  }

  void startDisplayThread(void)     
  {
    while (1) {
      do {
    displayFrame = display_queue.front();
    display_queue.pop_front();
      }
      while (displayFrame >= 0 && display_queue.size());
      
      /* Message to shut down */
      if (displayFrame == -1 || displayFrame >= nFrames) {
    
    // Closes all the windows
    destroyAllWindows();
    
    break;
      }
      else if (displayFrame == -2) {    // show
    namedWindow("Frame", WINDOW_AUTOSIZE);
    //  setMouseCallback("Frame", onMouse, NULL);
      }
      else if (displayFrame == -3) { // hide
    destroyWindow("Frame");
      }
      else if (show_frames) {
    /* See if the window was closed by the wm */
    if (getWindowProperty("Frame", WND_PROP_AUTOSIZE) >= 0) {
      // Use opencv highgui to display frame

      Mat dframe;
      if (scale != 1.0) {
        cv::resize(Frames[displayFrame], dframe,
               cv::Size(Frames[displayFrame].cols * scale,
                Frames[displayFrame].rows * scale),
               0, 0, cv::INTER_LINEAR);
      }
      else {
        dframe = Frames[displayFrame].clone();
      }

      annotate_frame(dframe, FrameInObs[displayFrame]);
      imshow( "Frame", dframe );
      
      // Press  ESC on keyboard to  exit
      char c = (char)waitKey(5);
      switch (c) {
      case 27:
        do_shutdown();
        break;
      case 'h':
        hide();
        break;
      default:
        break;
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

int show_display(void)
{
  return (int) displayThread.show();
}

int hide_display(void)
{
  return (int) displayThread.hide();
}

void add_shutdown_command(char *cmd)
{
  /* push shutdown command onto Tcl queue */
  shutdown_queue.push_back(std::string(cmd));
  return;
}

/* shutdown called by Tcl process */
int do_shutdown(void)
{
  done = true;
  return 1;
}


int setupTcl(char *name)
{
  int exitCode = 0;
  
  Tcl_FindExecutable(name);
  interp = Tcl_CreateInterp();
  
  if (!interp) {
    std::cerr << "Error initialializing tcl interpreter" << std::endl;
  }
  
  /*
   * Invoke application-specific initialization.
   */
  
  if (Tcl_AppInit(interp) != TCL_OK) {
    std::cerr << "application-specific initialization failed: ";
    std::cerr << Tcl_GetStringResult(interp) << std::endl;
  }
  else {
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
      if (rcstr) {
    //rqueue.push_back(std::string(rcstr));
    //std::cout << std::string(rcstr) << std::endl;
      }
    }
    else {
      const char *rcstr = Tcl_GetStringResult(interp);
      if (rcstr) {
    //rqueue.push_back("!TCL_ERROR "+std::string(rcstr));
    //std::cout << "Error: " + std::string(rcstr) << std::endl;
      }
    }
  }
  return n;
}

int processTclCommands(void) {
  int n = 0;
  while (cqueue.size()) {
    n++;
    std::string s(cqueue.front());
    cqueue.pop_front();
    const char *script = s.c_str();
      int retcode = Tcl_Eval(interp, script);
      if (retcode == TCL_OK) {
    const char *rcstr = Tcl_GetStringResult(interp);
    if (rcstr) {
      rqueue.push_back(std::string(rcstr));
      //std::cout << std::string(rcstr) << std::endl;
    }
    else {
      rqueue.push_back("");
    }
      }
      else {
    const char *rcstr = Tcl_GetStringResult(interp);
    if (rcstr) {
      rqueue.push_back("!TCL_ERROR "+std::string(rcstr));
      //std::cout << "Error: " + std::string(rcstr) << std::endl;
    }
    else {
      rqueue.push_back("Error:");
    }
      }
  }
  
  /* Do this here? */
  while (Tcl_DoOneEvent(TCL_DONT_WAIT)) ;
  
  return n;
}


int Tcl_AppInit(Tcl_Interp *interp)
{
  if (Tcl_Init(interp) == TCL_ERROR) return TCL_ERROR;
  addTclCommands(interp);
  return TCL_OK;
}

int sourceFile(const char *filename)
{
  if (!interp) {
    std::cerr << "no tcl interpreter" << std::endl;
    return TCL_ERROR;
  }
  
  return Tcl_EvalFile(interp, filename);
}


/***********************************************************************************/
/*                            DataServer Functions                                 */
/***********************************************************************************/

static int docmd(const char *dscmd)
{
  char *p;
  char varname[128];
  const char *cmd;
  
  /* Parse dataserver string and process */
  p = strchr((char *) dscmd, ' ');
  if (!p) return -1;
  
  memcpy(varname, dscmd, p-dscmd);
  varname[p-dscmd] = '\0';
  Tcl_SetVar2(interp, "dsVals", varname, dscmd, TCL_GLOBAL_ONLY);
  
  /* Dispatch callback if it exists */
  if ((cmd = Tcl_GetVar2(interp, "dsCmds", varname, TCL_GLOBAL_ONLY))) {
    Tcl_VarEval(interp, cmd, " ", varname, (char *) NULL);
    Tcl_ResetResult(interp);
  }
  return 0;
}


/*
 * processDSCommand - process input from dataserver supplying events
 */
int processDSCommand(const char *dscmd)
{
  static char buf[1024];
  const char *begin, *end;

  /* Need to handle cases where multiple dataserver messages are concatenated */

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



#ifdef USE_FLIR

bool get_linestatus(void)
{
  if (!nodeMapPtr) return -1;

  CBooleanPtr lineStatus = nodeMapPtr->GetNode("LineStatus");
  return lineStatus->GetValue();
}

int configure_exposure(float exposureTimeToSet)
{
  if (!nodeMapPtr) return -1;
  
  CEnumerationPtr ptrExposureAuto = nodeMapPtr->GetNode("ExposureAuto");
  if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
    return -1;
  
  CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
  if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
    return -1;
  ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
  
  CFloatPtr ptrExposureTime = nodeMapPtr->GetNode("ExposureTime");
  if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
    return -1;
  
  ptrExposureTime->SetValue(exposureTimeToSet);
  return 0;
}

int configure_ROI(int w, int h, int offsetx, int offsety)
{
  if (!nodeMapPtr) return -1;
  
  CIntegerPtr ptrWidth = nodeMapPtr->GetNode("Width");
  if (!IsAvailable(ptrWidth) || !IsWritable(ptrWidth))
    return -1;
  ptrWidth->SetValue(w);

  CIntegerPtr ptrHeight = nodeMapPtr->GetNode("Height");
  if (!IsAvailable(ptrHeight) || !IsWritable(ptrHeight))
    return -1;
  ptrHeight->SetValue(h);

  CIntegerPtr ptrOffsetX = nodeMapPtr->GetNode("OffsetX");
  if (!IsAvailable(ptrOffsetX) || !IsWritable(ptrOffsetX))
    return -1;
  ptrOffsetX->SetValue(offsetx);

  CIntegerPtr ptrOffsetY = nodeMapPtr->GetNode("OffsetY");
  if (!IsAvailable(ptrOffsetY) || !IsWritable(ptrOffsetY))
    return -1;
  ptrOffsetY->SetValue(offsety);
  
  return 0;
}

int configure_gain(float gainToSet)
{
  if (!nodeMapPtr) return -1;
  
  CEnumerationPtr ptrGainAuto = nodeMapPtr->GetNode("GainAuto");
  if (!IsAvailable(ptrGainAuto) || !IsWritable(ptrGainAuto))
    return -1;
  
  CEnumEntryPtr ptrGainAutoOff = ptrGainAuto->GetEntryByName("Off");
  if (!IsAvailable(ptrGainAutoOff) || !IsReadable(ptrGainAutoOff))
    return -1;
  ptrGainAuto->SetIntValue(ptrGainAutoOff->GetValue());
  
  CFloatPtr ptrGain = nodeMapPtr->GetNode("Gain");
  if (!IsAvailable(ptrGain) || !IsWritable(ptrGain))
    return -1;
  
  ptrGain->SetValue(gainToSet);
  return 0;
}

int configure_framerate(float frameRateToSet)
{
  if (!nodeMapPtr) return -1;
  
  CBooleanPtr ptrFrameRateEnable = nodeMapPtr->GetNode("AcquisitionFrameRateEnable");
  if (!IsAvailable(ptrFrameRateEnable) || !IsWritable(ptrFrameRateEnable))
    return -1;
  ptrFrameRateEnable->SetValue(true);
  
  CFloatPtr ptrFrameRate = nodeMapPtr->GetNode("AcquisitionFrameRate");
  if (!IsAvailable(ptrFrameRate) || !IsWritable(ptrFrameRate))
    return -1;
  
  ptrFrameRate->SetValue(frameRateToSet);
  return 0;
}


int configure_chunk_data(bool v)
{
  int result = 0;
  if (v) cout << endl << endl << "*** CONFIGURING CHUNK DATA ***" << endl << endl;
  try
    {
      //
      // Activate chunk mode
      //
      // *** NOTES ***
      // Once enabled, chunk data will be available at the end of the payload
      // of every image captured until it is disabled. Chunk data can also be 
      // retrieved from the nodemap.
      //

      CBooleanPtr ptrChunkModeActive = nodeMapPtr->GetNode("ChunkModeActive");
      if (!IsAvailable(ptrChunkModeActive) || !IsWritable(ptrChunkModeActive))
    {
      cout << "Unable to activate chunk mode. Aborting..." << endl << endl;
      return -1;
    }
      ptrChunkModeActive->SetValue(true);
      if (v) cout << "Chunk mode activated..." << endl;
      //
      // Enable all types of chunk data
      //
      // *** NOTES ***
      // Enabling chunk data requires working with nodes: "ChunkSelector"
      // is an enumeration selector node and "ChunkEnable" is a boolean. It
      // requires retrieving the selector node (which is of enumeration node 
      // type), selecting the entry of the chunk data to be enabled, retrieving 
      // the corresponding boolean, and setting it to true. 
      //
      // In this example, all chunk data is enabled, so these steps are 
      // performed in a loop. Once this is complete, chunk mode still needs to
      // be activated.
      //
      NodeList_t entries;
      // Retrieve the selector node
      CEnumerationPtr ptrChunkSelector = nodeMapPtr->GetNode("ChunkSelector");
      if (!IsAvailable(ptrChunkSelector) || !IsReadable(ptrChunkSelector))
    {
      cout << "Unable to retrieve chunk selector. Aborting..." << endl << endl;
      return -1;
    }
      // Retrieve entries
      ptrChunkSelector->GetEntries(entries);
      if (v) cout << "Enabling entries..." << endl;
      for (int i = 0; i < entries.size(); i++)
    {
      // Select entry to be enabled
      CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);
      // Go to next node if problem occurs
      if (!IsAvailable(ptrChunkSelectorEntry) || !IsReadable(ptrChunkSelectorEntry))
        {
          continue;
        }
      ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());
      if (v) cout << "\t" << ptrChunkSelectorEntry->GetSymbolic() << ": ";
      // Retrieve corresponding boolean
      CBooleanPtr ptrChunkEnable = nodeMapPtr->GetNode("ChunkEnable");
      // Enable the boolean, thus enabling the corresponding chunk data
      if (!IsAvailable(ptrChunkEnable))
        {
          if (v) cout << "Node not available" << endl;
        }
      else if (ptrChunkEnable->GetValue())
        {
          if (v) cout << "Enabled" << endl;
        }
      else if (IsWritable(ptrChunkEnable))
        {
          ptrChunkEnable->SetValue(true);
          if (v) cout << "Enabled" << endl;
        }
      else
        {
          if (v) cout << "Node not writable" << endl;
        }
    }
    }
  catch (Spinnaker::Exception &e)
        {
      cout << "Error: " << e.what() << endl;
      result = -1;
        }
  return result;
}
// This function displays a select amount of chunk data from the image. Unlike
// accessing chunk data via the nodemap, there is no way to loop through all 
// available data.
int display_chunk_data(ImagePtr pImage)
{
  int result = 0;
  cout << "Printing chunk data from image..." << endl;
  try
    {
      //
      // Retrieve chunk data from image
      //
      // *** NOTES ***
      // When retrieving chunk data from an image, the data is stored in a
      // a ChunkData object and accessed with getter functions.
      //
      ChunkData chunkData = pImage->GetChunkData();
      
      //
      // Retrieve exposure time; exposure time recorded in microseconds
      //
      // *** NOTES ***
      // Floating point numbers are returned as a float64_t. This can safely
      // and easily be statically cast to a double.
      //
      double exposureTime = static_cast<double>(chunkData.GetExposureTime());
      std::cout << "\tExposure time: " << exposureTime << endl;
      //
      // Retrieve frame ID
      //
      // *** NOTES ***
      // Integers are returned as an int64_t. As this is the typical integer
      // data type used in the Spinnaker SDK, there is no need to cast it.
      //
      int64_t frameID = chunkData.GetFrameID();
      cout << "\tFrame ID: " << frameID << endl;
      // Retrieve gain; gain recorded in decibels
      double gain = chunkData.GetGain();
      cout << "\tGain: " << gain << endl;
      // Retrieve height; height recorded in pixels
      int64_t height = chunkData.GetHeight();
      cout << "\tHeight: " << height << endl;
      // Retrieve offset X; offset X recorded in pixels
      int64_t offsetX = chunkData.GetOffsetX();
      cout << "\tOffset X: " << offsetX << endl;
      // Retrieve offset Y; offset Y recorded in pixels
      int64_t offsetY = chunkData.GetOffsetY();
      cout << "\tOffset Y: " << offsetY << endl;
      // Retrieve sequencer set active
      int64_t sequencerSetActive = chunkData.GetSequencerSetActive();
      cout << "\tSequencer set active: " << sequencerSetActive << endl;
      // Retrieve timestamp
      int64_t timestamp = chunkData.GetTimestamp();
      cout << "\tTimestamp: " << timestamp << endl;
      // Retrieve width; width recorded in pixels
      int64_t width = chunkData.GetWidth();
      cout << "\tWidth: " << width << endl;
    }
  catch (Spinnaker::Exception &e)
    {
      cout << "Error: " << e.what() << endl;
      result = -1;
    }
  
  return result;

}
#endif

int main(int argc, char **argv)
{
  int camera_id = 0;
  bool verbose = false;
  bool use_webcam = false;  // even if FLIR defined, use webcam
  int display_every = 1;
  bool help = false;
  bool init_display = false;
  bool flip_view = true;
  int  flip_code = -2;
  float scale = 1.0;
  const char *startup_file = NULL;
  int port = 4610;

  int64_t frameID = 0;
  int64_t timestamp = 0;
  bool linestatus = false;
  
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
    ("f,file", "File name", cxxopts::value<std::string>())
    ("e,flipcode", "Flip code (OpenCV)", cxxopts::value<int>(flip_code))
    ("l,flip", "Flip video(OpenCV)", cxxopts::value<bool>(flip_view))
    ("help", "Print help", cxxopts::value<bool>(help))
    ;

  try {
    auto result = options.parse(argc, argv);
    if (result.count("file")) {
      startup_file = strdup((result["file"].as<std::string>()).c_str());
    }
    else if (result.count("f")) {
      startup_file = strdup((result["f"].as<std::string>()).c_str());
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

  // Connect to Tcl variable
  displayEvery = display_every;

#ifdef USE_FLIR
  CameraPtr pCam;
  VideoCapture cap; 
  SystemPtr system;
  CameraList camList;
  
  if (!use_webcam) {
    useWebcam = 0;      // linked to Tcl
    
    // Retrieve singleton reference to system object
    system = System::GetInstance();
    // Retrieve list of cameras from the system
    camList = system->GetCameras();
    unsigned int numCameras = camList.GetSize();
    
    // Finish if there are no cameras
    if (numCameras <= camera_id) {
      // Clear camera list before releasing system
      camList.Clear();
      // Release system
      system->ReleaseInstance();
      cout << "Camera not found" << endl;
      return -1;
    }
    
    pCam = camList.GetByIndex(camera_id);
    INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
    
    // Initialize camera
    pCam->Init();
    
    // Retrieve GenICam nodemap
    INodeMap & nodeMap = pCam->GetNodeMap();
    nodeMapPtr = &nodeMap;


    // Configure Chunk Data
    configure_chunk_data(verbose);
    
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
      {
    cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
      return -1;
    }
                
    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrAcquisitionModeContinuous =
      ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) ||
    !IsReadable(ptrAcquisitionModeContinuous))
      {
    cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
    return -1;
      }
    
    // Retrieve integer value from entry node
    int64_t acquisitionModeContinuous =
      ptrAcquisitionModeContinuous->GetValue();
    
    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
    
    
    CFloatPtr ptrAcquisitionFrameRate = nodeMap.GetNode("AcquisitionFrameRate");
    frame_rate = static_cast<float>(ptrAcquisitionFrameRate->GetValue());
  }
  else {
    // Create a VideoCapture object and use camera to capture the video
    cap = VideoCapture(camera_id); 
    
    // Check if camera opened successfully
    if(!cap.isOpened())
      {
    cout << "Error opening video stream" << endl; 
    return -1; 
      } 
    
    frame_width = cap.get(CAP_PROP_FRAME_WIDTH); 
    frame_height = cap.get(CAP_PROP_FRAME_HEIGHT); 
    frame_rate = cap.get(CAP_PROP_FPS);
    if (frame_rate == 0.0) frame_rate = 30.0; // Some cameras don't supply
  }
#else
  use_webcam = true;
  useWebcam = 1;
  
  // Create a VideoCapture object and use camera to capture the video
  VideoCapture cap(camera_id); 
 
  // Check if camera opened successfully
  if(!cap.isOpened())
  {
    cout << "Error opening video stream" << endl; 
    return -1; 
  } 
 
  frame_width = cap.get(CAP_PROP_FRAME_WIDTH); 
  frame_height = cap.get(CAP_PROP_FRAME_HEIGHT);
  frame_rate = cap.get(CAP_PROP_FPS);
  if (frame_rate == 0.0) frame_rate = 30.0; // Some cameras don't supply
#endif
  
  int curFrame = 0;

  done = false;

  // Install a signal handler
  std::signal(SIGINT, signal_handler);
  
  setupTcl(argv[0]);

  // Start a watchdog thread

  WatchdogThread watchdogTimer;
  std::thread watchdog_thread(&WatchdogThread::startWatchdog, &watchdogTimer);

  TcpipThread tcpServer;
  tcpServer.port = port;
  if (verbose)
    cout << "Starting cmd server on port " << tcpServer.port << std::endl;

  std::thread net_thread(&TcpipThread::startTcpServer, &tcpServer);

  DSTcpipThread dstcpServer;
  dstcpServer.port = port+1;
  dsPort = dstcpServer.port;    // shared in tcl thread
  
  if (verbose)
    cout << "Starting ds server on port " << dstcpServer.port << std::endl;

  std::thread ds_thread(&DSTcpipThread::startTcpServer, &dstcpServer);

  // Source configuration file after TCP/IP threads are initialized
  if (startup_file) {
    if (sourceFile(startup_file) != TCL_OK) {
      std::cerr << Tcl_GetStringResult(interp) << std::endl;
    }
  }
  
  // Fire up processing thread
  std::thread process_thread(&ProcessThread::startProcessThread,
                 &processThread);
#if !defined(__APPLE__)
  // Fire up display thread
  std::thread display_thread(&DisplayThread::startDisplayThread,
                 &displayThread);
#endif
  
#ifdef USE_FLIR  
  if (!use_webcam) pCam->BeginAcquisition();
#endif

#if !defined(__APPLE__)
  if (init_display) displayThread.show();
  displayThread.setScale(scale);
#endif
  
  while(!done)
  { 
    Mat frame; 
    
#ifdef USE_FLIR
    if (!use_webcam) {
      ImagePtr pResultImage = pCam->GetNextImage();

      // If the image isn't complete don't process
      if (pResultImage->IsIncomplete()) {
    if (verbose) {
      cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << endl;
    }
    continue;
      }

      linestatus = get_linestatus();
      
      ImagePtr convertedImage =
    pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
      unsigned int XPadding =
    static_cast<unsigned int>(convertedImage->GetXPadding());
      unsigned int YPadding =
    static_cast<unsigned int>(convertedImage->GetYPadding());
      unsigned int rowsize =
    static_cast<unsigned int>(convertedImage->GetWidth());
      unsigned int colsize =
    static_cast<unsigned int>(convertedImage->GetHeight());
      
      // image data contains padding. When allocating Mat container size,
      // you need to account for the X,Y image data padding. 
      is_color = false;
      Mat cvimg = cv::Mat(colsize + YPadding, rowsize + XPadding,
              CV_8UC1, convertedImage->GetData(),
              convertedImage->GetStride());

      if ( flip_view ) {
    Mat flipped = cv::Mat(cvimg.rows, cvimg.cols, CV_8UC1);
    cv::flip(cvimg, flipped, flip_code);
    
    frame = flipped.clone();
      }
      else {
    frame = cvimg.clone();
      }

      if (ShowChunk) {
    display_chunk_data(pResultImage);
    ShowChunk = 0;
      }

      ChunkData chunkData = pResultImage->GetChunkData();
      
      frameID = chunkData.GetFrameID();
      timestamp = chunkData.GetTimestamp();
      
      pResultImage->Release();
    }
    else {
      cap >> frame;
    }
#else    

    // Capture frame-by-frame 
    cap >> frame;

    if (ShowChunk) {
      cout << "ShowChunk" << std::endl;
      ShowChunk = 0; // no chunks with web cam
    }
    
#endif

    // this will allow dataserver notification of obs on/off to be processed
    processDSCommands();

    frame_width = frame.cols;
    frame_height = frame.rows;

    Frames[curFrame] = frame;
    FrameInObs[curFrame] = in_obs;
    FrameLinestatus[curFrame] = linestatus;
      
    FrameIDs[curFrame] = frameID;
    FrameTimestamps[curFrame] = timestamp;
    SystemTimestamps[curFrame] = std::chrono::high_resolution_clock::now();
      

    // wake up write thread to add to potentially add to output file
    process_queue.push_back(curFrame);

    if (curFrame % displayEvery == 0) {
    // For OSX, the display code cannot be in a separate thread.
    // For other platforms, we just queue up the frame
#if !defined(__APPLE__)
      // wake up display thread to show this thread (if visible)
      display_queue.push_back(curFrame);
#else
      Mat dframe;
      if (scale != 1.0) {
    cv::resize(frame, dframe,
           cv::Size(frame.cols * scale,frame.rows * scale),
           0, 0, cv::INTER_LINEAR);
      }
      else {
    dframe = frame.clone();
      }
      
      DisplayThread::annotate_frame(dframe, FrameInObs[curFrame]);
      imshow( "Frame", dframe );
      
      // Press  ESC on keyboard to  exit
      char c = (char)waitKey(1);
      switch (c) {
      case 27:
    do_shutdown();
    break;
      default:
    break;
      }
#endif
    }
    
    // If the frame is empty, break because we have a problem!
    if (Frames[curFrame].empty())
      break;

    // Update write frame circular index
    curFrame = (curFrame+1)%nFrames;

    // If there are any pending Tcl commands, process
    processTclCommands();
  }

  /* We're done, close down helper threads */
  process_queue.push_back(-1); // signal shutdown
  process_thread.join();

#if !defined(__APPLE__)
  display_queue.push_back(-1); // signal shutdown
  display_thread.join();
#endif

  /* shutdown the watchdog timer */
  watchdogTimer.m_bDone = true;
  watchdog_thread.join();

  /* send event to TCP/IP threads to close */
  tcpServer.m_sig_cv.notify_all();
  dstcpServer.m_sig_cv.notify_all();

  /* don't leave until TCP/IP threads are finished */
  net_thread.detach();
  ds_thread.detach();

  if (verbose) std::cout << "Shutting down" << std::endl;

  /* If the user has added shutdown commands, execute those scripts here */
  processShutdownCommands();
  
#ifdef USE_FLIR
  if (!use_webcam) {
    pCam->EndAcquisition();
    
    // Deinitialize camera
    pCam->DeInit();
    pCam = NULL;
    
    // Clear camera list before releasing system
    camList.Clear();
    // Release system
    system->ReleaseInstance();
  }
  else {
    cap.release();
  }
#else
  // When everything done, release the video capture and write object
  cap.release();
#endif
  
  return 0;
}
