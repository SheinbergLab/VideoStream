// Declarations for VideoStream.cpp and tclproc.cpp

typedef struct _proginfo_t {
  char *name;
  Tcl_Interp *interp;
  int display;

  SourceManager* sourceManager;
  ReviewModeSource* reviewSource;
  FrameBufferManager *frameBuffer;
  int *displayFrame;
  IFrameSource** frameSource;  // Pointer to pointer so we can update it
  
  // Frame properties
  float* frame_rate;
  int* frame_width;
  int* frame_height;
  bool* is_color;  
} proginfo_t;


#ifdef __cplusplus
extern "C" {
#endif
  void addTclCommands(Tcl_Interp *interp, proginfo_t *p);
  int open_videoFile(char *filename);
  int close_videoFile(void);

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
#endif

