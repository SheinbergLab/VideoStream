/*
 * NAME
 *      tclproc.c - start up a tcl process inside a program
 *
 * DESCRIPTION
 *      Creates and initializes a tcl interpreter which can communicate
 * with a main process and the outside world.
 *
 * AUTHOR
 *     DLS
 *
 */

#if defined(_WIN32) || defined(_WIN64) 
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <tcl.h>

#include "IFrameSource.h"
#include "SourceManager.h"
#include "FlirCameraSource.h"
#include "ReviewModeSource.h"
#include "FrameBufferManager.h"
#include "VideoStream.h"

#ifdef WIN32
static int strcasecmp(char *a,char *b) { return stricmp(a,b); }
static int strncasecmp(char *a,char *b, int n) 
{ 
  return strnicmp(a,b,n);
}
#endif


/*********************************************************************/
/*                           Tcl Code                                */
/*********************************************************************/


/*********************************************************************/
/*                          Ping Command                             */
/*********************************************************************/

static int pingCmd(ClientData clientData, Tcl_Interp *interp,
           int argc, char *argv[])
{
  Tcl_AppendResult(interp, "pong", NULL);
  if (argc > 1) {
    Tcl_AppendResult(interp, " ", argv[1], NULL);
  }
  return TCL_OK;
}


/*********************************************************************/
/*                         Source Commands                           */
/*********************************************************************/


static int startSourceCmd(ClientData clientData, Tcl_Interp *interp,
                         int objc, Tcl_Obj *const objv[]) {
    proginfo_t *p = (proginfo_t *)clientData;
    
    if (objc < 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "type ?key value ...?");
        return TCL_ERROR;
    }
    
    std::string type = Tcl_GetString(objv[1]);
    std::map<std::string, std::string> params;
    
    // Parse key-value pairs
    for (int i = 2; i + 1 < objc; i += 2) {
        std::string key = Tcl_GetString(objv[i]);
        std::string val = Tcl_GetString(objv[i+1]);
        params[key] = val;
    }
    
    // Access through proginfo_t
    SourceManager* sm = p->sourceManager;
    IFrameSource** fs = p->frameSource;
    
    // Stop existing source first if running
    if (sm->getState() == SOURCE_RUNNING) {
        std::cout << "Stopping existing source before starting new one..." << std::endl;
        sm->stopSource();
        *fs = nullptr;
        // Small delay to ensure clean shutdown
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Start new source
    if (sm->startSource(type, params)) {
        *fs = sm->getCurrentSource();
        
        // Update global properties through proginfo_t pointers
        if (*fs) {
            *(p->frame_rate) = (*fs)->getFrameRate();
            *(p->frame_width) = (*fs)->getWidth();
            *(p->frame_height) = (*fs)->getHeight();
            *(p->is_color) = (*fs)->isColor();
            
            std::cout << "New source: " << *(p->frame_width) << "x" << *(p->frame_height) 
                      << " @ " << *(p->frame_rate) << " fps" << std::endl;
        }
        
        Tcl_SetObjResult(interp, Tcl_NewStringObj("ok", -1));
        return TCL_OK;
    } else {
        Tcl_SetObjResult(interp, Tcl_NewStringObj("failed to start source", -1));
        return TCL_ERROR;
    }
}

static int stopSourceCmd(ClientData clientData, Tcl_Interp *interp,
                        int objc, Tcl_Obj *const objv[]) {
    proginfo_t *p = (proginfo_t *)clientData;
    
    SourceManager* sm = p->sourceManager;
    IFrameSource** fs = p->frameSource;
    
    if (sm->stopSource()) {
        *fs = nullptr;
        Tcl_SetObjResult(interp, Tcl_NewStringObj("ok", -1));
        return TCL_OK;
    } else {
        Tcl_SetObjResult(interp, Tcl_NewStringObj("no active source or already stopped", -1));
        return TCL_ERROR;
    }
}

static int getSourceStatusCmd(ClientData clientData, Tcl_Interp *interp,
                             int objc, Tcl_Obj *const objv[]) {
    proginfo_t *p = (proginfo_t *)clientData;
    
    SourceManager* sm = p->sourceManager;
    IFrameSource** fs = p->frameSource;
    
    Tcl_Obj* statusDict = Tcl_NewDictObj();
    
    // State
    std::string state_str;
    switch (sm->getState()) {
        case SOURCE_IDLE: state_str = "idle"; break;
        case SOURCE_RUNNING: state_str = "running"; break;
        case SOURCE_PAUSED: state_str = "paused"; break;
        case SOURCE_STOPPING: state_str = "stopping"; break;
        case SOURCE_ERROR: state_str = "error"; break;
        default: state_str = "unknown"; break;
    }
    
    Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("state", -1),
                  Tcl_NewStringObj(state_str.c_str(), -1));
    
    // Type
    std::string type = sm->getSourceType();
    Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("type", -1),
                  Tcl_NewStringObj(type.c_str(), -1));
    
    // Frame properties (if source is active)
    if (*fs) {
        Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("width", -1),
                      Tcl_NewIntObj(*(p->frame_width)));
        Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("height", -1),
                      Tcl_NewIntObj(*(p->frame_height)));
        Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("fps", -1),
                      Tcl_NewDoubleObj(*(p->frame_rate)));
        Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("color", -1),
                      Tcl_NewBooleanObj(*(p->is_color)));
    }
    
    Tcl_SetObjResult(interp, statusDict);
    return TCL_OK;
}

/*********************************************************************/
/*                         Review Commands                           */
/*********************************************************************/

int reviewClearCmd(ClientData data, Tcl_Interp *interp, 
                    int objc, Tcl_Obj *const objv[])
{
  proginfo_t *p = (proginfo_t *)data;
  p->sourceManager->getReviewSource()->clearFrames();
  return TCL_OK;
}

int reviewSampleCmd(ClientData data, Tcl_Interp *interp,
		 int objc, Tcl_Obj *const objv[])
{
  proginfo_t *p = (proginfo_t *)data;
  
  cv::Mat frame_copy;
  FrameMetadata metadata;
  bool in_obs;
  
  // Thread-safe copy from the frame buffer
  if (p->frameBuffer->copyFrame(*(p->displayFrame), frame_copy, metadata, in_obs)) {
    // Add to review source
    if (p->sourceManager->sampleCurrentFrame(frame_copy, metadata)) {
      Tcl_SetObjResult(interp, Tcl_NewIntObj(
					     p->sourceManager->getReviewSource()->getFrameCount()));
      return TCL_OK;
    }
  }
  
  Tcl_SetObjResult(interp, Tcl_NewIntObj(0));
  return TCL_OK;
}

int reviewRemoveCmd(ClientData data, Tcl_Interp *interp,
                     int objc, Tcl_Obj *const objv[])
{
  proginfo_t *p = (proginfo_t *)data;
  if (!p->reviewSource) return TCL_OK;
  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "index");
    return TCL_ERROR;
  }
  
  int index;
  if (Tcl_GetIntFromObj(interp, objv[1], &index) != TCL_OK) {
    return TCL_ERROR;
  }
  p->sourceManager->getReviewSource()->removeFrame(index);

  return TCL_OK;
}

int reviewNextCmd(ClientData data, Tcl_Interp *interp,
		  int objc, Tcl_Obj *const objv[])
{
  proginfo_t *p = (proginfo_t *)data;
  p->sourceManager->getReviewSource()->nextFrame();
  return TCL_OK;
}

int reviewPreviousCmd(ClientData data, Tcl_Interp *interp,
                       int objc, Tcl_Obj *const objv[])
{
  proginfo_t *p = (proginfo_t *)data;
  p->sourceManager->getReviewSource()->previousFrame();
  return TCL_OK;
}

int reviewJumpToCmd(ClientData data, Tcl_Interp *interp,
		    int objc, Tcl_Obj *const objv[])
{
  proginfo_t *p = (proginfo_t *)data;
  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "index");
    return TCL_ERROR;
  }
  
  int index;
  if (Tcl_GetIntFromObj(interp, objv[1], &index) != TCL_OK) {
    return TCL_ERROR;
  }

  if (index < 0 ||
      index >= p->sourceManager->getReviewSource()->getFrameCount()) {
    Tcl_AppendResult(interp, Tcl_GetString(objv[0]),
		     ":index out of range", NULL);
    return TCL_ERROR;
  }
  
  p->sourceManager->getReviewSource()->jumpToFrame(index);

  return TCL_OK;
}

int reviewCountCmd(ClientData data, Tcl_Interp *interp,
                    int objc, Tcl_Obj *const objv[])
{
  proginfo_t *p = (proginfo_t *)data;

  int count = 0;
  Tcl_SetObjResult(interp,
		   Tcl_NewIntObj(p->sourceManager->getReviewSource()->getFrameCount()));
  return TCL_OK;
}

int reviewIndexCmd(ClientData data, Tcl_Interp *interp,
                    int objc, Tcl_Obj *const objv[])
{
  proginfo_t *p = (proginfo_t *)data;
  Tcl_SetObjResult(interp,
		   Tcl_NewIntObj(p->sourceManager->getReviewSource()->getCurrentIndex()));
  return TCL_OK;
}

/*********************************************************************/
/*                         FileIO Commands                           */
/*********************************************************************/

static int openFileCmd(ClientData clientData, Tcl_Interp *interp,
               int argc, char *argv[])
{
  int res;
  if (argc < 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " filename", NULL); 
    return TCL_ERROR;
  }
  res = open_videoFile(argv[1]);
  if (res) {
    Tcl_SetResult(interp, "1", TCL_STATIC);
  }
  else {
    Tcl_SetResult(interp, "0", TCL_STATIC);
  }
  return TCL_OK;
}

static int closeFileCmd(ClientData clientData, Tcl_Interp *interp,
            int argc, char *argv[])
{
  int res;
  close_videoFile();
  return TCL_OK;
}

/*********************************************************************/
/*                    Domain Socket Commands                         */
/*********************************************************************/

static int openDomainSocketCmd(ClientData clientData, Tcl_Interp *interp,
               int argc, char *argv[])
{
  int res;
  if (argc < 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " path", NULL); 
    return TCL_ERROR;
  }
  res = open_domainSocket(argv[1]);
  if (res) {
    Tcl_SetResult(interp, "1", TCL_STATIC);
  }
  else {
    Tcl_SetResult(interp, "0", TCL_STATIC);
  }
  return TCL_OK;
}

static int closeDomainSocketCmd(ClientData clientData, Tcl_Interp *interp,
            int argc, char *argv[])
{
  close_domainSocket();
  return TCL_OK;
}

static int sendNDomainSocketCmd(ClientData clientData, Tcl_Interp *interp,
            int argc, char *argv[])
{
  int n;
  if (argc < 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " n {-1|0|n}", NULL); 
    return TCL_ERROR;
  }
  if (Tcl_GetInt(interp, argv[1], &n) != TCL_OK) return TCL_ERROR;
  sendn_domainSocket(n);
  return TCL_OK;
}



static int setInObsCmd(ClientData clientData, Tcl_Interp *interp,
               int argc, char *argv[])
{
  int res;
  int status = -1;  /* used to get current value w/o setting */

  if (argc > 1) {
    if (Tcl_GetInt(interp, argv[1], &status) != TCL_OK)
      return TCL_ERROR;
  }
  res = set_inObs(status);
  if (res) {
    Tcl_SetResult(interp, "1", TCL_STATIC);
  }
  else {
    Tcl_SetResult(interp, "0", TCL_STATIC);
  }
  return TCL_OK;
}


static int setOnlySaveInObsCmd(ClientData clientData, Tcl_Interp *interp,
               int argc, char *argv[])
{
  int res;
  int status = -1;  /* used to get current value w/o setting */

  if (argc > 1) {
    if (Tcl_GetInt(interp, argv[1], &status) != TCL_OK)
      return TCL_ERROR;
  }
  res = set_onlySaveInObs(status);
  if (res) {
    Tcl_SetResult(interp, "1", TCL_STATIC);
  }
  else {
    Tcl_SetResult(interp, "0", TCL_STATIC);
  }
  return TCL_OK;
}

static int setFourCCCmd(ClientData clientData, Tcl_Interp *interp,
            int argc, char *argv[])
{
  int res;
  int status = -1;  /* used to get current value w/o setting */

  if (argc < 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " fourcc", TCL_STATIC);
    return TCL_ERROR;
  }
  res = set_fourCC(argv[1]);
  return TCL_OK;
}

static int addShutdownCmdCmd(ClientData clientData, Tcl_Interp *interp,
                 int argc, char *argv[])
{
  if (argc < 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " shutdownCmd", TCL_STATIC);
    return TCL_ERROR;
  }
  add_shutdown_command(argv[1]);
  return TCL_OK;
}

/*********************************************************************/
/*                         Display Commands                          */
/*********************************************************************/

static int showCmd(ClientData clientData, Tcl_Interp *interp,
           int argc, char *argv[])
{
  proginfo_t *p = (proginfo_t *) clientData;
  show_display(p);
  return TCL_OK;
}

static int hideCmd(ClientData clientData, Tcl_Interp *interp,
           int argc, char *argv[])
{
  proginfo_t *p = (proginfo_t *) clientData;
  hide_display(p);
  return TCL_OK;
}


/*********************************************************************/
/*                       Configure Commands                          */
/*********************************************************************/

static int configureExposureCmd(ClientData clientData, Tcl_Interp *interp,
                int argc, char *argv[])
{
  int res = 0;
  double exposure;

  if (argc < 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " exposure", TCL_STATIC);
    return TCL_ERROR;
  }
  if (Tcl_GetDouble(interp, argv[1], &exposure) != TCL_OK) return TCL_ERROR;
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* flirSource = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
  if (flirSource) {
    res = flirSource->configureExposure(exposure);
  }  
#endif
  if (res < 0) {
    Tcl_AppendResult(interp, argv[0], ": error configuring exposure", NULL);
    return TCL_ERROR;
  }
  return TCL_OK;
}

static int configureGainCmd(ClientData clientData, Tcl_Interp *interp,
                int argc, char *argv[])
{
  int res = 0;
  double gain;

  if (argc < 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " gain", TCL_STATIC);
    return TCL_ERROR;
  }
  if (Tcl_GetDouble(interp, argv[1], &gain) != TCL_OK) return TCL_ERROR;
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* flirSource = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
  if (flirSource) {
    res = flirSource->configureGain(gain);
  }  
#endif
  if (res < 0) {
    Tcl_AppendResult(interp, argv[0], ": error configuring gain", NULL);
    return TCL_ERROR;
  }
  return TCL_OK;
}

static int configureFrameRateCmd(ClientData clientData, Tcl_Interp *interp,
                 int argc, char *argv[])
{
  int res  = 0;
  double fr;

  if (argc < 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " framerate", TCL_STATIC);
    return TCL_ERROR;
  }
  if (Tcl_GetDouble(interp, argv[1], &fr) != TCL_OK) return TCL_ERROR;
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* flirSource = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
  if (flirSource) {
    res = flirSource->configureFrameRate(fr);
  }
#endif
  if (res < 0) {
    Tcl_AppendResult(interp, argv[0], ": error configuring framerate", NULL);
    return TCL_ERROR;
  }
  return TCL_OK;
}


static int configureROICmd(ClientData clientData, Tcl_Interp *interp,
               int argc, char *argv[])
{
  int res = 0;
  int w, h, x, y;

  if (argc < 5) {
    Tcl_AppendResult(interp, "usage: ", argv[0],
             " width height offsetx offsety", TCL_STATIC);
    return TCL_ERROR;
  }
  if (Tcl_GetInt(interp, argv[1], &w) != TCL_OK) return TCL_ERROR;
  if (Tcl_GetInt(interp, argv[2], &h) != TCL_OK) return TCL_ERROR;
  if (Tcl_GetInt(interp, argv[3], &x) != TCL_OK) return TCL_ERROR;
  if (Tcl_GetInt(interp, argv[4], &y) != TCL_OK) return TCL_ERROR;
#ifdef USE_FLIR
  extern IFrameSource* g_frameSource;
  FlirCameraSource* flirSource = 
    dynamic_cast<FlirCameraSource*>(g_frameSource);
  if (flirSource) {
    res = flirSource->configureROI(w, h, x, y);
  }
#endif
  if (res < 0) {
    Tcl_AppendResult(interp, argv[0], ": error configuring ROI", NULL);
    return TCL_ERROR;
  }
  return TCL_OK;
}



/*********************************************************************/
/*                       Shutdown Command                            */
/*********************************************************************/

static int shutdownCmd(ClientData clientData, Tcl_Interp *interp,
               int argc, char *argv[])
{
  do_shutdown();
  return TCL_OK;
}


void addTclCommands(Tcl_Interp *interp, proginfo_t *p)
{
  Tcl_CreateCommand(interp, "ping", (Tcl_CmdProc *) pingCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

  Tcl_Eval(interp, "namespace eval vstream {}");

  Tcl_CreateObjCommand(interp, "::vstream::startSource", startSourceCmd, p, NULL);
  Tcl_CreateObjCommand(interp, "::vstream::stopSource", stopSourceCmd, p, NULL);
  Tcl_CreateObjCommand(interp, "::vstream::getSourceStatus", getSourceStatusCmd, p, NULL);

  Tcl_CreateObjCommand(interp, "::vstream::reviewClear", reviewClearCmd, 
		       (ClientData)p, (Tcl_CmdDeleteProc *)NULL);
  Tcl_CreateObjCommand(interp, "::vstream::reviewSample", reviewSampleCmd,
		       (ClientData)p, (Tcl_CmdDeleteProc *)NULL);
  Tcl_CreateObjCommand(interp, "::vstream::reviewRemove", reviewRemoveCmd,
		       (ClientData)p, (Tcl_CmdDeleteProc *)NULL);
  Tcl_CreateObjCommand(interp, "::vstream::reviewNext", reviewNextCmd,
		       (ClientData)p, (Tcl_CmdDeleteProc *)NULL);
  Tcl_CreateObjCommand(interp, "::vstream::reviewPrevious", reviewPreviousCmd,
		       (ClientData)p, (Tcl_CmdDeleteProc *)NULL);
  Tcl_CreateObjCommand(interp, "::vstream::reviewJumpTo", reviewJumpToCmd,
		       (ClientData)p, (Tcl_CmdDeleteProc *)NULL);
  Tcl_CreateObjCommand(interp, "::vstream::reviewCount", reviewCountCmd,
		       (ClientData)p, (Tcl_CmdDeleteProc *)NULL);
  Tcl_CreateObjCommand(interp, "::vstream::reviewIndex", reviewIndexCmd,
		       (ClientData)p, (Tcl_CmdDeleteProc *)NULL);

  
  Tcl_CreateCommand(interp, "vstream::fileOpen", (Tcl_CmdProc *) openFileCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "vstream::fileClose",
            (Tcl_CmdProc *) closeFileCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

  Tcl_CreateCommand(interp, "vstream::domainSocketOpen", 
            (Tcl_CmdProc *) openDomainSocketCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "vstream::domainSocketClose",
            (Tcl_CmdProc *) closeDomainSocketCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "vstream::domainSocketSendN",
            (Tcl_CmdProc *) sendNDomainSocketCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
            

  Tcl_CreateCommand(interp, "vstream::inObs", (Tcl_CmdProc *) setInObsCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "vstream::onlySaveInObs", (Tcl_CmdProc *) setOnlySaveInObsCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "vstream::fourcc", (Tcl_CmdProc *) setFourCCCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

  Tcl_CreateCommand(interp, "vstream::addShutdownCmd",
            (Tcl_CmdProc *) addShutdownCmdCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  
  Tcl_CreateCommand(interp, "vstream::displayOpen", (Tcl_CmdProc *) showCmd, 
		    (ClientData) p, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "vstream::displayClose", (Tcl_CmdProc *) hideCmd, 
		    (ClientData) p, (Tcl_CmdDeleteProc *) NULL);

  Tcl_CreateCommand(interp, "vstream::configureExposure",
            (Tcl_CmdProc *) configureExposureCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "vstream::configureGain",
            (Tcl_CmdProc *) configureGainCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "vstream::configureFrameRate",
            (Tcl_CmdProc *) configureFrameRateCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "vstream::configureROI",
            (Tcl_CmdProc *) configureROICmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  
  Tcl_CreateCommand(interp, "vstream::shutdown", (Tcl_CmdProc *) shutdownCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateCommand(interp, "vstream::exit", (Tcl_CmdProc *) shutdownCmd, 
            (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

  extern int dsPort;
  Tcl_LinkVar(interp, "vstream::dsPort", (char *) &dsPort, TCL_LINK_INT);

  extern int displayEvery;
  Tcl_LinkVar(interp, "vstream::displayEvery", 
          (char *) &displayEvery, TCL_LINK_INT);

const char *ds_str = R"V0G0N(
  proc vstream::dsRegister { server { port 4620 } } {
	set s [socket $server $port]
	fconfigure $s -buffering line
	set our_ip [lindex [fconfigure $s -sockname] 0]
	puts $s "%reg $our_ip $::vstream::dsPort"
	catch {gets $s status}
	close $s
        if { $status } { 
           set ::vstream::dsServerIP $server 
           set ::vstream::dsServerPort $port
        }
	return $status
    }

    proc vstream::dsUnregister { } {
        if { ![info exists ::vstream::dsServerIP] } { 
             return 0 
        } else {
             set server $::vstream::dsServerIP
        }
        if { ![info exists ::vstream::dsServerPort] } { 
             return 0 
        } else {
             set port $::vstream::dsServerPort
        }
	set s [socket $server $port]
	fconfigure $s -buffering line
	set our_ip [lindex [fconfigure $s -sockname] 0]
	puts $s "%unreg $our_ip $::vstream::dsPort"
	catch {gets $s status}
	close $s
        unset ::vstream::dsServerIP ::vstream::dsServerPort
	return $status
    }

    proc vstream::dsAddMatch { server pattern { port 4620 } } {
	set s [socket $server $port]
	fconfigure $s -buffering line
	set our_ip [lindex [fconfigure $s -sockname] 0]
	puts $s "%match $our_ip $::vstream::dsPort $pattern 1"
	catch {gets $s status}
	close $s
	return $status
    }

    proc vstream::dsRemoveMatch { server pattern { port 4620 } } {
	set s [socket $server $port]
	fconfigure $s -buffering line
	set our_ip [lindex [fconfigure $s -sockname] 0]
	puts $s "%unmatch $our_ip $::vstream::dsPort $pattern"
	catch {gets $s status}
	close $s
	return $status
    }
)V0G0N";

  Tcl_Eval(interp, ds_str);

  Tcl_LinkVar(interp, "vstream::show_chunk",
          (char *) &ShowChunk, TCL_LINK_BOOLEAN);
}




