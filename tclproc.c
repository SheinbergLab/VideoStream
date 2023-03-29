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
  res = configure_exposure(exposure);
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
  res = configure_gain(gain);
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
  res = configure_framerate(fr);
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
  res = configure_ROI(w, h, x, y);
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

  extern int useWebcam;
  Tcl_LinkVar(interp, "vstream::useWebcam", (char *) &useWebcam, TCL_LINK_INT);

  extern int displayEvery;
  Tcl_LinkVar(interp, "vstream::displayEvery", 
          (char *) &useWebcam, TCL_LINK_INT);

#if 0
  const char *ds_str = R"V0G0N(
  proc vstream::dsRegister { server { port 4620 } } {
    set s [socket $server $port]
    fconfigure $s -buffering line
    set our_ip [lindex [fconfigure $s -sockname] 0]
    puts $s "%reg $our_ip $::vstream::dsPort"
    catch {gets $s status}
    close $s
    return $status
    }

    proc vstream::dsUnregister { server { port 4620 } } {
    set s [socket $server $port]
    fconfigure $s -buffering line
    set our_ip [lindex [fconfigure $s -sockname] 0]
    puts $s "%unreg $our_ip $::vstream::dsPort"
    catch {gets $s status}
    close $s
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
#endif
  extern const char *ds_str;
  Tcl_Eval(interp, ds_str);

  Tcl_LinkVar(interp, "vstream::show_chunk",
          (char *) &ShowChunk, TCL_LINK_BOOLEAN);
}




