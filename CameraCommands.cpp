// Vendor-neutral camera Tcl commands. Registered under one or more
// namespaces (camera::, flir::, lucid::); each command finds the active
// source through ICameraControl, so FLIR and Lucid share this file.

#include <iostream>
#include <map>
#include <string>
#include <tcl.h>

#include "VstreamEvent.h"
#include "IFrameSource.h"
#include "CameraControl.h"

extern IFrameSource* g_frameSource;

/*********************************************************************/
/*                     ICameraControl events                         */
/*********************************************************************/

void ICameraControl::fireSettingChanged(const std::string& setting_name,
                                        const std::string& value) {
  std::map<std::string, std::string> data;
  data["name"] = setting_name;
  data["value"] = value;

  VstreamEvent evt(std::string(vendorName()) + "/settings",
                   VstreamEventData::makeKeyValue(data));
  evt.rate_limit_exempt = true;
  fireEvent(evt);
}

void ICameraControl::fireAllSettings() {
  fireSettingChanged("exposure_time", std::to_string(settings_.exposure_time));
  fireSettingChanged("gain", std::to_string(settings_.gain));
  fireSettingChanged("frame_rate", std::to_string(settings_.frame_rate));
  fireSettingChanged("acquisition_running",
                     settings_.acquisition_running ? "1" : "0");
  fireSettingChanged("ttl_line", std::to_string(getTTLLine()));
}

/*********************************************************************/
/*                          helpers                                  */
/*********************************************************************/

struct CameraNamespace {
  std::string ns;
  bool available;
};

// Active source as a camera, or nullptr with the Tcl error set.
static ICameraControl* requireCamera(ClientData clientData, Tcl_Interp* interp,
                                     const char* cmd) {
  CameraNamespace* info = static_cast<CameraNamespace*>(clientData);
  if (!info->available) {
    Tcl_AppendResult(interp, cmd, ": ", info->ns.c_str(),
                     " support not compiled", NULL);
    return nullptr;
  }
  ICameraControl* cam = dynamic_cast<ICameraControl*>(g_frameSource);
  if (!cam) {
    Tcl_AppendResult(interp, cmd, ": camera not active", NULL);
  }
  return cam;
}

static Tcl_Obj* roiDict(Tcl_Interp* interp, ICameraControl* cam) {
  Tcl_Obj* d = Tcl_NewDictObj();
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("width", -1),
                 Tcl_NewIntObj(cam->getWidth()));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("height", -1),
                 Tcl_NewIntObj(cam->getHeight()));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("offset_x", -1),
                 Tcl_NewIntObj(cam->getOffsetX()));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("offset_y", -1),
                 Tcl_NewIntObj(cam->getOffsetY()));
  return d;
}

static Tcl_Obj* offsetDict(Tcl_Interp* interp, ICameraControl* cam) {
  Tcl_Obj* d = Tcl_NewDictObj();
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("offset_x", -1),
                 Tcl_NewIntObj(cam->getOffsetX()));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("offset_y", -1),
                 Tcl_NewIntObj(cam->getOffsetY()));
  return d;
}

static Tcl_Obj* binningDict(Tcl_Interp* interp, ICameraControl* cam) {
  Tcl_Obj* d = Tcl_NewDictObj();
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("horizontal", -1),
                 Tcl_NewIntObj(cam->getBinningH()));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("vertical", -1),
                 Tcl_NewIntObj(cam->getBinningV()));
  return d;
}

/*********************************************************************/
/*                      camera commands                              */
/*********************************************************************/

// <ns>::isAvailable -> was this backend compiled in
static int isAvailableCmd(ClientData clientData, Tcl_Interp *interp,
                          int objc, Tcl_Obj *const objv[]) {
  if (objc != 1) {
    Tcl_WrongNumArgs(interp, 1, objv, "");
    return TCL_ERROR;
  }
  CameraNamespace* info = static_cast<CameraNamespace*>(clientData);
  Tcl_SetObjResult(interp, Tcl_NewBooleanObj(info->available));
  return TCL_OK;
}

static int getSettingsCmd(ClientData clientData, Tcl_Interp *interp,
                          int objc, Tcl_Obj *const objv[]) {
  ICameraControl* cam = requireCamera(clientData, interp, Tcl_GetString(objv[0]));
  if (!cam) return TCL_ERROR;

  Tcl_Obj* settingsDict = Tcl_NewDictObj();
  Tcl_DictObjPut(interp, settingsDict,
                 Tcl_NewStringObj("exposure_time", -1),
                 Tcl_NewDoubleObj(cam->settings_.exposure_time));
  Tcl_DictObjPut(interp, settingsDict,
                 Tcl_NewStringObj("gain", -1),
                 Tcl_NewDoubleObj(cam->settings_.gain));
  Tcl_DictObjPut(interp, settingsDict,
                 Tcl_NewStringObj("frame_rate", -1),
                 Tcl_NewDoubleObj(cam->settings_.frame_rate));
  Tcl_DictObjPut(interp, settingsDict,
                 Tcl_NewStringObj("acquisition_running", -1),
                 Tcl_NewBooleanObj(cam->settings_.acquisition_running));
  Tcl_DictObjPut(interp, settingsDict,
                 Tcl_NewStringObj("ttl_line", -1),
                 Tcl_NewIntObj(cam->getTTLLine()));
  Tcl_DictObjPut(interp, settingsDict,
                 Tcl_NewStringObj("vendor", -1),
                 Tcl_NewStringObj(cam->vendorName(), -1));
  Tcl_SetObjResult(interp, settingsDict);
  return TCL_OK;
}

static int refreshSettingsCmd(ClientData clientData, Tcl_Interp *interp,
                              int objc, Tcl_Obj *const objv[]) {
  ICameraControl* cam = requireCamera(clientData, interp, Tcl_GetString(objv[0]));
  if (!cam) return TCL_ERROR;
  cam->fireAllSettings();
  return TCL_OK;
}

static int startAcquisitionCmd(ClientData clientData, Tcl_Interp *interp,
                               int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;

  if (!cam->startAcquisition()) {
    Tcl_AppendResult(interp, argv[0], ": error starting acquisition", NULL);
    return TCL_ERROR;
  }
  return TCL_OK;
}

static int stopAcquisitionCmd(ClientData clientData, Tcl_Interp *interp,
                              int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;

  if (!cam->stopAcquisition()) {
    Tcl_AppendResult(interp, argv[0], ": error stopping acquisition", NULL);
    return TCL_ERROR;
  }
  return TCL_OK;
}

static int isStreamingCmd(ClientData clientData, Tcl_Interp *interp,
                          int argc, char *argv[])
{
  ICameraControl* cam = dynamic_cast<ICameraControl*>(g_frameSource);
  Tcl_SetResult(interp, (char*)((cam && cam->isStreaming()) ? "1" : "0"), TCL_STATIC);
  return TCL_OK;
}

static int configureImageOrientationCmd(ClientData clientData, Tcl_Interp *interp,
                                        int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;

  if (argc == 1) {
    Tcl_AppendResult(interp, "Query not implemented - use: ", argv[0],
                     " reverseX reverseY", NULL);
    return TCL_ERROR;
  }

  if (argc != 3) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " reverseX reverseY", NULL);
    return TCL_ERROR;
  }

  int reverseX, reverseY;
  if (Tcl_GetInt(interp, argv[1], &reverseX) != TCL_OK ||
      Tcl_GetInt(interp, argv[2], &reverseY) != TCL_OK) {
    return TCL_ERROR;
  }

  if (!cam->configureImageOrientation(reverseX, reverseY)) {
    Tcl_AppendResult(interp, argv[0], ": failed to configure orientation", NULL);
    return TCL_ERROR;
  }
  return TCL_OK;
}

static int configureExposureCmd(ClientData clientData, Tcl_Interp *interp,
                                int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;

  // Query mode - return current exposure
  if (argc == 1) {
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(cam->settings_.exposure_time));
    return TCL_OK;
  }

  if (argc != 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " ?exposure?", NULL);
    return TCL_ERROR;
  }

  double exposure;
  if (Tcl_GetDouble(interp, argv[1], &exposure) != TCL_OK) {
    return TCL_ERROR;
  }

  if (!cam->configureExposure(exposure)) {
    Tcl_AppendResult(interp, argv[0], ": error configuring exposure", NULL);
    return TCL_ERROR;
  }
  Tcl_SetObjResult(interp, Tcl_NewDoubleObj(cam->settings_.exposure_time));
  return TCL_OK;
}

static int configureGainCmd(ClientData clientData, Tcl_Interp *interp,
                            int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;

  if (argc == 1) {
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(cam->settings_.gain));
    return TCL_OK;
  }

  if (argc != 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " ?gain?", NULL);
    return TCL_ERROR;
  }

  double gain;
  if (Tcl_GetDouble(interp, argv[1], &gain) != TCL_OK) {
    return TCL_ERROR;
  }

  if (!cam->configureGain(gain)) {
    Tcl_AppendResult(interp, argv[0], ": error configuring gain", NULL);
    return TCL_ERROR;
  }
  Tcl_SetObjResult(interp, Tcl_NewDoubleObj(cam->settings_.gain));
  return TCL_OK;
}

static int configureFrameRateCmd(ClientData clientData, Tcl_Interp *interp,
                                 int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;

  if (argc == 1) {
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(cam->getFrameRate()));
    return TCL_OK;
  }

  if (argc != 2) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " ?frameRate?", NULL);
    return TCL_ERROR;
  }

  double requested_fr;
  if (Tcl_GetDouble(interp, argv[1], &requested_fr) != TCL_OK) {
    return TCL_ERROR;
  }

  float actual_fr;
  if (!cam->configureFrameRate(requested_fr, &actual_fr)) {
    Tcl_AppendResult(interp, argv[0], ": error configuring frame rate", NULL);
    return TCL_ERROR;
  }
  Tcl_SetObjResult(interp, Tcl_NewDoubleObj(actual_fr));
  return TCL_OK;
}

static int getFrameRateRangeCmd(ClientData clientData, Tcl_Interp *interp,
                                int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;

  float min_fps, max_fps;
  if (!cam->getFrameRateRange(min_fps, max_fps)) {
    Tcl_AppendResult(interp, argv[0], ": failed to get frame rate range", NULL);
    return TCL_ERROR;
  }

  Tcl_Obj* resultDict = Tcl_NewDictObj();
  Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("min", -1),
                 Tcl_NewDoubleObj(min_fps));
  Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("max", -1),
                 Tcl_NewDoubleObj(max_fps));
  Tcl_SetObjResult(interp, resultDict);
  return TCL_OK;
}

static int getROIConstraintsCmd(ClientData clientData, Tcl_Interp *interp,
                                int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;

  ICameraControl::ROIConstraints c;
  if (!cam->getROIConstraints(c)) {
    Tcl_AppendResult(interp, argv[0], ": failed to get constraints", NULL);
    return TCL_ERROR;
  }

  Tcl_Obj* d = Tcl_NewDictObj();
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("width_min", -1), Tcl_NewIntObj(c.width_min));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("width_max", -1), Tcl_NewIntObj(c.width_max));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("width_inc", -1), Tcl_NewIntObj(c.width_inc));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("height_min", -1), Tcl_NewIntObj(c.height_min));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("height_max", -1), Tcl_NewIntObj(c.height_max));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("height_inc", -1), Tcl_NewIntObj(c.height_inc));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("offset_x_min", -1), Tcl_NewIntObj(c.offset_x_min));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("offset_x_max", -1), Tcl_NewIntObj(c.offset_x_max));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("offset_x_inc", -1), Tcl_NewIntObj(c.offset_x_inc));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("offset_y_min", -1), Tcl_NewIntObj(c.offset_y_min));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("offset_y_max", -1), Tcl_NewIntObj(c.offset_y_max));
  Tcl_DictObjPut(interp, d, Tcl_NewStringObj("offset_y_inc", -1), Tcl_NewIntObj(c.offset_y_inc));
  Tcl_SetObjResult(interp, d);
  return TCL_OK;
}

static int configureBinningCmd(ClientData clientData, Tcl_Interp *interp,
                               int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;

  if (argc == 1) {
    Tcl_SetObjResult(interp, binningDict(interp, cam));
    return TCL_OK;
  }

  if (argc != 3) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " ?horizontal vertical?", NULL);
    return TCL_ERROR;
  }

  int h, v;
  if (Tcl_GetInt(interp, argv[1], &h) != TCL_OK ||
      Tcl_GetInt(interp, argv[2], &v) != TCL_OK) {
    return TCL_ERROR;
  }

  if (!cam->configureBinning(h, v)) {
    Tcl_AppendResult(interp, argv[0], ": failed to configure binning", NULL);
    return TCL_ERROR;
  }
  Tcl_SetObjResult(interp, binningDict(interp, cam));
  return TCL_OK;
}

static int configureROICmd(ClientData clientData, Tcl_Interp *interp,
                           int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;

  if (argc == 1) {
    Tcl_SetObjResult(interp, roiDict(interp, cam));
    return TCL_OK;
  }

  if (argc != 5) {
    Tcl_AppendResult(interp, "usage: ", argv[0],
                     " ?width height offsetX offsetY?", NULL);
    return TCL_ERROR;
  }

  int w, h, x, y;
  if (Tcl_GetInt(interp, argv[1], &w) != TCL_OK) return TCL_ERROR;
  if (Tcl_GetInt(interp, argv[2], &h) != TCL_OK) return TCL_ERROR;
  if (Tcl_GetInt(interp, argv[3], &x) != TCL_OK) return TCL_ERROR;
  if (Tcl_GetInt(interp, argv[4], &y) != TCL_OK) return TCL_ERROR;

  if (!cam->configureROI(w, h, x, y)) {
    Tcl_AppendResult(interp, argv[0], ": error configuring ROI", NULL);
    return TCL_ERROR;
  }
  Tcl_SetObjResult(interp, roiDict(interp, cam));
  return TCL_OK;
}

static int getROICmd(ClientData clientData, Tcl_Interp *interp,
                     int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;
  Tcl_SetObjResult(interp, roiDict(interp, cam));
  return TCL_OK;
}

static int setROIOffsetCmd(ClientData clientData, Tcl_Interp *interp,
                           int argc, char *argv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, argv[0]);
  if (!cam) return TCL_ERROR;

  if (argc == 1) {
    Tcl_SetObjResult(interp, offsetDict(interp, cam));
    return TCL_OK;
  }

  if (argc != 3) {
    Tcl_AppendResult(interp, "usage: ", argv[0], " ?offsetX offsetY?", NULL);
    return TCL_ERROR;
  }

  int offset_x, offset_y;
  if (Tcl_GetInt(interp, argv[1], &offset_x) != TCL_OK ||
      Tcl_GetInt(interp, argv[2], &offset_y) != TCL_OK) {
    return TCL_ERROR;
  }

  if (!cam->setROIOffset(offset_x, offset_y)) {
    Tcl_AppendResult(interp, argv[0], ": failed to set offset", NULL);
    return TCL_ERROR;
  }
  Tcl_SetObjResult(interp, offsetDict(interp, cam));
  return TCL_OK;
}

// <ns>::ttlLine ?line? -> query/set which I/O line stamps frame line_status
static int ttlLineCmd(ClientData clientData, Tcl_Interp *interp,
                      int objc, Tcl_Obj *const objv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, Tcl_GetString(objv[0]));
  if (!cam) return TCL_ERROR;

  if (objc == 1) {
    Tcl_SetObjResult(interp, Tcl_NewIntObj(cam->getTTLLine()));
    return TCL_OK;
  }

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "?line?");
    return TCL_ERROR;
  }

  int line;
  if (Tcl_GetIntFromObj(interp, objv[1], &line) != TCL_OK) {
    return TCL_ERROR;
  }

  if (!cam->setTTLLine(line)) {
    Tcl_SetObjResult(interp, Tcl_ObjPrintf("cannot select line %d", line));
    return TCL_ERROR;
  }

  Tcl_SetObjResult(interp, Tcl_NewIntObj(cam->getTTLLine()));
  return TCL_OK;
}

// <ns>::lineStatusAll -> live bitfield of all I/O lines (bit N = LineN);
// toggle the TTL while watching this to find the wired line
static int lineStatusAllCmd(ClientData clientData, Tcl_Interp *interp,
                            int objc, Tcl_Obj *const objv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, Tcl_GetString(objv[0]));
  if (!cam) return TCL_ERROR;

  int64_t all = cam->getLineStatusAll();
  if (all < 0) {
    Tcl_SetResult(interp, (char*)"LineStatusAll not readable", TCL_STATIC);
    return TCL_ERROR;
  }

  Tcl_SetObjResult(interp, Tcl_NewWideIntObj((Tcl_WideInt)all));
  return TCL_OK;
}

// <ns>::vendor -> "flir" / "lucid" of the active camera
static int vendorCmd(ClientData clientData, Tcl_Interp *interp,
                     int objc, Tcl_Obj *const objv[])
{
  ICameraControl* cam = requireCamera(clientData, interp, Tcl_GetString(objv[0]));
  if (!cam) return TCL_ERROR;
  Tcl_SetObjResult(interp, Tcl_NewStringObj(cam->vendorName(), -1));
  return TCL_OK;
}

/*********************************************************************/
/*                        registration                               */
/*********************************************************************/

int add_camera_commands(Tcl_Interp *interp, const char* ns, bool available)
{
  // lives for the life of the interpreter
  CameraNamespace* info = new CameraNamespace{ns, available};
  ClientData cd = static_cast<ClientData>(info);
  std::string p = std::string(ns) + "::";

  auto objcmd = [&](const char* name, Tcl_ObjCmdProc* proc) {
    Tcl_CreateObjCommand(interp, (p + name).c_str(), proc, cd, NULL);
  };
  auto cmd = [&](const char* name, Tcl_CmdProc* proc) {
    Tcl_CreateCommand(interp, (p + name).c_str(), proc, cd,
                      (Tcl_CmdDeleteProc*)NULL);
  };

  objcmd("isAvailable", isAvailableCmd);
  objcmd("getSettings", getSettingsCmd);
  objcmd("refreshSettings", refreshSettingsCmd);
  objcmd("vendor", vendorCmd);

  cmd("startAcquisition", (Tcl_CmdProc*)startAcquisitionCmd);
  cmd("stopAcquisition", (Tcl_CmdProc*)stopAcquisitionCmd);
  cmd("isStreaming", (Tcl_CmdProc*)isStreamingCmd);
  cmd("configureImageOrientation", (Tcl_CmdProc*)configureImageOrientationCmd);
  cmd("configureExposure", (Tcl_CmdProc*)configureExposureCmd);
  cmd("configureGain", (Tcl_CmdProc*)configureGainCmd);
  cmd("configureFrameRate", (Tcl_CmdProc*)configureFrameRateCmd);
  cmd("getFrameRateRange", (Tcl_CmdProc*)getFrameRateRangeCmd);
  cmd("configureBinning", (Tcl_CmdProc*)configureBinningCmd);
  cmd("getROIConstraints", (Tcl_CmdProc*)getROIConstraintsCmd);
  cmd("configureROI", (Tcl_CmdProc*)configureROICmd);
  cmd("getROI", (Tcl_CmdProc*)getROICmd);
  cmd("setROIOffset", (Tcl_CmdProc*)setROIOffsetCmd);

  objcmd("ttlLine", ttlLineCmd);
  objcmd("lineStatusAll", lineStatusAllCmd);
  return TCL_OK;
}
