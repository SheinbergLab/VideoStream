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
#include "SamplingManager.h"
#include "FrameBufferManager.h"
#include "KeyboardCallbackRegistry.h"
#include "Widget.h"
#include "WidgetManager.h"

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
/*                       Keyboard bindings                           */
/*********************************************************************/

// tclproc.cpp

// bind_key key callback
// supports special keys like arrows, function keys, etc.
// In tclproc.cpp

int TclCmd_BindKey(ClientData clientData, Tcl_Interp *interp, 
                   int objc, Tcl_Obj *objv[]) {
    if (objc != 3) {
        Tcl_WrongNumArgs(interp, 1, objv, "key callback");
        return TCL_ERROR;
    }
    
    const char* keyStr = Tcl_GetString(objv[1]);
    std::string callback = Tcl_GetString(objv[2]);
    
    int key;
    size_t len = strlen(keyStr);
    
    if (len == 0) {
        // Empty string means catch-all handler
        key = -1;
    } else if (keyStr[0] == '<' && keyStr[len-1] == '>' && len > 2) {
        // Angle bracket syntax: <keycode>
        std::string codeStr(keyStr + 1, len - 2);  // Extract between < and >
        
        char* endptr;
        key = strtol(codeStr.c_str(), &endptr, 10);
        if (*endptr != '\0') {
            Tcl_SetObjResult(interp, 
                Tcl_NewStringObj("invalid keycode in angle brackets", -1));
            return TCL_ERROR;
        }
    } else if (len == 1) {
        // Single character - use its ASCII value
        key = static_cast<int>(keyStr[0]);
    } else {
        Tcl_SetObjResult(interp, 
            Tcl_NewStringObj("key must be single character, <keycode>, or empty string", -1));
        return TCL_ERROR;
    }
    
    g_keyboardCallbacks.registerCallback(key, callback);
    return TCL_OK;
}

int TclCmd_UnbindKey(ClientData clientData, Tcl_Interp *interp, 
                     int objc, Tcl_Obj *objv[]) {
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "key");
        return TCL_ERROR;
    }
    
    const char* keyStr = Tcl_GetString(objv[1]);
    
    int key;
    size_t len = strlen(keyStr);
    
    if (len == 0) {
        key = -1;
    } else if (keyStr[0] == '<' && keyStr[len-1] == '>' && len > 2) {
        // Angle bracket syntax
        std::string codeStr(keyStr + 1, len - 2);
        
        char* endptr;
        key = strtol(codeStr.c_str(), &endptr, 10);
        if (*endptr != '\0') {
            Tcl_SetObjResult(interp, 
                Tcl_NewStringObj("invalid keycode in angle brackets", -1));
            return TCL_ERROR;
        }
    } else if (len == 1) {
        key = static_cast<int>(keyStr[0]);
    } else {
        Tcl_SetObjResult(interp, 
            Tcl_NewStringObj("key must be single character, <keycode>, or empty string", -1));
        return TCL_ERROR;
    }
    
    g_keyboardCallbacks.unregisterCallback(key);
    return TCL_OK;
}

int TclCmd_ListKeyBindings(ClientData clientData, Tcl_Interp *interp, 
                           int objc, Tcl_Obj *objv[]) {
    auto keys = g_keyboardCallbacks.getRegisteredKeys();
    
    Tcl_Obj* resultDict = Tcl_NewDictObj();
    
    for (int key : keys) {
        Tcl_Obj* keyObj;
        
        if (key == -1) {
            // Catch-all handler
            keyObj = Tcl_NewStringObj("<catch-all>", -1);
        } else if (key >= 32 && key < 127) {
            // Printable ASCII - show as character
            char keyStr[2] = {static_cast<char>(key), '\0'};
            keyObj = Tcl_NewStringObj(keyStr, 1);
        } else {
            // Special key - show as <keycode>
            std::string keycode = "<" + std::to_string(key) + ">";
            keyObj = Tcl_NewStringObj(keycode.c_str(), -1);
        }
        
        // Get the callback for this key
        std::string callback = g_keyboardCallbacks.getCallback(key);
        Tcl_Obj* callbackObj = Tcl_NewStringObj(callback.c_str(), -1);
        
        Tcl_DictObjPut(interp, resultDict, keyObj, callbackObj);
    }
    
    Tcl_SetObjResult(interp, resultDict);
    return TCL_OK;
}

int TclCmd_ClearKeyBindings(ClientData clientData, Tcl_Interp *interp, 
                            int objc, Tcl_Obj *objv[]) {
    g_keyboardCallbacks.clearAll();
    return TCL_OK;
}


/*********************************************************************/
/*                          UI/UX Commands                           */
/*********************************************************************/

// Helper to parse color from Tcl list {r g b}
cv::Scalar parseTclColor(Tcl_Interp* interp, Tcl_Obj* colorObj) {
    Tcl_Size listLen;
    Tcl_Obj** listObjs;
    Tcl_ListObjGetElements(interp, colorObj, &listLen, &listObjs);
    
    int r = 0, g = 0, b = 0;
    if (listLen >= 3) {
        Tcl_GetIntFromObj(interp, listObjs[0], &r);
        Tcl_GetIntFromObj(interp, listObjs[1], &g);
        Tcl_GetIntFromObj(interp, listObjs[2], &b);
    }
    return cv::Scalar(b, g, r);
}

// add_toggle x y width height label callback
int TclCmd_AddToggle(ClientData clientData, Tcl_Interp *interp, 
                     int objc, Tcl_Obj *objv[]) {
    if (objc < 7) {
        Tcl_WrongNumArgs(interp, 1, objv, "x y width height label callback");
        return TCL_ERROR;
    }
    
    auto* mgr = static_cast<WidgetManager*>(clientData);
    int x, y, w, h;
    
    if (Tcl_GetIntFromObj(interp, objv[1], &x) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[2], &y) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[3], &w) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[4], &h) != TCL_OK) return TCL_ERROR;
    
    std::string label = Tcl_GetString(objv[5]);
    std::string callback = Tcl_GetString(objv[6]);
    bool toggle = true;
    
    auto btn = std::make_unique<Button>(x, y, w, h, label, callback, toggle);
    int id = mgr->addWidget(std::move(btn));
    
    Tcl_SetObjResult(interp, Tcl_NewIntObj(id));
    return TCL_OK;
}

// add_button x y width height label callback
int TclCmd_AddButton(ClientData clientData, Tcl_Interp *interp, 
                     int objc, Tcl_Obj *objv[]) {
    if (objc < 7) {
        Tcl_WrongNumArgs(interp, 1, objv, "x y width height label callback");
        return TCL_ERROR;
    }
    
    auto* mgr = static_cast<WidgetManager*>(clientData);
    int x, y, w, h;
    
    if (Tcl_GetIntFromObj(interp, objv[1], &x) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[2], &y) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[3], &w) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[4], &h) != TCL_OK) return TCL_ERROR;

    std::string label = Tcl_GetString(objv[5]);
    std::string callback = Tcl_GetString(objv[6]);
    bool toggle = false;

    auto btn = std::make_unique<Button>(x, y, w, h, label, callback, toggle);
    int id = mgr->addWidget(std::move(btn));
    
    Tcl_SetObjResult(interp, Tcl_NewIntObj(id));
    return TCL_OK;
}

// add_circle x y radius {r g b} ?thickness? ?callback? ?draggable?
int TclCmd_AddCircle(ClientData clientData, Tcl_Interp *interp, 
                     int objc, Tcl_Obj *objv[]) {
    if (objc < 5) {
        Tcl_WrongNumArgs(interp, 1, objv, 
                        "x y radius color ?thickness? ?callback? ?draggable?");
        return TCL_ERROR;
    }
    
    auto* mgr = static_cast<WidgetManager*>(clientData);
    int x, y, radius;
    
    if (Tcl_GetIntFromObj(interp, objv[1], &x) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[2], &y) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[3], &radius) != TCL_OK) return TCL_ERROR;
    
    cv::Scalar color = parseTclColor(interp, objv[4]);
    
    int thickness = 2;
    std::string callback = "";
    bool draggable = false;
    
    if (objc > 5) {
        if (Tcl_GetIntFromObj(interp, objv[5], &thickness) != TCL_OK) {
            return TCL_ERROR;
        }
    }
    if (objc > 6) callback = Tcl_GetString(objv[6]);
    if (objc > 7) {
        int drag_val;
        if (Tcl_GetBooleanFromObj(interp, objv[7], &drag_val) != TCL_OK) {
            return TCL_ERROR;
        }
        draggable = (drag_val != 0);
    }
    
    auto circle = std::make_unique<CircleWidget>(x, y, radius, color, 
                                                  thickness, callback,
                                                  draggable);
    int id = mgr->addWidget(std::move(circle));
    
    Tcl_SetObjResult(interp, Tcl_NewIntObj(id));
    return TCL_OK;
}

// add_rect x y width height {r g b} ?thickness? ?callback?
int TclCmd_AddRect(ClientData clientData, Tcl_Interp *interp, 
                   int objc, Tcl_Obj *objv[]) {
    if (objc < 6) {
        Tcl_WrongNumArgs(interp, 1, objv, 
                        "x y width height color ?thickness? ?callback?");
        return TCL_ERROR;
    }
    
    auto* mgr = static_cast<WidgetManager*>(clientData);
    int x, y, w, h;
    
    if (Tcl_GetIntFromObj(interp, objv[1], &x) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[2], &y) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[3], &w) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[4], &h) != TCL_OK) return TCL_ERROR;
    
    cv::Scalar color = parseTclColor(interp, objv[5]);
    
    int thickness = 2;
    std::string callback = "";
    
    if (objc > 6) {
        if (Tcl_GetIntFromObj(interp, objv[6], &thickness) != TCL_OK) {
            return TCL_ERROR;
        }
    }
    if (objc > 7) callback = Tcl_GetString(objv[7]);
    
    auto rect = std::make_unique<RectWidget>(x, y, w, h, color, 
                                              thickness, callback);
    int id = mgr->addWidget(std::move(rect));
    
    Tcl_SetObjResult(interp, Tcl_NewIntObj(id));
    return TCL_OK;
}

// add_text x y text {r g b} ?scale? ?thickness?
int TclCmd_AddText(ClientData clientData, Tcl_Interp *interp, 
                   int objc, Tcl_Obj *objv[]) {
    if (objc < 5) {
        Tcl_WrongNumArgs(interp, 1, objv, 
                        "x y text color ?scale? ?thickness?");
        return TCL_ERROR;
    }
    
    auto* mgr = static_cast<WidgetManager*>(clientData);
    int x, y;
    
    if (Tcl_GetIntFromObj(interp, objv[1], &x) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[2], &y) != TCL_OK) return TCL_ERROR;
    
    std::string text = Tcl_GetString(objv[3]);
    cv::Scalar color = parseTclColor(interp, objv[4]);
    
    double scale = 0.5;
    int thickness = 1;
    
    if (objc > 5) {
        if (Tcl_GetDoubleFromObj(interp, objv[5], &scale) != TCL_OK) {
            return TCL_ERROR;
        }
    }
    if (objc > 6) {
        if (Tcl_GetIntFromObj(interp, objv[6], &thickness) != TCL_OK) {
            return TCL_ERROR;
        }
    }
    
    auto txt = std::make_unique<TextWidget>(x, y, text, color, 
                                            scale, thickness);
    int id = mgr->addWidget(std::move(txt));
    
    Tcl_SetObjResult(interp, Tcl_NewIntObj(id));
    return TCL_OK;
}

// add_line x1 y1 x2 y2 {r g b} ?thickness?
int TclCmd_AddLine(ClientData clientData, Tcl_Interp *interp, 
                   int objc, Tcl_Obj *objv[]) {
    if (objc < 6) {
        Tcl_WrongNumArgs(interp, 1, objv, "x1 y1 x2 y2 color ?thickness?");
        return TCL_ERROR;
    }
    
    auto* mgr = static_cast<WidgetManager*>(clientData);
    int x1, y1, x2, y2;
    
    if (Tcl_GetIntFromObj(interp, objv[1], &x1) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[2], &y1) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[3], &x2) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[4], &y2) != TCL_OK) return TCL_ERROR;
    
    cv::Scalar color = parseTclColor(interp, objv[5]);
    
    int thickness = 2;
    if (objc > 6) {
        if (Tcl_GetIntFromObj(interp, objv[6], &thickness) != TCL_OK) {
            return TCL_ERROR;
        }
    }
    
    auto line = std::make_unique<LineWidget>(x1, y1, x2, y2, color, thickness);
    int id = mgr->addWidget(std::move(line));
    
    Tcl_SetObjResult(interp, Tcl_NewIntObj(id));
    return TCL_OK;
}

// add_slider x y width height label min max initial ?callback?
int TclCmd_AddSlider(ClientData clientData, Tcl_Interp *interp, 
                     int objc, Tcl_Obj *objv[]) {
    if (objc < 9) {
        Tcl_WrongNumArgs(interp, 1, objv, 
                        "x y width height label min max initial ?callback?");
        return TCL_ERROR;
    }
    
    auto* mgr = static_cast<WidgetManager*>(clientData);
    int x, y, width, height;
    double min_val, max_val, initial;
    
    if (Tcl_GetIntFromObj(interp, objv[1], &x) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[2], &y) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[3], &width) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[4], &height) != TCL_OK) return TCL_ERROR;
    
    std::string label = Tcl_GetString(objv[5]);
    
    if (Tcl_GetDoubleFromObj(interp, objv[6], &min_val) != TCL_OK) {
        return TCL_ERROR;
    }
    if (Tcl_GetDoubleFromObj(interp, objv[7], &max_val) != TCL_OK) {
        return TCL_ERROR;
    }
    if (Tcl_GetDoubleFromObj(interp, objv[8], &initial) != TCL_OK) {
        return TCL_ERROR;
    }
    
    std::string callback = "";
    if (objc > 9) callback = Tcl_GetString(objv[9]);
    
    auto slider = std::make_unique<SliderWidget>(x, y, width, height, 
                                                  label, min_val, max_val, 
                                                  initial, callback);
    int id = mgr->addWidget(std::move(slider));
    
    Tcl_SetObjResult(interp, Tcl_NewIntObj(id));
    return TCL_OK;
}

// update_widget id x y ?w? ?h?
int TclCmd_UpdateWidget(ClientData clientData, Tcl_Interp *interp, 
                        int objc, Tcl_Obj *objv[]) {
    if (objc < 4) {
        Tcl_WrongNumArgs(interp, 1, objv, "widget_id x y ?w? ?h?");
        return TCL_ERROR;
    }
    
    auto* mgr = static_cast<WidgetManager*>(clientData);
    int id, x, y, w = -1, h = -1;
    
    if (Tcl_GetIntFromObj(interp, objv[1], &id) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[2], &x) != TCL_OK) return TCL_ERROR;
    if (Tcl_GetIntFromObj(interp, objv[3], &y) != TCL_OK) return TCL_ERROR;

    if (objc > 4) {
        if (Tcl_GetIntFromObj(interp, objv[4], &w) != TCL_OK) return TCL_ERROR;
    }
    
    if (objc > 5) {
        if (Tcl_GetIntFromObj(interp, objv[5], &h) != TCL_OK) return TCL_ERROR;
    }
    
    bool success = mgr->updateWidget(id, x, y, w, h);
    
    Tcl_SetObjResult(interp, Tcl_NewBooleanObj(success));
    return TCL_OK;
}

// update_widget_text id txt?
int TclCmd_UpdateWidgetText(ClientData clientData, Tcl_Interp *interp, 
                        int objc, Tcl_Obj *objv[]) {
  if (objc < 3) {
    Tcl_WrongNumArgs(interp, 1, objv, "widget_id string");
    return TCL_ERROR;
  }
  
  auto* mgr = static_cast<WidgetManager*>(clientData);
  int id;
  
  if (Tcl_GetIntFromObj(interp, objv[1], &id) != TCL_OK) return TCL_ERROR;

  bool success = mgr->updateWidgetText(id, Tcl_GetString(objv[2]));
  
  Tcl_SetObjResult(interp, Tcl_NewBooleanObj(success));
  return TCL_OK;
}

// remove_widget id
int TclCmd_RemoveWidget(ClientData clientData, Tcl_Interp *interp, 
                        int objc, Tcl_Obj *objv[]) {
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "widget_id");
        return TCL_ERROR;
    }
    
    auto* mgr = static_cast<WidgetManager*>(clientData);
    int id;
    
    if (Tcl_GetIntFromObj(interp, objv[1], &id) != TCL_OK) return TCL_ERROR;
    
    mgr->removeWidget(id);
    
    return TCL_OK;
}

// clear_widgets
int TclCmd_ClearWidgets(ClientData clientData, Tcl_Interp *interp, 
                        int objc, Tcl_Obj *objv[]) {
  auto* mgr = static_cast<WidgetManager*>(clientData);
  mgr->clearAll();
  return TCL_OK;
}

int TclCmd_SetVariable(ClientData clientData, Tcl_Interp *interp, 
                       int objc, Tcl_Obj *objv[]) {
    if (objc != 3) {
        Tcl_WrongNumArgs(interp, 1, objv, "name value");
        return TCL_ERROR;
    }
    
    auto* mgr = static_cast<WidgetManager*>(clientData);
    std::string name = Tcl_GetString(objv[1]);
    std::string value = Tcl_GetString(objv[2]);
    
    mgr->setVariable(name, value);
    return TCL_OK;
}

// get_variable name
int TclCmd_GetVariable(ClientData clientData, Tcl_Interp *interp, 
                       int objc, Tcl_Obj *objv[]) {
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "name");
        return TCL_ERROR;
    }
    
    auto* mgr = static_cast<WidgetManager*>(clientData);
    std::string name = Tcl_GetString(objv[1]);
    std::string value = mgr->getVariable(name);
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj(value.c_str(), -1));
    return TCL_OK;
}

// clear_variables
int TclCmd_ClearVariables(ClientData clientData, Tcl_Interp *interp, 
                          int objc, Tcl_Obj *objv[]) {
    auto* mgr = static_cast<WidgetManager*>(clientData);
    mgr->clearVariables();
    return TCL_OK;
}

void registerWidgetCommands(Tcl_Interp* interp, WidgetManager* mgr) {
  Tcl_CreateObjCommand(interp, "add_button",
		       (Tcl_ObjCmdProc *) TclCmd_AddButton, mgr, NULL);
  Tcl_CreateObjCommand(interp, "add_toggle",
		       (Tcl_ObjCmdProc *) TclCmd_AddToggle, mgr, NULL);
  Tcl_CreateObjCommand(interp, "add_circle",
		       (Tcl_ObjCmdProc *) TclCmd_AddCircle, mgr, NULL);
  Tcl_CreateObjCommand(interp, "add_rect",
		       (Tcl_ObjCmdProc *) TclCmd_AddRect, mgr, NULL);
  Tcl_CreateObjCommand(interp, "add_text",
		       (Tcl_ObjCmdProc *) TclCmd_AddText, mgr, NULL);
  Tcl_CreateObjCommand(interp, "add_line",
		       (Tcl_ObjCmdProc *) TclCmd_AddLine, mgr, NULL);
  Tcl_CreateObjCommand(interp, "add_slider",
		       (Tcl_ObjCmdProc *) TclCmd_AddSlider, mgr, NULL);

  Tcl_CreateObjCommand(interp, "set_variable",
		       (Tcl_ObjCmdProc *) TclCmd_SetVariable, mgr, NULL);
  Tcl_CreateObjCommand(interp, "get_variable",
		       (Tcl_ObjCmdProc *) TclCmd_GetVariable, mgr, NULL);
  Tcl_CreateObjCommand(interp, "clear_variables",
		       (Tcl_ObjCmdProc *) TclCmd_ClearVariables, mgr, NULL);
    
  Tcl_CreateObjCommand(interp, "update_widget",
		       (Tcl_ObjCmdProc *) TclCmd_UpdateWidget, mgr, NULL);
  Tcl_CreateObjCommand(interp, "update_widgetText",
		       (Tcl_ObjCmdProc *) TclCmd_UpdateWidgetText, mgr, NULL);
  Tcl_CreateObjCommand(interp, "remove_widget",
		       (Tcl_ObjCmdProc *) TclCmd_RemoveWidget, mgr, NULL);
  Tcl_CreateObjCommand(interp, "clear_widgets",
		       (Tcl_ObjCmdProc *) TclCmd_ClearWidgets, mgr, NULL);


  
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


int reviewSampleMultipleCmd(ClientData data, Tcl_Interp *interp,
                            int objc, Tcl_Obj *const objv[])
{
  proginfo_t *p = (proginfo_t *)data;
    
  if (objc < 3) {
    Tcl_WrongNumArgs(interp, 1, objv, "n_frames interval_ms ?random?");
    return TCL_ERROR;
  }
    
  int n_frames, interval_ms;
  if (Tcl_GetIntFromObj(interp, objv[1], &n_frames) != TCL_OK ||
      Tcl_GetIntFromObj(interp, objv[2], &interval_ms) != TCL_OK) {
    return TCL_ERROR;
  }
  
  int random = 0;
  if (objc > 3 && strcmp(Tcl_GetString(objv[3]), "-random") == 0) random = 1;
  else if (objc > 3) {
    if (Tcl_GetIntFromObj(interp, objv[1], &random) != TCL_OK) {
      return TCL_ERROR;
    }
  }
    
  if (p->samplingManager->isActive()) {
    Tcl_SetObjResult(interp, Tcl_NewStringObj("Sampling already in progress", -1));
    return TCL_ERROR;
  }
  
  p->samplingManager->start(n_frames, interval_ms, random);
  return TCL_OK;
}

int reviewSampleStopCmd(ClientData data, Tcl_Interp *interp,
                        int objc, Tcl_Obj *const objv[])
{
  proginfo_t *p = (proginfo_t *)data;
  p->samplingManager->stop();
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
  Tcl_CreateObjCommand(interp, "::vstream::reviewSampleMultiple",
		       reviewSampleMultipleCmd,
		       (ClientData)p, (Tcl_CmdDeleteProc *)NULL);
  Tcl_CreateObjCommand(interp, "::vstream::reviewSampleStop", reviewSampleStopCmd,
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


  Tcl_CreateObjCommand(interp, "bind_key",
		       (Tcl_ObjCmdProc *) TclCmd_BindKey, 
		       (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateObjCommand(interp, "unbind_key",
		       (Tcl_ObjCmdProc *) TclCmd_UnbindKey, 
		       (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateObjCommand(interp, "clear_key_bindings",
		       (Tcl_ObjCmdProc *) TclCmd_ClearKeyBindings, 
		       (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
  Tcl_CreateObjCommand(interp, "list_key_bindings",
		       (Tcl_ObjCmdProc *) TclCmd_ListKeyBindings, 
		       (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    
  registerWidgetCommands(interp, p->widgetManager);

  // access to frame info
  Tcl_LinkVar(interp, "vstream::frame_width", (char *) p->frame_width,
	      TCL_LINK_INT | TCL_LINK_READ_ONLY);
  Tcl_LinkVar(interp, "vstream::frame_height", (char *) p->frame_height,
	      TCL_LINK_INT | TCL_LINK_READ_ONLY);
  Tcl_LinkVar(interp, "vstream::frame_rate", (char*)p->frame_rate, 
	      TCL_LINK_FLOAT | TCL_LINK_READ_ONLY);
  Tcl_LinkVar(interp, "vstream::is_color", (char*)p->is_color, 
	      TCL_LINK_BOOLEAN | TCL_LINK_READ_ONLY);
  
  extern int dsPort;
  Tcl_LinkVar(interp, "vstream::dsPort", (char *) &dsPort, TCL_LINK_INT);

  extern int displayEvery;
  Tcl_LinkVar(interp, "vstream::displayEvery", 
          (char *) &displayEvery, TCL_LINK_INT);

// In tclproc.cpp - add to addTclCommands function

const char *key_constants = R"TCL(
# OpenCV key codes for special keys with waitKeyEx()
# Platform-specific values detected automatically

namespace eval keys {
    # Detect platform
    variable platform [string tolower $::tcl_platform(os)]
    
    # Arrow keys - waitKeyEx() returns different codes per platform
    if {[string match "*darwin*" $platform] || [string match "*mac*" $platform]} {
        # macOS NSEvent codes (with waitKeyEx)
        variable UP      "<63232>"
        variable DOWN    "<63233>"
        variable LEFT    "<63234>"
        variable RIGHT   "<63235>"
        
        # Function keys on macOS
        variable F1      "<63236>"
        variable F2      "<63237>"
        variable F3      "<63238>"
        variable F4      "<63239>"
        variable F5      "<63240>"
        
        # Navigation keys
        variable PAGEUP   "<63276>"
        variable PAGEDOWN "<63277>"
        variable HOME     "<63273>"
        variable END      "<63275>"
    } elseif {[string match "*linux*" $platform]} {
        # Linux - depends on OpenCV build
        # With basic X11: arrows = Q/R/S/T (81-84)
        # With proper Qt/GTK: arrows = 2490368-2555904
        variable UP      "<82>"   ;# R
        variable DOWN    "<84>"   ;# T
        variable LEFT    "<81>"   ;# Q
        variable RIGHT   "<83>"   ;# S
        
        # Alternate codes if you have Qt/GTK build
        # variable UP      "<2490368>"
        # variable DOWN    "<2621440>"
        # variable LEFT    "<2424832>"
        # variable RIGHT   "<2555904>"
    } else {
        # Windows
        variable UP      "<2490368>"
        variable DOWN    "<2621440>"
        variable LEFT    "<2424832>"
        variable RIGHT   "<2555904>"
    }
    
    # Common keys (same across platforms)
    variable ESC     "<27>"
    variable ENTER   "<13>"
    variable TAB     "<9>"
    variable SPACE   " "
    variable BACKSPACE "<8>"
    variable DELETE  "<127>"
    
    # Storage for detected arrow codes
    variable arrow_codes [dict create]
    
    # Auto-detect and set arrow keys for the current platform
    proc auto_detect_arrows {} {
        variable arrow_codes
        variable UP
        variable DOWN
        variable LEFT
        variable RIGHT
        
        puts "Press arrow keys in order: UP, DOWN, LEFT, RIGHT"
        puts "(Press ESC to cancel)"
        set ::keys::_detect_step 0
        set ::keys::_detected [list]
        bind_key "" keys::_arrow_detect_callback
    }
    
    proc _arrow_detect_callback {keycode} {
        variable _detect_step
        variable _detected
        variable UP
        variable DOWN
        variable LEFT
        variable RIGHT
        variable arrow_codes
        
        if {$keycode == 27} {
            puts "Arrow detection cancelled"
            unbind_key ""
            return
        }
        
        lappend _detected $keycode
        incr _detect_step
        
        switch $_detect_step {
            1 { 
                set UP "<$keycode>"
                dict set arrow_codes "up" $keycode
                puts "UP: $keycode (bind with: bind_key <$keycode> <callback>)" 
            }
            2 { 
                set DOWN "<$keycode>"
                dict set arrow_codes "down" $keycode
                puts "DOWN: $keycode (bind with: bind_key <$keycode> <callback>)" 
            }
            3 { 
                set LEFT "<$keycode>"
                dict set arrow_codes "left" $keycode
                puts "LEFT: $keycode (bind with: bind_key <$keycode> <callback>)" 
            }
            4 { 
                set RIGHT "<$keycode>"
                dict set arrow_codes "right" $keycode
                puts "RIGHT: $keycode (bind with: bind_key <$keycode> <callback>)"
                puts ""
                puts "Arrow keys detected and updated!"
                puts "  keys::UP = $UP"
                puts "  keys::DOWN = $DOWN"
                puts "  keys::LEFT = $LEFT"
                puts "  keys::RIGHT = $RIGHT"
                unbind_key ""
            }
        }
    }
    
    # General key detection - shows code for any key pressed
    proc detect {} {
        puts "Key detection mode - press any key to see its code"
        puts "(Press ESC to quit)"
        bind_key "" keys::show_code
    }
    
    proc show_code {keycode} {
        if {$keycode == 27} {
            puts "Detection stopped"
            unbind_key ""
            return
        }
        
        set keyname ""
        set binding ""
        
        # Identify common keys
        switch $keycode {
            63232 { set keyname "UP (macOS)"; set binding "<63232>" }
            63233 { set keyname "DOWN (macOS)"; set binding "<63233>" }
            63234 { set keyname "LEFT (macOS)"; set binding "<63234>" }
            63235 { set keyname "RIGHT (macOS)"; set binding "<63235>" }
            63236 { set keyname "F1 (macOS)"; set binding "<63236>" }
            63237 { set keyname "F2 (macOS)"; set binding "<63237>" }
            63238 { set keyname "F3 (macOS)"; set binding "<63238>" }
            63239 { set keyname "F4 (macOS)"; set binding "<63239>" }
            63240 { set keyname "F5 (macOS)"; set binding "<63240>" }
            63273 { set keyname "HOME (macOS)"; set binding "<63273>" }
            63275 { set keyname "END (macOS)"; set binding "<63275>" }
            63276 { set keyname "PAGEUP (macOS)"; set binding "<63276>" }
            63277 { set keyname "PAGEDOWN (macOS)"; set binding "<63277>" }
            82 { set keyname "UP (Linux Q/R/S/T mode) or 'R'"; set binding "<82>" }
            84 { set keyname "DOWN (Linux Q/R/S/T mode) or 'T'"; set binding "<84>" }
            81 { set keyname "LEFT (Linux Q/R/S/T mode) or 'Q'"; set binding "<81>" }
            83 { set keyname "RIGHT (Linux Q/R/S/T mode) or 'S'"; set binding "<83>" }
            27 { set keyname "ESC"; set binding "<27>" }
            13 { set keyname "ENTER"; set binding "<13>" }
            10 { set keyname "ENTER (Linux variant)"; set binding "<10>" }
            9 { set keyname "TAB"; set binding "<9>" }
            32 { set keyname "SPACE"; set binding "\" \"" }
            8 { set keyname "BACKSPACE"; set binding "<8>" }
            127 { set keyname "DELETE"; set binding "<127>" }
            2490368 { set keyname "UP (Linux/Win with Qt/GTK)"; set binding "<2490368>" }
            2621440 { set keyname "DOWN (Linux/Win with Qt/GTK)"; set binding "<2621440>" }
            2424832 { set keyname "LEFT (Linux/Win with Qt/GTK)"; set binding "<2424832>" }
            2555904 { set keyname "RIGHT (Linux/Win with Qt/GTK)"; set binding "<2555904>" }
            default {
                if {$keycode >= 32 && $keycode < 127} {
                    set keyname "character '[format %c $keycode]'"
                    set binding "\"[format %c $keycode]\""
                } else {
                    set binding "<$keycode>"
                }
            }
        }
        
        if {$keyname ne ""} {
            puts "Key: $keyname (code: $keycode)"
            puts "  Bind with: bind_key $binding <callback>"
        } else {
            puts "Key code: $keycode"
            puts "  Bind with: bind_key $binding <callback>"
        }
    }
}
)TCL";

Tcl_Eval(interp, key_constants);
  
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




