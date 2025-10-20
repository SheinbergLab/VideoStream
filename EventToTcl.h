#pragma once

#include <tcl.h>
#include "VstreamEvent.h"

// Helper functions to convert Event to Tcl objects efficiently

// Convert EventData to Tcl_Obj based on type
inline Tcl_Obj* eventDataToTclObj(Tcl_Interp* interp, const EventData& data) {
    switch (data.type) {
        case EventDataType::NONE:
            return Tcl_NewObj();  // Empty object
            
        case EventDataType::STRING:
            return Tcl_NewStringObj(data.asString().c_str(), -1);
            
        case EventDataType::INTEGER:
            return Tcl_NewWideIntObj(data.asInt());
            
        case EventDataType::FLOAT:
            return Tcl_NewDoubleObj(data.asFloat());
            
        case EventDataType::INT_ARRAY: {
            Tcl_Obj* listObj = Tcl_NewListObj(0, nullptr);
            for (int64_t val : data.asIntArray()) {
                Tcl_ListObjAppendElement(interp, listObj, Tcl_NewWideIntObj(val));
            }
            return listObj;
        }
        
        case EventDataType::FLOAT_ARRAY: {
            Tcl_Obj* listObj = Tcl_NewListObj(0, nullptr);
            for (double val : data.asFloatArray()) {
                Tcl_ListObjAppendElement(interp, listObj, Tcl_NewDoubleObj(val));
            }
            return listObj;
        }
        
        case EventDataType::KEY_VALUE: {
            Tcl_Obj* dictObj = Tcl_NewDictObj();
            for (const auto& [key, value] : data.asKeyValue()) {
                Tcl_DictObjPut(interp, dictObj, 
                              Tcl_NewStringObj(key.c_str(), -1),
                              Tcl_NewStringObj(value.c_str(), -1));
            }
            return dictObj;
        }
        
        case EventDataType::BINARY: {
            // Return as byte array
            const auto& bytes = data.asBinary();
            return Tcl_NewByteArrayObj(bytes.data(), bytes.size());
        }
        
        default:
            return Tcl_NewObj();
    }
}

// Convert entire Event to a Tcl dict
inline Tcl_Obj* eventToTclDict(Tcl_Interp* interp, const Event& event) {
    Tcl_Obj* dictObj = Tcl_NewDictObj();
    
    // Add event type
    Tcl_DictObjPut(interp, dictObj,
                   Tcl_NewStringObj("type", -1),
                   Tcl_NewStringObj(event.type.c_str(), -1));
    
    // Add data
    Tcl_DictObjPut(interp, dictObj,
                   Tcl_NewStringObj("data", -1),
                   eventDataToTclObj(interp, event.data));
    
    // Add data type as string
    Tcl_DictObjPut(interp, dictObj,
                   Tcl_NewStringObj("data_type", -1),
                   Tcl_NewStringObj(eventDataTypeToString(event.data.type), -1));
    
    // Add timestamp (microseconds since epoch)
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(
        event.timestamp.time_since_epoch()).count();
    Tcl_DictObjPut(interp, dictObj,
                   Tcl_NewStringObj("timestamp", -1),
                   Tcl_NewWideIntObj(us));
    
    // Add source if present
    if (!event.source.empty()) {
        Tcl_DictObjPut(interp, dictObj,
                       Tcl_NewStringObj("source", -1),
                       Tcl_NewStringObj(event.source.c_str(), -1));
    }
    
    return dictObj;
}

// Call Tcl event handler: onEvent <type> <data>
// Returns TCL_OK or TCL_ERROR
inline int invokeTclEventHandler(Tcl_Interp* interp, const Event& event) {
    // Check if handler exists
    const char* check_script = "info commands onEvent";
    if (Tcl_Eval(interp, check_script) != TCL_OK) {
        return TCL_ERROR;
    }
    
    const char* result = Tcl_GetStringResult(interp);
    if (!result || strlen(result) == 0) {
        // Handler doesn't exist, that's okay
        Tcl_ResetResult(interp);
        return TCL_OK;
    }
    
    // Build argument list: onEvent <type> <data>
    Tcl_Obj* objv[3];
    objv[0] = Tcl_NewStringObj("onEvent", -1);
    objv[1] = Tcl_NewStringObj(event.type.c_str(), -1);
    objv[2] = eventDataToTclObj(interp, event.data);
    
    // Increment ref counts
    Tcl_IncrRefCount(objv[0]);
    Tcl_IncrRefCount(objv[1]);
    Tcl_IncrRefCount(objv[2]);
    
    // Call handler
    int retcode = Tcl_EvalObjv(interp, 3, objv, TCL_EVAL_GLOBAL);
    
    // Decrement ref counts
    Tcl_DecrRefCount(objv[0]);
    Tcl_DecrRefCount(objv[1]);
    Tcl_DecrRefCount(objv[2]);
    
    return retcode;
}
