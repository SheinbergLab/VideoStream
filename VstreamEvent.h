#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>
#include <variant>
#include <algorithm>

// VstreamEvent data types
enum class VstreamEventDataType {
    NONE,           // No data
    STRING,         // Text data
    INTEGER,        // Single int64_t
    FLOAT,          // Single double
    BINARY,         // Arbitrary binary blob
    INT_ARRAY,      // Array of int64_t
    FLOAT_ARRAY,    // Array of double
    KEY_VALUE       // String key-value pairs (like dict)
};

// Convert enum to string for serialization
inline const char* eventDataTypeToString(VstreamEventDataType type) {
    switch (type) {
        case VstreamEventDataType::NONE: return "none";
        case VstreamEventDataType::STRING: return "string";
        case VstreamEventDataType::INTEGER: return "integer";
        case VstreamEventDataType::FLOAT: return "float";
        case VstreamEventDataType::BINARY: return "binary";
        case VstreamEventDataType::INT_ARRAY: return "int_array";
        case VstreamEventDataType::FLOAT_ARRAY: return "float_array";
        case VstreamEventDataType::KEY_VALUE: return "key_value";
        default: return "unknown";
    }
}

// VstreamEvent data container (type-safe variant)
class VstreamEventData {
public:
    VstreamEventDataType type;
    
    // Union of possible data types
    std::variant<
        std::monostate,                              // NONE
        std::string,                                 // STRING
        int64_t,                                     // INTEGER
        double,                                      // FLOAT
        std::vector<uint8_t>,                        // BINARY
        std::vector<int64_t>,                        // INT_ARRAY
        std::vector<double>,                         // FLOAT_ARRAY
        std::map<std::string, std::string>           // KEY_VALUE
    > value;
    
    VstreamEventData() : type(VstreamEventDataType::NONE) {}
    
    // Factory methods for each type
    static VstreamEventData makeNone() {
        VstreamEventData e;
        e.type = VstreamEventDataType::NONE;
        return e;
    }
    
    static VstreamEventData makeString(const std::string& s) {
        VstreamEventData e;
        e.type = VstreamEventDataType::STRING;
        e.value = s;
        return e;
    }
    
    static VstreamEventData makeInt(int64_t i) {
        VstreamEventData e;
        e.type = VstreamEventDataType::INTEGER;
        e.value = i;
        return e;
    }
    
    static VstreamEventData makeFloat(double f) {
        VstreamEventData e;
        e.type = VstreamEventDataType::FLOAT;
        e.value = f;
        return e;
    }
    
    static VstreamEventData makeBinary(const std::vector<uint8_t>& data) {
        VstreamEventData e;
        e.type = VstreamEventDataType::BINARY;
        e.value = data;
        return e;
    }
    
    static VstreamEventData makeIntArray(const std::vector<int64_t>& arr) {
        VstreamEventData e;
        e.type = VstreamEventDataType::INT_ARRAY;
        e.value = arr;
        return e;
    }
    
    static VstreamEventData makeFloatArray(const std::vector<double>& arr) {
        VstreamEventData e;
        e.type = VstreamEventDataType::FLOAT_ARRAY;
        e.value = arr;
        return e;
    }
    
    static VstreamEventData makeKeyValue(const std::map<std::string, std::string>& kv) {
        VstreamEventData e;
        e.type = VstreamEventDataType::KEY_VALUE;
        e.value = kv;
        return e;
    }
    
    // Accessors with type checking
    const std::string& asString() const {
        return std::get<std::string>(value);
    }
    
    int64_t asInt() const {
        return std::get<int64_t>(value);
    }
    
    double asFloat() const {
        return std::get<double>(value);
    }
    
    const std::vector<uint8_t>& asBinary() const {
        return std::get<std::vector<uint8_t>>(value);
    }
    
    const std::vector<int64_t>& asIntArray() const {
        return std::get<std::vector<int64_t>>(value);
    }
    
    const std::vector<double>& asFloatArray() const {
        return std::get<std::vector<double>>(value);
    }
    
    const std::map<std::string, std::string>& asKeyValue() const {
        return std::get<std::map<std::string, std::string>>(value);
    }
};

// Main event structure
struct VstreamEvent {
    std::string type;                                    // Event type (hierarchical: "plugin/category/action")
    VstreamEventData data;                                      // Typed data
    std::chrono::system_clock::time_point timestamp;     // When event occurred
    std::string source;                                  // Optional: which plugin/component
    int priority;                                        // Optional: for prioritization (unused for now)
    bool rate_limit_exempt;                              // Critical events bypass rate limits
    
    // Convenience constructors - automatic type inference
    
    // No data
    explicit VstreamEvent(const std::string& t)
        : type(t), data(VstreamEventData::makeNone()), source(""), priority(0),
          timestamp(std::chrono::system_clock::now()),
          rate_limit_exempt(false) {}
    
    // String data
    VstreamEvent(const std::string& t, const std::string& str_data)
        : type(t), data(VstreamEventData::makeString(str_data)), source(""), priority(0),
          timestamp(std::chrono::system_clock::now()),
          rate_limit_exempt(false) {}
    
    // Integer data
    VstreamEvent(const std::string& t, int64_t int_data)
        : type(t), data(VstreamEventData::makeInt(int_data)), source(""), priority(0),
          timestamp(std::chrono::system_clock::now()),
          rate_limit_exempt(false) {}
    
    // Float data
    VstreamEvent(const std::string& t, double float_data)
        : type(t), data(VstreamEventData::makeFloat(float_data)), source(""), priority(0),
          timestamp(std::chrono::system_clock::now()),
          rate_limit_exempt(false) {}
    
    // Explicit VstreamEventData (for arrays, key-value, binary)
    VstreamEvent(const std::string& t, const VstreamEventData& d)
        : type(t), data(d), source(""), priority(0),
          timestamp(std::chrono::system_clock::now()),
          rate_limit_exempt(false) {}
    
    // With source
    VstreamEvent(const std::string& t, const VstreamEventData& d, const std::string& src)
        : type(t), data(d), source(src), priority(0),
          timestamp(std::chrono::system_clock::now()),
          rate_limit_exempt(false) {}
    
    // Check if event matches subscription pattern
    // Supports: "exact/match", "prefix/*", "*/suffix", "*"
    bool matchesPattern(const std::string& pattern) const {
        if (pattern == "*") return true;
        
        if (pattern.back() == '*') {
            // Prefix match: "eyetracking/*"
            std::string prefix = pattern.substr(0, pattern.length() - 1);
            return type.substr(0, prefix.length()) == prefix;
        }
        
        if (pattern.front() == '*') {
            // Suffix match: "*/settings"
            std::string suffix = pattern.substr(1);
            if (type.length() >= suffix.length()) {
                return type.substr(type.length() - suffix.length()) == suffix;
            }
            return false;
        }
        
        // Exact match
        return type == pattern;
    }
};

// Actual call to send events to our event system
void fireEvent(const VstreamEvent& event);
