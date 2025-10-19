#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>
#include <variant>
#include <algorithm>

// Event data types
enum class EventDataType {
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
inline const char* eventDataTypeToString(EventDataType type) {
    switch (type) {
        case EventDataType::NONE: return "none";
        case EventDataType::STRING: return "string";
        case EventDataType::INTEGER: return "integer";
        case EventDataType::FLOAT: return "float";
        case EventDataType::BINARY: return "binary";
        case EventDataType::INT_ARRAY: return "int_array";
        case EventDataType::FLOAT_ARRAY: return "float_array";
        case EventDataType::KEY_VALUE: return "key_value";
        default: return "unknown";
    }
}

// Event data container (type-safe variant)
class EventData {
public:
    EventDataType type;
    
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
    
    EventData() : type(EventDataType::NONE) {}
    
    // Factory methods for each type
    static EventData makeNone() {
        EventData e;
        e.type = EventDataType::NONE;
        return e;
    }
    
    static EventData makeString(const std::string& s) {
        EventData e;
        e.type = EventDataType::STRING;
        e.value = s;
        return e;
    }
    
    static EventData makeInt(int64_t i) {
        EventData e;
        e.type = EventDataType::INTEGER;
        e.value = i;
        return e;
    }
    
    static EventData makeFloat(double f) {
        EventData e;
        e.type = EventDataType::FLOAT;
        e.value = f;
        return e;
    }
    
    static EventData makeBinary(const std::vector<uint8_t>& data) {
        EventData e;
        e.type = EventDataType::BINARY;
        e.value = data;
        return e;
    }
    
    static EventData makeIntArray(const std::vector<int64_t>& arr) {
        EventData e;
        e.type = EventDataType::INT_ARRAY;
        e.value = arr;
        return e;
    }
    
    static EventData makeFloatArray(const std::vector<double>& arr) {
        EventData e;
        e.type = EventDataType::FLOAT_ARRAY;
        e.value = arr;
        return e;
    }
    
    static EventData makeKeyValue(const std::map<std::string, std::string>& kv) {
        EventData e;
        e.type = EventDataType::KEY_VALUE;
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
struct Event {
    std::string type;                                    // Event type (hierarchical: "plugin/category/action")
    EventData data;                                      // Typed data
    std::chrono::system_clock::time_point timestamp;     // When event occurred
    std::string source;                                  // Optional: which plugin/component
    int priority;                                        // Optional: for prioritization (unused for now)
    bool rate_limit_exempt;                              // Critical events bypass rate limits
    
    // Convenience constructors - automatic type inference
    
    // No data
    explicit Event(const std::string& t)
        : type(t), data(EventData::makeNone()), source(""), priority(0),
          timestamp(std::chrono::system_clock::now()),
          rate_limit_exempt(false) {}
    
    // String data
    Event(const std::string& t, const std::string& str_data)
        : type(t), data(EventData::makeString(str_data)), source(""), priority(0),
          timestamp(std::chrono::system_clock::now()),
          rate_limit_exempt(false) {}
    
    // Integer data
    Event(const std::string& t, int64_t int_data)
        : type(t), data(EventData::makeInt(int_data)), source(""), priority(0),
          timestamp(std::chrono::system_clock::now()),
          rate_limit_exempt(false) {}
    
    // Float data
    Event(const std::string& t, double float_data)
        : type(t), data(EventData::makeFloat(float_data)), source(""), priority(0),
          timestamp(std::chrono::system_clock::now()),
          rate_limit_exempt(false) {}
    
    // Explicit EventData (for arrays, key-value, binary)
    Event(const std::string& t, const EventData& d)
        : type(t), data(d), source(""), priority(0),
          timestamp(std::chrono::system_clock::now()),
          rate_limit_exempt(false) {}
    
    // With source
    Event(const std::string& t, const EventData& d, const std::string& src)
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
void fireEvent(const Event& event);
