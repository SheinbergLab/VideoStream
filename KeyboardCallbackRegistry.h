// KeyboardCallbackRegistry.h
#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>

class KeyboardCallbackRegistry {
private:
    std::unordered_map<int, std::string> callbacks;  // Changed from char to int
    mutable std::mutex mutex;
    
public:
    void registerCallback(int key, const std::string& callback);  // int instead of char
    void unregisterCallback(int key);
    void clearAll();
    
    // Returns empty string if no callback registered
    std::string getCallback(int key) const;
    
    // Get all registered keys (for debugging/introspection)
    std::vector<int> getRegisteredKeys() const;  // Changed from char to int
};

// Global instance declaration
extern KeyboardCallbackRegistry g_keyboardCallbacks;
