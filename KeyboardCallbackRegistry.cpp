// KeyboardCallbackRegistry.cpp

#include "KeyboardCallbackRegistry.h"
#include <iostream>

void KeyboardCallbackRegistry::registerCallback(int key, const std::string& callback) {
    std::lock_guard<std::mutex> lock(mutex);
    callbacks[key] = callback;
}

void KeyboardCallbackRegistry::unregisterCallback(int key) {
    std::lock_guard<std::mutex> lock(mutex);
    callbacks.erase(key);
}

void KeyboardCallbackRegistry::clearAll() {
    std::lock_guard<std::mutex> lock(mutex);
    callbacks.clear();
}

std::string KeyboardCallbackRegistry::getCallback(int key) const {
    std::lock_guard<std::mutex> lock(mutex);
    auto it = callbacks.find(key);
    return (it != callbacks.end()) ? it->second : "";
}

std::vector<int> KeyboardCallbackRegistry::getRegisteredKeys() const {
    std::lock_guard<std::mutex> lock(mutex);
    std::vector<int> keys;
    keys.reserve(callbacks.size());
    for (const auto& pair : callbacks) {
        keys.push_back(pair.first);
    }
    return keys;
}

// Global instance definition
KeyboardCallbackRegistry g_keyboardCallbacks;
