// WidgetManager.h

#include <tcl.h>
#include "SharedQueue.hpp"
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <unordered_map>

#pragma once
class WidgetManager {
  std::vector<std::unique_ptr<Widget>> widgets;
  mutable std::mutex widgetMutex;
  std::atomic<int> nextIdAtomic{0};
  
  std::queue<std::string> pendingCallbacks;
  std::mutex callbackMutex;
  
  std::unordered_map<std::string, std::string> variableCache;
  mutable std::mutex variableMutex;  
    
public:
  WidgetManager() {}

  void setVariable(const std::string& name, const std::string& value);
  std::string getVariable(const std::string& name) const;
  std::string substituteVariables(const std::string& text) const;
  void clearVariables();
  int addWidget(std::unique_ptr<Widget> widget);
  void drawAll(cv::Mat& frame);
  bool handleMouseDown(int x, int y);
  void handleMouseDrag(int x, int y);
  void handleMouseUp(int x, int y);
  bool handleClick(int x, int y);
  bool updateWidgetText(int id, std::string str);
  bool updateWidget(int id, int x, int y, int w, int h);
  void removeWidget(int id);
  void clearAll();
  void queueCallback(const std::string& script);
  void processPendingCallbacks();
  void executeTclCallback(const std::string& script);
};
    

