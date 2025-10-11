// WidgetManager.h

#include <tcl.h>
#include "Widget.h"
#include "WidgetManager.h"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <unordered_map>

extern WidgetManager g_widgetManager;

void WidgetManager::setVariable(const std::string& name, const std::string& value) {
  std::lock_guard<std::mutex> lock(variableMutex);
  variableCache[name] = value;
}

std::string WidgetManager::getVariable(const std::string& name) const {
  std::lock_guard<std::mutex> lock(variableMutex);
  auto it = variableCache.find(name);
  return (it != variableCache.end()) ? it->second : "";
}

// Substitute @{varname} patterns in text
std::string WidgetManager::substituteVariables(const std::string& text) const {
  std::lock_guard<std::mutex> lock(variableMutex);
  std::string result = text;
  size_t pos = 0;
  
  while ((pos = result.find("@{", pos)) != std::string::npos) {
    size_t end = result.find("}", pos + 2);
    if (end == std::string::npos) break;
    
    std::string varname = result.substr(pos + 2, end - pos - 2);
    auto it = variableCache.find(varname);
    std::string value = (it != variableCache.end()) ? it->second : "";
            
    result.replace(pos, end - pos + 1, value);
    pos += value.length();
  }
  return result;
}

void WidgetManager::clearVariables() {
  std::lock_guard<std::mutex> lock(variableMutex);
  variableCache.clear();
}

int WidgetManager::addWidget(std::unique_ptr<Widget> widget) {
  std::lock_guard<std::mutex> lock(widgetMutex);
  int id = nextIdAtomic.fetch_add(1);
  widget->id = id;
  widgets.push_back(std::move(widget));
  return id;
}

void WidgetManager::drawAll(cv::Mat& frame) {
  std::lock_guard<std::mutex> lock(widgetMutex);
  for (auto& w : widgets) {
    if (w->visible) {
      w->draw(frame);
    }
  }
}

bool WidgetManager::handleMouseDown(int x, int y) {
  std::lock_guard<std::mutex> lock(widgetMutex);
  for (auto& w : widgets) {
    if (w->visible && w->contains(x, y)) {
      w->onClick();
      return true;
    }
  }
  return false;
}

void WidgetManager::handleMouseDrag(int x, int y) {
  std::lock_guard<std::mutex> lock(widgetMutex);
  for (auto& w : widgets) {
    if (w->visible) {
      w->onDrag(x, y);
    }
  }
}

void WidgetManager::handleMouseUp(int x, int y) {
  std::lock_guard<std::mutex> lock(widgetMutex);
  for (auto& w : widgets) {
    if (w->visible) {
      w->onRelease();
    }
  }
}

bool WidgetManager::handleClick(int x, int y) {
  std::string callbackToExecute;
  {
    std::lock_guard<std::mutex> lock(widgetMutex);
    for (auto& w : widgets) {
      if (w->visible && w->contains(x, y)) {
	if (auto* btn = dynamic_cast<Button*>(w.get())) {
	  callbackToExecute = btn->tclCallback;
	  btn->pressed = !btn->pressed;
	} else if (auto* circ = dynamic_cast<CircleWidget*>(w.get())) {
	  callbackToExecute = circ->tclCallback;
	} else if (auto* rect = dynamic_cast<RectWidget*>(w.get())) {
	  callbackToExecute = rect->tclCallback;
	}
	break;
      }
    }
  }  // Release lock before callback
  
  if (!callbackToExecute.empty()) {
    executeTclCallback(callbackToExecute);
    return true;
  }
  return false;
}

bool WidgetManager::updateWidgetText(int id, std::string str) {
  std::lock_guard<std::mutex> lock(widgetMutex);
  
  for (auto& widget : widgets) {
    if (widget->id == id) {
      if (auto* txt = dynamic_cast<TextWidget*>(widget.get())) {
	txt->text = str;
	return true;
      }
      else if (auto* slider = dynamic_cast<AbstractSliderWidget*>(widget.get())) {
	slider->label = str;
	return true;
      }
      return false;
    }
  }
  return false;  // Widget not found
}

bool WidgetManager::updateWidgetValue(int id, const std::string& str) {
    std::lock_guard<std::mutex> lock(widgetMutex);

    std::istringstream iss(str);

    for (auto& widget : widgets) {
        if (widget->id == id) {
            if (auto* slider = dynamic_cast<FloatSliderWidget*>(widget.get())) {
                float min_v, max_v, val;
                if (!(iss >> min_v >> max_v >> val)) return false;

                slider->min_val = min_v;
                slider->max_val = max_v;
                slider->value = std::clamp((val - min_v) / (max_v - min_v), 0.0f, 1.0f);
                slider->updateHandlePosition();
                return true;
            }
            else if (auto* slider = dynamic_cast<IntSliderWidget*>(widget.get())) {
                int min_i, max_i, val_i;
                if (!(iss >> min_i >> max_i >> val_i)) return false;

                slider->min_val = min_i;
                slider->max_val = max_i;
                slider->value = std::clamp(val_i, min_i, max_i);
                slider->lastValue = slider->value;
                slider->updateHandlePosition();
                return true;
            }
            return false;
        }
    }
    return false;  // Widget not found
}

bool WidgetManager::updateWidget(int id, int x, int y, int w, int h) {
  std::lock_guard<std::mutex> lock(widgetMutex);
  
  for (auto& widget : widgets) {
    if (widget->id == id) {
      // Update based on widget type
      if (auto* btn = dynamic_cast<Button*>(widget.get())) {
	if (x >= 0) btn->bounds.x = x;
	if (y >= 0) btn->bounds.y = y;
	if (w >= 0) btn->bounds.width = w;
	if (h >= 0) btn->bounds.height = h;
	return true;
      }
      else if (auto* circ = dynamic_cast<CircleWidget*>(widget.get())) {
	if (x >= 0) circ->center.x = x;
	if (y >= 0) circ->center.y = y;
	if (w >= 0) circ->radius = w;  // Use w as radius
	// Clamp radius to reasonable bounds
	if (circ->radius > 1000) circ->radius = 1000;
	return true;
      }
      else if (auto* rect = dynamic_cast<RectWidget*>(widget.get())) {
	if (x >= 0) rect->bounds.x = x;
	if (y >= 0) rect->bounds.y = y;
	if (w >= 0) rect->bounds.width = w;
	if (h >= 0) rect->bounds.height = h;
	return true;
      }
      else if (auto* txt = dynamic_cast<TextWidget*>(widget.get())) {
	if (x >= 0) txt->position.x = x;
	if (y >= 0) txt->position.y = y;
	return true;
      }
      else if (auto* line = dynamic_cast<LineWidget*>(widget.get())) {
	if (x >= 0) line->pt1.x = x;
	if (y >= 0) line->pt1.y = y;
	if (w >= 0) line->pt2.x = w;  // Use w,h as x2,y2
	if (h >= 0) line->pt2.y = h;
	return true;
      }
      else if (auto* slider = dynamic_cast<AbstractSliderWidget*>(widget.get())) {
	if (x >= 0) slider->bounds.x = x;
	if (y >= 0) slider->bounds.y = y;
	if (w >= 0) slider->bounds.width = w;
	if (h >= 0) slider->bounds.height = h;
        
	// Recalculate track and handle if position/size changed
	if (x >= 0 || y >= 0 || w >= 0 || h >= 0) {
	  int track_height = 4;
	  slider->track = cv::Rect(
				   slider->bounds.x + 10, 
				   slider->bounds.y + slider->bounds.height/2 - track_height/2, 
				   slider->bounds.width - 20, 
				   track_height);
	  slider->updateHandlePosition();
	}
	return true;
      }
      return false;
    }
  }
  return false;  // Widget not found
}

void WidgetManager::removeWidget(int id) {
  std::lock_guard<std::mutex> lock(widgetMutex);
  widgets.erase(
		std::remove_if(widgets.begin(), widgets.end(),
			       [id](const auto& w) { return w->id == id; }),
		widgets.end());
}

void WidgetManager::clearAll() {
  std::lock_guard<std::mutex> lock(widgetMutex);
  widgets.clear();
}

void WidgetManager::queueCallback(const std::string& script) {
  std::lock_guard<std::mutex> lock(callbackMutex);
  if (!script.empty()) {
    pendingCallbacks.push(script);
  }
}

void WidgetManager::processPendingCallbacks() {
  std::queue<std::string> callbacks;
  {
    std::lock_guard<std::mutex> lock(callbackMutex);
    callbacks.swap(pendingCallbacks);
  }
  
  // Push to mouse_queue instead of calling tcl_eval directly
  extern SharedQueue<std::string> mouse_queue;
  
  while (!callbacks.empty()) {
    mouse_queue.push_back(callbacks.front());
    callbacks.pop();
  }
}    

void WidgetManager::executeTclCallback(const std::string& script) {
  queueCallback(script);
}    
