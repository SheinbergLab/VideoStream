// WidgetManager.h

#include <tcl.h>
#include "SharedQueue.hpp"

#pragma once
class WidgetManager {
    std::vector<std::unique_ptr<Widget>> widgets;
    mutable std::mutex widgetMutex;
    std::atomic<int> nextIdAtomic{0};

    std::queue<std::string> pendingCallbacks;
    std::mutex callbackMutex;
    
public:
    WidgetManager() {}
    
    int addWidget(std::unique_ptr<Widget> widget) {
      std::lock_guard<std::mutex> lock(widgetMutex);
      int id = nextIdAtomic.fetch_add(1);
      widget->id = id;
      widgets.push_back(std::move(widget));
      return id;
    }
    
    void drawAll(cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(widgetMutex);
        for (auto& w : widgets) {
            if (w->visible) {
                w->draw(frame);
            }
        }
    }

    bool handleMouseDown(int x, int y) {
        std::lock_guard<std::mutex> lock(widgetMutex);
        for (auto& w : widgets) {
            if (w->visible && w->contains(x, y)) {
                w->onClick();
                return true;
            }
        }
        return false;
    }
    
    void handleMouseDrag(int x, int y) {
        std::lock_guard<std::mutex> lock(widgetMutex);
        for (auto& w : widgets) {
            if (w->visible) {
                w->onDrag(x, y);
            }
        }
    }
    
    void handleMouseUp(int x, int y) {
        std::lock_guard<std::mutex> lock(widgetMutex);
        for (auto& w : widgets) {
            if (w->visible) {
                w->onRelease();
            }
        }
    }
    
    bool handleClick(int x, int y) {
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

    bool updateWidgetText(int id, std::string str) {
      std::lock_guard<std::mutex> lock(widgetMutex);

      for (auto& widget : widgets) {
	if (widget->id == id) {
	  if (auto* txt = dynamic_cast<TextWidget*>(widget.get())) {
	    txt->text = str;
	    return true;
	  }
	  else if (auto* slider = dynamic_cast<SliderWidget*>(widget.get())) {
	    slider->label = str;
	    return true;
	  }
	  return false;
	}
      }
      return false;  // Widget not found
    }
    
    bool updateWidget(int id, int x, int y, int w, int h) {
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
	  else if (auto* slider = dynamic_cast<SliderWidget*>(widget.get())) {
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
   
    void removeWidget(int id) {
        std::lock_guard<std::mutex> lock(widgetMutex);
        widgets.erase(
            std::remove_if(widgets.begin(), widgets.end(),
                [id](const auto& w) { return w->id == id; }),
            widgets.end());
    }
    
    void clearAll() {
        std::lock_guard<std::mutex> lock(widgetMutex);
        widgets.clear();
    }

    void queueCallback(const std::string& script) {
        std::lock_guard<std::mutex> lock(callbackMutex);
        if (!script.empty()) {
            pendingCallbacks.push(script);
        }
    }

    void processPendingCallbacks() {
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
    
    void executeTclCallback(const std::string& script) {
        queueCallback(script);
    }    
};
    

