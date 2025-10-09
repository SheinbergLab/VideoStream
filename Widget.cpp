#include "Widget.h"
#include "WidgetManager.h"
#include <sstream>
#include <iomanip>
#include <algorithm>

extern WidgetManager g_widgetManager;

/***************************************************************/
/*                         UI Support                          */
/***************************************************************/

void CircleWidget::onClick() {
  if (draggable) {
    dragging = true;
  } else if (!tclCallback.empty()) {
    extern WidgetManager g_widgetManager;
    g_widgetManager.executeTclCallback(tclCallback);
  }
}

void CircleWidget::onDrag(int x, int y) {
  if (dragging) {
    center = cv::Point(x, y);
    
    // notify via callback (could have separate drag callback?)
    if (!tclCallback.empty()) {
      std::ostringstream cmd;
      cmd << tclCallback << " " << center.x << " " << center.y;
      extern WidgetManager g_widgetManager;
      g_widgetManager.executeTclCallback(cmd.str());
    }
  }
}

void CircleWidget::onRelease() {
  dragging = false;
}

void RectWidget::onClick() {
    extern WidgetManager* g_widgetMgr;
    g_widgetManager.executeTclCallback(tclCallback);
}


void Button::onClick() {
  mouse_down_inside = true;
  
  if (toggle_mode) {
    // Toggle immediately on press
    pressed = !pressed;
    extern WidgetManager g_widgetManager;
    std::string cmd = tclCallback + " " + (pressed ? "1" : "0");
    g_widgetManager.executeTclCallback(cmd);
  }
  // Push buttons wait for release
}

void Button::onDrag(int x, int y) {
  // Update visual state based on whether mouse is still inside
  if (!toggle_mode && mouse_down_inside) {
    mouse_down_inside = contains(x, y);
  }
}

void Button::onRelease() {
  if (!toggle_mode && mouse_down_inside) {
    // Trigger callback only if released inside button
    extern WidgetManager g_widgetManager;
    g_widgetManager.executeTclCallback(tclCallback);
  }
  mouse_down_inside = false;
}


// SliderWidget implementation
SliderWidget::SliderWidget(int x, int y, int width, int height,
                           const std::string& lbl,
                           float min_v, float max_v, float initial,
                           const std::string& cb)
    : label(lbl), min_val(min_v), max_val(max_v), 
      tclCallback(cb), dragging(false),
      trackColor(cv::Scalar(100, 100, 100)),
      handleColor(cv::Scalar(200, 200, 200))
{
    type = Widget::SLIDER;
    bounds = cv::Rect(x, y, width, height);
    
    int track_height = 4;
    track = cv::Rect(x + 10, y + height/2 - track_height/2, 
                    width - 20, track_height);
    
    value = (initial - min_val) / (max_val - min_val);
    value = std::max(0.0f, std::min(1.0f, value));
    updateHandlePosition();
}

void SliderWidget::updateHandlePosition() {
    int handle_width = 12;
    int handle_height = 20;
    int handle_x = track.x + (int)(value * track.width) - handle_width/2;
    int handle_y = bounds.y + bounds.height/2 - handle_height/2;
    handle = cv::Rect(handle_x, handle_y, handle_width, handle_height);
}

void SliderWidget::draw(cv::Mat& frame) {
    cv::putText(frame, label, 
               cv::Point(bounds.x, bounds.y - 5),
               cv::FONT_HERSHEY_SIMPLEX, 0.4,
               cv::Scalar(200, 200, 200), 1);
    
    cv::rectangle(frame, track, trackColor, -1);
    
    cv::Rect filled(track.x, track.y, 
                   (int)(value * track.width), track.height);
    cv::rectangle(frame, filled, cv::Scalar(0, 200, 0), -1);
    
    cv::Scalar hColor = dragging ? cv::Scalar(255, 255, 0) : handleColor;
    cv::rectangle(frame, handle, hColor, -1);
    cv::rectangle(frame, handle, cv::Scalar(0, 0, 0), 1);
    
    float current = min_val + value * (max_val - min_val);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << current;
    cv::putText(frame, oss.str(),
               cv::Point(bounds.x + bounds.width + 5, 
                        bounds.y + bounds.height/2 + 5),
               cv::FONT_HERSHEY_SIMPLEX, 0.4,
               cv::Scalar(200, 200, 200), 1);
}

bool SliderWidget::contains(int x, int y) {
    return handle.contains(cv::Point(x, y)) || 
           track.contains(cv::Point(x, y));
}

void SliderWidget::onClick() {
    dragging = true;
}

void SliderWidget::onDrag(int x, int y) {
    if (dragging) {
        int rel_x = std::max(track.x, std::min(track.x + track.width, x));
        value = (float)(rel_x - track.x) / track.width;
        updateHandlePosition();
        
        if (!tclCallback.empty()) {
            float current = min_val + value * (max_val - min_val);
            std::ostringstream cmd;
            cmd << tclCallback << " " << std::fixed 
                << std::setprecision(6) << current;
            
            // Now we can use g_widgetManager because WidgetManager.h is included
            g_widgetManager.executeTclCallback(cmd.str());
        }
    }
}

void SliderWidget::onRelease() {
    dragging = false;
}
