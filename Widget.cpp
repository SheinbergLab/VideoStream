#include "Widget.h"
#include "WidgetManager.h"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <condition_variable>

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

void TextWidget::draw(cv::Mat& frame) {
  std::string displayText = g_widgetManager.substituteVariables(text);
  
  cv::putText(frame, displayText, position, cv::FONT_HERSHEY_SIMPLEX,
	      scale, color, thickness);
}

/***********************************************************************/
/****************************** SLIDERS ********************************/
/***********************************************************************/

AbstractSliderWidget::AbstractSliderWidget(int x, int y,
					   int width, int height,
                                           const std::string& lbl,
                                           const std::string& cb)
    : label(lbl), tclCallback(cb) {
    bounds = cv::Rect(x, y, width, height);
    int track_height = 4;
    track = cv::Rect(x + 10, y + height / 2 - track_height / 2,
                     width - 20, track_height);
}

void AbstractSliderWidget::draw(cv::Mat& frame) {
    extern WidgetManager g_widgetManager;
    std::string displayLabel = g_widgetManager.substituteVariables(label);

    cv::putText(frame, displayLabel,
                cv::Point(bounds.x, bounds.y - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(200, 200, 200), 1);

    cv::rectangle(frame, track, trackColor, -1);
    cv::rectangle(frame, handle, dragging ? cv::Scalar(255, 255, 0) : handleColor, -1);
    cv::rectangle(frame, handle, cv::Scalar(0, 0, 0), 1);

    drawValue(frame);
}

bool AbstractSliderWidget::contains(int x, int y) {
    return handle.contains(cv::Point(x, y)) || track.contains(cv::Point(x, y));
}

void AbstractSliderWidget::onClick() {
    dragging = true;
}

void AbstractSliderWidget::onDrag(int x, int y) {
    if (dragging) {
        int rel_x = std::max(track.x, std::min(track.x + track.width, x));
        updateValueFromPosition(rel_x);
        updateHandlePosition();
        invokeCallback();
    }
}

void AbstractSliderWidget::onRelease() {
    dragging = false;
}

FloatSliderWidget::FloatSliderWidget(int x, int y, int width, int height,
                                     const std::string& lbl,
                                     float min_v, float max_v, float initial,
                                     const std::string& cb)
    : AbstractSliderWidget(x, y, width, height, lbl, cb),
      min_val(min_v), max_val(max_v) {
    type = Widget::FLOAT_SLIDER;
    value = std::clamp((initial - min_val) / (max_val - min_val), 0.0f, 1.0f);
    updateHandlePosition();
}

void FloatSliderWidget::updateHandlePosition() {
    int handle_width = 12;
    int handle_height = 20;
    int handle_x = track.x + (int)(value * track.width) - handle_width / 2;
    int handle_y = bounds.y + bounds.height / 2 - handle_height / 2;
    handle = cv::Rect(handle_x, handle_y, handle_width, handle_height);
}

void FloatSliderWidget::drawValue(cv::Mat& frame) {
    float current = min_val + value * (max_val - min_val);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << current;
    cv::putText(frame, oss.str(),
                cv::Point(bounds.x + bounds.width + 5,
                          bounds.y + bounds.height / 2 + 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(200, 200, 200), 1);
}

void FloatSliderWidget::updateValueFromPosition(int x) {
    value = (float)(x - track.x) / track.width;
    value = std::clamp(value, 0.0f, 1.0f);
}

void FloatSliderWidget::invokeCallback() {
    if (!tclCallback.empty()) {
        float current = min_val + value * (max_val - min_val);
        std::ostringstream cmd;
        cmd << tclCallback << " " << std::fixed << std::setprecision(6) << current;
        g_widgetManager.executeTclCallback(cmd.str());
    }
}

IntSliderWidget::IntSliderWidget(int x, int y, int width, int height,
                                 const std::string& lbl,
                                 int min_v, int max_v, int initial,
                                 const std::string& cb)
    : AbstractSliderWidget(x, y, width, height, lbl, cb),
      min_val(min_v), max_val(max_v) {
    type = Widget::INT_SLIDER;
    value = std::clamp(initial, min_val, max_val);
    lastValue = value;    
    updateHandlePosition();
}

void IntSliderWidget::updateHandlePosition() {
    int handle_width = 12;
    int handle_height = 20;
    float norm = (float)(value - min_val) / (max_val - min_val);
    int handle_x = track.x + (int)(norm * track.width) - handle_width / 2;
    int handle_y = bounds.y + bounds.height / 2 - handle_height / 2;
    handle = cv::Rect(handle_x, handle_y, handle_width, handle_height);
}

void IntSliderWidget::drawValue(cv::Mat& frame) {
    std::ostringstream oss;
    oss << value;
    cv::putText(frame, oss.str(),
                cv::Point(bounds.x + bounds.width + 5,
                          bounds.y + bounds.height / 2 + 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(200, 200, 200), 1);
}

void IntSliderWidget::updateValueFromPosition(int x) {
    float norm = (float)(x - track.x) / track.width;
    norm = std::clamp(norm, 0.0f, 1.0f);
    value = min_val + (int)(norm * (max_val - min_val) + 0.5f);
}

void IntSliderWidget::invokeCallback() {
    if (value != lastValue && !tclCallback.empty()) {
        std::ostringstream cmd;
        cmd << tclCallback << " " << value;
        g_widgetManager.executeTclCallback(cmd.str());
        lastValue = value;
    }
}
