// Widget.h

#pragma once
#include "opencv2/opencv.hpp"
#include <string>
#include <functional>
#include <memory>

// Forward declaration
class WidgetManager;

class Widget {
public:
  enum Type { BUTTON, CIRCLE, RECTANGLE, TEXT, LINE, SLIDER };
    Type type;
    bool visible = true;
    int id;
    
    virtual ~Widget() = default;
    virtual void draw(cv::Mat& frame) = 0;
    virtual bool contains(int x, int y) { return false; }
    virtual void onClick() {}
    virtual void onDrag(int x, int y) {}
    virtual void onRelease() {}
};

class Button : public Widget {
public:
    cv::Rect bounds;
    std::string label;
    std::string tclCallback;
    bool pressed = false;
    bool toggle_mode = true;
    bool mouse_down_inside = false;  // ← Track if mouse pressed inside
    
    Button(int x, int y, int w, int h, const std::string& text, 
           const std::string& cb, bool toggle = true)
        : bounds(x, y, w, h), label(text), tclCallback(cb), 
          toggle_mode(toggle), mouse_down_inside(false)
    {
        type = BUTTON;
    }
    
    void draw(cv::Mat& frame) override {
        // Show pressed state for toggle buttons or during mouse press
        bool show_pressed = (toggle_mode && pressed) || mouse_down_inside;
        cv::Scalar color = show_pressed ? cv::Scalar(0, 200, 0) 
                                        : cv::Scalar(180, 180, 180);
        cv::rectangle(frame, bounds, color, -1);
        cv::rectangle(frame, bounds, cv::Scalar(0, 0, 0), 2);
        
        int baseline;
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 
                                            0.5, 1, &baseline);
        cv::Point textOrg(bounds.x + (bounds.width - textSize.width) / 2,
                         bounds.y + (bounds.height + textSize.height) / 2);
        cv::putText(frame, label, textOrg, cv::FONT_HERSHEY_SIMPLEX, 
                    0.5, cv::Scalar(0, 0, 0), 1);
    }
    
    bool contains(int x, int y) override {
        return bounds.contains(cv::Point(x, y));
    }
    
    void onClick() override;
    void onDrag(int, int) override;
    void onRelease() override;
};

class CircleWidget : public Widget {
public:
    cv::Point center;
    int radius;
    cv::Scalar color;
    int thickness;
    std::string tclCallback;
    bool draggable = false;
    bool dragging = false;
    cv::Point drag_offset;
    
    CircleWidget(int x, int y, int r, cv::Scalar c, int thick = 2,
                 const std::string& cb = "", bool drag = false) 
        : center(x, y), radius(r), color(c), thickness(thick), 
          tclCallback(cb), draggable(drag) {
        type = CIRCLE;
    }
    
    void draw(cv::Mat& frame) override {
        cv::circle(frame, center, radius, color, thickness);
    }
    
    bool contains(int x, int y) override {
        if (tclCallback.empty()) return false;
        int dx = x - center.x;
        int dy = y - center.y;
        return (dx*dx + dy*dy) <= radius*radius;
    }
    
    void onClick() override;
    void onRelease() override;
    void onDrag(int, int) override;
};

class RectWidget : public Widget {
public:
    cv::Rect bounds;
    cv::Scalar color;
    int thickness;
    std::string tclCallback;
    
    RectWidget(int x, int y, int w, int h, cv::Scalar c, int thick = 2,
               const std::string& cb = "") 
        : bounds(x, y, w, h), color(c), thickness(thick), tclCallback(cb) {
        type = RECTANGLE;
    }
    
    void draw(cv::Mat& frame) override {
        cv::rectangle(frame, bounds, color, thickness);
    }
    
    bool contains(int x, int y) override {
        if (tclCallback.empty()) return false;
        return bounds.contains(cv::Point(x, y));
    }
    
    void onClick() override;
};

class TextWidget : public Widget {
public:
    cv::Point position;
    std::string text;
    cv::Scalar color;
    double scale;
    int thickness;
    
    TextWidget(int x, int y, const std::string& txt, cv::Scalar c,
               double s = 0.5, int thick = 1, const std::string& var = "") 
      : position(x, y), text(txt), color(c), scale(s), thickness(thick) {
        type = TEXT;
    }

    
    void draw(cv::Mat& frame) override {
        cv::putText(frame, text, position, cv::FONT_HERSHEY_SIMPLEX,
                    scale, color, thickness);
    }
    
    // Optional: make text clickable with bounding box
    bool contains(int x, int y) override { return false; }
};

class LineWidget : public Widget {
public:
    cv::Point pt1, pt2;
    cv::Scalar color;
    int thickness;
    
    LineWidget(int x1, int y1, int x2, int y2, cv::Scalar c, int thick = 2)
        : pt1(x1, y1), pt2(x2, y2), color(c), thickness(thick) {
        type = LINE;
    }
    
    void draw(cv::Mat& frame) override {
        cv::line(frame, pt1, pt2, color, thickness);
    }
};

class SliderWidget : public Widget {
public:
    cv::Rect bounds;
    cv::Rect track;
    cv::Rect handle;
    std::string label;
    float value;
    float min_val, max_val;
    std::string tclCallback;
    bool dragging;
    cv::Scalar trackColor;
    cv::Scalar handleColor;
    
    SliderWidget(int x, int y, int width, int height,
                 const std::string& lbl,
                 float min_v, float max_v, float initial,
                 const std::string& cb = "");
    
    void updateHandlePosition();
    void draw(cv::Mat& frame) override;
    bool contains(int x, int y) override;
    void onClick() override;
    void onDrag(int x, int y) override;
    void onRelease() override;
};
