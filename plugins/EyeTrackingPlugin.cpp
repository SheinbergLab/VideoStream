#include <tcl.h>
#include "opencv2/opencv.hpp"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <algorithm>
#include <cmath>

#include "SharedQueue.hpp"
#include "IAnalysisPlugin.h"
#include "AnalysisPluginRegistry.h"

// Forward declare the registry
extern AnalysisPluginRegistry g_pluginRegistry;

// Add to class private members:
struct PurkinjeValidator {
  bool pupil_initialized;
  cv::Point2f pupil_last;  
  
  bool p1_initialized;
  cv::Point2f p1_last;

  bool p4_initialized;
  cv::Point2f p4_last;
  
  int p4_confidence_count;  
  
  PurkinjeValidator() : p1_initialized(false), p4_initialized(false) {}
  
  void reset() {
    p1_initialized = false;
    p4_initialized = false;
  }
  
  
  bool isValidP1(const cv::Point2f& new_pos, float max_jump_pixels = 15.0f) {
    if (!p1_initialized) return true;
    float dist = cv::norm(new_pos - p1_last);
    return dist < max_jump_pixels;
  }
  
  void updateP1(const cv::Point2f& pos) {
    p1_last = pos;
    p1_initialized = true;
  }

    bool isValidP4(const cv::Point2f& new_pos, const cv::Point2f& pupil_center, 
                   float max_jump_pixels = 10.0f) {
        if (!p4_initialized) return true;
        
        float p4_movement = cv::norm(new_pos - p4_last);
        
        // If pupil also moved, allow more P4 movement
        if (pupil_initialized) {
            float pupil_movement = cv::norm(pupil_center - pupil_last);
            
            // P4 can move up to: base threshold + pupil movement
            float adjusted_max = max_jump_pixels + pupil_movement;
            
            if (p4_movement > adjusted_max) {
                std::cout << "P4 rejected: moved " << p4_movement 
                          << " (pupil moved " << pupil_movement 
                          << ", allowed " << adjusted_max << ")" << std::endl;
                return false;
            }
        } else {
            // First frame with pupil
            if (p4_movement > max_jump_pixels) {
                return false;
            }
        }
        
        return true;
    }
  
  void updateP4(const cv::Point2f& pos, const cv::Point2f& pupil_center) {
    p4_last = pos;
    p4_initialized = true;
    pupil_last = pupil_center;
    pupil_initialized = true;    
  }
};

// Analysis result structures
struct PupilData {
    cv::Point2f center;
    float radius;
    bool detected;
};

struct PurkinjeData {
    cv::Point2f p1_center;
    cv::Point2f p4_center;
    bool p1_detected, p4_detected;
};

struct AnalysisResults {
    int frame_idx;
    PupilData pupil;
    PurkinjeData purkinje;
    bool valid;
};

class EyeTrackingPlugin : public IAnalysisPlugin {
private:
  // Results storage
  std::mutex results_mutex_;
  AnalysisResults latest_results_;
  
  // Debug visualization
  std::vector<cv::Point2f> debug_bright_spots_;
  std::vector<int> debug_spot_intensities_;
  
  // ROI management
  cv::Rect current_roi_;
  bool roi_enabled_;
  
  // Processing control
  std::atomic<bool> running_;
  std::thread analysis_thread_;
  
  struct FrameData {
      cv::Mat frame;
      int frame_idx;
      FrameMetadata metadata;
    };
  
  SharedQueue<FrameData> frame_queue_;
  
  // Pre-allocated working buffers
  cv::Mat gray_buffer_;
  cv::Mat binary_buffer_;
  cv::Mat bright_spots_buffer_;
  cv::Mat dim_spots_buffer_;
  cv::Size frame_size_;
  bool buffers_initialized_;
  
  // Pupil detection parameters
  int pupil_threshold_;
  int pupil_min_area_;
  int pupil_max_area_;
  float pupil_min_circularity_;
  
  // Purkinje detection parameters
  int purkinje_threshold_;

  PurkinjeValidator purkinje_validator_;
  float max_p1_jump_;  // Initialize to ~15 pixels for 250Hz
  float max_p4_jump_;  // Initialize to ~10 pixels for 250Hz
  float p1_exclusion_ratio_;  // Ratio of pupil radius to exclude around P1 (default 0.2)
  float min_p1_p4_ratio_;     // Minimum P1-P4 distance as ratio of pupil radius (default 0.6)
  float p1_max_distance_ratio_;
  
  float p4_search_radius_ratio_;      // Ratio of pupil radius for P4 search area (default 0.85)
  float p4_min_separation_ratio_;     // Minimum P1-P4 separation as ratio of pupil radius (default 0.3)
  int p4_min_brightness_;             // Minimum brightness threshold for P4 (default 160)
  float p4_angular_bonus_;            // Bonus weight for opposite-side P4 (default 1.0)
  int p4_max_area_;                   // Maximum area for P4 blob (default 100)
  int p4_min_area_;                   // Minimum area for P4 blob (default 2)

  std::vector<std::pair<float, cv::Point2f>> last_p4_candidates_;

  int p4_loss_counter_;
  cv::Point2f p4_last_stable_;
  int p4_loss_tolerance_ = 3;  // Frames to tolerate loss

  cv::Mat p4_template_;
  bool use_p4_template_;
  cv::Point2f p4_template_center_;
  float p4_template_threshold_;
  
  cv::Rect p1_seed_roi_;
  bool use_p1_seed_;
  
  // Blob detector
  cv::SimpleBlobDetector::Params blob_params_;
  cv::Ptr<cv::SimpleBlobDetector> detector_;
    
    int frame_count_;
    
    void initializeBlobDetector() {
        blob_params_.filterByArea = true;
        blob_params_.minArea = 5;
        blob_params_.maxArea = 200;
        
        blob_params_.filterByCircularity = true;
        blob_params_.minCircularity = 0.8f;
        
        blob_params_.filterByConvexity = true;
        blob_params_.minConvexity = 0.8f;
        
        blob_params_.filterByInertia = true;
        blob_params_.minInertiaRatio = 0.5f;
        
        detector_ = cv::SimpleBlobDetector::create(blob_params_);
    }
    
    void ensureBuffersAllocated(const cv::Mat& frame) {
        cv::Size current_size(frame.cols, frame.rows);
        
        // Only allocate/reallocate if size changed or not yet initialized
        if (!buffers_initialized_ || current_size != frame_size_) {
            frame_size_ = current_size;
            
            gray_buffer_ = cv::Mat(frame_size_, CV_8UC1);
            binary_buffer_ = cv::Mat(frame_size_, CV_8UC1);
            bright_spots_buffer_ = cv::Mat(frame_size_, CV_8UC1);
            dim_spots_buffer_ = cv::Mat(frame_size_, CV_8UC1);
            
            buffers_initialized_ = true;
            
            std::cout << "Allocated eye tracking buffers: " 
                      << frame_size_.width << "x" << frame_size_.height << std::endl;
        }
    }
    
    PupilData detectPupil(const cv::Mat& frame) {
        PupilData result = {cv::Point2f(-1,-1), -1, false};
        
        cv::Mat roi_frame = roi_enabled_ ? frame(current_roi_) : frame;
        
        // Reuse pre-allocated buffers instead of creating new ones
        cv::Mat gray_roi, binary_roi;
        if (roi_enabled_) {
            // For ROI, use a subset of the buffer
            gray_roi = gray_buffer_(current_roi_);
            binary_roi = binary_buffer_(current_roi_);
        } else {
            gray_roi = gray_buffer_;
            binary_roi = binary_buffer_;
        }
        
        if (roi_frame.channels() > 1) {
            cv::cvtColor(roi_frame, gray_roi, cv::COLOR_BGR2GRAY);
        } else {
            roi_frame.copyTo(gray_roi);
        }
        
        cv::threshold(gray_roi, binary_roi, pupil_threshold_, 255, 
                     cv::THRESH_BINARY_INV);
        
        // Calculate moments for center of mass
        cv::Moments m = cv::moments(binary_roi, true);
        
        if (m.m00 > 0) {
            cv::Point2f center;
            center.x = m.m10 / m.m00;
            center.y = m.m01 / m.m00;
            
            float area = m.m00;
            float radius = sqrt(area / M_PI);
            
            if (roi_enabled_) {
                center.x += current_roi_.x;
                center.y += current_roi_.y;
            }
            
            result = {center, radius, true};
        }
        
        return result;
    }
    

  // Fast brightness detection - finds top N brightest spots
  std::vector<cv::Point2f> findBrightestSpots(const cv::Mat& intensity_region, 
					      int n_spots = 3,
					      float min_separation = 5.0f,
					      float min_brightness = 200.0f) {  // Made configurable
    std::vector<cv::Point2f> spots;
    cv::Mat working = intensity_region.clone();
    
    for (int i = 0; i < n_spots; ++i) {
      cv::Point max_loc;
      double max_val;
      cv::minMaxLoc(working, nullptr, &max_val, nullptr, &max_loc);
      
      if (max_val < min_brightness) break;
      
      // For bloomed spots, find center of mass around peak
      int win_size = 5;  // Larger window for bloomed spots
      cv::Rect window(std::max(0, max_loc.x - win_size),
		      std::max(0, max_loc.y - win_size),
		      win_size * 2 + 1, win_size * 2 + 1);
      window = window & cv::Rect(0, 0, intensity_region.cols, intensity_region.rows);
      
      cv::Mat window_region = intensity_region(window);
      
      // Create binary mask of bright pixels in window
      cv::Mat bright_mask;
      double threshold_val = max_val * 0.7;  // Pixels at least 70% of peak
      cv::threshold(window_region, bright_mask, threshold_val, 255, cv::THRESH_BINARY);
      
      // Get center of mass of bright region
      cv::Moments m = cv::moments(bright_mask, true);
      
      cv::Point2f refined(max_loc.x, max_loc.y);
      if (m.m00 > 3) {  // At least a few bright pixels
	refined.x = window.x + m.m10 / m.m00;
	refined.y = window.y + m.m01 / m.m00;
      }
      
      // Check if too close to existing spots
      bool too_close = false;
      for (const auto& existing : spots) {
	if (cv::norm(refined - existing) < min_separation) {
	  too_close = true;
	  break;
	}
      }
      
      if (!too_close) {
	spots.push_back(refined);
      }
      
      // Mask out larger region for bloomed spots
      cv::circle(working, max_loc, 10, 0, -1);  // Increased from 8
    }
    
    return spots;
  }


  float scoreP4Candidate(const cv::Point2f& candidate,
			 const cv::Mat& gray_roi,
			 const cv::Point2f& p1_pos,
			 const cv::Point2f& pupil_center,
			 float pupil_radius) {
    
    // Bounds check
    if (candidate.x < 3 || candidate.y < 3 || 
	candidate.x >= gray_roi.cols - 3 || 
	candidate.y >= gray_roi.rows - 3) {
      return 0;
    }
    
    // Local peak analysis
    int win = 3;
    cv::Rect local_rect(candidate.x - win, candidate.y - win, win*2+1, win*2+1);
    local_rect = local_rect & cv::Rect(0, 0, gray_roi.cols, gray_roi.rows);
    
    if (local_rect.area() == 0) return 0;
    
    cv::Mat local = gray_roi(local_rect);
    double center_val = gray_roi.at<uchar>(candidate);
    double mean_val = cv::mean(local)[0];
    float peak_ratio = center_val / (mean_val + 1);
    
    if (peak_ratio < 1.05) return 0;
    
    // Compactness analysis
    cv::Mat binary;
    cv::threshold(local, binary, center_val * 0.8, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    float compactness = 1.0f;
    if (!contours.empty() && contours[0].size() > 2) {
      cv::Rect bbox = cv::boundingRect(contours[0]);
      if (bbox.width > 0 && bbox.height > 0) {
        // For tiny spots (1-3 pixels), don't penalize shape
        if (bbox.area() < 10) {
	  compactness = 0.8f;  // Fixed decent score
        } else {
	  float aspect = (float)std::min(bbox.width, bbox.height) / 
	    std::max(bbox.width, bbox.height);
	  compactness = aspect;
        }
      }
    }
    
    // Geometric constraints
    float dist_from_p1 = cv::norm(candidate - p1_pos);
    float dist_from_center = cv::norm(candidate - pupil_center);
    
    float dist_score = 0;
    
    // For P4 near center, relax the constraints
    if (dist_from_center < pupil_radius * 0.4f) {  // P4 is near center
      // Just need to be away from P1
      if (dist_from_p1 > pupil_radius * 0.15f) {  // Very small minimum separation
        dist_score = 1.0f;
      }
    } else {
      // Original constraints for P4 farther from center
      if (dist_from_p1 > pupil_radius * p4_min_separation_ratio_ && 
	  dist_from_p1 < pupil_radius * 0.9f &&
	  dist_from_center < pupil_radius * 0.7f) {
        dist_score = 1.0f;
      }
    }        
    return peak_ratio * compactness * dist_score;
  }

  // Template capture function
  void captureP4Template(cv::Point click_pos) {
    if (gray_buffer_.empty()) return;
    
    // Get ROI around click
    int template_size = 11;
    cv::Rect template_roi(
			  click_pos.x - template_size/2,
			  click_pos.y - template_size/2,
			  template_size,
			  template_size
			  );
    
    // Adjust for ROI if enabled
    cv::Mat source = gray_buffer_;
    if (roi_enabled_) {
      source = gray_buffer_(current_roi_);
      template_roi.x -= current_roi_.x;
      template_roi.y -= current_roi_.y;
    }
    
    // Ensure ROI is within bounds
    template_roi = template_roi & cv::Rect(0, 0, source.cols, source.rows);
    
    if (template_roi.area() > 0) {
      p4_template_ = source(template_roi).clone();
      p4_template_center_ = cv::Point2f(template_size/2, template_size/2);
      use_p4_template_ = true;
      p4_template_threshold_ = 0.7f;  // Correlation threshold
      
      // Normalize template for correlation
      cv::normalize(p4_template_, p4_template_, 0, 1, cv::NORM_MINMAX, CV_32F);
      
      std::cout << "P4 template captured: " << template_roi.width 
		<< "x" << template_roi.height << std::endl;
    }
  }
  
  PurkinjeData detectPurkinje(const cv::Mat& frame, const PupilData& pupil) {
    PurkinjeData result = {{-1,-1}, {-1,-1}, false, false};
    
    debug_bright_spots_.clear();
    debug_spot_intensities_.clear();
    
    if (!pupil.detected) {
        return result;
    }
    
    cv::Mat roi_frame = roi_enabled_ ? frame(current_roi_) : frame;
    cv::Mat gray_roi;
    if (roi_enabled_) {
        gray_roi = gray_buffer_(current_roi_);
    } else {
        gray_roi = gray_buffer_;
    }
    
    if (roi_frame.channels() > 1) {
        cv::cvtColor(roi_frame, gray_roi, cv::COLOR_BGR2GRAY);
    } else {
        roi_frame.copyTo(gray_roi);
    }
    
    // Light Gaussian blur to reduce noise but preserve small spots
    cv::GaussianBlur(gray_roi, gray_roi, cv::Size(3, 3), 0.5);
    
    cv::Point2f pupil_center_local = pupil.center;
    if (roi_enabled_) {
        pupil_center_local.x -= current_roi_.x;
        pupil_center_local.y -= current_roi_.y;
    }
    
    // ===== ROBUST P1 DETECTION =====
    // Search entire pupil area + small margin
    float search_radius = pupil.radius * 1.2f;
    
    int x1 = std::max(0, (int)(pupil_center_local.x - search_radius));
    int y1 = std::max(0, (int)(pupil_center_local.y - search_radius));
    int x2 = std::min(gray_roi.cols, (int)(pupil_center_local.x + search_radius));
    int y2 = std::min(gray_roi.rows, (int)(pupil_center_local.y + search_radius));
    
    if (use_p1_seed_ && p1_seed_roi_.area() > 0) {
        cv::Rect seed_local = p1_seed_roi_;
        if (roi_enabled_) {
            seed_local.x -= current_roi_.x;
            seed_local.y -= current_roi_.y;
        }
        x1 = std::max(0, seed_local.x);
        y1 = std::max(0, seed_local.y);
        x2 = std::min(gray_roi.cols, seed_local.x + seed_local.width);
        y2 = std::min(gray_roi.rows, seed_local.y + seed_local.height);
    }
    
    cv::Rect p1_search_rect(x1, y1, x2 - x1, y2 - y1);
    
    if (p1_search_rect.width > 0 && p1_search_rect.height > 0) {
        cv::Mat p1_region = gray_roi(p1_search_rect);
        
        // Multi-threshold approach to handle both bright and dim spots
        std::vector<int> thresholds = {220, 200, 180};  // Try progressively lower thresholds
        cv::Point2f best_p1(-1, -1);
        float best_score = 0;
        
        for (int thresh : thresholds) {
            cv::Mat p1_thresh;
            cv::threshold(p1_region, p1_thresh, thresh, 255, cv::THRESH_BINARY);
            
            // Find connected components
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(p1_thresh.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            for (const auto& contour : contours) {
                float area = cv::contourArea(contour);
                if (area < 2 || area > 500) continue;  // Allow very small spots
                
                // Get centroid
                cv::Moments m = cv::moments(contour);
                if (m.m00 < 1) continue;
                
                cv::Point2f center_rel(m.m10 / m.m00, m.m01 / m.m00);
                
                // Calculate weighted center using intensity
                cv::Rect bbox = cv::boundingRect(contour);
                bbox = bbox & cv::Rect(0, 0, p1_region.cols, p1_region.rows);
                
                if (bbox.area() == 0) continue;
                
                cv::Mat spot_region = p1_region(bbox);
                cv::Mat spot_mask = p1_thresh(bbox);
                
                // Get mean and max intensity
                cv::Scalar mean_int = cv::mean(spot_region, spot_mask);
                double max_int;
                cv::minMaxLoc(spot_region, nullptr, &max_int);
                
                // Refine center using intensity-weighted centroid
                cv::Mat spot_float;
                spot_region.convertTo(spot_float, CV_32F);
                spot_float.setTo(0, spot_mask == 0);
                
                // Subtract background to enhance peak
                spot_float = spot_float - (float)(thresh - 20);
                cv::threshold(spot_float, spot_float, 0, 255, cv::THRESH_TOZERO);
                
                cv::Moments m_weighted = cv::moments(spot_float, false);
                cv::Point2f refined_center = center_rel;
                
                if (m_weighted.m00 > 1) {
                    refined_center.x = bbox.x + m_weighted.m10 / m_weighted.m00;
                    refined_center.y = bbox.y + m_weighted.m01 / m_weighted.m00;
                }
                
                // Score based on: brightness, compactness, circularity
                float circularity = 4.0 * M_PI * area / (cv::arcLength(contour, true) * cv::arcLength(contour, true));
                circularity = std::max(0.0f, std::min(1.0f, circularity));
                
                float compactness = area / (bbox.width * bbox.height);
                
                float score = max_int * circularity * compactness;
                
                // Prefer spots in lower region if IR is from below (bonus for lower position)
                cv::Point2f spot_global(p1_search_rect.x + refined_center.x,
                                        p1_search_rect.y + refined_center.y);
                float vertical_bias = (spot_global.y > pupil_center_local.y) ? 1.2f : 1.0f;
                score *= vertical_bias;
                
                if (score > best_score) {
                    best_score = score;
                    best_p1 = refined_center;
                    
                    // Store debug info
                    debug_bright_spots_.clear();
                    debug_spot_intensities_.clear();
                    debug_bright_spots_.push_back(cv::Point2f(p1_search_rect.x + refined_center.x,
                                                               p1_search_rect.y + refined_center.y));
                    debug_spot_intensities_.push_back((int)max_int);
                }
            }
            
            // If we found a good candidate at high threshold, don't try lower ones
            if (best_score > 0 && thresh > 200) break;
        }
        
        if (best_p1.x > 0) {
            cv::Point2f p1_local(p1_search_rect.x + best_p1.x,
                                 p1_search_rect.y + best_p1.y);
            
            cv::Point2f p1_full = p1_local;
            if (roi_enabled_) {
                p1_full.x += current_roi_.x;
                p1_full.y += current_roi_.y;
            }
            
            // Validate distance from pupil center (allow very close for small CRs)
	    float dist = cv::norm(p1_local - pupil_center_local);
	    if (dist < pupil.radius * p1_max_distance_ratio_) {
	      if (purkinje_validator_.isValidP1(p1_full, max_p1_jump_)) {
		result.p1_center = p1_full;
		result.p1_detected = true;
		purkinje_validator_.updateP1(p1_full);
	      } else {
		std::cout << "P1 rejected by validator: jump too large" << std::endl;
	      }
	    } else {
	      std::cout << "P1 rejected: dist=" << dist << " > " << (pupil.radius * p1_max_distance_ratio_) << std::endl;
	    }
        } else {
	  std::cout << "No P1 candidate found in search region" << std::endl;
        }
    }
    
    // ===== P4 DETECTION =====
    if (result.p1_detected) {
      cv::Mat p4_search = gray_roi.clone();
      
      cv::Point2f p1_local = result.p1_center;
      if (roi_enabled_) {
        p1_local.x -= current_roi_.x;
        p1_local.y -= current_roi_.y;
      }
      
      // Block out P1 region
      float p1_block_radius = pupil.radius * 0.3f;
      cv::circle(p4_search, p1_local, p1_block_radius, 0, -1);
      
      // Create search mask
      cv::Mat p4_mask = cv::Mat::zeros(gray_roi.size(), CV_8UC1);
      float search_radius = pupil.radius * p4_search_radius_ratio_;
      cv::circle(p4_mask, pupil_center_local, search_radius, 255, -1);
      cv::circle(p4_mask, p1_local, p1_block_radius, 0, -1);
      
      // Apply mask and normalize
      cv::Mat p4_enhanced;
      p4_search.copyTo(p4_enhanced, p4_mask);
      cv::normalize(p4_enhanced, p4_enhanced, 0, 255, cv::NORM_MINMAX, CV_8UC1, p4_mask);
      
      // Collect ALL candidates with scoring function
      std::vector<std::pair<float, cv::Point2f>> p4_candidates;

      if (use_p4_template_ && !p4_template_.empty()) {
        // Convert search region to float
        cv::Mat search_float;
        p4_enhanced.convertTo(search_float, CV_32F, 1.0/255.0);
        
        // Template matching
        cv::Mat correlation;
        cv::matchTemplate(search_float, p4_template_, correlation, cv::TM_CCORR_NORMED);
        
        // Find peaks in correlation
        double min_val, max_val;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(correlation, &min_val, &max_val, &min_loc, &max_loc);
        
        if (max_val > p4_template_threshold_) {
	  // Convert to full coordinates
	  cv::Point2f template_match(
				     max_loc.x + p4_template_.cols/2,
				     max_loc.y + p4_template_.rows/2
				     );
	  
	  // Verify it's in valid P4 region
	  float dist_from_p1 = cv::norm(template_match - p1_local);
	  float dist_from_center = cv::norm(template_match - pupil_center_local);
          
	  if (dist_from_p1 > pupil.radius * 0.15f && 
	      dist_from_center < pupil.radius * 0.7f) {
	    
	    // High score for template match
	    float template_score = 3.0f * max_val;  // Boost template matches
	    p4_candidates.push_back({template_score, template_match});
            
	    if (frame_count_ % 100 == 0) {
	      std::cout << "P4 template match: correlation=" << max_val 
			<< " at " << template_match << std::endl;
	    }
	  }
        }
        
        // Also look for secondary peaks
        cv::Mat mask = cv::Mat::zeros(correlation.size(), CV_8UC1);
        cv::circle(mask, max_loc, 5, 255, -1);  // Mask out primary peak
        correlation.setTo(0, mask);
        
        cv::minMaxLoc(correlation, &min_val, &max_val, &min_loc, &max_loc);
        if (max_val > p4_template_threshold_ * 0.9f) {
	  cv::Point2f secondary(
				max_loc.x + p4_template_.cols/2,
				max_loc.y + p4_template_.rows/2
				);
	  p4_candidates.push_back({2.5f * max_val, secondary});
        }
      }
      
      // Multi-threshold approach
      std::vector<int> thresholds;
      for (int t = 240; t >= p4_min_brightness_; t -= 20) {
        thresholds.push_back(t);
      }
      
      for (int thresh : thresholds) {
        cv::Mat p4_binary;
        cv::threshold(p4_enhanced, p4_binary, thresh, 255, cv::THRESH_BINARY);
        p4_binary = p4_binary & p4_mask;
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(p4_binary.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
	  float area = cv::contourArea(contour);
	  if (area < p4_min_area_ || area > p4_max_area_) continue;
          
	  cv::Moments m = cv::moments(contour);
	  if (m.m00 < 1) continue;
          
	  cv::Point2f candidate(m.m10 / m.m00, m.m01 / m.m00);
          
	  float score = scoreP4Candidate(candidate, gray_roi, p1_local, 
					 pupil_center_local, pupil.radius);
	  
	  if (score > 0.1f) {  // LOWERED from 0.3f for better detection
	    // Check if we already have a candidate near this position
	    bool duplicate = false;
	    for (auto& candidate_pair : p4_candidates) {
	      float& existing_score = candidate_pair.first;
	      cv::Point2f& existing_pos = candidate_pair.second;
	      if (cv::norm(candidate - existing_pos) < 5.0f) {
		if (score > existing_score) {
		  existing_score = score;
		}
		duplicate = true;
		break;
	      }
	    }
            
	    if (!duplicate) {
	      p4_candidates.push_back({score, candidate});
	    }
	  }
        }
      }
      
      // Sort candidates by score
      std::sort(p4_candidates.begin(), p4_candidates.end(),
		[](const auto& a, const auto& b) { return a.first > b.first; });
      
      // Store for debugging/analysis
      last_p4_candidates_ = p4_candidates;
      
      // Try to find valid P4 from candidates
      bool found_valid_p4 = false;
      cv::Point2f pupil_global = pupil.center;
      
      for (const auto& candidate_pair : p4_candidates) {
        float score = candidate_pair.first;
        cv::Point2f candidate = candidate_pair.second;
        cv::Point2f p4_full = candidate;
        if (roi_enabled_) {
	  p4_full.x += current_roi_.x;
	  p4_full.y += current_roi_.y;
        }
        
        if (purkinje_validator_.isValidP4(p4_full, pupil_global, max_p4_jump_)) {
	  result.p4_center = p4_full;
	  result.p4_detected = true;
	  purkinje_validator_.updateP4(p4_full, pupil_global);
	  p4_last_stable_ = p4_full;  // Update stable position
	  p4_loss_counter_ = 0;       // Reset loss counter
	  found_valid_p4 = true;
          
	  if (frame_count_ % 100 == 0) {
	    std::cout << "P4 selected with score: " << score 
		      << " from " << p4_candidates.size() << " candidates" << std::endl;
	  }
	  break;
        }
      }
      
      // Hysteresis: If no valid candidate but recently had P4, maintain position
      if (!found_valid_p4) {
        if (p4_loss_counter_ < p4_loss_tolerance_ && purkinje_validator_.p4_initialized) {
	  // Use last stable position
	  result.p4_center = p4_last_stable_;
	  result.p4_detected = true;
	  p4_loss_counter_++;
	  
	  if (frame_count_ % 50 == 0) {
	    std::cout << "P4 maintained from history (loss count: " 
		      << p4_loss_counter_ << "/" << p4_loss_tolerance_ << ")" << std::endl;
	  }
        } else {
	  // Really lost P4
	  result.p4_detected = false;
          
	  if (!p4_candidates.empty() && frame_count_ % 100 == 0) {
	    std::cout << "P4 candidates found but rejected. Best score: " 
		      << p4_candidates[0].first << std::endl;
	  }
        }
      }
    }
    
    return result;
  }
  
  void analysisThreadFunc() {
    std::cout << "Eye tracking analysis thread started" << std::endl;
    
    while (running_) {
      try {
                FrameData frame_data = frame_queue_.front();
                frame_queue_.pop_front();

                // Check for shutdown sentinel
                if (frame_data.frame_idx < 0) {
                    break;
                }
                
                if (frame_data.frame.empty()) {
                    continue;
                }
                
                // Ensure buffers are allocated before first use
                ensureBuffersAllocated(frame_data.frame);
                
                auto start = std::chrono::high_resolution_clock::now();
                
                // Detect pupil first
                PupilData pupil = detectPupil(frame_data.frame);
                
                // Use pupil location to constrain Purkinje search
                PurkinjeData purkinje = detectPurkinje(frame_data.frame, pupil);
                
                {
                    std::lock_guard<std::mutex> lock(results_mutex_);
                    latest_results_.frame_idx = frame_data.frame_idx;
                    latest_results_.pupil = pupil;
                    latest_results_.purkinje = purkinje;
                    latest_results_.valid = true;
                }
                
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                
                frame_count_++;
                
                if (frame_count_ % 100 == 0) {
                    std::cout << "Analysis time: " << duration.count() << "µs" << std::endl;
                    if (pupil.detected) {
                        std::cout << "Pupil: (" << pupil.center.x << "," << pupil.center.y 
                                  << ") r=" << pupil.radius << std::endl;
                    }
                }
            } catch (...) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
    
    // Tcl command callbacks
    static int setPupilThresholdCmd(ClientData clientData, Tcl_Interp *interp,
                                    int objc, Tcl_Obj *const objv[]) {
        if (objc != 2) {
            Tcl_WrongNumArgs(interp, 1, objv, "threshold");
            return TCL_ERROR;
        }
        
        int threshold;
        if (Tcl_GetIntFromObj(interp, objv[1], &threshold) != TCL_OK) {
            return TCL_ERROR;
        }
        
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        plugin->pupil_threshold_ = threshold;
        return TCL_OK;
    }
    
    static int setROICmd(ClientData clientData, Tcl_Interp *interp,
                        int objc, Tcl_Obj *const objv[]) {
        if (objc != 5) {
            Tcl_WrongNumArgs(interp, 1, objv, "x y width height");
            return TCL_ERROR;
        }
        
        int x, y, width, height;
        if (Tcl_GetIntFromObj(interp, objv[1], &x) != TCL_OK ||
            Tcl_GetIntFromObj(interp, objv[2], &y) != TCL_OK ||
            Tcl_GetIntFromObj(interp, objv[3], &width) != TCL_OK ||
            Tcl_GetIntFromObj(interp, objv[4], &height) != TCL_OK) {
            return TCL_ERROR;
        }
        
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        plugin->current_roi_ = cv::Rect(x, y, width, height);
        plugin->roi_enabled_ = true;
        return TCL_OK;
    }
    
    static int disableROICmd(ClientData clientData, Tcl_Interp *interp,
                            int objc, Tcl_Obj *const objv[]) {
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        plugin->roi_enabled_ = false;
        return TCL_OK;
    }

  static int setP1SeedROICmd(ClientData clientData, Tcl_Interp *interp,
			     int objc, Tcl_Obj *const objv[]) {
    if (objc != 5) {
      Tcl_WrongNumArgs(interp, 1, objv, "center_x center_y width height");
      return TCL_ERROR;
    }
    
    int cx, cy, w, h;
    if (Tcl_GetIntFromObj(interp, objv[1], &cx) != TCL_OK ||
        Tcl_GetIntFromObj(interp, objv[2], &cy) != TCL_OK ||
        Tcl_GetIntFromObj(interp, objv[3], &w) != TCL_OK ||
        Tcl_GetIntFromObj(interp, objv[4], &h) != TCL_OK) {
      return TCL_ERROR;
    }
    
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    plugin->p1_seed_roi_ = cv::Rect(cx - w/2, cy - h/2, w, h);
    plugin->use_p1_seed_ = true;
    return TCL_OK;
  }

  static int setValidatorThresholdsCmd(ClientData clientData, Tcl_Interp *interp,
				       int objc, Tcl_Obj *const objv[]) {
    if (objc != 3) {
      Tcl_WrongNumArgs(interp, 1, objv, "max_p1_jump max_p4_jump");
      return TCL_ERROR;
    }
    
    double p1_jump, p4_jump;
    if (Tcl_GetDoubleFromObj(interp, objv[1], &p1_jump) != TCL_OK ||
        Tcl_GetDoubleFromObj(interp, objv[2], &p4_jump) != TCL_OK) {
      return TCL_ERROR;
    }
    
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    plugin->max_p1_jump_ = (float)p1_jump;
    plugin->max_p4_jump_ = (float)p4_jump;
    
    std::cout << "Validator thresholds updated: P1=" << p1_jump 
              << " P4=" << p4_jump << std::endl;
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj("Validator thresholds updated", -1));
    return TCL_OK;
  }

  static int resetValidatorCmd(ClientData clientData, Tcl_Interp *interp,
			       int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    plugin->purkinje_validator_.reset();
    std::cout << "Purkinje validator reset" << std::endl;
    return TCL_OK;
  }

  static int setP1MaxDistanceCmd(ClientData clientData, Tcl_Interp *interp,
				 int objc, Tcl_Obj *const objv[]) {
    if (objc != 2) {
      Tcl_WrongNumArgs(interp, 1, objv, "distance_ratio");
      return TCL_ERROR;
    }
    
    double ratio;
    if (Tcl_GetDoubleFromObj(interp, objv[1], &ratio) != TCL_OK) {
      return TCL_ERROR;
    }
    
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    plugin->p1_max_distance_ratio_ = (float)ratio;
    
    std::cout << "P1 max distance ratio updated: " << ratio << std::endl;
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj("P1 max distance updated", -1));
    return TCL_OK;
  }

  static int setP4TemplateCmd(ClientData clientData, Tcl_Interp *interp,
			      int objc, Tcl_Obj *const objv[]) {
    if (objc != 3) {
      Tcl_WrongNumArgs(interp, 1, objv, "x y");
      return TCL_ERROR;
    }
    
    int x, y;
    if (Tcl_GetIntFromObj(interp, objv[1], &x) != TCL_OK ||
        Tcl_GetIntFromObj(interp, objv[2], &y) != TCL_OK) {
      return TCL_ERROR;
    }
    
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    // Capture template from current frame around clicked point
    plugin->captureP4Template(cv::Point(x, y));
    
    std::cout << "P4 template captured at: " << x << "," << y << std::endl;
    return TCL_OK;
  }


  
  static int setP4DetectionParamsCmd(ClientData clientData, Tcl_Interp *interp,
				     int objc, Tcl_Obj *const objv[]) {
    if (objc != 5) {
      Tcl_WrongNumArgs(interp, 1, objv, "search_radius min_separation min_brightness angular_bonus");
      return TCL_ERROR;
    }
    
    double search_radius, min_sep, angular;
    int min_bright;
    
    if (Tcl_GetDoubleFromObj(interp, objv[1], &search_radius) != TCL_OK ||
        Tcl_GetDoubleFromObj(interp, objv[2], &min_sep) != TCL_OK ||
        Tcl_GetIntFromObj(interp, objv[3], &min_bright) != TCL_OK ||
        Tcl_GetDoubleFromObj(interp, objv[4], &angular) != TCL_OK) {
      return TCL_ERROR;
    }
    
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    plugin->p4_search_radius_ratio_ = (float)search_radius;
    plugin->p4_min_separation_ratio_ = (float)min_sep;
    plugin->p4_min_brightness_ = min_bright;
    plugin->p4_angular_bonus_ = (float)angular;
    
    std::cout << "P4 detection params updated: search=" << search_radius 
              << " sep=" << min_sep << " bright=" << min_bright 
              << " angular=" << angular << std::endl;
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj("P4 detection parameters updated", -1));
    return TCL_OK;
  }
  
  static int setP4AreaConstraintsCmd(ClientData clientData, Tcl_Interp *interp,
				     int objc, Tcl_Obj *const objv[]) {
    if (objc != 3) {
      Tcl_WrongNumArgs(interp, 1, objv, "min_area max_area");
      return TCL_ERROR;
    }
    
    int min_area, max_area;
    if (Tcl_GetIntFromObj(interp, objv[1], &min_area) != TCL_OK ||
        Tcl_GetIntFromObj(interp, objv[2], &max_area) != TCL_OK) {
      return TCL_ERROR;
    }
    
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    plugin->p4_min_area_ = min_area;
    plugin->p4_max_area_ = max_area;
    
    std::cout << "P4 area constraints: " << min_area << " - " << max_area << std::endl;
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj("P4 area constraints updated", -1));
    return TCL_OK;
  }
  
  static int getResultsCmd(ClientData clientData, Tcl_Interp *interp,
                             int objc, Tcl_Obj *const objv[]) {
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        
        std::lock_guard<std::mutex> lock(plugin->results_mutex_);
        
        if (!plugin->latest_results_.valid) {
            Tcl_SetObjResult(interp, Tcl_NewStringObj("no results", -1));
            return TCL_OK;
        }
        
        // Build result dict
        Tcl_Obj* resultDict = Tcl_NewDictObj();
        
        // Pupil data
        if (plugin->latest_results_.pupil.detected) {
            Tcl_Obj* pupilDict = Tcl_NewDictObj();
            Tcl_DictObjPut(interp, pupilDict, Tcl_NewStringObj("x", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.pupil.center.x));
            Tcl_DictObjPut(interp, pupilDict, Tcl_NewStringObj("y", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.pupil.center.y));
            Tcl_DictObjPut(interp, pupilDict, Tcl_NewStringObj("radius", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.pupil.radius));
            Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("pupil", -1), pupilDict);
        }
        
        // Purkinje data
        if (plugin->latest_results_.purkinje.p1_detected) {
            Tcl_Obj* p1Dict = Tcl_NewDictObj();
            Tcl_DictObjPut(interp, p1Dict, Tcl_NewStringObj("x", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.purkinje.p1_center.x));
            Tcl_DictObjPut(interp, p1Dict, Tcl_NewStringObj("y", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.purkinje.p1_center.y));
            Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("p1", -1), p1Dict);
        }
        
        if (plugin->latest_results_.purkinje.p4_detected) {
            Tcl_Obj* p4Dict = Tcl_NewDictObj();
            Tcl_DictObjPut(interp, p4Dict, Tcl_NewStringObj("x", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.purkinje.p4_center.x));
            Tcl_DictObjPut(interp, p4Dict, Tcl_NewStringObj("y", -1),
                          Tcl_NewDoubleObj(plugin->latest_results_.purkinje.p4_center.y));
            Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("p4", -1), p4Dict);
        }
        
        Tcl_SetObjResult(interp, resultDict);
        return TCL_OK;
    }  
    
public:
    EyeTrackingPlugin() 
        : roi_enabled_(false),
          running_(false),
          pupil_threshold_(65),
          pupil_min_area_(5000),
          pupil_max_area_(100000),
          pupil_min_circularity_(0.65),
          purkinje_threshold_(240),
          frame_count_(0),
	  max_p1_jump_(30.0f),
	  max_p4_jump_(30.0f),
	  p1_exclusion_ratio_(0.3f),
	  min_p1_p4_ratio_(0.1f),
	  use_p1_seed_(false),
	  p1_max_distance_ratio_(1.3f),
	  p4_search_radius_ratio_(0.85f),
	  p4_min_separation_ratio_(0.3f),
	  p4_min_brightness_(160),
	  p4_angular_bonus_(1.0f),
	  p4_max_area_(100),
	  p4_min_area_(2),
	  p4_loss_counter_(0),
	  p4_loss_tolerance_(3),  // Allow 3 frames of loss
	  p4_last_stable_(cv::Point2f(-1, -1)),	  
          buffers_initialized_(false) {
        
        latest_results_.valid = false;

    }
    
    ~EyeTrackingPlugin() {
        shutdown();
    }
    
    bool initialize(Tcl_Interp* interp) override {
        initializeBlobDetector();
        running_ = true;
        analysis_thread_ = std::thread(&EyeTrackingPlugin::analysisThreadFunc, this);
        return true;
    }
    
    void shutdown() override {
        running_ = false;

        // Push sentinel to unblock thread
        FrameData sentinel;
        sentinel.frame_idx = -1;  // Signal to exit
        frame_queue_.push_back(sentinel);
        
        if (analysis_thread_.joinable()) {
            analysis_thread_.join();
        }
    }
    
    void analyzeFrame(const cv::Mat& frame, int frameIdx, 
                     const FrameMetadata& metadata) override {
        FrameData frame_data;
        frame_data.frame = frame.clone();
        frame_data.frame_idx = frameIdx;
        frame_data.metadata = metadata;
        
        frame_queue_.push_back(frame_data);
    }
    
    void registerTclCommands(Tcl_Interp* interp) override {
        Tcl_Eval(interp, "namespace eval ::eyetracking {}");

        Tcl_CreateObjCommand(interp, "::eyetracking::setPupilThreshold", 
                            setPupilThresholdCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setROI", 
                            setROICmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::disableROI", 
                            disableROICmd, this, NULL);
	Tcl_CreateObjCommand(interp, "::eyetracking::setP4Template", 
			     setP4TemplateCmd, this, NULL);	
	Tcl_CreateObjCommand(interp, "::eyetracking::setP1SeedROI", 
			     setP1SeedROICmd, this, NULL);
	Tcl_CreateObjCommand(interp, "::eyetracking::setP1MaxDistance", 
			     setP1MaxDistanceCmd, this, NULL);
	Tcl_CreateObjCommand(interp, "::eyetracking::setP4DetectionParams", 
			     setP4DetectionParamsCmd, this, NULL);
	Tcl_CreateObjCommand(interp, "::eyetracking::setP4AreaConstraints", 
			     setP4AreaConstraintsCmd, this, NULL);
	Tcl_CreateObjCommand(interp, "::eyetracking::setValidatorThresholds", 
			     setValidatorThresholdsCmd, this, NULL);
	Tcl_CreateObjCommand(interp, "::eyetracking::resetValidator", 
			     resetValidatorCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::getResults",
                            getResultsCmd, this, NULL);      
    }
    
    const char* getName() override { return "eye_tracking"; }
    const char* getVersion() override { return "1.0"; }
    const char* getDescription() override { 
        return "Pupil and Purkinje reflection tracking"; 
    }
    
    bool drawOverlay(cv::Mat& frame, int frame_idx) override {
        std::lock_guard<std::mutex> lock(results_mutex_);
        
        if (!latest_results_.valid) return false;
        
        // Draw ROI if enabled
        if (roi_enabled_) {
            cv::rectangle(frame, current_roi_, cv::Scalar(255, 255, 0), 2);
            cv::putText(frame, "ROI", 
                       cv::Point(current_roi_.x + 5, current_roi_.y + 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                       cv::Scalar(255, 255, 0), 1);
        }
        
        // Draw pupil
        if (latest_results_.pupil.detected) {
            cv::circle(frame, latest_results_.pupil.center, 
                      (int)latest_results_.pupil.radius, 
                      cv::Scalar(0, 255, 0), 2);
            cv::drawMarker(frame, latest_results_.pupil.center, 
                          cv::Scalar(0, 255, 0), 
                          cv::MARKER_CROSS, 10, 2);
            cv::putText(frame, "Pupil", 
                       cv::Point(latest_results_.pupil.center.x + 
                                latest_results_.pupil.radius + 5, 
                                latest_results_.pupil.center.y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                       cv::Scalar(0, 255, 0), 1);
        }

#ifdef DEBUG_OVERLAY
	if (latest_results_.pupil.detected && !debug_bright_spots_.empty()) {
	  for (size_t i = 0; i < debug_bright_spots_.size(); ++i) {
            cv::Point2f spot = debug_bright_spots_[i];
            int intensity = debug_spot_intensities_[i];
            
            // Draw spot with different colors based on rank
            cv::Scalar color;
            if (i == 0) color = cv::Scalar(255, 255, 0); // Cyan - brightest
            else if (i == 1) color = cv::Scalar(255, 0, 255); // Magenta - 2nd
            else color = cv::Scalar(128, 128, 128); // Gray - others
            
            cv::circle(frame, spot, 6, color, 2);
            cv::circle(frame, spot, 2, color, -1);
            
            // Show intensity value
            char text[32];
            snprintf(text, sizeof(text), "%d", intensity);
            cv::putText(frame, text,
			cv::Point(spot.x + 8, spot.y - 8),
			cv::FONT_HERSHEY_SIMPLEX, 0.35,
			color, 1);
	  }
	}
#endif	
	
        // Draw Purkinje reflections
        if (latest_results_.purkinje.p1_detected) {
	  cv::circle(frame, latest_results_.purkinje.p1_center, 
		     5, cv::Scalar(255, 0, 0), 2);
	  cv::putText(frame, "P1", 
		      cv::Point(latest_results_.purkinje.p1_center.x + 8, 
                                latest_results_.purkinje.p1_center.y),
		      cv::FONT_HERSHEY_SIMPLEX, 0.4, 
		      cv::Scalar(255, 0, 0), 1);
        }
        
        if (latest_results_.purkinje.p4_detected) {
            cv::circle(frame, latest_results_.purkinje.p4_center, 
                      5, cv::Scalar(0, 0, 255), 2);
            cv::putText(frame, "P4", 
                       cv::Point(latest_results_.purkinje.p4_center.x + 8, 
                                latest_results_.purkinje.p4_center.y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, 
                       cv::Scalar(0, 0, 255), 1);
        }
        
        return true;
    }
};

// Standard Tcl package entry point
extern "C" {
    int Eyetracking_Init(Tcl_Interp *interp) {
        EyeTrackingPlugin* plugin = new EyeTrackingPlugin();
        
        if (!plugin->initialize(interp)) {
            delete plugin;
            return TCL_ERROR;
        }
        
        // Register with the global registry
        g_pluginRegistry.registerPlugin(plugin);
        
        // Register Tcl commands
        plugin->registerTclCommands(interp);
        
        // Declare package
        Tcl_PkgProvide(interp, "Eyetracking", "1.0");
        
        return TCL_OK;
    }
}
