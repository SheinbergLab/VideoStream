#include <tcl.h>
#include "opencv2/opencv.hpp"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "SharedQueue.hpp"
#include "IAnalysisPlugin.h"
#include "AnalysisPluginRegistry.h"

extern AnalysisPluginRegistry g_pluginRegistry;

// ============================================================================
// P1 VALIDATOR
// ============================================================================

class P1Validator {
private:
    bool initialized_;
    cv::Point2f last_position_;
    float max_jump_pixels_;

public:
    P1Validator(float max_jump = 15.0f) 
        : initialized_(false), max_jump_pixels_(max_jump) {}

    bool isValid(const cv::Point2f& new_pos) {
        if (!initialized_) return true;
        float dist = cv::norm(new_pos - last_position_);
        bool valid = dist < max_jump_pixels_;        
        return valid;
    }

    void update(const cv::Point2f& pos) {
        last_position_ = pos;
        initialized_ = true;
    }

    void reset() {
        initialized_ = false;
    }

    void setMaxJump(float max_jump) {
        max_jump_pixels_ = max_jump;
    }
};

// ============================================================================
// P4 VALIDATOR
// ============================================================================

class P4Validator {
private:
    bool initialized_;
    cv::Point2f last_p4_position_;
    cv::Point2f last_pupil_center_;
    float max_jump_pixels_;

public:
    P4Validator(float max_jump = 10.0f) 
        : initialized_(false), max_jump_pixels_(max_jump) {}

    bool isValid(const cv::Point2f& new_p4, const cv::Point2f& pupil_center) {
        if (!initialized_) return true;

        float p4_movement = cv::norm(new_p4 - last_p4_position_);
        float pupil_movement = cv::norm(pupil_center - last_pupil_center_);

        // Allow P4 to move more if pupil also moved
        float adjusted_max = max_jump_pixels_ + pupil_movement;
        return p4_movement < adjusted_max;
    }

    void update(const cv::Point2f& p4_pos, const cv::Point2f& pupil_center) {
        last_p4_position_ = p4_pos;
        last_pupil_center_ = pupil_center;
        initialized_ = true;
    }

    void reset() {
        initialized_ = false;
    }

    void setMaxJump(float max_jump) {
        max_jump_pixels_ = max_jump;
    }
};

// ============================================================================
// P1-P4 ROTATIONAL MODEL
// ============================================================================

class P1P4RotationalModel {
private:
    bool initialized_;
    bool frozen_;
    float magnitude_ratio_;
    float angle_offset_;
    float learning_rate_;
    
    std::vector<cv::Point2f> calibration_pupil_centers_;
    std::vector<cv::Point2f> calibration_p1_centers_;
    std::vector<cv::Point2f> calibration_p4_centers_;
    int target_samples_;

    static float wrapAngle(float angle) {
        angle = std::fmod(angle + M_PI, 2.0f * M_PI);
        if (angle < 0) angle += 2.0f * M_PI;
        return angle - M_PI;
    }

public:
    P1P4RotationalModel(int target_samples = 50) 
        : initialized_(false),
          frozen_(false),
          magnitude_ratio_(0.0f),
          angle_offset_(0.0f),
          learning_rate_(0.1f),
          target_samples_(target_samples) {}

    cv::Point2f predict(const cv::Point2f& pupil_center, const cv::Point2f& p1_center) const {
        if (!initialized_) return cv::Point2f(-1, -1);

        cv::Point2f v_pc_p1 = p1_center - pupil_center;
        float mag_p1 = cv::norm(v_pc_p1);
        
        if (mag_p1 < 1e-6) return cv::Point2f(-1, -1);

        float angle_p1 = std::atan2(v_pc_p1.y, v_pc_p1.x);
        float predicted_mag_p4 = mag_p1 * magnitude_ratio_;
        float predicted_angle_p4 = angle_p1 + angle_offset_;

        cv::Point2f predicted_v_p4(
            predicted_mag_p4 * std::cos(predicted_angle_p4),
            predicted_mag_p4 * std::sin(predicted_angle_p4)
        );

        return pupil_center + predicted_v_p4;
    }

    void updateModel(const cv::Point2f& pupil_center, 
                    const cv::Point2f& p1_center, 
                    const cv::Point2f& p4_center) {
        if (frozen_ || !initialized_) return;

        cv::Point2f v_pc_p1 = p1_center - pupil_center;
        cv::Point2f v_pc_p4 = p4_center - pupil_center;
        
        float mag_p1 = cv::norm(v_pc_p1);
        if (mag_p1 < 1e-6) return;

        float obs_mag_ratio = cv::norm(v_pc_p4) / mag_p1;
        float obs_angle_p1 = std::atan2(v_pc_p1.y, v_pc_p1.x);
        float obs_angle_p4 = std::atan2(v_pc_p4.y, v_pc_p4.x);
        float obs_angle_offset = wrapAngle(obs_angle_p4 - obs_angle_p1);

        magnitude_ratio_ = learning_rate_ * obs_mag_ratio + 
                          (1.0f - learning_rate_) * magnitude_ratio_;

        float old_x = std::cos(angle_offset_);
        float old_y = std::sin(angle_offset_);
        float new_x = std::cos(obs_angle_offset);
        float new_y = std::sin(obs_angle_offset);
        
        float avg_x = learning_rate_ * new_x + (1.0f - learning_rate_) * old_x;
        float avg_y = learning_rate_ * new_y + (1.0f - learning_rate_) * old_y;
        
        angle_offset_ = std::atan2(avg_y, avg_x);
    }

    void addCalibrationSample(const cv::Point2f& pupil_center,
                             const cv::Point2f& p1_center,
                             const cv::Point2f& p4_center) {
        calibration_pupil_centers_.push_back(pupil_center);
        calibration_p1_centers_.push_back(p1_center);
        calibration_p4_centers_.push_back(p4_center);
    }

    bool calibrateFromSamples() {
        if (calibration_pupil_centers_.size() < 10) {
            return false;
        }

        std::vector<float> mag_ratios;
        std::vector<float> angle_offsets;

        for (size_t i = 0; i < calibration_pupil_centers_.size(); ++i) {
            cv::Point2f v_pc_p1 = calibration_p1_centers_[i] - calibration_pupil_centers_[i];
            cv::Point2f v_pc_p4 = calibration_p4_centers_[i] - calibration_pupil_centers_[i];
            
            float mag_p1 = cv::norm(v_pc_p1);
            if (mag_p1 < 1e-6) continue;

            mag_ratios.push_back(cv::norm(v_pc_p4) / mag_p1);
            
            float angle_p1 = std::atan2(v_pc_p1.y, v_pc_p1.x);
            float angle_p4 = std::atan2(v_pc_p4.y, v_pc_p4.x);
            angle_offsets.push_back(wrapAngle(angle_p4 - angle_p1));
        }

        if (mag_ratios.empty()) return false;

        float sum = 0;
        for (float r : mag_ratios) sum += r;
        magnitude_ratio_ = sum / mag_ratios.size();

        float sum_sin = 0, sum_cos = 0;
        for (float a : angle_offsets) {
            sum_cos += std::cos(a);
            sum_sin += std::sin(a);
        }
        angle_offset_ = std::atan2(sum_sin / angle_offsets.size(), 
                                   sum_cos / angle_offsets.size());

        initialized_ = true;
        calibration_pupil_centers_.clear();
        calibration_p1_centers_.clear();
        calibration_p4_centers_.clear();

        return true;
    }

    bool isInitialized() const { return initialized_; }
    bool isFrozen() const { return frozen_; }
    void freeze() { frozen_ = true; }
    void unfreeze() { frozen_ = false; }
    void reset() { 
        initialized_ = false; 
        frozen_ = false;
        calibration_pupil_centers_.clear();
        calibration_p1_centers_.clear();
        calibration_p4_centers_.clear();
    }
    
    int getCalibrationSampleCount() const { 
        return calibration_pupil_centers_.size(); 
    }
    
    int getTargetSamples() const { return target_samples_; }
    float getMagnitudeRatio() const { return magnitude_ratio_; }
    float getAngleOffset() const { return angle_offset_; }
};

// ============================================================================
// BLINK DETECTOR (WITH RADIUS CHECK)
// ============================================================================

class BlinkDetector {
private:
    bool in_blink_;
    int recovery_countdown_;
    static constexpr int RECOVERY_FRAMES = 5;
    
    float baseline_radius_;
    bool baseline_initialized_;
    static constexpr float BLINK_ENTER_RATIO = 0.5f;
    static constexpr float BLINK_EXIT_RATIO = 0.7f;
    static constexpr int BASELINE_UPDATE_FRAMES = 10;
    int frames_since_update_;

public:
    BlinkDetector() : in_blink_(false), recovery_countdown_(0),
                     baseline_radius_(0), baseline_initialized_(false),
                     frames_since_update_(0) {}

    void update(bool pupil_detected, float pupil_radius) {
        if (pupil_detected && !in_blink_ && recovery_countdown_ == 0) {
            if (!baseline_initialized_) {
                baseline_radius_ = pupil_radius;
                baseline_initialized_ = true;
            } else {
                frames_since_update_++;
                if (frames_since_update_ >= BASELINE_UPDATE_FRAMES) {
                    baseline_radius_ = 0.95f * baseline_radius_ + 0.05f * pupil_radius;
                    frames_since_update_ = 0;
                }
            }
        }
        
        bool blink_condition;
        if (in_blink_) {
            blink_condition = !pupil_detected || 
                             (baseline_initialized_ && 
                              pupil_radius < baseline_radius_ * BLINK_EXIT_RATIO);
        } else {
            blink_condition = !pupil_detected || 
                             (baseline_initialized_ && 
                              pupil_radius < baseline_radius_ * BLINK_ENTER_RATIO);
        }
        
        if (blink_condition) {
            if (!in_blink_) {
                in_blink_ = true;
                recovery_countdown_ = RECOVERY_FRAMES;
            }
        } else {
            if (in_blink_) {
                in_blink_ = false;
                recovery_countdown_ = RECOVERY_FRAMES;
            }
        }
    }
    
    void decrementRecovery() {
        if (recovery_countdown_ > 0) {
            recovery_countdown_--;
        }
    }
    
    bool isInBlink() const { return in_blink_; }
    bool isRecovering() const { return recovery_countdown_ > 0; }
    
    bool shouldResetValidators() const {
        return in_blink_ && recovery_countdown_ == RECOVERY_FRAMES;
    }
    
    float getBaselineRadius() const { return baseline_radius_; }
};

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct PupilData {
    cv::Point2f center;
    float radius;
    bool detected;
};

struct PurkinjeData {
    cv::Point2f p1_center;
    cv::Point2f p4_center;
    bool p1_detected;
    bool p4_detected;
};

struct AnalysisResults {
    int frame_idx;
    PupilData pupil;
    PurkinjeData purkinje;
    bool in_blink;
    bool valid;
};

// ============================================================================
// MAIN PLUGIN CLASS
// ============================================================================

class EyeTrackingPlugin : public IAnalysisPlugin {
private:
    // Threading
    std::atomic<bool> running_;
    std::thread analysis_thread_;
    
    struct FrameData {
        cv::Mat frame;
        int frame_idx;
        FrameMetadata metadata;
    };
    SharedQueue<FrameData> frame_queue_;
    
    // Results
    std::mutex results_mutex_;
    AnalysisResults latest_results_;
    
    // Buffers
    cv::Mat gray_buffer_;
    cv::Mat binary_buffer_;
    cv::Size frame_size_;
    bool buffers_initialized_;
    
    // ROI
    cv::Rect current_roi_;
    bool roi_enabled_;
    
    // Pupil detection
    int pupil_threshold_;
    
    // Blink detection
    BlinkDetector blink_detector_;
    
    // P1 detection
    P1Validator p1_validator_;
    int p1_min_intensity_;
    float p1_max_distance_ratio_;
    cv::Size p1_centroid_roi_size_;
    
    // P4 detection
    P4Validator p4_validator_;
    P1P4RotationalModel p4_model_;
    cv::Size p4_search_roi_size_;
    float p4_max_prediction_error_;
    int p4_min_intensity_;
    
    // P4 Bootstrap mode
    bool p4_bootstrap_mode_;
    cv::Point2f p4_last_known_position_;
    
    // Statistics
    int frame_count_;

    // ========================================================================
    // BUFFER MANAGEMENT
    // ========================================================================
    
    void ensureBuffersAllocated(const cv::Mat& frame) {
        cv::Size current_size(frame.cols, frame.rows);
        
        if (!buffers_initialized_ || current_size != frame_size_) {
            frame_size_ = current_size;
            gray_buffer_ = cv::Mat(frame_size_, CV_8UC1);
            binary_buffer_ = cv::Mat(frame_size_, CV_8UC1);
            buffers_initialized_ = true;
            
            std::cout << "Allocated buffers: " 
                      << frame_size_.width << "x" << frame_size_.height << std::endl;
        }
    }

    // ========================================================================
    // PUPIL DETECTION
    // ========================================================================
    
    PupilData detectPupil(const cv::Mat& frame) {
        PupilData result = {cv::Point2f(-1, -1), -1, false};
        
        cv::Mat roi_frame = roi_enabled_ ? frame(current_roi_) : frame;
        cv::Mat gray_roi = roi_enabled_ ? gray_buffer_(current_roi_) : gray_buffer_;
        cv::Mat binary_roi = roi_enabled_ ? binary_buffer_(current_roi_) : binary_buffer_;
        
        if (roi_frame.channels() > 1) {
            cv::cvtColor(roi_frame, gray_roi, cv::COLOR_BGR2GRAY);
        } else {
            roi_frame.copyTo(gray_roi);
        }
        
        cv::threshold(gray_roi, binary_roi, pupil_threshold_, 255, cv::THRESH_BINARY_INV);
        
        cv::Moments m = cv::moments(binary_roi, true);
        
        if (m.m00 > 0) {
            cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
            float radius = std::sqrt(m.m00 / M_PI);
            
            if (roi_enabled_) {
                center.x += current_roi_.x;
                center.y += current_roi_.y;
            }
            
            result = {center, radius, true};
        }
        
        return result;
    }

    // ========================================================================
    // P1 SUB-PIXEL REFINEMENT
    // ========================================================================
    
    cv::Point2f refineP1SubPixel(const cv::Mat& gray_roi,
                                 const cv::Rect& search_rect,
                                 const cv::Point2f& candidate) {
        cv::Rect centroid_roi(
            candidate.x - p1_centroid_roi_size_.width / 2,
            candidate.y - p1_centroid_roi_size_.height / 2,
            p1_centroid_roi_size_.width,
            p1_centroid_roi_size_.height
        );
        
        centroid_roi &= cv::Rect(0, 0, search_rect.width, search_rect.height);
        
        if (centroid_roi.area() == 0) {
            return candidate;
        }
        
        cv::Mat p1_glint_roi = gray_roi(search_rect)(centroid_roi);
        
        double roi_min_val, roi_max_val;
        cv::minMaxLoc(p1_glint_roi, &roi_min_val, &roi_max_val);
        
        cv::Mat p1_glint_thresh;
        cv::threshold(p1_glint_roi, p1_glint_thresh, roi_max_val * 0.8, 255, cv::THRESH_BINARY);
        
        std::vector<std::vector<cv::Point>> sub_contours;
        cv::findContours(p1_glint_thresh, sub_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        cv::Point2f refined_p1 = candidate;
        
        if (!sub_contours.empty()) {
            auto largest_contour = std::max_element(sub_contours.begin(), sub_contours.end(),
                [](const auto& a, const auto& b) { 
                    return cv::contourArea(a) < cv::contourArea(b); 
                });
            
            cv::Moments M = cv::moments(*largest_contour);
            if (M.m00 > 0) {
                cv::Point2f centroid_in_roi(M.m10 / M.m00, M.m01 / M.m00);
                refined_p1 = centroid_in_roi + cv::Point2f(centroid_roi.x, centroid_roi.y);
            }
        }
        
        return refined_p1;
    }

    // ========================================================================
    // P1 MAIN DETECTION
    // ========================================================================
    
    cv::Point2f detectP1(const cv::Mat& gray_roi, 
                        const cv::Point2f& pupil_center_local,
                        float pupil_radius) {
        
        float search_radius = pupil_radius * 1.2f;
        
        cv::Rect p1_search_rect(
            pupil_center_local.x - search_radius,
            pupil_center_local.y - search_radius,
            search_radius * 2,
            search_radius * 2
        );
        
        p1_search_rect &= cv::Rect(0, 0, gray_roi.cols, gray_roi.rows);
        
        if (p1_search_rect.area() == 0) {
            return cv::Point2f(-1, -1);
        }
        
        cv::Mat p1_region = gray_roi(p1_search_rect);
        
        std::vector<int> thresholds = {220, 200, 180, 160, 140};
        cv::Point2f best_candidate(-1, -1);
        float best_score = 0;
        
        int total_candidates = 0;
        
        for (int thresh : thresholds) {
            if (thresh < p1_min_intensity_) continue;
            
            cv::Mat p1_thresh;
            cv::threshold(p1_region, p1_thresh, thresh, 255, cv::THRESH_BINARY);
            
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(p1_thresh.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            for (const auto& contour : contours) {
                float area = cv::contourArea(contour);
                
                if (area < 1 || area > 400) continue;
                
                total_candidates++;
                
                cv::Moments m = cv::moments(contour);
                if (m.m00 < 1) continue;
                
                cv::Point2f center_rel(m.m10 / m.m00, m.m01 / m.m00);
                
                float perimeter = cv::arcLength(contour, true);
                float circularity = 0;
                if (perimeter > 0) {
                    circularity = 4 * M_PI * area / (perimeter * perimeter);
                    circularity = std::max(0.0f, std::min(1.0f, circularity));
                }
                
                float circularity_threshold = (area > 100) ? 0.3f : 0.5f;
                if (circularity < circularity_threshold) continue;
                
                cv::Rect bbox = cv::boundingRect(contour);
                float compactness = (bbox.width > 0 && bbox.height > 0) ? 
                                    area / (bbox.width * bbox.height) : 0;
                
                bbox &= cv::Rect(0, 0, p1_region.cols, p1_region.rows);
                if (bbox.area() == 0) continue;
                
                cv::Mat spot_region = p1_region(bbox);
                cv::Mat spot_mask = p1_thresh(bbox);
                
                double max_intensity;
                cv::minMaxLoc(spot_region, nullptr, &max_intensity, nullptr, nullptr, spot_mask);
                
                cv::Scalar mean_intensity = cv::mean(spot_region, spot_mask);
                
                double effective_intensity = (max_intensity >= 254) ? 
                                            mean_intensity[0] * 1.2 : max_intensity;
                
                cv::Point2f spot_global = center_rel + cv::Point2f(p1_search_rect.x, p1_search_rect.y);
                float vertical_bias = (spot_global.y > pupil_center_local.y) ? 1.2f : 1.0f;
                
                float size_penalty = (area > 150) ? 0.8f : 1.0f;
                
                float score = effective_intensity * circularity * compactness * 
                             vertical_bias * size_penalty;
                
                if (score > best_score) {
                    best_score = score;
                    best_candidate = center_rel;
                }
            }
            
            if (best_score > 0 && thresh > 180) break;
        }
        
        static int frame_counter = 0;
        if (++frame_counter % 100 == 0) {
            std::cout << "P1 search: " << total_candidates << " candidates, "
                      << "best_score=" << best_score << std::endl;
        }
        
        if (best_candidate.x < 0) {
            return cv::Point2f(-1, -1);
        }
        
        cv::Point2f refined = refineP1SubPixel(gray_roi, p1_search_rect, best_candidate);
        
        cv::Point2f p1_local(
            p1_search_rect.x + refined.x,
            p1_search_rect.y + refined.y
        );
        
        float dist = cv::norm(p1_local - pupil_center_local);
        if (dist > pupil_radius * 1.5f) {
            if (frame_counter % 100 == 0) {
                std::cout << "P1 rejected: dist=" << dist 
                          << " > " << (pupil_radius * 1.5f) << std::endl;
            }
            return cv::Point2f(-1, -1);
        }
        
        return p1_local;
    }

    // ========================================================================
    // P4 BRIGHT SPOT DETECTION
    // ========================================================================
    
    cv::Point2f findP4ByBrightestSpot(const cv::Mat& search_region,
                                       const cv::Mat& search_mask) {
        if (search_region.empty()) {
            return cv::Point2f(-1, -1);
        }
        
        // Find brightest point in masked region
        double min_val, max_val;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(search_region, &min_val, &max_val, &min_loc, &max_loc, search_mask);
        
        static int debug_counter = 0;
        if (++debug_counter % 100 == 0) {
            std::cout << "P4 bright spot: max_intensity=" << max_val 
                      << " at (" << max_loc.x << "," << max_loc.y << ")" << std::endl;
        }
        
        // Validate intensity (P4 glints are very bright)
        if (max_val < p4_min_intensity_) {
            return cv::Point2f(-1, -1);
        }
        
        // Sub-pixel refinement: intensity-weighted centroid in 3x3 neighborhood
        cv::Rect refinement_roi(max_loc.x - 1, max_loc.y - 1, 3, 3);
        refinement_roi &= cv::Rect(0, 0, search_region.cols, search_region.rows);
        
        if (refinement_roi.area() < 9) {
            return cv::Point2f(max_loc);  // Edge case: just use brightest pixel
        }
        
        cv::Mat neighborhood = search_region(refinement_roi);
        
        // Intensity-weighted centroid for sub-pixel accuracy
        cv::Moments m = cv::moments(neighborhood, false);
        
        if (m.m00 > 0) {
            cv::Point2f refined(
                refinement_roi.x + m.m10 / m.m00,
                refinement_roi.y + m.m01 / m.m00
            );
            return refined;
        }
        
        return cv::Point2f(max_loc);
    }

    // ========================================================================
    // P4 DETECTION
    // ========================================================================
    
    cv::Point2f detectP4(const cv::Mat& gray_roi,
                        const cv::Point2f& pupil_center_local,
                        const cv::Point2f& p1_local,
                        float pupil_radius) {
        
        // ===== STEP 1: CREATE SEARCH MASK =====
        cv::Mat search_mask = cv::Mat::zeros(gray_roi.size(), CV_8UC1);
        cv::Point2f predicted_p4_local(-1, -1);
        
        if (p4_model_.isInitialized()) {
            // MODEL-GUIDED SEARCH
            predicted_p4_local = p4_model_.predict(pupil_center_local, p1_local);
            
            if (predicted_p4_local.x > 0) {
                // Create small search ROI around prediction
                cv::Rect predictive_roi(
                    predicted_p4_local.x - p4_search_roi_size_.width / 2,
                    predicted_p4_local.y - p4_search_roi_size_.height / 2,
                    p4_search_roi_size_.width,
                    p4_search_roi_size_.height
                );
                
                predictive_roi &= cv::Rect(0, 0, gray_roi.cols, gray_roi.rows);
                
                if (predictive_roi.area() > 0) {
                    search_mask(predictive_roi) = 255;
                    
                    static int debug_counter = 0;
                    if (++debug_counter % 100 == 0) {
                        std::cout << "P4 model prediction: (" << predicted_p4_local.x 
                                  << "," << predicted_p4_local.y << ")" << std::endl;
                    }
                }
            }
        } else if (p4_bootstrap_mode_) {
            // BOOTSTRAP MODE: Search near last known position
            cv::Point2f last_known_local = p4_last_known_position_;
            if (roi_enabled_) {
                last_known_local.x -= current_roi_.x;
                last_known_local.y -= current_roi_.y;
            }
            
            // Small search area around last known position
            cv::Rect bootstrap_roi(
                last_known_local.x - p4_search_roi_size_.width / 2,
                last_known_local.y - p4_search_roi_size_.height / 2,
                p4_search_roi_size_.width,
                p4_search_roi_size_.height
            );
            
            bootstrap_roi &= cv::Rect(0, 0, gray_roi.cols, gray_roi.rows);
            
            if (bootstrap_roi.area() > 0) {
                search_mask(bootstrap_roi) = 255;
                
                static int debug_counter = 0;
                if (++debug_counter % 50 == 0) {
                    std::cout << "P4 bootstrap search near (" << last_known_local.x 
                             << "," << last_known_local.y << ")" << std::endl;
                }
            }
        } else {
            // WIDE-AREA SEARCH (model not yet initialized)
            float search_radius = pupil_radius * 0.85f;
            cv::circle(search_mask, pupil_center_local, search_radius, 255, -1);
            
            static int debug_counter = 0;
            if (++debug_counter % 100 == 0) {
                std::cout << "P4 wide-area search (model not initialized)" << std::endl;
            }
        }
        
        // Always exclude P1 region to avoid false detections
        float p1_exclusion_radius = pupil_radius * 0.3f;
        cv::circle(search_mask, p1_local, p1_exclusion_radius, 0, -1);
        
        // ===== STEP 2: BRIGHT SPOT DETECTION =====
        cv::Point2f p4_candidate = findP4ByBrightestSpot(gray_roi, search_mask);
        
        if (p4_candidate.x < 0) {
            return cv::Point2f(-1, -1);
        }
        
        // Update last known position for next bootstrap search
        if (p4_bootstrap_mode_) {
            cv::Point2f p4_full = p4_candidate;
            if (roi_enabled_) {
                p4_full.x += current_roi_.x;
                p4_full.y += current_roi_.y;
            }
            p4_last_known_position_ = p4_full;
        }
        
        // ===== STEP 3: VALIDATE PREDICTION ERROR =====
        if (p4_model_.isInitialized() && predicted_p4_local.x > 0) {
            float prediction_error = cv::norm(p4_candidate - predicted_p4_local);
            
            if (prediction_error > p4_max_prediction_error_) {
                static int debug_counter = 0;
                if (++debug_counter % 50 == 0) {
                    std::cout << "P4 rejected: prediction_error=" << prediction_error 
                             << " > " << p4_max_prediction_error_ << std::endl;
                }
                return cv::Point2f(-1, -1);
            }
        }
        
        return p4_candidate;
    }

    // ========================================================================
    // MAIN PURKINJE COORDINATOR
    // ========================================================================
    
    PurkinjeData detectPurkinje(const cv::Mat& frame, const PupilData& pupil) {
        PurkinjeData result = {{-1, -1}, {-1, -1}, false, false};
        
        if (!pupil.detected || pupil.radius <= 0) {
            return result;
        }
        
        cv::Mat roi_frame = roi_enabled_ ? frame(current_roi_) : frame;
        cv::Mat gray_roi = roi_enabled_ ? gray_buffer_(current_roi_) : gray_buffer_;
        
        if (roi_frame.channels() > 1) {
            cv::cvtColor(roi_frame, gray_roi, cv::COLOR_BGR2GRAY);
        } else {
            roi_frame.copyTo(gray_roi);
        }
        
        cv::GaussianBlur(gray_roi, gray_roi, cv::Size(3, 3), 0.5);
        
        cv::Point2f pupil_center_local = pupil.center;
        if (roi_enabled_) {
            pupil_center_local.x -= current_roi_.x;
            pupil_center_local.y -= current_roi_.y;
        }
        
        // ===== P1 DETECTION =====
        cv::Point2f p1_local = detectP1(gray_roi, pupil_center_local, pupil.radius);
        
        // Track P1 loss
        static int p1_loss_counter = 0;
        static int p1_recovery_countdown = 0;
        const int P1_LOSS_THRESHOLD = 5;
        const int P1_RECOVERY_FRAMES = 5;
        
        static int diag_counter = 0;
        bool should_print = (++diag_counter % 50 == 0) || 
                           (p1_local.x < 0) || 
                           blink_detector_.isInBlink() || 
                           blink_detector_.isRecovering();
        
        if (should_print) {
            std::cout << "P1 diagnostics:" << std::endl;
            std::cout << "  detectP1 returned: (" << p1_local.x << "," << p1_local.y << ")" << std::endl;
            std::cout << "  Pupil detected: " << pupil.detected << std::endl;
            std::cout << "  In blink: " << blink_detector_.isInBlink() << std::endl;
            std::cout << "  In blink recovery: " << blink_detector_.isRecovering() << std::endl;
            std::cout << "  P1 recovery countdown: " << p1_recovery_countdown << std::endl;
        }
        
        if (p1_local.x < 0) {
            p1_loss_counter++;
            if (p1_loss_counter == P1_LOSS_THRESHOLD) {
                p1_validator_.reset();
                p1_recovery_countdown = P1_RECOVERY_FRAMES;
                std::cout << "P1 lost for " << P1_LOSS_THRESHOLD 
                         << " frames - resetting validator and starting recovery" << std::endl;
            }
        } else {
            p1_loss_counter = 0;
            if (p1_recovery_countdown > 0) {
                p1_recovery_countdown--;
            }
        }
        
        if (p1_local.x > 0) {
            cv::Point2f p1_full = p1_local;
            if (roi_enabled_) {
                p1_full.x += current_roi_.x;
                p1_full.y += current_roi_.y;
            }
            
            bool is_valid = blink_detector_.isRecovering() || 
                           p1_recovery_countdown > 0 ||
                           p1_validator_.isValid(p1_full);
            
            if (is_valid) {
                result.p1_detected = true;
                result.p1_center = p1_full;
                
                if (!blink_detector_.isInBlink() && !blink_detector_.isRecovering() && p1_recovery_countdown == 0) {
                    p1_validator_.update(p1_full);
                }
            } else {
                static int frame_counter = 0;
                if (++frame_counter % 100 == 0) {
                    std::cout << "P1 rejected by validator (temporal jump too large)" << std::endl;
                }
            }
        }
        
        // ===== P4 DETECTION =====
        if (result.p1_detected) {
            cv::Point2f p4_local = detectP4(gray_roi, pupil_center_local, p1_local, pupil.radius);
            
            if (p4_local.x > 0) {
                cv::Point2f p4_full = p4_local;
                if (roi_enabled_) {
                    p4_full.x += current_roi_.x;
                    p4_full.y += current_roi_.y;
                }
                
                // During bootstrap, be more lenient with validation
                bool is_valid = blink_detector_.isRecovering() || 
                               p4_bootstrap_mode_ ||  // Accept all detections during bootstrap
                               p4_validator_.isValid(p4_full, pupil.center);
                
                static int p4_debug_counter = 0;
                bool should_debug = (++p4_debug_counter % 50 == 0) || !is_valid;
                
                if (should_debug && !is_valid) {
                    std::cout << "P4 REJECTED by validator: pos=(" << p4_full.x << "," << p4_full.y << ")"
                              << " bootstrap=" << p4_bootstrap_mode_ << std::endl;
                }
                
                if (is_valid) {
                    result.p4_detected = true;
                    result.p4_center = p4_full;
                    
                    if (should_debug) {
                        std::cout << "P4 ACCEPTED: pos=(" << p4_full.x << "," << p4_full.y << ")"
                                  << " model_init=" << p4_model_.isInitialized() << std::endl;
                    }
                    
                    if (!blink_detector_.isInBlink() && !blink_detector_.isRecovering()) {
                        p4_validator_.update(p4_full, pupil.center);
                        
                        // ===== MODEL LEARNING =====
                        if (!p4_model_.isInitialized()) {
                            // Auto-collect samples during bootstrap
                            p4_model_.addCalibrationSample(pupil.center, result.p1_center, p4_full);
                            
                            int count = p4_model_.getCalibrationSampleCount();
                            int target = p4_model_.getTargetSamples();
                            
                            if (count % 10 == 0) {
                                std::cout << "Auto-collecting P4 samples: " << count << "/" << target << std::endl;
                            }
                            
                            if (count >= target) {
                                if (p4_model_.calibrateFromSamples()) {
                                    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
                                    std::cout << "✓✓✓ P4 MODEL CALIBRATED! ✓✓✓" << std::endl;
                                    std::cout << "  Magnitude ratio: " << p4_model_.getMagnitudeRatio() << std::endl;
                                    std::cout << "  Angle offset: " << (p4_model_.getAngleOffset() * 180.0 / M_PI) << "°" << std::endl;
                                    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
                                    
                                    p4_bootstrap_mode_ = false;  // Exit bootstrap mode
                                }
                            }
                        } else if (!p4_model_.isFrozen()) {
                            // Incremental learning
                            p4_model_.updateModel(pupil.center, result.p1_center, p4_full);
                        }
                    }
                }
            }
        }
        
        return result;
    }

    // ========================================================================
    // ANALYSIS THREAD
    // ========================================================================
    
    void analysisThreadFunc() {
        std::cout << "Eye tracking analysis thread started" << std::endl;
        
        while (running_) {
            try {
                FrameData frame_data = frame_queue_.front();
                frame_queue_.pop_front();

                if (frame_data.frame_idx < 0) {
                    break;
                }
                
                if (frame_data.frame.empty()) {
                    continue;
                }
                
                ensureBuffersAllocated(frame_data.frame);
                
                auto start = std::chrono::high_resolution_clock::now();
                
                PupilData pupil = detectPupil(frame_data.frame);
                
                bool was_in_blink = blink_detector_.isInBlink();
                
                blink_detector_.update(pupil.detected, pupil.radius);
                
                if (blink_detector_.shouldResetValidators()) {
                    p1_validator_.reset();
                    p4_validator_.reset();                
                }
                
                PurkinjeData purkinje = detectPurkinje(frame_data.frame, pupil);
                
                {
                    std::lock_guard<std::mutex> lock(results_mutex_);
                    latest_results_.frame_idx = frame_data.frame_idx;
                    latest_results_.pupil = pupil;
                    latest_results_.purkinje = purkinje;
                    latest_results_.in_blink = blink_detector_.isInBlink();
                    latest_results_.valid = true;
                }
                
                blink_detector_.decrementRecovery();
                
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                
                frame_count_++;
                
                if (frame_count_ % 100 == 0) {
                    std::cout << "Analysis time: " << duration.count() << "µs" << std::endl;
                    if (pupil.detected) {
                        std::cout << "Pupil: (" << pupil.center.x << "," << pupil.center.y 
                                  << ") r=" << pupil.radius << std::endl;
                    }
                    if (blink_detector_.isInBlink()) {
                        std::cout << "Blink detected" << std::endl;
                    }
                }
            } catch (...) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    // ========================================================================
    // TCL COMMANDS
    // ========================================================================
    
    static int setPupilThresholdCmd(ClientData clientData, Tcl_Interp *interp,
                                    int objc, Tcl_Obj *const objv[]) {
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        
        if (objc > 2) {
            Tcl_WrongNumArgs(interp, 1, objv, "?threshold?");
            return TCL_ERROR;
        }
        
        int last_threshold = plugin->pupil_threshold_;
        
        if (objc > 1) {
            int threshold;
            if (Tcl_GetIntFromObj(interp, objv[1], &threshold) != TCL_OK) {
                return TCL_ERROR;
            }
            plugin->pupil_threshold_ = threshold;
        }
        
        Tcl_SetObjResult(interp, Tcl_NewIntObj(last_threshold));
        return TCL_OK;
    }
    
    static int setROICmd(ClientData clientData, Tcl_Interp *interp,
                         int objc, Tcl_Obj *const objv[]) {
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        
        if (objc == 1) {
            const cv::Rect& roi = plugin->current_roi_;
            Tcl_Obj* resultList = Tcl_NewListObj(0, nullptr);
            Tcl_ListObjAppendElement(interp, resultList, Tcl_NewIntObj(roi.x));
            Tcl_ListObjAppendElement(interp, resultList, Tcl_NewIntObj(roi.y));
            Tcl_ListObjAppendElement(interp, resultList, Tcl_NewIntObj(roi.width));
            Tcl_ListObjAppendElement(interp, resultList, Tcl_NewIntObj(roi.height));
            Tcl_SetObjResult(interp, resultList);
            return TCL_OK;
        }
        
        if (objc != 5) {
            Tcl_WrongNumArgs(interp, 1, objv, "?x y width height?");
            return TCL_ERROR;
        }
        
        int x, y, width, height;
        if (Tcl_GetIntFromObj(interp, objv[1], &x) != TCL_OK ||
            Tcl_GetIntFromObj(interp, objv[2], &y) != TCL_OK ||
            Tcl_GetIntFromObj(interp, objv[3], &width) != TCL_OK ||
            Tcl_GetIntFromObj(interp, objv[4], &height) != TCL_OK) {
            return TCL_ERROR;
        }
        
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
    
    static int addP4CalibrationSampleCmd(ClientData clientData, Tcl_Interp *interp,
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
        
        std::lock_guard<std::mutex> lock(plugin->results_mutex_);
        
        if (!plugin->latest_results_.pupil.detected || 
            !plugin->latest_results_.purkinje.p1_detected) {
            Tcl_SetObjResult(interp, Tcl_NewStringObj(
                "Cannot add calibration sample: need valid pupil and P1", -1));
            return TCL_ERROR;
        }
        
        // Add the clicked position as the first calibration sample immediately
        cv::Point2f p4_pos(x, y);
        plugin->p4_model_.addCalibrationSample(
            plugin->latest_results_.pupil.center,
            plugin->latest_results_.purkinje.p1_center,
            p4_pos
        );
        
        // Store the position and enable bootstrap mode
        plugin->p4_last_known_position_ = p4_pos;
        plugin->p4_bootstrap_mode_ = true;
        
        int count = plugin->p4_model_.getCalibrationSampleCount();
        std::cout << "P4 bootstrap initiated at (" << x << "," << y << ")" << std::endl;
        std::cout << "Added first calibration sample (1/" << plugin->p4_model_.getTargetSamples() << ")" << std::endl;
        std::cout << "Will auto-collect remaining samples..." << std::endl;
        
        Tcl_SetObjResult(interp, Tcl_NewStringObj("P4 bootstrap started", -1));
        return TCL_OK;
    }

    static int freezeP4ModelCmd(ClientData clientData, Tcl_Interp *interp,
                                int objc, Tcl_Obj *const objv[]) {
        if (objc != 2) {
            Tcl_WrongNumArgs(interp, 1, objv, "0|1");
            return TCL_ERROR;
        }
        
        int freeze_flag;
        if (Tcl_GetIntFromObj(interp, objv[1], &freeze_flag) != TCL_OK) {
            return TCL_ERROR;
        }
        
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        if (freeze_flag) {
            plugin->p4_model_.freeze();
        } else {
            plugin->p4_model_.unfreeze();
        }
        
        Tcl_SetObjResult(interp, Tcl_NewStringObj(
            plugin->p4_model_.isFrozen() ? "P4 model frozen" : "P4 model unfrozen", -1));
        return TCL_OK;
    }
    
    static int resetP4ModelCmd(ClientData clientData, Tcl_Interp *interp,
                               int objc, Tcl_Obj *const objv[]) {
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        plugin->p4_model_.reset();
        plugin->p4_validator_.reset();
        plugin->p4_bootstrap_mode_ = false;
        Tcl_SetObjResult(interp, Tcl_NewStringObj("P4 model and validator reset", -1));
        return TCL_OK;
    }
    
    static int getP4ModelStatusCmd(ClientData clientData, Tcl_Interp *interp,
                                   int objc, Tcl_Obj *const objv[]) {
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        
        Tcl_Obj* statusDict = Tcl_NewDictObj();
        
        Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("initialized", -1),
                      Tcl_NewBooleanObj(plugin->p4_model_.isInitialized()));
        Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("frozen", -1),
                      Tcl_NewBooleanObj(plugin->p4_model_.isFrozen()));
        Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("samples", -1),
                      Tcl_NewIntObj(plugin->p4_model_.getCalibrationSampleCount()));
        Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("bootstrap_mode", -1),
                      Tcl_NewBooleanObj(plugin->p4_bootstrap_mode_));
        
        if (plugin->p4_model_.isInitialized()) {
            Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("magnitude_ratio", -1),
                          Tcl_NewDoubleObj(plugin->p4_model_.getMagnitudeRatio()));
            Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("angle_offset_deg", -1),
                          Tcl_NewDoubleObj(plugin->p4_model_.getAngleOffset() * 180.0 / M_PI));
        }
        
        Tcl_SetObjResult(interp, statusDict);
        return TCL_OK;
    }
    
    static int setP4MinIntensityCmd(ClientData clientData, Tcl_Interp *interp,
                                     int objc, Tcl_Obj *const objv[]) {
        if (objc != 2) {
            Tcl_WrongNumArgs(interp, 1, objv, "intensity");
            return TCL_ERROR;
        }
        
        int intensity;
        if (Tcl_GetIntFromObj(interp, objv[1], &intensity) != TCL_OK) {
            return TCL_ERROR;
        }
        
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        plugin->p4_min_intensity_ = intensity;
        
        Tcl_SetObjResult(interp, Tcl_NewStringObj("P4 min intensity updated", -1));
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
        
        Tcl_Obj* resultDict = Tcl_NewDictObj();
        
        Tcl_DictObjPut(interp, resultDict, Tcl_NewStringObj("in_blink", -1),
                      Tcl_NewBooleanObj(plugin->latest_results_.in_blink));
        
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
        : running_(false),
          roi_enabled_(false),
          pupil_threshold_(45),
          p1_validator_(15.0f),
          p1_min_intensity_(140),
          p1_max_distance_ratio_(1.5f),
          p1_centroid_roi_size_(cv::Size(7, 7)),
          p4_validator_(20.0f),  // Increased from 10.0f
          p4_model_(50),
          p4_search_roi_size_(cv::Size(30, 30)),
          p4_max_prediction_error_(13.0f),
          p4_min_intensity_(140),
          p4_bootstrap_mode_(false),
          frame_count_(0),
          buffers_initialized_(false) {
        latest_results_.valid = false;
    }
    
    ~EyeTrackingPlugin() {
        shutdown();
    }
    
    bool initialize(Tcl_Interp* interp) override {
        running_ = true;
        analysis_thread_ = std::thread(&EyeTrackingPlugin::analysisThreadFunc, this);
        return true;
    }
    
    void shutdown() override {
        running_ = false;

        FrameData sentinel;
        sentinel.frame_idx = -1;
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
        Tcl_CreateObjCommand(interp, "::eyetracking::addP4CalibrationSample", 
                            addP4CalibrationSampleCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::freezeP4Model", 
                            freezeP4ModelCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::resetP4Model", 
                            resetP4ModelCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::getP4ModelStatus", 
                            getP4ModelStatusCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setP4MinIntensity", 
                            setP4MinIntensityCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::getResults",
                            getResultsCmd, this, NULL);
    }
    
    const char* getName() override { return "eye_tracking"; }
    const char* getVersion() override { return "1.0"; }
    const char* getDescription() override { 
        return "Pupil and Purkinje reflection tracking with bright spot detection"; 
    }
    
    bool drawOverlay(cv::Mat& frame, int frame_idx) override {
        std::lock_guard<std::mutex> lock(results_mutex_);
        
        if (!latest_results_.valid) return false;
        
        if (roi_enabled_) {
            cv::rectangle(frame, current_roi_, cv::Scalar(255, 255, 0), 2);
            cv::putText(frame, "ROI", 
                       cv::Point(current_roi_.x + 5, current_roi_.y + 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                       cv::Scalar(255, 255, 0), 1);
        }
        
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
        
        if (latest_results_.purkinje.p1_detected) {
            cv::circle(frame, latest_results_.purkinje.p1_center, 
                       5, cv::Scalar(255, 0, 0), 2);
            cv::drawMarker(frame, latest_results_.purkinje.p1_center,
                          cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 8, 1);
            cv::putText(frame, "P1", 
                       cv::Point(latest_results_.purkinje.p1_center.x + 8, 
                                latest_results_.purkinje.p1_center.y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, 
                       cv::Scalar(255, 0, 0), 1);
        }
        
        if (latest_results_.purkinje.p4_detected) {
            cv::circle(frame, latest_results_.purkinje.p4_center, 
                       5, cv::Scalar(0, 0, 255), 2);
            cv::drawMarker(frame, latest_results_.purkinje.p4_center,
                          cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 8, 1);
            cv::putText(frame, "P4", 
                       cv::Point(latest_results_.purkinje.p4_center.x + 8, 
                                latest_results_.purkinje.p4_center.y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, 
                       cv::Scalar(0, 0, 255), 1);
        }
        
        // Draw prediction ROI when in bootstrap or model mode
        if (p4_bootstrap_mode_ && latest_results_.pupil.detected) {
            cv::Rect bootstrap_roi(
                p4_last_known_position_.x - p4_search_roi_size_.width / 2,
                p4_last_known_position_.y - p4_search_roi_size_.height / 2,
                p4_search_roi_size_.width,
                p4_search_roi_size_.height
            );
            cv::rectangle(frame, bootstrap_roi, cv::Scalar(255, 165, 0), 1);
            cv::putText(frame, "Bootstrap", 
                       cv::Point(bootstrap_roi.x + 5, bootstrap_roi.y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, 
                       cv::Scalar(255, 165, 0), 1);
        } else if (p4_model_.isInitialized() && latest_results_.pupil.detected && 
                   latest_results_.purkinje.p1_detected) {
            cv::Point2f predicted_p4 = p4_model_.predict(
                latest_results_.pupil.center, 
                latest_results_.purkinje.p1_center
            );
            
            if (predicted_p4.x > 0) {
                cv::Rect pred_roi(
                    predicted_p4.x - p4_search_roi_size_.width / 2,
                    predicted_p4.y - p4_search_roi_size_.height / 2,
                    p4_search_roi_size_.width,
                    p4_search_roi_size_.height
                );
                cv::rectangle(frame, pred_roi, cv::Scalar(255, 255, 0), 1);
                cv::circle(frame, predicted_p4, 3, cv::Scalar(255, 255, 0), 1);
            }
        }
        
        return true;
    }
};

// ============================================================================
// TCL PACKAGE ENTRY POINT
// ============================================================================

extern "C" {
    int Eyetracking_Init(Tcl_Interp *interp) {
        EyeTrackingPlugin* plugin = new EyeTrackingPlugin();
        
        if (!plugin->initialize(interp)) {
            delete plugin;
            return TCL_ERROR;
        }
        
        g_pluginRegistry.registerPlugin(plugin);
        plugin->registerTclCommands(interp);
        Tcl_PkgProvide(interp, "Eyetracking", "1.0");
        
        return TCL_OK;
    }
}
