#include <tcl.h>
#include "opencv2/opencv.hpp"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <unordered_map>

#include <jansson.h>

#include "SharedQueue.hpp"
#include "IAnalysisPlugin.h"
#include "AnalysisPluginRegistry.h"
#include "SourceManager.h"
#include "DataserverForwarder.h"
#include "VstreamEvent.h"

extern AnalysisPluginRegistry g_pluginRegistry;
extern SourceManager* g_sourceManager;

// ============================================================================
// DEBUG LEVELS
// ============================================================================
enum DebugLevel {
    DEBUG_SILENT = 0,      // No output
    DEBUG_CRITICAL = 1,    // Only critical events (calibration, losses)
    DEBUG_NORMAL = 2,      // + Warnings and important state changes
    DEBUG_VERBOSE = 3,     // + Detailed detection info
    DEBUG_PROFILE = 4      // + Timing information
};

// Profiling flags (can be combined with debug levels)
enum ProfileFlags {
    PROFILE_NONE = 0,
    PROFILE_TIMING = 1 << 0,     // Frame timing
    PROFILE_DETECTION = 1 << 1    // Detection details
};

// ============================================================================
// P1 VALIDATOR
// ============================================================================

class P1Validator {
private:
    bool initialized_;
    cv::Point2f last_position_;
    float max_jump_pixels_;
    float last_jump_distance_;
    
    // Candidate tracking for relocation
    cv::Point2f candidate_position_;
    int candidate_count_;
    int relocation_threshold_;

public:
    P1Validator(float max_jump = 15.0f, int relocation_threshold = 3) 
        : initialized_(false), max_jump_pixels_(max_jump), last_jump_distance_(0.0f),
          candidate_position_(-1, -1), candidate_count_(0), relocation_threshold_(relocation_threshold) {}

    bool isValid(const cv::Point2f& new_pos) {
        if (!initialized_) {
            last_jump_distance_ = 0.0f;
            return true;
        }
        
        last_jump_distance_ = cv::norm(new_pos - last_position_);
        
        // Hard threshold - definitely good
        if (last_jump_distance_ < max_jump_pixels_) {
            candidate_count_ = 0;
            candidate_position_ = cv::Point2f(-1, -1);
            return true;
        }
        
        // Reject if TOO far (more than 2x threshold - probably not real P1)
        if (last_jump_distance_ > max_jump_pixels_ * 2.0f) {
            candidate_count_ = 0;
            candidate_position_ = cv::Point2f(-1, -1);
            return false;
        }
        
        // In the soft zone (1x to 2x threshold) - need consistency
        if (candidate_count_ == 0) {
            // First time seeing this new location
            candidate_position_ = new_pos;
            candidate_count_ = 1;
            return false;
        }
        
        // Check if new detection is near our candidate location
        float candidate_dist = cv::norm(new_pos - candidate_position_);
        if (candidate_dist < max_jump_pixels_ * 0.3f) {
            // Consistent with candidate location
            candidate_count_++;
            
            // Update candidate to running average for sub-pixel precision
            candidate_position_ = (candidate_position_ * (candidate_count_ - 1) + new_pos) / 
                                  static_cast<float>(candidate_count_);
            
            if (candidate_count_ >= relocation_threshold_) {
                // Accept the relocation!
                return true;
            }
            return false;  // Not enough evidence yet
        } else {
            // New position doesn't match candidate - start tracking new candidate
            candidate_position_ = new_pos;
            candidate_count_ = 1;
            return false;
        }
    }

    void update(const cv::Point2f& pos) {
        last_position_ = pos;
        initialized_ = true;
        candidate_count_ = 0;
        candidate_position_ = cv::Point2f(-1, -1);
    }

    void reset() {
        initialized_ = false;
        last_jump_distance_ = 0.0f;
        candidate_count_ = 0;
        candidate_position_ = cv::Point2f(-1, -1);
    }

    void setMaxJump(float max_jump) {
        max_jump_pixels_ = max_jump;
    }
    
    float getMaxJump() const {
        return max_jump_pixels_;
    }
    
    float getJumpDistance() const {
        return last_jump_distance_;
    }
    
    cv::Point2f getLastPosition() const {
        return last_position_;
    }
    
    bool isInitialized() const {
        return initialized_;
    }
    
    int getCandidateCount() const {
        return candidate_count_;
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
    
    bool isValid(const cv::Point2f& new_p4, const cv::Point2f& pupil_center, 
                 float pupil_radius = -1.0f) {  // ADD pupil_radius parameter
        
        // CRITICAL: P4 must be inside pupil
        if (pupil_radius > 0) {
            float dist_from_center = cv::norm(new_p4 - pupil_center);
            if (dist_from_center > pupil_radius * 0.95f) {  // Allow 5% margin for edge cases
                return false;
            }
        }
        
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
    
    float getMaxJump() const {
        return max_jump_pixels_;
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
  
  static float wrapAngle(float angle) {
    angle = std::fmod(angle + M_PI, 2.0f * M_PI);
    if (angle < 0) angle += 2.0f * M_PI;
    return angle - M_PI;
  }
  
public:
  P1P4RotationalModel() 
    : initialized_(false),
      frozen_(false),
      magnitude_ratio_(0.0f),
      angle_offset_(0.0f),
      learning_rate_(0.1f) {}
  
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
    float mag_p4 = cv::norm(v_pc_p4);

    const float min_reliable_distance = 10.0f;
    if (mag_p1 < min_reliable_distance || mag_p4 < min_reliable_distance) {
      return;
    }
    
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
    if (calibration_pupil_centers_.size() < 1) {
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
  
  void setParameters(float mag_ratio, float angle_offset_rad) {
    magnitude_ratio_ = mag_ratio;
    angle_offset_ = angle_offset_rad;
    initialized_ = true;
    frozen_ = false;
    
    // Clear any calibration samples
    calibration_pupil_centers_.clear();
    calibration_p1_centers_.clear();
    calibration_p4_centers_.clear();
  }
  
  int getCalibrationSampleCount() const { 
    return calibration_pupil_centers_.size(); 
  }
  
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
    int recovery_frames_;
    
    float baseline_radius_;
    bool baseline_initialized_;
    static constexpr float BLINK_ENTER_RATIO = 0.6f;
    static constexpr float BLINK_EXIT_RATIO = 0.85f;
    int baseline_update_frames_;
    int frames_since_update_;

    // Self-heal: how long the "blink" gate may persist with a pupil present
    // before we assume the situation changed (subject left & returned) and
    // re-learn the baseline from the live pupil. Far longer than any real blink.
    int frames_in_blink_;
    int long_blink_reset_frames_;

public:
    BlinkDetector(int recovery_frames = 15, int baseline_update_frames = 10,
                  int long_blink_reset_frames = 375)
        : in_blink_(false), recovery_countdown_(0), recovery_frames_(recovery_frames),
          baseline_radius_(0), baseline_initialized_(false),
          frames_since_update_(0), baseline_update_frames_(baseline_update_frames),
          frames_in_blink_(0), long_blink_reset_frames_(long_blink_reset_frames) {}

    void update(bool pupil_detected, float pupil_radius) {
        // Track how long we've been "in blink". A real blink is short; a very
        // long one with a pupil actually present means the baseline is stale
        // (e.g. subject went out of frame and came back, or signal was lost).
        // Re-learn the baseline from the live pupil so the gate releases and
        // P1/P4 detection resumes — without needing a manual reset/restart.
        if (in_blink_) {
            frames_in_blink_++;
            if (pupil_detected && pupil_radius > 0 &&
                frames_in_blink_ > long_blink_reset_frames_) {
                baseline_radius_ = pupil_radius;
                baseline_initialized_ = true;
                in_blink_ = false;
                recovery_countdown_ = recovery_frames_;
                frames_in_blink_ = 0;
                frames_since_update_ = 0;
                return;  // resume normal tracking next frame
            }
        } else {
            frames_in_blink_ = 0;
        }

        if (pupil_detected && !in_blink_ && recovery_countdown_ == 0) {
            if (!baseline_initialized_) {
                baseline_radius_ = pupil_radius;
                baseline_initialized_ = true;
            } else {
                frames_since_update_++;
                if (frames_since_update_ >= baseline_update_frames_) {
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
                recovery_countdown_ = recovery_frames_;
            }
        } else {
            if (in_blink_) {
                in_blink_ = false;
                recovery_countdown_ = recovery_frames_;
            }
        }
    }
    
    void decrementRecovery() {
        if (recovery_countdown_ > 0) {
            recovery_countdown_--;
        }
    }

  void reset() {
    in_blink_ = false;
    recovery_countdown_ = 0;
    // Also forget the learned baseline so it re-learns from the live pupil
    // (matches a program restart). Without this, a baseline corrupted during an
    // extended signal loss (subject out of frame) keeps the blink gate stuck on
    // after the subject returns, so P1/P4 are never re-acquired until restart.
    baseline_radius_ = 0;
    baseline_initialized_ = false;
    frames_since_update_ = 0;
    frames_in_blink_ = 0;
  }

    bool isInBlink() const { return in_blink_; }
    bool isRecovering() const { return recovery_countdown_ > 0; }
    
    bool shouldResetValidators() const {
        return in_blink_ && recovery_countdown_ == recovery_frames_;
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
    // Ellipse fit of the pupil component's filled outer contour, for
    // pupillometry. Semi-axes in px; angle is cv::fitEllipse's RotatedRect
    // angle in degrees. Valid only when has_ellipse. The tracked center stays
    // the component centroid (P4-model geometry input unchanged); the ellipse
    // contributes the SIZE measures: radius above is the equivalent radius of
    // the FILLED contour (glint holes no longer deflate the area), and
    // sqrt(a*b) is robust to the foreshortening of an off-axis camera.
    bool has_ellipse = false;
    float ellipse_a = -1.0f;      // semi-major axis
    float ellipse_b = -1.0f;      // semi-minor axis
    float ellipse_angle = -1.0f;  // degrees
};

struct PurkinjeData {
    cv::Point2f p1_center;
    cv::Point2f p4_center;
    bool p1_detected;
    bool p4_detected;
};

// Why P4 was not reported for a frame (for reprocess diagnostics)
enum P4RejectReason {
    P4_OK = 0,            // P4 detected and accepted
    P4_NOT_FULL_MODE,     // detection mode < full
    P4_NO_P1,             // no P1 this frame (P4 search needs P1)
    P4_BLINK,             // blink / blink-recovery gate suppressed detection
    P4_NO_CANDIDATE,      // no pixel above threshold in search region
    P4_PRED_ERROR,        // candidate too far from model prediction
    P4_OUT_OF_BOUNDS,     // candidate outside ROI bounds
    P4_INSIDE_PUPIL,      // validator: candidate outside 0.95*pupil_radius
    P4_JUMP               // validator: candidate moved too far from last P4
};

// Why P1 was not reported for a frame (for reprocess diagnostics)
enum P1RejectReason {
    P1_OK = 0,            // P1 detected and accepted
    P1_BLINK,             // blink / blink-recovery gate suppressed detection
    P1_NOT_DETECTED,      // detectP1 returned no candidate (search/threshold/area)
    P1_REJECTED_FAR,      // validator: jump > 2x max_jump (hard reject vs last P1)
    P1_REJECTED_SOFT,     // validator: in 1x-2x soft zone, still accumulating evidence
    P1_DESPERATION_OK     // P1 REPORTED, but accepted blind in desperation mode —
                          // low-trust; offline analysis can discount these frames
};

struct AnalysisResults {
    int frame_idx;
    PupilData pupil;
    PurkinjeData purkinje;
    bool in_blink;
    bool tracking_lost = false;  // no plausible pupil longer than any real blink
    bool valid;
    // Monotonic frame id of this analysis (metadata.frameID - session anchor);
    // keys the reference-overlay lookup in drawOverlay.
    long long abs_frame_id = -1;
    // reprocess diagnostics
    int p4_reject_reason = P4_OK;
    cv::Point2f p4_predicted{-1, -1};
    bool p4_candidate_found = false;
    int p1_reject_reason = P1_OK;
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
  
  // Detection Modes
  enum DetectionMode {
    MODE_PUPIL_ONLY = 0,
    MODE_PUPIL_P1 = 1,
    MODE_FULL = 2
  };  
  DetectionMode detection_mode_;
  
  // Results
  std::mutex results_mutex_;
  AnalysisResults latest_results_;
  
  // Buffers
  cv::Mat gray_buffer_;
  cv::Mat blur_buffer_;   // Gaussian-blurred copy of gray for P1/P4 search
  cv::Mat binary_buffer_;
  cv::Size frame_size_;
  bool buffers_initialized_;
  
  // ROI
  cv::Rect current_roi_;
  bool roi_enabled_;
  std::mutex roi_mutex_;
  
  // Pupil detection
  int pupil_threshold_;

  // Recenter the ROI on P4-sample accept (setup flow): each accepted sample is
  // a moment when the pupil is known-good, so the crop follows the eye without
  // any extra operator action. eyetracking::autoCenterROI 0|1 to disable.
  bool auto_center_roi_on_p4_ = true;

  // Dilation reference for CONSERVATIVE model learning: a leaky MAXIMUM of the
  // pupil radius (half-life ~10 min), seeded by P4 calibration accepts. The
  // blink detector's baseline cannot serve here — it deliberately TRACKS the
  // current pupil (that's what blink detection needs), so under a sustained
  // bright-screen constriction it follows the radius down within seconds and
  // "well-dilated" quietly becomes "near the constricted size" — re-admitting
  // exactly the noisy-center frames the learning gate exists to exclude. A
  // decaying max can only soften on a ~10-minute clock, never from
  // constriction itself; if it is ever too strict, learning simply pauses and
  // the model holds (proven safe frozen), which is the right failure mode.
  // Survives resetTrackingState (session physiology, like the model); cleared
  // on reset()/fileOpen. Decay factor is computed from the frame rate.
  float learn_ref_radius_ = 0.0f;

  // Pupil plausibility gate. The old detector took moments of EVERY dark pixel
  // in the ROI, so lashes/shadows/hair biased the centroid, and when the
  // subject looked away or left, whatever dark junk remained still read as a
  // confident "pupil" — corrupting the blink baseline (the stuck-blink-after-
  // return failure) and feeding garbage centers to the P4 model. Instead we
  // pick the best connected component and require it to look pupil-like; if
  // nothing qualifies the frame reports NO pupil, which the blink gate then
  // handles correctly (and the baseline is preserved for a clean re-lock).
  float pupil_min_area_ = 200.0f;     // px^2 (equiv radius ~8px)
  float pupil_max_area_ = 150000.0f;  // px^2 (equiv radius ~218px)
  float pupil_min_extent_ = 0.40f;    // area/bbox; filled circle = pi/4 = 0.785
  float pupil_max_aspect_ = 3.0f;     // bbox elongation limit (lash streaks)
  cv::Mat cc_labels_;                 // reused label buffer for the CC pass
  cv::Mat p4_rect_mask_;              // reused rect-local P4 search mask

  // Blink detection
  BlinkDetector blink_detector_;
  
  // P1 detection
  P1Validator p1_validator_;
  int p1_min_intensity_;
  int p1_min_area_;
  int p1_max_area_;
  cv::Size p1_centroid_roi_size_;
  float last_p1_area_ = -1.0f;
  float p1_pupil_radius_max_;
  float last_p1_intensity_ = -1;
  
  // P4 detection
  P4Validator p4_validator_;
  P1P4RotationalModel p4_model_;
  cv::Size p4_search_roi_size_;
  float p4_max_prediction_error_;
  int p4_min_intensity_;

  float proximity_weight_ = 1.5f;  // bonus for being close to center

  // P4 is only ever visible INSIDE the pupil. The predictive search ROI is a
  // fixed size, so when the pupil constricts the ROI spills onto the bright
  // iris/sclera and the search can pick an out-of-pupil bright spot over the
  // real (dim) P4. Constrain the search to this fraction of the pupil radius.
  // (Validated 2026-07-10 on coherence-sweep: without this disk an iris spot
  // outscores the true P4 on ~1/3 of frames.)
  float p4_pupil_search_margin_ = 0.95f;

  // Floor for the disk above: P4's offset from the pupil center is fixed by
  // eye geometry, but the disk scales with the measured pupil radius, so a
  // constricted (or under-measured) pupil could exclude the very location the
  // model predicts. Never shrink the searchable disk below the predicted P4
  // orbit plus this slack (covers sub-pixel refinement + pupil-center noise).
  // Kept small on purpose: it makes the PREDICTED spot searchable without
  // re-admitting the far-from-prediction iris spots the disk exists to block.
  static constexpr float P4_DISK_PRED_SLACK = 6.0f;

  cv::Point2f p4_last_known_position_;
  bool p4_pending_sample_active_;
  cv::Point2f p4_pending_sample_position_;

  // P4 loss/recovery tracking (must be member vars so reset can clear them)
  int p4_loss_counter_ = 0;
  int p4_recovery_countdown_ = 0;
  bool p4_was_lost_ = false;
  int frames_without_p1_for_p4_ = 0;
  bool p1_was_absent_extended_ = false;

  // P4 auto-reacquire: counts consecutive accepted frames where P4 sits far
  // from the model prediction (near the search-box edge). When it persists,
  // we reset the P4 validator + arm recovery — the same re-lock a blink does —
  // so a stuck/cornered P4 recenters without waiting for a blink. Does not
  // touch learned geometry, so it is safe even when the model is frozen.
  int p4_offset_streak_ = 0;

  // P1 loss/recovery tracking — member vars (NOT function-static) so reset()
  // and resetTrackingState clear them. Previously function-static in
  // detectPurkinje, which is why a stuck P1-loss state survived "reset" and only
  // a program restart cleared it.
  int p1_loss_counter_ = 0;
  int p1_recovery_countdown_ = 0;
  bool p1_was_lost_ = false;

  // P1 desperation state — LATCHED (hysteresis) so it doesn't oscillate around
  // the entry threshold and reprint diagnostics every frame. Entered when the
  // loss counter blows past the threshold; only cleared once a spatially
  // consistent lock is re-established across several frames.
  bool p1_desperation_ = false;
  int p1_desperation_streak_ = 0;
  cv::Point2f p1_desperation_last_pos_{-1, -1};

  // Desperation give-up: if a desperation episode runs this long without ever
  // achieving a consistent lock, P1 is genuinely gone (occluded illuminator,
  // extreme gaze) — stop accepting blind candidates so junk isn't streamed as
  // P1, wait out a cooldown, then allow a fresh attempt. Accepted-in-desperation
  // frames are flagged P1_DESPERATION_OK in the data either way.
  int p1_desperation_frames_ = 0;    // frames spent in the current episode
  int p1_desperation_cooldown_ = 0;  // frames until desperation may re-latch

  // LOST state: no plausible pupil for longer than any real blink (subject
  // looked far away, closed eyes for an extended period, or left). Distinct
  // from blink both in the data (a 30 s "blink" is misleading) and in behavior
  // (on return we force the same clean re-lock a blink does). The plausibility
  // gate in detectPupil is what makes "no pupil" trustworthy here.
  bool tracking_lost_ = false;
  int frames_without_pupil_ = 0;

  // Synchronous (deterministic) analysis for offline reprocess.
  // When true, analyzeFrame runs the pipeline inline on the caller's thread
  // instead of pushing to frame_queue_. Live path leaves this false.
  std::atomic<bool> synchronous_{false};

  // Per-frame P4 diagnostic scratch (set during detectPurkinje/detectP4,
  // copied into latest_results_ for the reprocess decision log)
  int p4_reject_reason_ = P4_OK;
  cv::Point2f p4_predicted_{-1, -1};
  bool p4_candidate_found_ = false;
  int p1_reject_reason_ = P1_OK;

  // Statistics
  int frame_count_;
  
  int64_t first_frameID_ = -1;
  int64_t first_timestamp_ = -1;

  // Monotonic frame identity for outgoing events: metadata.frameID relative
  // to the session anchor (same domain as the em/frame_id stream and the
  // db's relative_frame_id). The frame_idx the host passes in is the capture
  // ring-buffer SLOT (wraps at the buffer size, 500 by default) — fine as a
  // queue key, useless in an event log.
  int64_t event_frame_id_ = 0;
  
  // Debug control
  int debug_level_;
  int profile_flags_;
  std::atomic<bool> debug_next_frame_;
  std::atomic<bool> show_insets_;

  // Focus-assist mode: a large magnified view of the P4 region to help the
  // experimenter set camera focus per subject/session. Anchored on the P4-model
  // prediction (with fallbacks) so it works even before P4 is detectable.
  // Tri-state so a single key can cycle annotations off for "pure" focusing:
  //   0 = off, 1 = annotated (reticle + sharpness + sparkline), 2 = clean image.
  // Drawn only on the display thread (drawOverlay), so focus_history_/focus_best_
  // need no extra locking.
  enum { FOCUS_OFF = 0, FOCUS_ANNOTATED = 1, FOCUS_CLEAN = 2 };
  std::atomic<int> focus_mode_;
  std::vector<float> focus_history_;   // sharpness score trace for sparkline
  float focus_best_ = 0.0f;            // running max since mode was enabled

  // Frame source for getting frame rate
  IFrameSource* frame_source_;

  // Reference ("ghost") overlay: detections loaded from a recorded session
  // .db so playback shows what WAS stored — live recording or any reprocess —
  // alongside what the CURRENT code detects on the same frames. The overlay
  // draws the reference in orange under the live markers, and running
  // agreement counters quantify what a code change won or lost.
  struct RefFrame {
    float px, py, pr;      // pupil
    float p1x, p1y;
    float p4x, p4y;
    uint8_t has;           // bitmask of the HAS_* flags below
    bool blink;
  };
  enum { REF_HAS_PUPIL = 1, REF_HAS_P1 = 2, REF_HAS_P4 = 4 };
  std::unordered_map<long long, RefFrame> reference_;
  std::mutex reference_mutex_;
  std::atomic<bool> show_reference_{true};
  // Fast-path gate so the per-frame hot path costs ONE relaxed atomic read
  // when no reference is loaded (the live-rig case) — no mutex, no lookup.
  std::atomic<bool> reference_loaded_{false};
  // agreement counters (reset on load; updated on the analysis thread)
  long long ref_frames_matched_ = 0;
  long long ref_p4_ref_ = 0, ref_p4_live_ = 0, ref_p4_both_ = 0;

  // Cached prepared statement for storeFrameData — preparing the INSERT once
  // per recording instead of once per frame (250 Hz). Prepared lazily against
  // the db the host passes in; finalized in endStorageBatch, which the host
  // calls before closing the db.
  sqlite3_stmt* store_stmt_ = nullptr;
  sqlite3* store_stmt_db_ = nullptr;

  // Settings storage
  struct Settings {
    int pupil_threshold = 45;
    float p1_pupil_radius_max = 1.5;
    float p1_max_jump = 15.0f;
    int p1_min_intensity = 140;
    float p1_min_area = 40.0f;
    float p1_max_area = 600.0f;
    // NOTE: these must match the live members they mirror (p4_validator_ ctor,
    // p4_max_prediction_error_ init) — the UI syncs from settings_, and a
    // mismatch here shows the operator a different value than the gate uses.
    float p4_max_jump = 20.0f;
    int p4_min_intensity = 140;
    int p4_search_width = 50;
    int p4_search_height = 50;
    float p4_max_prediction_error = 13.0f;
    std::string detection_mode = "pupil_p1";
  } settings_;
  
  // Frame-rate-dependent timing thresholds (in frames)
  struct TimingThresholds {
    int p1_loss_threshold;        // Frames without P1 before reset
    int p1_recovery_frames;       // Frames to skip validation after P1 loss
    int p1_relocation_frames;     // Frames needed to confirm P1 relocation
    int p4_loss_threshold;        // Frames without P4 before reset
    int p4_recovery_frames;       // Frames to skip validation after P4 loss
    int p4_retry_interval;        // Frames between periodic P4 recovery retries
    int blink_recovery_frames;    // Frames to recover after blink
    int baseline_update_frames;   // Frames between baseline updates
    int tracking_lost_frames;     // No-pupil frames before declaring LOST
    int p1_desperation_timeout;   // Desperation frames before giving up
    int p1_desperation_cooldown;  // Frames before desperation may re-latch
    int long_blink_reset_frames;  // Blink-with-pupil frames before self-heal
    float learn_ref_decay;        // Per-frame decay of the dilation reference

    // Per-frame jump limits (max_jump) are DISTANCES an object may move in
    // one frame, so they scale with frame time: the tuned values are
    // referenced to 250 Hz, and at lower rates the eye moves proportionally
    // farther between frames. Applied when timing is (re)calculated:
    // validator limit = user setting * jump_scale.
    float jump_scale;

    // Initialize with default time constants (assuming 250 Hz)
    TimingThresholds()
      : p1_loss_threshold(5),
        p1_recovery_frames(5),
        p1_relocation_frames(3),
	p4_loss_threshold(5),
        p4_recovery_frames(8),
	p4_retry_interval(50),
	blink_recovery_frames(15),
        baseline_update_frames(10),
        tracking_lost_frames(125),
        p1_desperation_timeout(500),
        p1_desperation_cooldown(250),
        long_blink_reset_frames(375),
        jump_scale(1.0f),
        learn_ref_decay(0.999995f) {}
    
    // Calculate frame counts from time constants and frame rate
    void calculateFromFrameRate(float fps) {
      float frame_time_ms = 1000.0f / fps;
      
      // Time constants in milliseconds
      constexpr float P1_LOSS_TIME_MS = 20.0f;
      constexpr float P1_RECOVERY_TIME_MS = 15.0f;
      constexpr float P1_RELOCATION_TIME_MS = 15.0f;
      constexpr float P4_LOSS_TIME_MS = 15.0f;
      constexpr float P4_RECOVERY_TIME_MS = 15.0f;
      constexpr float P4_RETRY_TIME_MS = 200.0f;
      constexpr float BLINK_RECOVERY_TIME_MS = 20.0f;
      constexpr float BASELINE_UPDATE_TIME_MS = 20.0f;
      // Longest plausible real blink; beyond this with no pupil = LOST
      constexpr float TRACKING_LOST_TIME_MS = 500.0f;
      constexpr float P1_DESPERATION_TIMEOUT_MS = 2000.0f;
      constexpr float P1_DESPERATION_COOLDOWN_MS = 1000.0f;
      // Stuck-blink self-heal: blink persisting this long WITH a pupil present
      constexpr float LONG_BLINK_RESET_MS = 1500.0f;
      // Frame rate the per-frame jump limits were tuned at
      constexpr float JUMP_REFERENCE_FPS = 250.0f;
      // Half-life of the learning dilation reference (leaky max)
      constexpr float LEARN_REF_HALFLIFE_MS = 600000.0f;  // 10 minutes

      // Convert to frame counts
      p1_loss_threshold = static_cast<int>(std::ceil(P1_LOSS_TIME_MS / frame_time_ms));
      p1_recovery_frames = static_cast<int>(std::ceil(P1_RECOVERY_TIME_MS / frame_time_ms));
      p1_relocation_frames = static_cast<int>(std::ceil(P1_RELOCATION_TIME_MS / frame_time_ms));
      p4_loss_threshold = static_cast<int>(std::ceil(P4_LOSS_TIME_MS / frame_time_ms));
      p4_recovery_frames = static_cast<int>(std::ceil(P4_RECOVERY_TIME_MS / frame_time_ms));
      p4_retry_interval = static_cast<int>(std::ceil(P4_RETRY_TIME_MS / frame_time_ms));

      blink_recovery_frames = static_cast<int>(std::ceil(BLINK_RECOVERY_TIME_MS / frame_time_ms));
      baseline_update_frames = static_cast<int>(std::ceil(BASELINE_UPDATE_TIME_MS / frame_time_ms));
      tracking_lost_frames = static_cast<int>(std::ceil(TRACKING_LOST_TIME_MS / frame_time_ms));
      p1_desperation_timeout = static_cast<int>(std::ceil(P1_DESPERATION_TIMEOUT_MS / frame_time_ms));
      p1_desperation_cooldown = static_cast<int>(std::ceil(P1_DESPERATION_COOLDOWN_MS / frame_time_ms));
      long_blink_reset_frames = static_cast<int>(std::ceil(LONG_BLINK_RESET_MS / frame_time_ms));
      jump_scale = (fps > 0.0f) ? (JUMP_REFERENCE_FPS / fps) : 1.0f;
      // decay^(frames_per_halflife) = 0.5
      learn_ref_decay = std::exp(-0.6931472f * frame_time_ms / LEARN_REF_HALFLIFE_MS);
    }
  } timing_;
  
    void fireSettingChanged(const std::string& setting_name, const std::string& value) {
        std::map<std::string, std::string> data;
        data["name"] = setting_name;
        data["value"] = value;
        
        VstreamEvent evt("eyetracking/settings",
			 VstreamEventData::makeKeyValue(data));
        evt.rate_limit_exempt = true;  // Always deliver
        fireEvent(evt);
    }

    void fireAllSettings() {
        fireSettingChanged("pupil_threshold",
			   std::to_string(settings_.pupil_threshold));
        fireSettingChanged("p1_max_jump",
			   std::to_string(settings_.p1_max_jump));
        fireSettingChanged("p1_pupil_radius_max",
			   std::to_string(settings_.p1_pupil_radius_max));
        fireSettingChanged("p1_min_intensity",
			   std::to_string(settings_.p1_min_intensity));
        fireSettingChanged("p4_max_jump",
			   std::to_string(settings_.p4_max_jump));
        fireSettingChanged("p4_min_intensity",
			   std::to_string(settings_.p4_min_intensity));
        fireSettingChanged("p4_max_prediction_error",
			   std::to_string(settings_.p4_max_prediction_error));
        fireSettingChanged("detection_mode",
			   settings_.detection_mode);
    }  

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
            
            if (debug_level_ >= DEBUG_NORMAL) {
                std::cout << "Allocated buffers: " 
                          << frame_size_.width << "x" << frame_size_.height << std::endl;
            }
        }
    }

    // Recenter the ROI (size unchanged) on a full-frame point, clamped to the
    // frame. Safe mid-session: validators, the P4 model, and all loss/recovery
    // anchors operate in FULL-FRAME coordinates, so moving the crop does not
    // disturb tracking state — it simply takes effect on the next frame.
    bool centerROIOn(const cv::Point2f& center) {
        std::lock_guard<std::mutex> lock(roi_mutex_);
        if (!roi_enabled_) {
            return false;
        }
        int nx = cvRound(center.x - current_roi_.width / 2.0f);
        int ny = cvRound(center.y - current_roi_.height / 2.0f);
        if (buffers_initialized_) {
            nx = std::max(0, std::min(nx, frame_size_.width - current_roi_.width));
            ny = std::max(0, std::min(ny, frame_size_.height - current_roi_.height));
        } else {
            nx = std::max(0, nx);
            ny = std::max(0, ny);
        }
        current_roi_.x = nx;
        current_roi_.y = ny;
        return true;
    }

    // ========================================================================
    // PUPIL DETECTION
    // ========================================================================
  // gray_roi is the (already ROI-cropped, already grayscale) working image
  // prepared once per frame in processFrameInline; roi/use_roi describe how it
  // maps back to full-frame coordinates.
  PupilData detectPupil(const cv::Mat& gray_roi, const cv::Rect& roi,
                        bool use_roi) {
    PupilData result = {cv::Point2f(-1, -1), -1, false};

    if (binary_buffer_.size() != gray_roi.size()) {
        binary_buffer_ = cv::Mat(gray_roi.size(), CV_8UC1);
    }

    // Threshold
    cv::threshold(gray_roi, binary_buffer_, pupil_threshold_, 255, cv::THRESH_BINARY_INV);

    // Pick the best pupil-like connected component (plausibility gate) rather
    // than taking moments of every dark pixel — see the member comment block.
    // 8-connectivity so a pupil pinched by a glint at its edge stays one blob.
    // NOTE: keep CV_32S labels — OpenCV's fast (SIMD/BBDT) implementation is
    // 32-bit-only; CV_16U falls back to a scalar path measured 2x slower here.
    cv::Mat stats, centroids;
    int n = cv::connectedComponentsWithStats(binary_buffer_, cc_labels_,
                                             stats, centroids, 8, CV_32S);
    int best = -1;
    int best_area = 0;
    for (int i = 1; i < n; i++) {  // label 0 = background
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < pupil_min_area_ || area > pupil_max_area_) continue;
        if (area <= best_area) continue;  // only bother testing larger blobs

        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        if (w <= 0 || h <= 0) continue;

        float extent = static_cast<float>(area) / (static_cast<float>(w) * h);
        if (extent < pupil_min_extent_) continue;

        float aspect = (w > h) ? static_cast<float>(w) / h
                               : static_cast<float>(h) / w;
        if (aspect > pupil_max_aspect_) continue;

        best = i;
        best_area = area;
    }

    if (best >= 0) {
        cv::Point2f center(
            static_cast<float>(centroids.at<double>(best, 0)),
            static_cast<float>(centroids.at<double>(best, 1)));
        float radius = std::sqrt(best_area / M_PI);

        // Adjust coordinates back to full frame if using ROI
        if (use_roi) {
            center.x += roi.x;
            center.y += roi.y;
        }

        result = {center, radius, true};

        // Pupillometry: outer contour of the selected component. Its FILLED
        // area is unbiased by the bright glint holes (P1/P4/illuminator)
        // punched in the thresholded pupil, and it gives an ellipse fit.
        // All of this runs on the component's small bbox, not the frame.
        int bx = stats.at<int>(best, cv::CC_STAT_LEFT);
        int by = stats.at<int>(best, cv::CC_STAT_TOP);
        int bw = stats.at<int>(best, cv::CC_STAT_WIDTH);
        int bh = stats.at<int>(best, cv::CC_STAT_HEIGHT);
        cv::Mat comp_mask = (cc_labels_(cv::Rect(bx, by, bw, bh)) == best);

        std::vector<std::vector<cv::Point>> comp_contours;
        cv::findContours(comp_mask, comp_contours,
                         cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (!comp_contours.empty()) {
            auto outer = std::max_element(
                comp_contours.begin(), comp_contours.end(),
                [](const auto& a, const auto& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });
            double filled_area = cv::contourArea(*outer);
            if (filled_area >= best_area) {  // sanity: filled >= pixel count
                result.radius = std::sqrt(filled_area / M_PI);
            }
            if (outer->size() >= 5) {
                cv::RotatedRect e = cv::fitEllipse(*outer);
                result.has_ellipse = true;
                result.ellipse_a = std::max(e.size.width, e.size.height) * 0.5f;
                result.ellipse_b = std::min(e.size.width, e.size.height) * 0.5f;
                result.ellipse_angle = e.angle;
            }
        }
    }

    return result;
}

    // ========================================================================
    // P1 SUB-PIXEL REFINEMENT
    // ========================================================================

  cv::Point2f refineP1SubPixel(const cv::Mat& gray_roi,
			       const cv::Rect& search_rect,
			       const cv::Point2f& candidate) {

    bool use_ellipse_p1_refinement_ = false;
      
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
        
        if (use_ellipse_p1_refinement_ && largest_contour->size() >= 5) {
            // ELLIPSE METHOD - better for saturated reflections
            cv::RotatedRect ellipse = cv::fitEllipse(*largest_contour);
            
            // Sanity check: reasonable aspect ratio
            float aspect = std::max(ellipse.size.width, ellipse.size.height) / 
                          std::min(ellipse.size.width, ellipse.size.height);
            
            if (aspect < 3.0) {  // Not too elongated
                refined_p1 = ellipse.center + cv::Point2f(centroid_roi.x, centroid_roi.y);
            } else {
                // Weird shape - fall back to centroid
                cv::Moments M = cv::moments(*largest_contour);
                if (M.m00 > 0) {
                    cv::Point2f centroid_in_roi(M.m10 / M.m00, M.m01 / M.m00);
                    refined_p1 = centroid_in_roi + cv::Point2f(centroid_roi.x, centroid_roi.y);
                }
            }
        } else {
            // CENTROID METHOD - original approach
            cv::Moments M = cv::moments(*largest_contour);
            if (M.m00 > 0) {
                cv::Point2f centroid_in_roi(M.m10 / M.m00, M.m01 / M.m00);
                refined_p1 = centroid_in_roi + cv::Point2f(centroid_roi.x, centroid_roi.y);
            }
        }
    }
    
    return refined_p1;
}
  
    // ========================================================================
    // P1 MAIN DETECTION
    // ========================================================================
  
  cv::Point2f detectP1(const cv::Mat& gray_roi, 
		       const cv::Point2f& pupil_center_local,
		       float pupil_radius,
		       bool in_desperation = false) {  
    
    float search_radius = pupil_radius * p1_pupil_radius_max_;
    
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
    
    // Store candidates with their scores
    struct P1Candidate {
      cv::Point2f position;
      float score;
      float area;
      float intensity;
      bool operator<(const P1Candidate& other) const {
	return score > other.score;  // Sort descending
      }
    };
    std::vector<P1Candidate> candidates;
    
    int total_candidates = 0;
    
    for (int thresh : thresholds) {
      if (thresh < p1_min_intensity_) continue;
      
      cv::Mat p1_thresh;
      cv::threshold(p1_region, p1_thresh, thresh, 255, cv::THRESH_BINARY);

      std::vector<std::vector<cv::Point>> contours;
      // findContours has not modified its input since OpenCV 3.2 — no clone.
      cv::findContours(p1_thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      
      for (const auto& contour : contours) {
	float area = cv::contourArea(contour);
        
	if (area < 20) continue;
	if (area > 800) {
	  continue;
	}

	// Relax area constraints in desperation mode
	if (!in_desperation) {
	  if (area < p1_min_area_ || area > p1_max_area_) {
	    if (debug_level_ >= DEBUG_VERBOSE) {
	      std::cout << "P1 candidate rejected: area=" << area 
			<< " outside [" << p1_min_area_ << "," << p1_max_area_ << "]" 
			<< std::endl;
	    }
	    continue;
	  }
	}
    	
	// Relax size ratio check in desperation mode
	if (last_p1_area_ > 0 && !in_desperation) {
	  float size_ratio = area / last_p1_area_;
	  if (size_ratio < 0.7f || size_ratio > 2.0f) continue;
	}

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
        
	// Relax circularity in desperation mode
	float circularity_threshold = in_desperation ? 0.2f : 
	  ((area > 100) ? 0.3f : 0.5f);
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

	float size_score = 1.0f;
	if (last_p1_area_ < 0 || in_desperation) {
	  // Acquisition mode: STRONGLY prefer larger P1
	  if (area >= 100) {
	    size_score = 1.5f;  // Big bonus for large spots
	  } else if (area >= 50) {
	    size_score = 0.7f;  // Penalty for medium
	  } else {
	    size_score = 0.3f;  // Strong penalty for tiny spots
	  }
	} else {
	  // Tracking mode
	  size_score = 1.0f;
	}	
	
	
	// reward candidates that are about the same size
	float size_consistency = 1.0f;
	if (last_p1_area_ > 0 && !in_desperation) {
	  float size_ratio = area / last_p1_area_;
	  if (size_ratio < 0.6f || size_ratio > 1.7f) {
	    size_consistency = 0.5f;
	  }
	}
        
	float score = effective_intensity * circularity * compactness * 
	  vertical_bias * size_score * size_consistency;
	
	// Add to candidate list instead of immediately picking best
	candidates.push_back({center_rel, score, area, (float) effective_intensity});
      }
    }
    
    if (debug_level_ >= DEBUG_VERBOSE) {
      std::cout << "P1: " << total_candidates << " candidates" 
		<< (in_desperation ? " (DESPERATION)" : "") << std::endl;
    }
    
    if (candidates.empty()) {
      return cv::Point2f(-1, -1);
    }
    
    // Sort candidates by score
    std::sort(candidates.begin(), candidates.end());
    
    // Try candidates in order until one passes validation
    for (size_t i = 0; i < candidates.size(); ++i) {
      cv::Point2f refined = refineP1SubPixel(gray_roi, p1_search_rect, candidates[i].position);
      
      cv::Point2f p1_local(
			   p1_search_rect.x + refined.x,
			   p1_search_rect.y + refined.y
			   );
      
      // Distance check
      float dist = cv::norm(p1_local - pupil_center_local);
      if (dist > pupil_radius * p1_pupil_radius_max_) {
	if (debug_level_ >= DEBUG_VERBOSE && i == 0) {
	  std::cout << "P1 candidate #" << (i+1) << " rejected: dist=" << dist 
		    << " > " << (pupil_radius * 2.0f) << std::endl;
	}
	continue;  // Try next candidate
      }
      
      // Convert to full frame coordinates for validator
      cv::Point2f p1_full = p1_local;
      if (roi_enabled_) {
	p1_full.x += current_roi_.x;
	p1_full.y += current_roi_.y;
      }
      
      // SKIP VALIDATOR CHECK IN DESPERATION MODE
      if (!in_desperation) {
	// Validator check - if rejected, try next candidate
	if (!p1_validator_.isValid(p1_full)) {
	  if (debug_level_ >= DEBUG_VERBOSE && i == 0) {
	    std::cout << "P1 candidate #" << (i+1) << " rejected: "
		      << "jump=" << std::fixed << std::setprecision(1) 
		      << p1_validator_.getJumpDistance() << "px "
		      << "(max=" << p1_validator_.getMaxJump() << "px, "
		      << (p1_validator_.getJumpDistance() / p1_validator_.getMaxJump() * 100) 
		      << "% over) "
		      << "pos=(" << p1_full.x << "," << p1_full.y << ")" << std::endl;
	  }
	  continue;  // Try next candidate
	}
      } else {
	// In desperation mode - just report what we're accepting.
	// VERBOSE only: entering desperation and recovering from it are logged
	// once (edge-triggered) in detectPurkinje; per-frame accepts here would
	// otherwise spam the log for the whole duration of a desperation episode.
	if (debug_level_ >= DEBUG_VERBOSE && i == 0) {
	  std::cout << "P1 desperation mode accepting candidate #" << (i+1)
		    << " (skipping validation)" << std::endl;
	}
      }
      
      // Found a valid candidate!
      if (debug_level_ >= DEBUG_VERBOSE && i > 0) {
	std::cout << "P1 candidate #" << (i+1) << " accepted (score=" 
		  << candidates[i].score << ")" << std::endl;
      }
      
      last_p1_area_ = candidates[i].area;
      last_p1_intensity_ = candidates[i].intensity;      
      return p1_local;
    }
    
    // All candidates rejected
    if (debug_level_ >= DEBUG_VERBOSE) {
      std::cout << "All " << candidates.size() << " P1 candidates rejected" << std::endl;
    }
    
    // don't have P1 area to track
    last_p1_area_ = -1.0f;
    last_p1_intensity_ = -1.0f;
    
    return cv::Point2f(-1, -1);
  }

  // ========================================================================
  // P4 BRIGHT SPOT DETECTION
  // ========================================================================

cv::Point2f refineP4SubPixelWeighted(const cv::Mat& search_region,
                                     cv::Point max_loc) {
    int x = max_loc.x;
    int y = max_loc.y;
    
    // Need 2-pixel border for 5x5
    if (x < 2 || x >= search_region.cols - 2 ||
        y < 2 || y >= search_region.rows - 2) {
        return cv::Point2f(max_loc);
    }
    
    // Find max intensity in 5x5 to set threshold
    double max_intensity = 0;
    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            max_intensity = std::max(max_intensity, 
                (double)search_region.at<uchar>(y + dy, x + dx));
        }
    }
    
    // Only use pixels above 70% of local max
    double threshold = max_intensity * 0.7;
    
    double sum_wx = 0, sum_wy = 0, sum_w = 0;
    
    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            float intensity = search_region.at<uchar>(y + dy, x + dx);
            
            if (intensity > threshold) {
                // Weight by intensity (squared for emphasis on bright pixels)
                double weight = intensity * intensity;
                
                sum_wx += weight * (x + dx);
                sum_wy += weight * (y + dy);
                sum_w += weight;
            }
        }
    }
    
    if (sum_w > 0) {
        return cv::Point2f(sum_wx / sum_w, sum_wy / sum_w);
    }
    
    return cv::Point2f(max_loc);
}

// search_rect is the predictive box (already clipped to the image); rect_mask
// is rect-sized, 255 = searchable (P1 exclusion already carved out). Only the
// rect is scanned — the old full-image raster visited ~100x more pixels per
// frame than the box contains, plus a full-frame mask allocation, for the
// same result.
cv::Point2f findP4ByProximityWeightedSearch(const cv::Mat& search_region,
                                             const cv::Mat& rect_mask,
                                             const cv::Rect& search_rect,
                                             const cv::Point2f& predicted_center_local,
                                             const cv::Point2f& pupil_center_local,
                                             float pupil_max_dist) {
    if (search_region.empty()) {
        return cv::Point2f(-1, -1);
    }

    int center_x = cvRound(predicted_center_local.x);
    int center_y = cvRound(predicted_center_local.y);

    // Validate center is in bounds
    if (center_x < 0 || center_x >= search_region.cols ||
        center_y < 0 || center_y >= search_region.rows) {
        // Fallback to global search (degenerate case: prediction off-image).
        // Reconstruct the full-size mask the brightest-spot search expects.
        cv::Mat full_mask = cv::Mat::zeros(search_region.size(), CV_8UC1);
        rect_mask.copyTo(full_mask(search_rect));
        return findP4ByBrightestSpot(search_region, full_mask);
    }

    // P4 is only ever inside the pupil; reject candidates beyond this distance
    // from the pupil center so out-of-pupil iris spots are never preferred.
    float max_d2 = (pupil_max_dist > 0) ? pupil_max_dist * pupil_max_dist : -1.0f;

    cv::Point best_loc(-1, -1);
    double best_score = -1e9;
    double best_intensity = 0;
    float best_distance = 0;

    for (int y = search_rect.y; y < search_rect.y + search_rect.height; y++) {
        const uchar* mask_row = rect_mask.ptr<uchar>(y - search_rect.y);
        const uchar* img_row = search_region.ptr<uchar>(y);
        for (int x = search_rect.x; x < search_rect.x + search_rect.width; x++) {
            // Skip masked pixels
            if (mask_row[x - search_rect.x] == 0) continue;

            // Skip pixels outside the pupil disk
            if (max_d2 > 0) {
                float ddx = x - pupil_center_local.x;
                float ddy = y - pupil_center_local.y;
                if (ddx*ddx + ddy*ddy > max_d2) continue;
            }

            double intensity = img_row[x];

            // Skip pixels below threshold
            if (intensity < p4_min_intensity_) continue;

            // Calculate distance from predicted center
            float dx = x - center_x;
            float dy = y - center_y;
            float distance = std::sqrt(dx*dx + dy*dy);

            // Score = intensity - distance penalty
            // Higher proximity_weight_ means stronger preference for center
            double score = intensity - distance * proximity_weight_;

            if (score > best_score) {
                best_score = score;
                best_loc = cv::Point(x, y);
                best_intensity = intensity;
                best_distance = distance;
            }
        }
    }

    if (debug_level_ >= DEBUG_VERBOSE) {
        std::cout << "P4 proximity search: best at distance=" << best_distance
                  << " intensity=" << best_intensity
                  << " threshold=" << p4_min_intensity_
                  << " score=" << best_score << std::endl;
    }

    if (best_loc.x < 0) {
        return cv::Point2f(-1, -1);
    }

    return refineP4SubPixelWeighted(search_region, best_loc);
}
  

 cv::Point2f findP4ByBrightestSpot(const cv::Mat& search_region,
                                   const cv::Mat& search_mask) {
    if (search_region.empty()) {
        return cv::Point2f(-1, -1);
    }
    
    double min_val, max_val;
    cv::Point min_loc, max_loc;
    cv::minMaxLoc(search_region, &min_val, &max_val, &min_loc, &max_loc, search_mask);
    
    if (debug_level_ >= DEBUG_VERBOSE) {
        std::cout << "P4 bright spot: intensity=" << max_val
                  << " threshold=" << p4_min_intensity_ << std::endl;
    }

    if (max_val < p4_min_intensity_) {
        return cv::Point2f(-1, -1);
    }
    
    return refineP4SubPixelWeighted(search_region, max_loc);
}
  
    // ========================================================================
    // P4 DETECTION
    // ========================================================================

  cv::Point2f detectP4(const cv::Mat& gray_roi,
		       const cv::Point2f& pupil_center_local,
		       const cv::Point2f& p1_local,
		       float pupil_radius,
		       bool use_roi,
		       const cv::Rect& roi,
		       bool in_recovery = false) {
    
    cv::Point2f predicted_p4_local(-1, -1);
    cv::Rect predictive_roi;
    bool have_predictive_rect = false;
    float p1_exclusion_radius = pupil_radius * 0.3f;

    if (p4_model_.isInitialized()) {
      // Convert ROI-local coordinates to full-frame for model prediction
      cv::Point2f pupil_full = pupil_center_local;
      cv::Point2f p1_full = p1_local;
      
      if (use_roi) {
        pupil_full.x += roi.x;
        pupil_full.y += roi.y;
        p1_full.x += roi.x;
        p1_full.y += roi.y;
      }
      
      if (debug_level_ >= DEBUG_VERBOSE) {
          std::cout << "P4 model inputs:" << std::endl;
          std::cout << "  Pupil (local): (" << pupil_center_local.x << "," << pupil_center_local.y 
                    << ") full: (" << pupil_full.x << "," << pupil_full.y << ")" << std::endl;
          std::cout << "  P1 (local): (" << p1_local.x << "," << p1_local.y 
                    << ") full: (" << p1_full.x << "," << p1_full.y << ")" << std::endl;
          if (use_roi) {
              std::cout << "  ROI offset: (" << roi.x << "," << roi.y << ")" << std::endl;
          }
      }
      
      // Predict in full-frame coordinates (matching training data)
      cv::Point2f predicted_p4_full = p4_model_.predict(pupil_full, p1_full);

      // Stash full-frame prediction for the reprocess decision log
      p4_predicted_ = predicted_p4_full;

      // Convert prediction back to ROI-local coordinates for search
      predicted_p4_local = predicted_p4_full;
      if (use_roi && predicted_p4_full.x > 0) {
        predicted_p4_local.x -= roi.x;
        predicted_p4_local.y -= roi.y;
      }
      
      if (predicted_p4_local.x > 0) {
        // Expand search ROI during recovery to accommodate gaze shifts
        cv::Size effective_search_size = p4_search_roi_size_;
        if (in_recovery) {
	  effective_search_size.width = static_cast<int>(effective_search_size.width * 1.5);
	  effective_search_size.height = static_cast<int>(effective_search_size.height * 1.5);
        }

        predictive_roi = cv::Rect(
				predicted_p4_local.x - effective_search_size.width / 2,
				predicted_p4_local.y - effective_search_size.height / 2,
				effective_search_size.width,
				effective_search_size.height
				);

        predictive_roi &= cv::Rect(0, 0, gray_roi.cols, gray_roi.rows);

        if (predictive_roi.area() > 0) {
	  // Rect-local mask (the search only ever visits this box): 255 =
	  // searchable, with the P1 glint region carved out. Same circle
	  // rasterization as the old full-frame mask, center shifted into
	  // rect coordinates — behavior-identical, ~100x fewer mask pixels.
	  if (p4_rect_mask_.size() != predictive_roi.size()) {
	    p4_rect_mask_.create(predictive_roi.size(), CV_8UC1);
	  }
	  p4_rect_mask_.setTo(255);
	  cv::circle(p4_rect_mask_, cv::Point(p1_local) - predictive_roi.tl(),
		     p1_exclusion_radius, 0, -1);
	  have_predictive_rect = true;

	  if (debug_level_ >= DEBUG_VERBOSE) {
	    cv::Point2f predicted_p4_full = predicted_p4_local;
	    if (use_roi) {
	      predicted_p4_full.x += roi.x;
	      predicted_p4_full.y += roi.y;
	    }
	    std::cout << "P4 prediction (local): (" << predicted_p4_local.x 
		      << "," << predicted_p4_local.y << ") full: ("
		      << predicted_p4_full.x << "," << predicted_p4_full.y << ")" << std::endl;
	    std::cout << "  Search ROI (local): [" << predictive_roi.x << "," << predictive_roi.y 
		      << "] size " << predictive_roi.width << "x" << predictive_roi.height << std::endl;
	    std::cout << "  Gray ROI size: " << gray_roi.cols << "x" << gray_roi.rows << std::endl;
	  }
        }
      }
    }

    // With a model prediction, search near it (proximity-weighted, confined
    // to the predictive rect); otherwise fall back to the global brightest
    // spot within the pupil (pre-calibration only — cold path, full mask).
    cv::Point2f p4_candidate(-1, -1);
    if (p4_model_.isInitialized() && predicted_p4_local.x > 0) {
      if (have_predictive_rect) {
        // Pupil-disk bound, floored at the predicted P4 orbit: a constricted
        // pupil must never exclude the location the model itself predicts.
        float pred_orbit = cv::norm(predicted_p4_local - pupil_center_local);
        float disk_limit = std::max(pupil_radius * p4_pupil_search_margin_,
                                    pred_orbit + P4_DISK_PRED_SLACK);
        p4_candidate = findP4ByProximityWeightedSearch(
            gray_roi, p4_rect_mask_, predictive_roi, predicted_p4_local,
            pupil_center_local, disk_limit);
      }
      // else: prediction fell entirely off-image — nothing searchable,
      // p4_candidate stays (-1,-1), exactly as the all-zero mask produced.
    } else if (!p4_model_.isInitialized()) {
        cv::Mat search_mask = cv::Mat::zeros(gray_roi.size(), CV_8UC1);
        float search_radius = pupil_radius * 0.85f;
        cv::circle(search_mask, pupil_center_local, search_radius, 255, -1);
        cv::circle(search_mask, p1_local, p1_exclusion_radius, 0, -1);
        p4_candidate = findP4ByBrightestSpot(gray_roi, search_mask);
    }
    // (model initialized but prediction invalid: no searchable region — the
    // old code reached the brightest-spot search with an all-zero mask and
    // found nothing; p4_candidate stays (-1,-1).)
    
    if (p4_candidate.x < 0) {
        p4_reject_reason_ = P4_NO_CANDIDATE;
        if (in_recovery && debug_level_ >= DEBUG_NORMAL) {
            std::cout << "  P4 recovery: no candidate found in search region" << std::endl;
        }
        return cv::Point2f(-1, -1);
    }
    p4_candidate_found_ = true;

    // Log candidate details during recovery for diagnostics
    if (in_recovery && debug_level_ >= DEBUG_NORMAL) {
        int intensity = gray_roi.at<uchar>(
            cv::saturate_cast<int>(p4_candidate.y),
            cv::saturate_cast<int>(p4_candidate.x));
        float pred_error = (predicted_p4_local.x > 0) ?
            cv::norm(p4_candidate - predicted_p4_local) : -1.0f;
        std::cout << "  P4 recovery candidate: (" << p4_candidate.x << ","
                  << p4_candidate.y << ") intensity=" << intensity
                  << " pred_error=" << pred_error << std::endl;
    }

    // Validate that P4 is within ROI bounds
    if (p4_candidate.x < 0 || p4_candidate.x >= gray_roi.cols ||
        p4_candidate.y < 0 || p4_candidate.y >= gray_roi.rows) {
        p4_reject_reason_ = P4_OUT_OF_BOUNDS;
        if (debug_level_ >= DEBUG_NORMAL) {
            std::cout << "P4 candidate outside ROI bounds: ("
                      << p4_candidate.x << "," << p4_candidate.y 
                      << ") ROI size: " << gray_roi.cols << "x" << gray_roi.rows << std::endl;
        }
        return cv::Point2f(-1, -1);
    }
    
    // Debug: Show found P4 in both coordinate systems
    if (debug_level_ >= DEBUG_VERBOSE) {
        cv::Point2f p4_candidate_full = p4_candidate;
        if (use_roi) {
            p4_candidate_full.x += roi.x;
            p4_candidate_full.y += roi.y;
        }
        std::cout << "P4 candidate (local): (" << p4_candidate.x 
                  << "," << p4_candidate.y << ") full: ("
                  << p4_candidate_full.x << "," << p4_candidate_full.y << ")" << std::endl;
    }
    
    // Validate prediction error if model is initialized
    if (p4_model_.isInitialized() && predicted_p4_local.x > 0) {
        float prediction_error = cv::norm(p4_candidate - predicted_p4_local);
        
        // DIAGNOSTIC OUTPUT
        if (debug_level_ >= DEBUG_VERBOSE) {
            std::cout << "  Prediction error calculation:" << std::endl;
            std::cout << "    predicted_p4_local: (" << predicted_p4_local.x << ", " << predicted_p4_local.y << ")" << std::endl;
            std::cout << "    p4_candidate: (" << p4_candidate.x << ", " << p4_candidate.y << ")" << std::endl;
            std::cout << "    difference: (" << (p4_candidate.x - predicted_p4_local.x) << ", " 
                      << (p4_candidate.y - predicted_p4_local.y) << ")" << std::endl;
            std::cout << "    error (norm): " << prediction_error << " pixels" << std::endl;
        }
        
        // Relax prediction error threshold during recovery
        float effective_max_error = p4_max_prediction_error_;
        if (in_recovery) {
            effective_max_error *= 1.5f;
        }

        if (prediction_error > effective_max_error) {
            p4_reject_reason_ = P4_PRED_ERROR;
            if (debug_level_ >= DEBUG_NORMAL) {
                std::cout << "P4 prediction error too large: " << prediction_error
                         << " > " << effective_max_error
                         << (in_recovery ? " (recovery)" : "")
                         << " (both in local ROI coordinates)" << std::endl;
            }
            return cv::Point2f(-1, -1);
        }
    }
    
    return p4_candidate;
}
  
  // ========================================================================
  // MAIN PURKINJE COORDINATOR
  // ========================================================================

  // gray_roi is the shared per-frame grayscale prepared in processFrameInline
  // (same image detectPupil thresholded); the blurred copy for glint search is
  // made here into blur_buffer_ so the raw gray stays untouched.
  PurkinjeData detectPurkinje(const cv::Mat& gray_roi, const PupilData& pupil,
			      int frame_idx, const cv::Rect& roi, bool use_roi) {
    PurkinjeData result = {{-1, -1}, {-1, -1}, false, false};

    if (!pupil.detected || pupil.radius <= 0) {
      return result;
    }

    // Early return if we're in pupil-only mode
    if (detection_mode_ == MODE_PUPIL_ONLY) {
      return result;
    }

    cv::GaussianBlur(gray_roi, blur_buffer_, cv::Size(3, 3), 0.5);

    // Adjust pupil center to local coordinates if using ROI
    cv::Point2f pupil_center_local = pupil.center;
    if (use_roi) {
      pupil_center_local.x -= roi.x;
      pupil_center_local.y -= roi.y;
    }
        
    // P1 DETECTION (only if mode >= MODE_PUPIL_P1)
    // Loss/recovery state lives in member vars (p1_loss_counter_ etc.) so reset
    // clears it. Local references keep the body below readable.
    int& p1_loss_counter = p1_loss_counter_;
    int& p1_recovery_countdown = p1_recovery_countdown_;
    bool& p1_was_lost = p1_was_lost_;

    // Use dynamic thresholds from timing_ struct
    const int P1_LOSS_THRESHOLD = timing_.p1_loss_threshold;
    const int P1_RECOVERY_FRAMES = timing_.p1_recovery_frames;

    // Latched desperation state with hysteresis. Entered when the loss counter
    // blows past the threshold; cleared only after a stable re-lock (below).
    // Previously this was recomputed every frame as a pure threshold on an
    // oscillating counter, which made it flip-flop across the boundary and
    // reprint the desperation diagnostics forever.
    if (p1_desperation_cooldown_ > 0) {
      p1_desperation_cooldown_--;
    }
    if (!p1_desperation_ && p1_desperation_cooldown_ == 0 &&
        p1_loss_counter > P1_LOSS_THRESHOLD * 5) {
      p1_desperation_ = true;
      p1_desperation_streak_ = 0;
      p1_desperation_frames_ = 0;
      p1_desperation_last_pos_ = cv::Point2f(-1, -1);
      if (debug_level_ >= DEBUG_CRITICAL) {
	std::cout << "P1 entering desperation mode (loss_counter="
		  << p1_loss_counter << ")" << std::endl;
      }
    }

    // Desperation give-up: an episode that never achieves a consistent lock
    // means P1 is genuinely gone; stop accepting blind candidates (junk would
    // stream out as P1 indefinitely), rest for a cooldown, then allow a fresh
    // attempt. Loss counter restarts so re-entry needs the full threshold.
    if (p1_desperation_ &&
        ++p1_desperation_frames_ > timing_.p1_desperation_timeout) {
      p1_desperation_ = false;
      p1_desperation_streak_ = 0;
      p1_desperation_frames_ = 0;
      p1_desperation_cooldown_ = timing_.p1_desperation_cooldown;
      p1_loss_counter = 0;
      if (debug_level_ >= DEBUG_CRITICAL) {
	std::cout << "P1 desperation gave up (no consistent lock in "
		  << timing_.p1_desperation_timeout
		  << " frames) - cooling down" << std::endl;
      }
    }
    bool in_desperation = p1_desperation_;

    cv::Point2f p1_local = detectP1(blur_buffer_, pupil_center_local, pupil.radius,
				    in_desperation);
    
    if (debug_level_ >= DEBUG_VERBOSE && p1_local.x < 0) {
      std::cout << "P1: not detected" << std::endl;
    }
    
    // (Blink start/end events fire from processFrameInline, where the
    // transition is observable — this function only runs outside blinks.)

    // Handle P1 detection/validation
    if (p1_local.x < 0) {
      // No P1 candidate detected
      p1_reject_reason_ = P1_NOT_DETECTED;
      p1_loss_counter++;
      if (p1_loss_counter == P1_LOSS_THRESHOLD) {
	p1_validator_.reset();
	p1_recovery_countdown = P1_RECOVERY_FRAMES;
	
	fireEvent(VstreamEvent("eyetracking/p1_lost",
			       "frame " + std::to_string(event_frame_id_)));
	p1_was_lost = true;		
	
	if (debug_level_ >= DEBUG_CRITICAL) {
	  std::cout << "P1 lost - resetting validator" << std::endl;
	}
      }
    } else {
      // P1 candidate detected - convert to full-frame coordinates
      cv::Point2f p1_full = p1_local;
      if (use_roi) {
	p1_full.x += roi.x;
	p1_full.y += roi.y;
      }
      
      // Debug validation state
      if (debug_level_ >= DEBUG_NORMAL) {
	std::cout << "P1 validation check: "
		  << "loss_counter=" << p1_loss_counter
		  << " recovery_countdown=" << p1_recovery_countdown
		  << " in_desperation=" << in_desperation	  
		  << " p1_recovery=" << p1_recovery_countdown
		  << " validator_initialized=" << p1_validator_.isInitialized()
		  << " candidate_count=" << p1_validator_.getCandidateCount();
	if (p1_validator_.isInitialized()) {
	  std::cout << " last_pos=(" << p1_validator_.getLastPosition().x 
		    << "," << p1_validator_.getLastPosition().y << ")";
	}
	std::cout << std::endl;
      }

      // Determine validation mode
      // Note: blink recovery is handled by the outer gate in analysisThreadFunc —
      // detectPurkinje is only called when NOT in blink and NOT in blink recovery.
      // Recovery here refers to P1 loss/reacquisition recovery only.
      bool in_recovery = (p1_recovery_countdown > 0);

      // Validate the candidate
      bool is_valid = in_desperation || in_recovery || p1_validator_.isValid(p1_full);      
      
      if (is_valid) {
	// Success! Desperation accepts are flagged low-trust in the data —
	// the validator was bypassed, so offline analysis can discount them.
	p1_reject_reason_ = in_desperation ? P1_DESPERATION_OK : P1_OK;
	result.p1_detected = true;
	result.p1_center = p1_full;

	if (in_desperation) {
	  // We're accepting candidates blind (validator bypassed). Don't declare
	  // recovery — or hand back to the validator — until we've seen a
	  // spatially CONSISTENT lock across several frames. This latches
	  // desperation (hysteresis) instead of dropping out the instant the
	  // counter dips under the entry threshold, which is what caused the
	  // enter/exit oscillation and endless diagnostic reprints.
	  bool consistent =
	    (p1_desperation_last_pos_.x >= 0.0f) &&
	    (cv::norm(p1_full - p1_desperation_last_pos_) < p1_validator_.getMaxJump());
	  p1_desperation_streak_ = consistent ? (p1_desperation_streak_ + 1) : 0;
	  p1_desperation_last_pos_ = p1_full;

	  if (p1_desperation_streak_ >= timing_.p1_relocation_frames) {
	    // Stable re-lock: exit desperation and hand back to the normal
	    // validator so ordinary validation sustains the track from here.
	    p1_desperation_ = false;
	    p1_desperation_streak_ = 0;
	    p1_desperation_frames_ = 0;
	    p1_reject_reason_ = P1_OK;  // lock confirmed — this frame is trusted
	    p1_loss_counter = 0;
	    p1_validator_.update(p1_full);
	    if (p1_was_lost) {
	      fireEvent(VstreamEvent("eyetracking/p1_recovered",
				     "frame " + std::to_string(event_frame_id_)));
	      p1_was_lost = false;
	    }
	    if (debug_level_ >= DEBUG_CRITICAL) {
	      std::cout << "P1 recovered from desperation mode at ("
			<< p1_full.x << "," << p1_full.y << ")" << std::endl;
	    }
	  }
	  // else: still latched — reporting P1 but not yet trusting it enough to
	  // fire recovered or seed the validator.
	} else {
	  // Normal operation - reset counter and adapt validator to new position
	  p1_loss_counter = 0;
	  if (p1_was_lost) {
	    fireEvent(VstreamEvent("eyetracking/p1_recovered",
				   "frame " + std::to_string(event_frame_id_)));
	    p1_was_lost = false;
	  }
	  p1_validator_.update(p1_full);
	}

	// Decrement recovery countdown on successful detection
	if (p1_recovery_countdown > 0) {
	  p1_recovery_countdown--;
	}
      } else {
	// Rejected by validator - treat as a loss. Classify for the reprocess log:
	// FAR = hard reject (>2x max_jump vs stale last P1); SOFT = relocation
	// candidate still accumulating evidence.
	p1_reject_reason_ =
	  (p1_validator_.getJumpDistance() > p1_validator_.getMaxJump() * 2.0f)
	  ? P1_REJECTED_FAR : P1_REJECTED_SOFT;
	p1_loss_counter++;

	if (debug_level_ >= DEBUG_NORMAL) {
	  float jump_dist = p1_validator_.getJumpDistance();
	  float max_allowed = p1_validator_.getMaxJump();
	  int candidate_count = p1_validator_.getCandidateCount();
	  
	  std::cout << "P1 rejected: "
		    << "jump=" << std::fixed << std::setprecision(1) << jump_dist << "px "
		    << "(max=" << max_allowed << "px, "
		    << (jump_dist / max_allowed * 100) << "% over) "
		    << "pos=(" << p1_full.x << "," << p1_full.y << ")";
	  
	  if (candidate_count > 0) {
	    std::cout << " [tracking " << candidate_count << "/3]";
	  }
	  
	  std::cout << std::endl;
	}
      }
    }
        
    // P4 DETECTION

    // P4 loss/recovery tracking uses member variables so reset can clear them
    const int P4_LOSS_THRESHOLD = timing_.p4_loss_threshold;
    const int P4_RECOVERY_FRAMES = timing_.p4_recovery_frames;
    const int P4_RETRY_INTERVAL = timing_.p4_retry_interval;

    if (detection_mode_ == MODE_FULL) {
      if (!result.p1_detected) {
        frames_without_p1_for_p4_++;
        if (frames_without_p1_for_p4_ >= timing_.p1_loss_threshold * 3) {
	  p1_was_absent_extended_ = true;
        }
      } else {
        // P1 just returned after extended absence — reset P4 for clean reacquisition
        if (p1_was_absent_extended_) {
	  p4_validator_.reset();
	  p4_loss_counter_ = 0;
	  p4_recovery_countdown_ = P4_RECOVERY_FRAMES;

	  if (debug_level_ >= DEBUG_CRITICAL) {
	    std::cout << "P1 returned after extended absence"
		      << " — resetting P4 for reacquisition"
		      << " (absent " << frames_without_p1_for_p4_ << " frames)"
		      << std::endl;
	  }
        }
        frames_without_p1_for_p4_ = 0;
        p1_was_absent_extended_ = false;
      }
    }

    // Diagnostic: record why P4 won't be attempted this frame
    if (detection_mode_ != MODE_FULL) {
      p4_reject_reason_ = P4_NOT_FULL_MODE;
    } else if (!result.p1_detected) {
      p4_reject_reason_ = P4_NO_P1;
    }

    if (detection_mode_ == MODE_FULL && result.p1_detected) {

      bool p4_in_recovery = (p4_recovery_countdown_ > 0);

      cv::Point2f p4_local = detectP4(blur_buffer_, pupil_center_local,
				      p1_local, pupil.radius,
				      use_roi, roi, p4_in_recovery);
      if (p4_local.x < 0) {
        p4_loss_counter_++;

        // Trigger recovery: initially after P4_LOSS_THRESHOLD,
        // then retry every P4_RETRY_INTERVAL frames using existing model
        bool initial_loss = (p4_loss_counter_ == P4_LOSS_THRESHOLD);
        bool periodic_retry = (p4_loss_counter_ > P4_LOSS_THRESHOLD &&
                               (p4_loss_counter_ % P4_RETRY_INTERVAL == 0));

        if (initial_loss || periodic_retry) {
	  p4_validator_.reset();
	  p4_recovery_countdown_ = P4_RECOVERY_FRAMES;

	  if (initial_loss) {
	    fireEvent(VstreamEvent("eyetracking/p4_lost",
				   "frame " + std::to_string(event_frame_id_)));
	    p4_was_lost_ = true;
	  }

	  if (debug_level_ >= DEBUG_CRITICAL) {
	    std::cout << "P4 " << (initial_loss ? "lost" : "retry")
		      << " - resetting validator (lost " << p4_loss_counter_
		      << " frames)" << std::endl;
	  }
        }
      } else {
        if (p4_was_lost_) {
	  fireEvent(VstreamEvent("eyetracking/p4_recovered",
				 "frame " + std::to_string(event_frame_id_)));
	  p4_was_lost_ = false;
        }

        p4_loss_counter_ = 0;
        if (p4_recovery_countdown_ > 0) {
	  p4_recovery_countdown_--;
        }
      }

      if (p4_local.x > 0) {
	cv::Point2f p4_full = p4_local;
	if (use_roi) {
	  p4_full.x += roi.x;
	  p4_full.y += roi.y;
	}

	// The old inside-pupil check (dist > 0.95*pupil_radius) rejected valid P4s
	// when the pupil constricted (bright screen): measured 77% of P4 losses on
	// coherence-sweep, all INSIDE_PUPIL, with PRED_ERROR/NO_CANDIDATE = 0. P4's
	// offset from the pupil center is fixed by eye geometry, not pupil size.
	// In full mode the model prediction + prediction-error gate already
	// constrain P4 location, so the radius check is redundant — disable it (pass
	// -1). Keep it only as a spatial fallback when there is no model prediction.
	float gate_radius = p4_model_.isInitialized() ? -1.0f : pupil.radius;

	bool is_valid =
	  p4_in_recovery ||
	  p4_validator_.isValid(p4_full, pupil.center, gate_radius);

	if (!is_valid) {
	  // Distinguish the validator's two gates for the reprocess log
	  if (gate_radius > 0 &&
	      cv::norm(p4_full - pupil.center) > gate_radius * 0.95f) {
	    p4_reject_reason_ = P4_INSIDE_PUPIL;
	  } else {
	    p4_reject_reason_ = P4_JUMP;
	  }
	  if (debug_level_ >= DEBUG_NORMAL) {
	    std::cout << "P4 rejected by validator" << std::endl;
	  }
	}

	if (is_valid) {
	  p4_reject_reason_ = P4_OK;
	  result.p4_detected = true;
	  result.p4_center = p4_full;

	  if (debug_level_ >= DEBUG_VERBOSE) {
	    std::cout << "P4 accepted (full-frame): (" << p4_full.x << "," << p4_full.y
		      << ") local: (" << p4_local.x << "," << p4_local.y << ")" << std::endl;
	  }
	  if (!p4_in_recovery) {
	    p4_validator_.update(p4_full, pupil.center);
	  }

	  // LIVE-SAFE LEARNING: detection above is permissive (the radius-scaled
	  // inside-pupil gate is gone), but online model updates are CONSERVATIVE —
	  // learn geometry only from reliable frames. Updating on constricted /
	  // off-prediction detections drifts the model and causes runaway P4 loss.
	  // Conditions: not in recovery, pupil well-dilated (near baseline), and the
	  // detection close to the model prediction.
	  if (!p4_in_recovery &&
	      p4_model_.isInitialized() && !p4_model_.isFrozen()) {
	    const float LEARN_MIN_DILATION = 0.85f;  // pupil >= 85% of dilation ref
	    const float LEARN_MAX_ERR_FRAC = 0.5f;   // within half the max pred error
	    // Dilation gate anchored to the leaky-max reference, NOT the blink
	    // baseline: the baseline tracks a sustained constriction down within
	    // seconds, which silently re-admitted constricted (noisy-center)
	    // frames into model learning — the drift the gate exists to prevent.
	    bool well_dilated = (learn_ref_radius_ > 0.0f) &&
	                        (pupil.radius >= learn_ref_radius_ * LEARN_MIN_DILATION);
	    bool close_to_prediction =
	        (p4_predicted_.x < 0.0f) ||
	        (cv::norm(p4_full - p4_predicted_) <=
	         p4_max_prediction_error_ * LEARN_MAX_ERR_FRAC);
	    if (well_dilated && close_to_prediction) {
	      p4_model_.updateModel(pupil.center, result.p1_center, p4_full);
	    }
	  }

	  // AUTO-REACQUIRE when P4 is stuck near the search-box edge.
	  // Detection accepts anything within p4_max_prediction_error_ of the
	  // prediction, but learning only nudges the model within a tighter band
	  // (and is off entirely when frozen). A P4 that persistently lands far
	  // from the prediction therefore sticks in a corner of the search box:
	  // the model bias never self-corrects, and only a blink (which resets the
	  // P4 validator) breaks it. Replicate that re-lock on a timer: when the
	  // offset stays large for several consecutive frames, reset the validator
	  // and arm recovery so the box re-acquires the true P4 — without a blink,
	  // and without touching learned geometry (safe even when frozen).
	  if (!p4_in_recovery &&
	      p4_model_.isInitialized() && p4_predicted_.x >= 0.0f) {
	    const float RECENTER_OFFSET_FRAC = 0.7f;  // fraction of max pred error
	    float pred_offset = cv::norm(p4_full - p4_predicted_);
	    if (pred_offset > p4_max_prediction_error_ * RECENTER_OFFSET_FRAC) {
	      p4_offset_streak_++;
	    } else {
	      p4_offset_streak_ = 0;
	    }
	    if (p4_offset_streak_ >= P4_LOSS_THRESHOLD) {
	      p4_validator_.reset();
	      p4_recovery_countdown_ = P4_RECOVERY_FRAMES;
	      p4_offset_streak_ = 0;
	      if (debug_level_ >= DEBUG_CRITICAL) {
		std::cout << "P4 auto-reacquire: stuck " << pred_offset
			  << "px off prediction — resetting validator to re-lock"
			  << std::endl;
	      }
	    }
	  } else {
	    p4_offset_streak_ = 0;
	  }
	}
      }
    }

    return result;
  }
  
    // ========================================================================
    // PER-FRAME PIPELINE (runs on analysis thread, or inline when synchronous_)
    // ========================================================================

    void processFrameInline(const cv::Mat& frame, int frame_idx,
                            const FrameMetadata& metadata) {
        if (frame.empty()) {
            return;
        }

        // Frame identity for events fired anywhere in this frame's pipeline.
        // On the very first frame the anchor isn't set yet (forwardResults
        // sets it below) — relative id 0 is correct for that frame either way.
        event_frame_id_ =
            (first_frameID_ < 0) ? 0 : (metadata.frameID - first_frameID_);

        ensureBuffersAllocated(frame);

        // Handle one-shot debug
        int saved_debug_level = debug_level_;
        if (debug_next_frame_.exchange(false)) {  // Atomic swap to false
          debug_level_ = DEBUG_VERBOSE;
          std::cout << "\n======== FRAME " << frame_idx
                    << " DEBUG ========" << std::endl;
        }

        auto start = std::chrono::high_resolution_clock::now();

        // Capture the ROI once and convert to grayscale once per frame; the
        // whole pipeline (pupil, P1, P4) works from this shared view.
        // detectPupil and detectPurkinje used to each re-lock, re-crop, and
        // re-convert the same region.
        cv::Rect roi;
        bool use_roi;
        {
            std::lock_guard<std::mutex> lock(roi_mutex_);
            roi = current_roi_;
            use_roi = roi_enabled_;
        }
        if (use_roi) {
            if (roi.x < 0 || roi.y < 0 ||
                roi.x + roi.width > frame.cols ||
                roi.y + roi.height > frame.rows ||
                roi.width <= 0 || roi.height <= 0) {
                use_roi = false;  // Invalid ROI, process full frame
            }
        }
        cv::Mat roi_frame = use_roi ? frame(roi) : frame;
        if (gray_buffer_.size() != roi_frame.size()) {
            gray_buffer_ = cv::Mat(roi_frame.size(), CV_8UC1);
        }
        if (roi_frame.channels() > 1) {
            cv::cvtColor(roi_frame, gray_buffer_, cv::COLOR_BGR2GRAY);
        } else {
            roi_frame.copyTo(gray_buffer_);
        }

        PupilData pupil = detectPupil(gray_buffer_, roi, use_roi);

        bool was_in_blink = blink_detector_.isInBlink();

        blink_detector_.update(pupil.detected, pupil.radius);

        // Blink transitions must be detected HERE, around the update: once a
        // blink starts, detectPurkinje is gated off entirely, so an edge
        // detector inside it can never observe in_blink==true and the events
        // silently never fire (the web UI blink indicator was dead).
        bool now_in_blink = blink_detector_.isInBlink();
        if (!was_in_blink && now_in_blink) {
            fireEvent(VstreamEvent("eyetracking/blink_start",
                                   "frame " + std::to_string(event_frame_id_)));
        } else if (was_in_blink && !now_in_blink) {
            fireEvent(VstreamEvent("eyetracking/blink_end",
                                   "frame " + std::to_string(event_frame_id_)));
        }

        if (debug_level_ >= DEBUG_NORMAL && pupil.detected) {
          std::cout << "Frame " << frame_idx
                    << ": pupil_r=" << std::fixed << std::setprecision(1) << pupil.radius
                    << " baseline=" << blink_detector_.getBaselineRadius()
                    << " ratio=" << std::setprecision(2) << (pupil.radius / blink_detector_.getBaselineRadius())
                    << " blink=" << blink_detector_.isInBlink()
                    << " recovering=" << blink_detector_.isRecovering()
                    << std::endl;
        }

        if (blink_detector_.shouldResetValidators()) {
            p1_validator_.reset();
            p4_validator_.reset();
        }

        // LOST-state bookkeeping. "No plausible pupil" for longer than any
        // real blink means the subject looked far away, closed their eyes for
        // an extended period, or left. The blink gate already suppresses
        // Purkinje detection during this time; LOST adds the correct semantic
        // label to the data/events, and on return forces the same clean
        // re-lock a blink does so P1/P4 reacquire from scratch.
        if (!pupil.detected) {
            frames_without_pupil_++;
            if (!tracking_lost_ &&
                frames_without_pupil_ >= timing_.tracking_lost_frames) {
                tracking_lost_ = true;
                fireEvent(VstreamEvent("eyetracking/tracking_lost",
                                       "frame " + std::to_string(event_frame_id_)));
                if (debug_level_ >= DEBUG_CRITICAL) {
                    std::cout << "Tracking LOST (no plausible pupil for "
                              << frames_without_pupil_ << " frames)" << std::endl;
                }
            }
        } else {
            if (tracking_lost_) {
                tracking_lost_ = false;
                // Clean re-lock, exactly as after a blink: fresh validators,
                // recovery windows armed, desperation cleared.
                p1_validator_.reset();
                p4_validator_.reset();
                p1_loss_counter_ = 0;
                p1_recovery_countdown_ = timing_.p1_recovery_frames;
                p4_loss_counter_ = 0;
                p4_recovery_countdown_ = timing_.p4_recovery_frames;
                p1_desperation_ = false;
                p1_desperation_streak_ = 0;
                p1_desperation_frames_ = 0;
                p1_desperation_cooldown_ = 0;
                fireEvent(VstreamEvent("eyetracking/tracking_recovered",
                                       "frame " + std::to_string(event_frame_id_)));
                if (debug_level_ >= DEBUG_CRITICAL) {
                    std::cout << "Tracking RECOVERED (pupil re-acquired)"
                              << std::endl;
                }
            }
            frames_without_pupil_ = 0;
        }

        // Dilation reference upkeep: decays on a wall-clock-derived schedule
        // every frame; ratchets up on clean frames only. See the member docs.
        learn_ref_radius_ *= timing_.learn_ref_decay;
        if (pupil.detected && !blink_detector_.isInBlink() &&
            !blink_detector_.isRecovering() &&
            pupil.radius > learn_ref_radius_) {
            learn_ref_radius_ = pupil.radius;
        }

        // Reset per-frame diagnostics; detectPurkinje/detectP4 set them.
        p4_reject_reason_ = P4_OK;
        p4_predicted_ = cv::Point2f(-1, -1);
        p4_candidate_found_ = false;
        p1_reject_reason_ = P1_OK;

        PurkinjeData purkinje;

        if (!blink_detector_.isInBlink() && !blink_detector_.isRecovering()) {
          purkinje = detectPurkinje(gray_buffer_, pupil, frame_idx, roi, use_roi);
        } else {
          p4_reject_reason_ = P4_BLINK;
          p1_reject_reason_ = P1_BLINK;
          if (debug_level_ >= DEBUG_NORMAL) {
            std::cout << "Frame " << frame_idx
                      << ": Skipping Purkinje detection (blink/recovery)"
                      << std::endl;
          }
        }

        {
            std::lock_guard<std::mutex> lock(results_mutex_);
            latest_results_.frame_idx = frame_idx;
            latest_results_.pupil = pupil;
            latest_results_.purkinje = purkinje;
            latest_results_.in_blink = blink_detector_.isInBlink();
            latest_results_.tracking_lost = tracking_lost_;
            latest_results_.valid = true;
            latest_results_.abs_frame_id = event_frame_id_;
            latest_results_.p4_reject_reason = p4_reject_reason_;
            latest_results_.p4_predicted = p4_predicted_;
            latest_results_.p4_candidate_found = p4_candidate_found_;
            latest_results_.p1_reject_reason = p1_reject_reason_;
        }

        // Reference-overlay agreement bookkeeping: how the current code's P4
        // compares to what was stored for this same frame. Gated on a relaxed
        // atomic so the live path (no reference loaded) pays one flag read.
        if (reference_loaded_.load(std::memory_order_relaxed)) {
            std::lock_guard<std::mutex> ref_lock(reference_mutex_);
            auto it = reference_.find(event_frame_id_);
            if (it != reference_.end()) {
                ref_frames_matched_++;
                bool ref_p4 = (it->second.has & REF_HAS_P4) != 0;
                if (ref_p4) ref_p4_ref_++;
                if (purkinje.p4_detected) ref_p4_live_++;
                if (ref_p4 && purkinje.p4_detected) ref_p4_both_++;
            }
        }

        forwardResults(frame_idx, metadata, pupil, purkinje);

        if (debug_level_ == DEBUG_VERBOSE && saved_debug_level != DEBUG_VERBOSE) {
          std::cout << "========================================\n" << std::endl;
          debug_level_ = saved_debug_level;
        }

        blink_detector_.decrementRecovery();

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        frame_count_++;

        // Profiling output - timing only
        if ((profile_flags_ & PROFILE_TIMING) && frame_count_ % 100 == 0) {
            std::cout << "[Profile] Frame " << frame_count_
                      << ": " << duration.count() << "µs" << std::endl;
        }

        // Profiling output - full details (original DEBUG_PROFILE behavior)
        if (debug_level_ >= DEBUG_PROFILE && frame_count_ % 100 == 0) {
            std::cout << "Frame " << frame_count_ << ": " << duration.count() << "µs";
            if (pupil.detected) {
                std::cout << " | Pupil: r=" << pupil.radius;
            }
            if (blink_detector_.isInBlink()) {
                std::cout << " | BLINK";
            }
            std::cout << std::endl;
        }

        // Periodic summary at normal level
        if (debug_level_ >= DEBUG_NORMAL && frame_count_ % 1000 == 0) {
            std::cout << "Processed " << frame_count_ << " frames" << std::endl;
        }
    }

    // ========================================================================
    // ANALYSIS THREAD
    // ========================================================================

    void analysisThreadFunc() {
        if (debug_level_ >= DEBUG_CRITICAL) {
            std::cout << "Eye tracking analysis thread started" << std::endl;
        }
#ifdef __linux__
    // Pin to a specific core (avoid CCX hopping on AMD)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(4, &cpuset);  // Core 4, for example
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    
    // Increase priority
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
#elif __APPLE__
    // macOS: Set thread priority (no CPU pinning API)
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    
    // Alternative: Use QoS (Quality of Service) classes
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
    
        while (running_) {
            try {
                FrameData frame_data = frame_queue_.front();
                frame_queue_.pop_front();

                if (frame_data.frame_idx < 0) {
                    break;
                }
                
                processFrameInline(frame_data.frame, frame_data.frame_idx,
                                   frame_data.metadata);
            } catch (...) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    // ========================================================================
    // TCL COMMANDS
    // ========================================================================

static int setDetectionModeCmd(ClientData clientData, Tcl_Interp *interp,
                                int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    if (objc == 1) {
        const char* mode_names[] = {"pupil_only", "pupil_p1", "full"};
        Tcl_SetObjResult(interp, Tcl_NewStringObj(
            mode_names[plugin->detection_mode_], -1));
        return TCL_OK;
    }
    
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "?pupil_only|pupil_p1|full?");
        return TCL_ERROR;
    }
    
    const char* mode_str = Tcl_GetString(objv[1]);
    DetectionMode old_mode = plugin->detection_mode_;
    
    if (strcmp(mode_str, "pupil_only") == 0) {
        plugin->detection_mode_ = MODE_PUPIL_ONLY;
    } else if (strcmp(mode_str, "pupil_p1") == 0) {
        plugin->detection_mode_ = MODE_PUPIL_P1;
    } else if (strcmp(mode_str, "full") == 0) {
        plugin->detection_mode_ = MODE_FULL;
    } else {
        Tcl_SetObjResult(interp, Tcl_NewStringObj(
            "Invalid mode. Use: pupil_only, pupil_p1, or full", -1));
        return TCL_ERROR;
    }
    
    plugin->settings_.detection_mode = std::string(mode_str);
    
    // Reset validators if downgrading
    if (plugin->detection_mode_ < old_mode) {
        if (plugin->detection_mode_ < MODE_FULL) {
            plugin->p4_validator_.reset();
            plugin->p4_model_.reset();
            plugin->p4_pending_sample_active_ = false;
            plugin->p4_loss_counter_ = 0;
            plugin->p4_recovery_countdown_ = 0;
            plugin->p4_was_lost_ = false;
            plugin->frames_without_p1_for_p4_ = 0;
            plugin->p1_was_absent_extended_ = false;
        }
        if (plugin->detection_mode_ < MODE_PUPIL_P1) {
            plugin->p1_validator_.reset();
        }
    }
    
    // Fire event
    plugin->fireSettingChanged("detection_mode", mode_str);
    
    if (plugin->debug_level_ >= DEBUG_CRITICAL) {
        const char* mode_names[] = {"PUPIL_ONLY", "PUPIL_P1", "FULL"};
        std::cout << "Detection mode changed to: " 
                  << mode_names[plugin->detection_mode_] << std::endl;
    }
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj("Mode changed", -1));
    return TCL_OK;
}

static int resetDetectionCmd(ClientData clientData, Tcl_Interp *interp,
                              int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    plugin->detection_mode_ = MODE_PUPIL_ONLY;
    plugin->p1_validator_.reset();
    plugin->p4_validator_.reset();
    plugin->p4_model_.reset();
    
    if (plugin->debug_level_ >= DEBUG_CRITICAL) {
        std::cout << "Detection reset to PUPIL_ONLY mode" << std::endl;
    }
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj("Detection reset to pupil_only", -1));
    return TCL_OK;
}

static int resetTrackingStateCmd(ClientData clientData, Tcl_Interp *interp,
                                  int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    // Check if we're currently in a blink state
    bool was_in_blink = plugin->blink_detector_.isInBlink();

    // Full reset of transient tracking state (validators, blink detector +
    // baseline, P1/P4 loss-recovery counters). Keeps detection mode and the
    // calibrated P4 model intact. This is what lets an operator recover from a
    // stuck blink/loss state without restarting the program.
    plugin->clearTrackingState();
    plugin->updateTimingThresholds();

    // If we were in a blink, fire event to clear UI indicators
    if (was_in_blink) {
      fireEvent(VstreamEvent("eyetracking/blink_end", "frame -1"));
    }
    
    if (plugin->debug_level_ >= DEBUG_CRITICAL) {
      std::cout << "Tracking state reset (validators and blink cleared)" << std::endl;
    }
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj("Tracking state reset", -1));
    return TCL_OK;
}

// Enable/disable synchronous (deterministic) analysis for offline reprocess.
// eyetracking::setSynchronous ?0|1?  -> returns current state.
static int setSynchronousCmd(ClientData clientData, Tcl_Interp *interp,
                             int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    if (objc >= 2) {
        int enable;
        if (Tcl_GetBooleanFromObj(interp, objv[1], &enable) != TCL_OK) {
            return TCL_ERROR;
        }
        plugin->synchronous_.store(enable != 0);
    }
    Tcl_SetObjResult(interp, Tcl_NewBooleanObj(plugin->synchronous_.load()));
    return TCL_OK;
}
      
    static int setROICmd(ClientData clientData, Tcl_Interp *interp,
                         int objc, Tcl_Obj *const objv[]) {
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        
        if (objc == 1) {
	  std::lock_guard<std::mutex> lock(plugin->roi_mutex_);	  
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

static int setP4PendingSampleCmd(ClientData clientData, Tcl_Interp *interp,
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
    plugin->p4_pending_sample_active_ = true;
    plugin->p4_pending_sample_position_ = cv::Point2f(x, y);
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj("P4 sample position marked", -1));
    return TCL_OK;
}

static int clearP4PendingSampleCmd(ClientData clientData, Tcl_Interp *interp,
                                    int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    plugin->p4_pending_sample_active_ = false;
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj("P4 pending sample cleared", -1));
    return TCL_OK;
}

static int acceptP4SampleCmd(ClientData clientData, Tcl_Interp *interp,
                              int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);

    if (!plugin->p4_pending_sample_active_) {
        Tcl_SetObjResult(interp, Tcl_NewStringObj(
            "No P4 sample position marked", -1));
        return TCL_ERROR;
    }

    cv::Point2f pupil_center;
    {
        std::lock_guard<std::mutex> lock(plugin->results_mutex_);

        if (!plugin->latest_results_.pupil.detected ||
            !plugin->latest_results_.purkinje.p1_detected) {
            Tcl_SetObjResult(interp, Tcl_NewStringObj(
                "Cannot add calibration sample: need valid pupil and P1", -1));
            return TCL_ERROR;
        }

        pupil_center = plugin->latest_results_.pupil.center;

        // Add the calibration sample
        plugin->p4_model_.addCalibrationSample(
            plugin->latest_results_.pupil.center,
            plugin->latest_results_.purkinje.p1_center,
            plugin->p4_pending_sample_position_
        );
    }

    // Store for display purposes
    plugin->p4_last_known_position_ = plugin->p4_pending_sample_position_;

    // Clear the pending sample
    plugin->p4_pending_sample_active_ = false;

    // Setup flow: the pupil is known-good at sample time — seed the learning
    // dilation reference from it, and follow it with the crop
    // (once per accept; no-op when the ROI is already centered).
    {
        std::lock_guard<std::mutex> lock(plugin->results_mutex_);
        if (plugin->latest_results_.pupil.radius > plugin->learn_ref_radius_) {
            plugin->learn_ref_radius_ = plugin->latest_results_.pupil.radius;
        }
    }
    if (plugin->auto_center_roi_on_p4_ && plugin->centerROIOn(pupil_center)) {
        if (plugin->debug_level_ >= DEBUG_CRITICAL) {
            std::cout << "ROI recentered on pupil ("
                      << pupil_center.x << "," << pupil_center.y << ")"
                      << std::endl;
        }
    }

    int sample_count = plugin->p4_model_.getCalibrationSampleCount();

    Tcl_SetObjResult(interp, Tcl_NewIntObj(sample_count));
    return TCL_OK;
}

// Load a recorded session's eyetracking_frames as the reference ("ghost")
// overlay. eyetracking::loadReference <path.db> -> returns frame count.
static int loadReferenceCmd(ClientData clientData, Tcl_Interp *interp,
                            int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "path.db");
        return TCL_ERROR;
    }
    const char* path = Tcl_GetString(objv[1]);

    sqlite3* db = nullptr;
    if (sqlite3_open_v2(path, &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
        std::string msg = std::string("cannot open ") + path;
        if (db) sqlite3_close(db);
        Tcl_SetObjResult(interp, Tcl_NewStringObj(msg.c_str(), -1));
        return TCL_ERROR;
    }

    const char* sql =
        "SELECT frame_number, pupil_x, pupil_y, pupil_radius, "
        "p1_x, p1_y, p4_x, p4_y, in_blink FROM eyetracking_frames";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        std::string msg = std::string("no eyetracking_frames table in ") + path;
        sqlite3_close(db);
        Tcl_SetObjResult(interp, Tcl_NewStringObj(msg.c_str(), -1));
        return TCL_ERROR;
    }

    std::unordered_map<long long, RefFrame> loaded;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        RefFrame rf{};
        long long fn = sqlite3_column_int64(stmt, 0);
        if (sqlite3_column_type(stmt, 1) != SQLITE_NULL) {
            rf.px = (float)sqlite3_column_double(stmt, 1);
            rf.py = (float)sqlite3_column_double(stmt, 2);
            rf.pr = (float)sqlite3_column_double(stmt, 3);
            rf.has |= REF_HAS_PUPIL;
        }
        if (sqlite3_column_type(stmt, 4) != SQLITE_NULL) {
            rf.p1x = (float)sqlite3_column_double(stmt, 4);
            rf.p1y = (float)sqlite3_column_double(stmt, 5);
            rf.has |= REF_HAS_P1;
        }
        if (sqlite3_column_type(stmt, 6) != SQLITE_NULL) {
            rf.p4x = (float)sqlite3_column_double(stmt, 6);
            rf.p4y = (float)sqlite3_column_double(stmt, 7);
            rf.has |= REF_HAS_P4;
        }
        rf.blink = sqlite3_column_int(stmt, 8) != 0;
        loaded.emplace(fn, rf);
    }
    sqlite3_finalize(stmt);
    sqlite3_close(db);

    int count = (int)loaded.size();
    {
        std::lock_guard<std::mutex> lock(plugin->reference_mutex_);
        plugin->reference_ = std::move(loaded);
        plugin->ref_frames_matched_ = 0;
        plugin->ref_p4_ref_ = 0;
        plugin->ref_p4_live_ = 0;
        plugin->ref_p4_both_ = 0;
    }
    plugin->show_reference_ = true;
    plugin->reference_loaded_.store(count > 0);

    Tcl_SetObjResult(interp, Tcl_NewIntObj(count));
    return TCL_OK;
}

static int clearReferenceCmd(ClientData clientData, Tcl_Interp *interp,
                             int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    std::lock_guard<std::mutex> lock(plugin->reference_mutex_);
    plugin->reference_.clear();
    plugin->reference_loaded_.store(false);
    plugin->ref_frames_matched_ = 0;
    plugin->ref_p4_ref_ = 0;
    plugin->ref_p4_live_ = 0;
    plugin->ref_p4_both_ = 0;
    Tcl_SetObjResult(interp, Tcl_NewStringObj("reference cleared", -1));
    return TCL_OK;
}

// eyetracking::toggleReference ?0|1? -> show/hide ghosts (state retained)
static int toggleReferenceCmd(ClientData clientData, Tcl_Interp *interp,
                              int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    if (objc == 1) {
        plugin->show_reference_ = !plugin->show_reference_;
    } else {
        int value;
        if (Tcl_GetBooleanFromObj(interp, objv[1], &value) != TCL_OK) {
            return TCL_ERROR;
        }
        plugin->show_reference_ = (value != 0);
    }
    Tcl_SetObjResult(interp, Tcl_NewBooleanObj(plugin->show_reference_));
    return TCL_OK;
}

// eyetracking::referenceStats -> dict of running agreement counters
static int referenceStatsCmd(ClientData clientData, Tcl_Interp *interp,
                             int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    std::lock_guard<std::mutex> lock(plugin->reference_mutex_);
    Tcl_Obj* d = Tcl_NewDictObj();
    Tcl_DictObjPut(interp, d, Tcl_NewStringObj("loaded", -1),
                   Tcl_NewWideIntObj((Tcl_WideInt)plugin->reference_.size()));
    Tcl_DictObjPut(interp, d, Tcl_NewStringObj("frames", -1),
                   Tcl_NewWideIntObj(plugin->ref_frames_matched_));
    Tcl_DictObjPut(interp, d, Tcl_NewStringObj("p4_ref", -1),
                   Tcl_NewWideIntObj(plugin->ref_p4_ref_));
    Tcl_DictObjPut(interp, d, Tcl_NewStringObj("p4_live", -1),
                   Tcl_NewWideIntObj(plugin->ref_p4_live_));
    Tcl_DictObjPut(interp, d, Tcl_NewStringObj("p4_both", -1),
                   Tcl_NewWideIntObj(plugin->ref_p4_both_));
    Tcl_SetObjResult(interp, d);
    return TCL_OK;
}

// Recenter the ROI on the current pupil position (size unchanged, clamped to
// the frame). eyetracking::centerROI -> returns the new {x y w h}.
static int centerROICmd(ClientData clientData, Tcl_Interp *interp,
                        int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);

    cv::Point2f pupil_center;
    {
        std::lock_guard<std::mutex> lock(plugin->results_mutex_);
        if (!plugin->latest_results_.valid ||
            !plugin->latest_results_.pupil.detected) {
            Tcl_SetObjResult(interp, Tcl_NewStringObj(
                "Cannot center ROI: no pupil detected", -1));
            return TCL_ERROR;
        }
        pupil_center = plugin->latest_results_.pupil.center;
    }

    if (!plugin->centerROIOn(pupil_center)) {
        Tcl_SetObjResult(interp, Tcl_NewStringObj(
            "Cannot center ROI: ROI not enabled", -1));
        return TCL_ERROR;
    }

    std::lock_guard<std::mutex> lock(plugin->roi_mutex_);
    Tcl_Obj* result = Tcl_NewListObj(0, nullptr);
    Tcl_ListObjAppendElement(interp, result, Tcl_NewIntObj(plugin->current_roi_.x));
    Tcl_ListObjAppendElement(interp, result, Tcl_NewIntObj(plugin->current_roi_.y));
    Tcl_ListObjAppendElement(interp, result, Tcl_NewIntObj(plugin->current_roi_.width));
    Tcl_ListObjAppendElement(interp, result, Tcl_NewIntObj(plugin->current_roi_.height));
    Tcl_SetObjResult(interp, result);
    return TCL_OK;
}

// eyetracking::autoCenterROI ?0|1? — recenter the ROI automatically when a P4
// calibration sample is accepted. Returns current state.
static int autoCenterROICmd(ClientData clientData, Tcl_Interp *interp,
                            int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    if (objc >= 2) {
        int enable;
        if (Tcl_GetBooleanFromObj(interp, objv[1], &enable) != TCL_OK) {
            return TCL_ERROR;
        }
        plugin->auto_center_roi_on_p4_ = (enable != 0);
    }
    Tcl_SetObjResult(interp, Tcl_NewBooleanObj(plugin->auto_center_roi_on_p4_));
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
    
  static int calibrateP4ModelCmd(ClientData clientData, Tcl_Interp *interp,
				 int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);

    plugin->updateTimingThresholds();

    // If model already initialized, reset it so we can recalibrate
    if (plugin->p4_model_.isInitialized()) {
      plugin->p4_model_.reset();
    }

    // Check if we have samples
    int sample_count = plugin->p4_model_.getCalibrationSampleCount();
    if (sample_count < 1) {
      Tcl_SetObjResult(interp, Tcl_NewStringObj(
						"No calibration samples available. Add samples first.", -1));
      return TCL_ERROR;
    }
    
    // Attempt calibration
    if (plugin->p4_model_.calibrateFromSamples()) {
      std::ostringstream data;
      data << "samples " << plugin->p4_model_.getCalibrationSampleCount()
	   << " magnitude " << plugin->p4_model_.getMagnitudeRatio()
	   << " angle " << (plugin->p4_model_.getAngleOffset() * 180.0 / M_PI);
      
      // Clear loss/recovery tracking so fresh model starts clean
      plugin->p4_loss_counter_ = 0;
      plugin->p4_recovery_countdown_ = 0;
      plugin->p4_was_lost_ = false;
      plugin->p4_validator_.reset();

      fireEvent(VstreamEvent("eyetracking/p4_calibrated", data.str()));
      return TCL_OK;
    }
    else {
      Tcl_SetObjResult(interp, Tcl_NewStringObj(
						"Calibration failed. Check sample quality.", -1));
      return TCL_ERROR;
    }
  }

  static int resetP4ModelCmd(ClientData clientData, Tcl_Interp *interp,
			     int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    plugin->p4_model_.reset();
    plugin->p4_validator_.reset();
    plugin->p4_pending_sample_active_ = false;

    // Clear loss/recovery tracking so we start fresh
    plugin->p4_loss_counter_ = 0;
    plugin->p4_recovery_countdown_ = 0;
    plugin->p4_was_lost_ = false;
    plugin->frames_without_p1_for_p4_ = 0;
    plugin->p1_was_absent_extended_ = false;

    Tcl_SetObjResult(interp, Tcl_NewStringObj("P4 model and validator reset", -1));
    return TCL_OK;
  }

  static int setP4ModelCmd(ClientData clientData, Tcl_Interp *interp,
                           int objc, Tcl_Obj *const objv[]) {
    if (objc != 3) {
      Tcl_WrongNumArgs(interp, 1, objv, "magnitude_ratio angle_offset_degrees");
      return TCL_ERROR;
    }
    
    double mag_ratio, angle_deg;
    if (Tcl_GetDoubleFromObj(interp, objv[1], &mag_ratio) != TCL_OK ||
        Tcl_GetDoubleFromObj(interp, objv[2], &angle_deg) != TCL_OK) {
      return TCL_ERROR;
    }
    
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    // Directly set the model parameters
    plugin->p4_model_.setParameters(mag_ratio, angle_deg * M_PI / 180.0);
    
    std::ostringstream msg;
    msg << "P4 model set: magnitude=" << mag_ratio 
        << " angle=" << angle_deg << "°";
    
    if (plugin->debug_level_ >= DEBUG_CRITICAL) {
      std::cout << msg.str() << std::endl;
    }
    
    std::ostringstream data;
    data << "magnitude " << mag_ratio << " angle " << angle_deg;
    fireEvent(VstreamEvent("eyetracking/p4_model_set", data.str()));
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj(msg.str().c_str(), -1));
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
    
    if (plugin->p4_model_.isInitialized()) {
        Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("magnitude_ratio", -1),
                      Tcl_NewDoubleObj(plugin->p4_model_.getMagnitudeRatio()));
        Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("angle_offset_deg", -1),
                      Tcl_NewDoubleObj(plugin->p4_model_.getAngleOffset() * 180.0 / M_PI));
    }
    // What the learning gate currently considers "dilated" (leaky max, px)
    Tcl_DictObjPut(interp, statusDict, Tcl_NewStringObj("learn_ref_radius", -1),
                  Tcl_NewDoubleObj(plugin->learn_ref_radius_));
    
    Tcl_SetObjResult(interp, statusDict);
    return TCL_OK;
}

static int setPupilThresholdCmd(ClientData clientData, Tcl_Interp *interp,
                                int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    if (objc > 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "?threshold?");
        return TCL_ERROR;
    }
    
    int last_threshold = plugin->settings_.pupil_threshold;
    
    if (objc > 1) {
        int threshold;
        if (Tcl_GetIntFromObj(interp, objv[1], &threshold) != TCL_OK) {
            return TCL_ERROR;
        }
        plugin->pupil_threshold_ = threshold;
        plugin->settings_.pupil_threshold = threshold;
        
        // Fire event for all listeners (web UI, widgets, etc.)
        plugin->fireSettingChanged("pupil_threshold", std::to_string(threshold));
    }
    
    Tcl_SetObjResult(interp, Tcl_NewIntObj(last_threshold));
    return TCL_OK;
}

// Pupil plausibility gate parameters.
// eyetracking::setPupilGate ?min_area max_area min_extent max_aspect?
//   -> returns current {min_area max_area min_extent max_aspect}
static int setPupilGateCmd(ClientData clientData, Tcl_Interp *interp,
                           int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);

    if (objc != 1 && objc != 5) {
        Tcl_WrongNumArgs(interp, 1, objv, "?min_area max_area min_extent max_aspect?");
        return TCL_ERROR;
    }

    if (objc == 5) {
        double min_area, max_area, min_extent, max_aspect;
        if (Tcl_GetDoubleFromObj(interp, objv[1], &min_area) != TCL_OK ||
            Tcl_GetDoubleFromObj(interp, objv[2], &max_area) != TCL_OK ||
            Tcl_GetDoubleFromObj(interp, objv[3], &min_extent) != TCL_OK ||
            Tcl_GetDoubleFromObj(interp, objv[4], &max_aspect) != TCL_OK) {
            return TCL_ERROR;
        }
        plugin->pupil_min_area_ = min_area;
        plugin->pupil_max_area_ = max_area;
        plugin->pupil_min_extent_ = min_extent;
        plugin->pupil_max_aspect_ = max_aspect;
    }

    Tcl_Obj* result = Tcl_NewListObj(0, nullptr);
    Tcl_ListObjAppendElement(interp, result, Tcl_NewDoubleObj(plugin->pupil_min_area_));
    Tcl_ListObjAppendElement(interp, result, Tcl_NewDoubleObj(plugin->pupil_max_area_));
    Tcl_ListObjAppendElement(interp, result, Tcl_NewDoubleObj(plugin->pupil_min_extent_));
    Tcl_ListObjAppendElement(interp, result, Tcl_NewDoubleObj(plugin->pupil_max_aspect_));
    Tcl_SetObjResult(interp, result);
    return TCL_OK;
}

static int setP1MinAreaCmd(ClientData clientData, Tcl_Interp *interp,
			   int objc, Tcl_Obj *const objv[]) {
  EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
  
  if (objc == 1) {
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(plugin->p1_min_area_));
    return TCL_OK;
  }
  
  double area;
  if (Tcl_GetDoubleFromObj(interp, objv[1], &area) != TCL_OK) {
    return TCL_ERROR;
  }
  
  plugin->p1_min_area_ = area;
  Tcl_SetObjResult(interp, Tcl_NewDoubleObj(area));
  return TCL_OK;
}


static int setP1MaxAreaCmd(ClientData clientData, Tcl_Interp *interp,
			   int objc, Tcl_Obj *const objv[]) {
  EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
  
  if (objc == 1) {
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(plugin->p1_max_area_));
    return TCL_OK;
  }
  
  double area;
  if (Tcl_GetDoubleFromObj(interp, objv[1], &area) != TCL_OK) {
    return TCL_ERROR;
  }
  
  plugin->p1_max_area_ = area;
  Tcl_SetObjResult(interp, Tcl_NewDoubleObj(area));
  return TCL_OK;
}  


static int setP4SearchSizeCmd(ClientData clientData, Tcl_Interp *interp,
                              int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    if (objc > 3) {
        Tcl_WrongNumArgs(interp, 1, objv, "?width? ?height?");
        return TCL_ERROR;
    }
    
    // Return current value
    if (objc == 1) {
        Tcl_Obj* result = Tcl_NewListObj(0, NULL);
        Tcl_ListObjAppendElement(interp, result, 
            Tcl_NewIntObj(plugin->p4_search_roi_size_.width));
        Tcl_ListObjAppendElement(interp, result, 
            Tcl_NewIntObj(plugin->p4_search_roi_size_.height));
        Tcl_SetObjResult(interp, result);
        return TCL_OK;
    }
    
    // Set new value
    int width, height;
    if (Tcl_GetIntFromObj(interp, objv[1], &width) != TCL_OK) {
        return TCL_ERROR;
    }
    if (objc == 3) {
        if (Tcl_GetIntFromObj(interp, objv[2], &height) != TCL_OK) {
            return TCL_ERROR;
        }
    } else {
        height = width;  // Square if only one dimension given
    }
    
    // Validate reasonable bounds
    if (width < 10 || width > 200 || height < 10 || height > 200) {
        Tcl_SetResult(interp, "Search size must be between 10 and 200 pixels", TCL_STATIC);
        return TCL_ERROR;
    }
    
    plugin->p4_search_roi_size_ = cv::Size(width, height);
    plugin->settings_.p4_search_width = width;
    plugin->settings_.p4_search_height = height;
    
    // Fire event for UI updates
    std::string value = std::to_string(width) + "x" + std::to_string(height);
    plugin->fireSettingChanged("p4_search_size", value);
    
    Tcl_SetObjResult(interp, Tcl_NewIntObj(width));
    return TCL_OK;
}
  
static int setP4MaxPredictionErrorCmd(ClientData clientData, Tcl_Interp *interp,
                                       int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    if (objc == 1) {
        Tcl_SetObjResult(interp, Tcl_NewDoubleObj(plugin->p4_max_prediction_error_));
        return TCL_OK;
    }
    
    double error;
    if (Tcl_GetDoubleFromObj(interp, objv[1], &error) != TCL_OK) {
        return TCL_ERROR;
    }

    plugin->p4_max_prediction_error_ = error;
    plugin->settings_.p4_max_prediction_error = error;

    // Fire event
    plugin->fireSettingChanged("p4_max_prediction_error", std::to_string(error));

    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(error));
    return TCL_OK;
}

// Fraction of the pupil radius the P4 search is confined to (P4 must be inside
// the pupil). eyetracking::setP4PupilMargin ?frac?  -> returns current.
static int setP4PupilMarginCmd(ClientData clientData, Tcl_Interp *interp,
                               int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    if (objc >= 2) {
        double m;
        if (Tcl_GetDoubleFromObj(interp, objv[1], &m) != TCL_OK) return TCL_ERROR;
        plugin->p4_pupil_search_margin_ = (float)m;
    }
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(plugin->p4_pupil_search_margin_));
    return TCL_OK;
}

static int setP1PupilRadiusMaxCmd(ClientData clientData, Tcl_Interp *interp,
				  int objc, Tcl_Obj *const objv[]) {
  EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);

  if (objc == 1) {
    Tcl_SetObjResult(interp,
		     Tcl_NewDoubleObj(plugin->settings_.p1_pupil_radius_max));
    return TCL_OK;
  }
  
  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "radius_multiplier");
    return TCL_ERROR;
  }
  
  double radius_max;
  if (Tcl_GetDoubleFromObj(interp, objv[1], &radius_max) != TCL_OK) {
    return TCL_ERROR;
  }
    
  plugin->p1_pupil_radius_max_ = radius_max;
  plugin->settings_.p1_pupil_radius_max = radius_max;
    
  // Fire event
  plugin->fireSettingChanged("p1_pupil_radius_max",
			     std::to_string(radius_max));
  
  Tcl_SetObjResult(interp, Tcl_NewDoubleObj(radius_max));
  return TCL_OK;
}
  
static int setP1MinIntensityCmd(ClientData clientData, Tcl_Interp *interp,
                                 int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);

    if (objc == 1) {
        Tcl_SetObjResult(interp,
			 Tcl_NewIntObj(plugin->settings_.p1_min_intensity));
        return TCL_OK;
    }
    
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "intensity");
        return TCL_ERROR;
    }
    
    int intensity;
    if (Tcl_GetIntFromObj(interp, objv[1], &intensity) != TCL_OK) {
        return TCL_ERROR;
    }
    
    plugin->p1_min_intensity_ = intensity;
    plugin->settings_.p1_min_intensity = intensity;
    
    // Fire event
    plugin->fireSettingChanged("p1_min_intensity", std::to_string(intensity));
    
    Tcl_SetObjResult(interp, Tcl_NewIntObj(intensity));
    return TCL_OK;
}
  
static int setP4MinIntensityCmd(ClientData clientData, Tcl_Interp *interp,
                                 int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);

    int old_intensity = plugin->p4_min_intensity_;
    
    if (objc == 1) {
        Tcl_SetObjResult(interp, Tcl_NewIntObj(plugin->settings_.p4_min_intensity));
        return TCL_OK;
    }
    
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "intensity");
        return TCL_ERROR;
    }
    
    int intensity;
    if (Tcl_GetIntFromObj(interp, objv[1], &intensity) != TCL_OK) {
        return TCL_ERROR;
    }
    
    plugin->p4_min_intensity_ = intensity;
    plugin->settings_.p4_min_intensity = intensity;
    
    // Fire event
    plugin->fireSettingChanged("p4_min_intensity", std::to_string(intensity));
    
    Tcl_SetObjResult(interp, Tcl_NewIntObj(old_intensity));
    return TCL_OK;
}

static int setP1MaxJumpCmd(ClientData clientData, Tcl_Interp *interp,
                            int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    if (objc == 1) {
        // User units: px per frame at the 250 Hz reference rate
        Tcl_SetObjResult(interp, Tcl_NewDoubleObj(plugin->settings_.p1_max_jump));
        return TCL_OK;
    }

    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "?pixels?");
        return TCL_ERROR;
    }

    double pixels;
    if (Tcl_GetDoubleFromObj(interp, objv[1], &pixels) != TCL_OK) {
        return TCL_ERROR;
    }

    plugin->p1_validator_.setMaxJump(pixels * plugin->timing_.jump_scale);
    plugin->settings_.p1_max_jump = pixels;
    
    // Fire event
    plugin->fireSettingChanged("p1_max_jump", std::to_string(pixels));
    
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(pixels));
    return TCL_OK;
}

static int setP4MaxJumpCmd(ClientData clientData, Tcl_Interp *interp,
                            int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    if (objc == 1) {
        // User units: px per frame at the 250 Hz reference rate
        Tcl_SetObjResult(interp, Tcl_NewDoubleObj(plugin->settings_.p4_max_jump));
        return TCL_OK;
    }

    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "?pixels?");
        return TCL_ERROR;
    }

    double pixels;
    if (Tcl_GetDoubleFromObj(interp, objv[1], &pixels) != TCL_OK) {
        return TCL_ERROR;
    }

    plugin->p4_validator_.setMaxJump(pixels * plugin->timing_.jump_scale);
    plugin->settings_.p4_max_jump = pixels;
    
    // Fire event
    plugin->fireSettingChanged("p4_max_jump", std::to_string(pixels));
    
    Tcl_SetObjResult(interp, Tcl_NewDoubleObj(pixels));
    return TCL_OK;
}  

  static int debugNextFrameCmd(ClientData clientData, Tcl_Interp *interp,
                              int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    plugin->debug_next_frame_.store(true);
    
    Tcl_SetObjResult(interp, Tcl_NewStringObj(
        "Next frame will be analyzed with verbose debug output", -1));
    return TCL_OK;
}
  
    static int setDebugLevelCmd(ClientData clientData, Tcl_Interp *interp,
                                 int objc, Tcl_Obj *const objv[]) {
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        
        if (objc == 1) {
            // Return current level
            Tcl_SetObjResult(interp, Tcl_NewIntObj(plugin->debug_level_));
            return TCL_OK;
        }
        
        if (objc != 2) {
            Tcl_WrongNumArgs(interp, 1, objv, "?level?");
            return TCL_ERROR;
        }
        
        int level;
        if (Tcl_GetIntFromObj(interp, objv[1], &level) != TCL_OK) {
            return TCL_ERROR;
        }
        
        plugin->debug_level_ = std::max(0, std::min(4, level));
        
        const char* level_names[] = {"SILENT", "CRITICAL", "NORMAL", "VERBOSE", "PROFILE"};
        std::string msg = "Debug level set to " + std::to_string(plugin->debug_level_) + 
                         " (" + level_names[plugin->debug_level_] + ")";
        Tcl_SetObjResult(interp, Tcl_NewStringObj(msg.c_str(), -1));
        return TCL_OK;
    }
    
    static int enableProfilingCmd(ClientData clientData, Tcl_Interp *interp,
                                   int objc, Tcl_Obj *const objv[]) {
        EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
        
        if (objc == 1) {
            // Return current flags
            Tcl_SetObjResult(interp, Tcl_NewIntObj(plugin->profile_flags_));
            return TCL_OK;
        }
        
        if (objc != 2) {
            Tcl_WrongNumArgs(interp, 1, objv, "?0|1?");
            return TCL_ERROR;
        }
        
        int enable;
        if (Tcl_GetIntFromObj(interp, objv[1], &enable) != TCL_OK) {
            return TCL_ERROR;
        }
        
        if (enable) {
            plugin->profile_flags_ |= PROFILE_TIMING;
            Tcl_SetObjResult(interp, Tcl_NewStringObj("Frame timing enabled", -1));
        } else {
            plugin->profile_flags_ &= ~PROFILE_TIMING;
            Tcl_SetObjResult(interp, Tcl_NewStringObj("Frame timing disabled", -1));
        }
        
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

static int getSettingsCmd(ClientData clientData, Tcl_Interp *interp,
                          int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    Tcl_Obj* settingsDict = Tcl_NewDictObj();
    
    Tcl_DictObjPut(interp, settingsDict, 
                   Tcl_NewStringObj("pupil_threshold", -1),
                   Tcl_NewIntObj(plugin->settings_.pupil_threshold));
    
    Tcl_DictObjPut(interp, settingsDict,
                   Tcl_NewStringObj("p1_max_jump", -1),
                   Tcl_NewDoubleObj(plugin->settings_.p1_max_jump));
    
    Tcl_DictObjPut(interp, settingsDict,
                   Tcl_NewStringObj("p1_min_intensity", -1),
                   Tcl_NewIntObj(plugin->settings_.p1_min_intensity));
    
    Tcl_DictObjPut(interp, settingsDict,
                   Tcl_NewStringObj("p1_pupil_radius_max", -1),
                   Tcl_NewDoubleObj(plugin->settings_.p1_pupil_radius_max));
    
    Tcl_DictObjPut(interp, settingsDict,
                   Tcl_NewStringObj("p4_max_jump", -1),
                   Tcl_NewDoubleObj(plugin->settings_.p4_max_jump));
    
    Tcl_DictObjPut(interp, settingsDict,
                   Tcl_NewStringObj("p4_min_intensity", -1),
                   Tcl_NewIntObj(plugin->settings_.p4_min_intensity));
    
    Tcl_DictObjPut(interp, settingsDict,
                   Tcl_NewStringObj("p4_max_prediction_error", -1),
                   Tcl_NewDoubleObj(plugin->settings_.p4_max_prediction_error));
    
    Tcl_DictObjPut(interp, settingsDict,
                   Tcl_NewStringObj("detection_mode", -1),
                   Tcl_NewStringObj(plugin->settings_.detection_mode.c_str(), -1));
    
    Tcl_SetObjResult(interp, settingsDict);
    return TCL_OK;
}

static int refreshSettingsCmd(ClientData clientData, Tcl_Interp *interp,
                                         int objc, Tcl_Obj *const objv[]) {
  EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
  // Fire events for all current settings
  plugin->fireAllSettings();
  
  return TCL_OK;
}

  static int toggleInsetsCmd(ClientData clientData, Tcl_Interp *interp,
			     int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);
    
    if (objc == 1) {
      // Toggle
      plugin->show_insets_ = !plugin->show_insets_;
    } else if (objc == 2) {
      // Set explicitly
      int value;
      if (Tcl_GetIntFromObj(interp, objv[1], &value) != TCL_OK) {
	return TCL_ERROR;
      }
      plugin->show_insets_ = (value != 0);
    } else {
      Tcl_WrongNumArgs(interp, 1, objv, "?0|1?");
      return TCL_ERROR;
    }
    
    const char* status = plugin->show_insets_ ? "enabled" : "disabled";
    Tcl_SetObjResult(interp, Tcl_NewStringObj(status, -1));
    return TCL_OK;
  }

  // ::eyetracking::focusMode            -> cycle off -> annotated -> clean -> off
  // ::eyetracking::focusMode 0|1|2      -> set explicitly (off/annotated/clean)
  static int focusModeCmd(ClientData clientData, Tcl_Interp *interp,
			  int objc, Tcl_Obj *const objv[]) {
    EyeTrackingPlugin* plugin = static_cast<EyeTrackingPlugin*>(clientData);

    int prev = plugin->focus_mode_;
    if (objc == 1) {
      // Cycle through the three states.
      plugin->focus_mode_ = (prev + 1) % 3;
    } else if (objc == 2) {
      int value;
      if (Tcl_GetIntFromObj(interp, objv[1], &value) != TCL_OK) {
	return TCL_ERROR;
      }
      if (value < 0) value = 0;
      if (value > 2) value = 2;
      plugin->focus_mode_ = value;
    } else {
      Tcl_WrongNumArgs(interp, 1, objv, "?0|1|2?");
      return TCL_ERROR;
    }

    // Reset the sharpness history/best whenever we transition from OFF into an
    // active state, so the running peak reflects only the current focus session.
    if (prev == FOCUS_OFF && plugin->focus_mode_ != FOCUS_OFF) {
      plugin->focus_history_.clear();
      plugin->focus_best_ = 0.0f;
    }

    const char* status = (plugin->focus_mode_ == FOCUS_ANNOTATED) ? "annotated"
		       : (plugin->focus_mode_ == FOCUS_CLEAN)     ? "clean"
								  : "off";
    Tcl_SetObjResult(interp, Tcl_NewStringObj(status, -1));
    return TCL_OK;
  }

public:
    EyeTrackingPlugin() 
        : running_(false),
	  detection_mode_(MODE_PUPIL_P1),
          roi_enabled_(false),
          pupil_threshold_(45),
          p1_validator_(15.0f, 3),
	  p1_pupil_radius_max_(1.5),
          p1_min_intensity_(140),
	  p1_min_area_(40.0f),
	  p1_max_area_(600.0f),
          p1_centroid_roi_size_(cv::Size(19, 19)),
          p4_validator_(20.0f),
          p4_model_(),
          p4_search_roi_size_(cv::Size(50, 50)),
          p4_max_prediction_error_(13.0f),
          p4_min_intensity_(140),
	  p4_pending_sample_active_(false),
	  p4_pending_sample_position_(-1, -1),
          frame_count_(0),
          buffers_initialized_(false),
	  show_insets_(false),
	  focus_mode_(FOCUS_OFF),
          debug_level_(DEBUG_CRITICAL),
	  debug_next_frame_(false),	  
          profile_flags_(PROFILE_NONE),
          frame_source_(nullptr) {
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

  // ========================================================================
  // FRAME TIMING
  // ========================================================================
  
  float getCurrentFrameRate() const {
    if (g_sourceManager) {
      IFrameSource* source = g_sourceManager->getCurrentSource();
      if (source && source->isOpen()) {
	float fps = source->getFrameRate();
	if (fps > 0) {
	  return fps;
	}
      }
    }
    return 250.0f;  // Default fallback if source unavailable
  }
  
  // Recalculate timing thresholds based on current frame rate
  void updateTimingThresholds() {
    float fps = getCurrentFrameRate();
    
    if (debug_level_ >= DEBUG_CRITICAL) {
      std::cout << "Updating timing for " << fps << " Hz" << std::endl;
    }
    
    timing_.calculateFromFrameRate(fps);

    // Recreate BlinkDetector with new timing
    blink_detector_ = BlinkDetector(timing_.blink_recovery_frames,
				    timing_.baseline_update_frames,
				    timing_.long_blink_reset_frames);

    // Per-frame jump limits scale with frame time: the user-facing settings
    // are in "px per frame at 250 Hz" (the rate they were tuned at); the
    // validators get the equivalent for the actual rate.
    p1_validator_.setMaxJump(settings_.p1_max_jump * timing_.jump_scale);
    p4_validator_.setMaxJump(settings_.p4_max_jump * timing_.jump_scale);

    if (debug_level_ >= DEBUG_CRITICAL) {
      std::cout << "   P1 loss: " << timing_.p1_loss_threshold << " frames" << std::endl;
      std::cout << "   P1 recovery: " << timing_.p1_recovery_frames << " frames" << std::endl;
      std::cout << "   P1 relocation: " << timing_.p1_relocation_frames << " frames" << std::endl;
      std::cout << "   Blink recovery: " << timing_.blink_recovery_frames << " frames" << std::endl;
      std::cout << "   Baseline update: " << timing_.baseline_update_frames << " frames" << std::endl;
      if (timing_.jump_scale != 1.0f) {
        std::cout << "   Jump scale: " << timing_.jump_scale
                  << " (max_jump settings are px/frame at 250 Hz)" << std::endl;
      }
    }
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

  // Clear ALL transient tracking state so a "reset" behaves like a fresh start
  // (validators, blink detector + baseline, P1/P4 loss-recovery counters).
  // Does NOT touch the calibrated P4 model or detection mode.
  void clearTrackingState() {
    p1_validator_.reset();
    p4_validator_.reset();
    blink_detector_.reset();
    p1_loss_counter_ = 0;
    p1_recovery_countdown_ = 0;
    p1_was_lost_ = false;
    p1_desperation_ = false;
    p1_desperation_streak_ = 0;
    p1_desperation_last_pos_ = cv::Point2f(-1, -1);
    p1_desperation_frames_ = 0;
    p1_desperation_cooldown_ = 0;
    tracking_lost_ = false;
    frames_without_pupil_ = 0;
    p4_loss_counter_ = 0;
    p4_recovery_countdown_ = 0;
    p4_was_lost_ = false;
    p4_offset_streak_ = 0;
    frames_without_p1_for_p4_ = 0;
    p1_was_absent_extended_ = false;
  }

  void reset() override {
    first_frameID_ = -1;
    first_timestamp_ = -1;
    // New session: forget the dilation reference (deliberately NOT part of
    // clearTrackingState — an operator resetTrackingState mid-session keeps
    // it, like the calibrated model).
    learn_ref_radius_ = 0.0f;
    clearTrackingState();
  }


  void fileOpen(const std::string& filename) override {
    reset();
  }
  
    void analyzeFrame(const cv::Mat& frame, int frameIdx,
                     const FrameMetadata& metadata) override {
        // Deterministic offline path: run the pipeline inline so this frame's
        // result is ready before the caller stores it (no queue, no reordering).
        if (synchronous_.load()) {
            processFrameInline(frame, frameIdx, metadata);
            return;
        }

        FrameData frame_data;
        frame_data.frame = frame.clone();
        frame_data.frame_idx = frameIdx;
        frame_data.metadata = metadata;

        frame_queue_.push_back(frame_data);
    }
    
    void registerTclCommands(Tcl_Interp* interp) override {
        Tcl_Eval(interp, "namespace eval ::eyetracking {}");
	
	Tcl_CreateObjCommand(interp, "eyetracking::refreshSettings",
			     refreshSettingsCmd, this, NULL);
	Tcl_CreateObjCommand(interp, "::eyetracking::getSettings",
			     getSettingsCmd, this, NULL);
 
        Tcl_CreateObjCommand(interp, "::eyetracking::setDetectionMode", 
                            setDetectionModeCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::resetDetection", 
                            resetDetectionCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::resetTrackingState",
                            resetTrackingStateCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setSynchronous",
                            setSynchronousCmd, this, NULL);

        Tcl_CreateObjCommand(interp, "::eyetracking::setPupilThreshold",
                            setPupilThresholdCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setPupilGate",
                            setPupilGateCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setROI", 
                            setROICmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::disableROI",
                            disableROICmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::centerROI",
                            centerROICmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::autoCenterROI",
                            autoCenterROICmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::loadReference",
                            loadReferenceCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::clearReference",
                            clearReferenceCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::toggleReference",
                            toggleReferenceCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::referenceStats",
                            referenceStatsCmd, this, NULL);

	Tcl_CreateObjCommand(interp, "::eyetracking::markP4Sample", 
			     setP4PendingSampleCmd, this, NULL);
	Tcl_CreateObjCommand(interp, "::eyetracking::clearP4Sample", 
			     clearP4PendingSampleCmd, this, NULL);
	Tcl_CreateObjCommand(interp, "::eyetracking::acceptP4Sample", 
			     acceptP4SampleCmd, this, NULL);

	Tcl_CreateObjCommand(interp, "::eyetracking::calibrateP4Model", 
			     calibrateP4ModelCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::freezeP4Model", 
                            freezeP4ModelCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::resetP4Model", 
                            resetP4ModelCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setP4Model", 
                            setP4ModelCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::getP4ModelStatus", 
                            getP4ModelStatusCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setP1PupilRadiusMax", 
                            setP1PupilRadiusMaxCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setP4MaxPredictionError",
                            setP4MaxPredictionErrorCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setP4PupilMargin",
                            setP4PupilMarginCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setP1MinIntensity", 
                            setP1MinIntensityCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setP1MinArea", 
                            setP1MinAreaCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setP1MaxArea", 
                            setP1MaxAreaCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setP4MinIntensity", 
                            setP4MinIntensityCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setP1MaxJump", 
                            setP1MaxJumpCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::setP4MaxJump", 
			     setP4MaxJumpCmd, this, NULL);
	
	Tcl_CreateObjCommand(interp, "::eyetracking::debugNextFrame", 
			     debugNextFrameCmd, this, NULL);	
        Tcl_CreateObjCommand(interp, "::eyetracking::setDebugLevel", 
			     setDebugLevelCmd, this, NULL);
        Tcl_CreateObjCommand(interp, "::eyetracking::enableProfiling", 
			     enableProfilingCmd, this, NULL);
	Tcl_CreateObjCommand(interp, "::eyetracking::toggleInsets",
			     toggleInsetsCmd, this, NULL);
	Tcl_CreateObjCommand(interp, "::eyetracking::focusMode",
			     focusModeCmd, this, NULL);

	Tcl_CreateObjCommand(interp, "::eyetracking::getResults",
			     getResultsCmd, this, NULL);
    }
  
  const char* getName() override { return "eye_tracking"; }
  const char* getVersion() override { return "1.0"; }
  const char* getDescription() override { 
    return "Pupil and Purkinje reflection tracking with bright spot detection"; 
  }
  
  bool hasWebUI() const override { 
    return true; 
  }
  

std::string getUIHTML() const override {
    return R"EYETRACK_HTML(
<div class="control-section">
    <h3>Detection Mode</h3>
    <div class="control-row">
        <div class="input-group">
            <label>Mode</label>
            <select id="et-detection-mode">
                <option value="pupil_only">Pupil Only</option>
                <option value="pupil_p1" selected>Pupil + P1</option>
                <option value="full">Full (P1+P4)</option>
            </select>
        </div>
        
        <div class="button-group" style="align-self: flex-end;">
            <button onclick="etApplyMode()">Apply Mode</button>
            <button class="secondary" onclick="etResetTracking()">Reset Tracking</button>
        </div>
    </div>
</div>

<div class="control-section">
    <h3>Detection Parameters</h3>
    <div class="three-column-grid">
        <!-- Pupil Column -->
        <div class="param-column">
            <h4>Pupil</h4>
            <div class="param-group">
                <label>Threshold: <span id="et-pupil-thresh-val">45</span></label>
                <input type="range" id="et-pupil-threshold" min="0" max="255" value="45" 
                       oninput="etUpdateLabel(this, 'et-pupil-thresh-val')" 
                       onchange="etApplyPupilThreshold(this.value)">
            </div>
        </div>
        
        <!-- P1 Column -->
        <div class="param-column" id="et-p1-column">
            <h4>P1</h4>
            <div class="param-group">
                <label>Max Jump: <span id="et-p1-jump-val">15.0</span> px</label>
                <input type="range" id="et-p1-max-jump" min="5" max="50" step="0.5" value="15.0"
                       oninput="etUpdateLabel(this, 'et-p1-jump-val')"
                       onchange="etApplyP1MaxJump(this.value)">
            </div>
            <div class="param-group">
                <label>Min Intensity: <span id="et-p1-intensity-val">140</span></label>
                <input type="range" id="et-p1-min-intensity" min="80" max="250" value="140"
                       oninput="etUpdateLabel(this, 'et-p1-intensity-val')"
                       onchange="etApplyP1MinIntensity(this.value)">
            </div>
        </div>
        
        <!-- P4 Column -->
        <div class="param-column" id="et-p4-column" style="display: none;">
            <h4>P4</h4>
            <div class="param-group">
                <label>Max Jump: <span id="et-p4-jump-val">20.0</span> px</label>
                <input type="range" id="et-p4-max-jump" min="5" max="50" step="0.5" value="20.0"
                       oninput="etUpdateLabel(this, 'et-p4-jump-val')"
                       onchange="etApplyP4MaxJump(this.value)">
            </div>
            <div class="param-group">
                <label>Min Intensity: <span id="et-p4-intensity-val">140</span></label>
                <input type="range" id="et-p4-min-intensity" min="80" max="250" value="140"
                       oninput="etUpdateLabel(this, 'et-p4-intensity-val')"
                       onchange="etApplyP4MinIntensity(this.value)">
            </div>
            <div class="param-group">
                <label>Max Pred Error: <span id="et-p4-pred-error-val">13.0</span> px</label>
                <input type="range" id="et-p4-pred-error" min="5" max="30" step="0.5" value="13.0"
                       oninput="etUpdateLabel(this, 'et-p4-pred-error-val')"
                       onchange="etApplyP4PredictionError(this.value)">
            </div>
        </div>
    </div>
</div>

<div class="control-section">
    <h3>Status</h3>
    <div class="status-row">
        <div class="status-indicator">
            <span class="status-dot" id="et-blink-indicator"></span>
            <span id="et-blink-text">Tracking active</span>
        </div>
        <div class="status-indicator">
            <span class="status-dot" id="et-p1-indicator"></span>
            <span id="et-p1-text">P1 tracking</span>
        </div>
        <div class="status-indicator">
            <span class="status-dot" id="et-p4-indicator"></span>
            <span id="et-p4-text">P4 model</span>
        </div>
    </div>
</div>

<div class="control-section" id="et-p4-calibration" style="display: none;">
    <h3>P4 Calibration</h3>
    <div class="control-row">
        <div class="button-group">
            <button onclick="etCalibrateP4()">Calibrate P4 Model</button>
            <button class="secondary" onclick="etResetP4Model()">Reset P4 Model</button>
        </div>
    </div>
    <div class="control-row">
        <div id="et-p4-status" style="font-size: 12px; color: #999;"></div>
    </div>
</div>

<div class="control-section">
    <h3>Debug</h3>
    <div class="control-row">
        <div class="input-group" style="max-width: 300px;">
            <label>Debug Level: <span id="et-debug-level-val">1</span></label>
            <input type="range" id="et-debug-level" min="0" max="4" step="1" value="1"
                   oninput="etUpdateLabel(this, 'et-debug-level-val')"
                   onchange="etApplyDebugLevel(this.value)">
            <small style="color: #666; margin-top: 4px; display: block;">
                0=Silent, 1=Critical, 2=Normal, 3=Verbose, 4=Profile
            </small>
        </div>
    </div>
    <div class="control-row">
        <button class="secondary" onclick="etDebugNextFrame()">Debug Next Frame</button>
        <small style="color: #666; margin-left: 12px; align-self: center;">
            Verbose debug output for next frame (check terminal)
        </small>
    </div>
    <div class="control-row">
        <div class="input-group">
            <label style="display: flex; align-items: center; gap: 8px;">
                <input type="checkbox" id="et-profiling" onchange="etToggleProfiling(this.checked)">
                <span>Enable Frame Timing</span>
            </label>
        </div>
    </div>
</div>

<div class="control-section">
    <div style="font-size: 11px; color: #666; line-height: 1.6;">
        <strong>Note:</strong> Real-time tracking visualization appears on the OpenCV display window.
        Results are also forwarded to the dataserver if configured.
    </div>
</div>
)EYETRACK_HTML";
}

std::string getUIScript() const override {
    return R"EYETRACK_JS(
// Eye Tracking Plugin UI Script

let etSettingsInitialized = false;

// ============================================================================
// UI UPDATE FUNCTIONS
// ============================================================================

window.etUpdateLabel = function(slider, labelId) {
    const label = document.getElementById(labelId);
    if (label) {
        const val = parseFloat(slider.value);
        label.textContent = Number.isInteger(val) ? val : val.toFixed(1);
    }
};

window.etSyncSettingToUI = function(settingName, value) {
    console.log('Syncing setting:', settingName, '=', value);
    
    switch(settingName) {
        case 'pupil_threshold':
            const pupilSlider = document.getElementById('et-pupil-threshold');
            if (pupilSlider && pupilSlider.value != value) {
                pupilSlider.value = value;
                window.etUpdateLabel(pupilSlider, 'et-pupil-thresh-val');
            }
            break;
            
        case 'p1_max_jump':
            const p1Slider = document.getElementById('et-p1-max-jump');
            if (p1Slider && p1Slider.value != value) {
                p1Slider.value = value;
                window.etUpdateLabel(p1Slider, 'et-p1-jump-val');
            }
            break;
            
        case 'p1_min_intensity':
            const p1IntSlider = document.getElementById('et-p1-min-intensity');
            if (p1IntSlider && p1IntSlider.value != value) {
                p1IntSlider.value = value;
                window.etUpdateLabel(p1IntSlider, 'et-p1-intensity-val');
            }
            break;
            
        case 'p4_max_jump':
            const p4Slider = document.getElementById('et-p4-max-jump');
            if (p4Slider && p4Slider.value != value) {
                p4Slider.value = value;
                window.etUpdateLabel(p4Slider, 'et-p4-jump-val');
            }
            break;
            
        case 'p4_min_intensity':
            const p4IntSlider = document.getElementById('et-p4-min-intensity');
            if (p4IntSlider && p4IntSlider.value != value) {
                p4IntSlider.value = value;
                window.etUpdateLabel(p4IntSlider, 'et-p4-intensity-val');
            }
            break;
            
        case 'p4_max_prediction_error':
            const p4PredSlider = document.getElementById('et-p4-pred-error');
            if (p4PredSlider && p4PredSlider.value != value) {
                p4PredSlider.value = value;
                window.etUpdateLabel(p4PredSlider, 'et-p4-pred-error-val');
            }
            break;
            
        case 'detection_mode':
            const modeSelect = document.getElementById('et-detection-mode');
            if (modeSelect && modeSelect.value != value) {
                modeSelect.value = value;
                window.etUpdateModeControls(value);
            }
            break;
    }
};

window.etUpdateModeControls = function(mode) {
    const p1Column = document.getElementById('et-p1-column');
    const p4Column = document.getElementById('et-p4-column');
    const calibSection = document.getElementById('et-p4-calibration');
    
    if (p1Column) p1Column.style.display = (mode !== 'pupil_only') ? 'block' : 'none';
    if (p4Column) p4Column.style.display = (mode === 'full') ? 'block' : 'none';
    if (calibSection) calibSection.style.display = (mode === 'full') ? 'block' : 'none';
};

// ============================================================================
// SETTINGS MANAGEMENT
// ============================================================================

window.etLoadCurrentSettings = function() {
    console.log('etLoadCurrentSettings called, ws state:', window.ws ? window.ws.readyState : 'no ws');
    
    if (window.ws && window.ws.readyState === WebSocket.OPEN) {
        console.log('Sending refreshSettings command...');
        if (window.sendCommand) {
            window.sendCommand('eyetracking::refreshSettings');
        } else {
            window.ws.send(JSON.stringify({
                cmd: 'eval',
                script: 'eyetracking::refreshSettings'
            }));
        }
    } else {
        console.warn('WebSocket not ready, cannot load settings');
    }
};

// ============================================================================
// COMMAND FUNCTIONS (called from HTML UI)
// ============================================================================

window.etApplyMode = function() {
    const mode = document.getElementById('et-detection-mode').value;
    if (window.sendCommand) {
        window.sendCommand('eyetracking::setDetectionMode ' + mode);
    }
};

window.etApplyPupilThreshold = function(value) {
    if (window.sendCommand) {
        window.sendCommand('eyetracking::setPupilThreshold ' + value);
    }
};

window.etApplyP1MaxJump = function(value) {
    if (window.sendCommand) {
        window.sendCommand('eyetracking::setP1MaxJump ' + value);
    }
};

window.etApplyP1MinIntensity = function(value) {
    if (window.sendCommand) {
        window.sendCommand('eyetracking::setP1MinIntensity ' + value);
    }
};

window.etApplyP4MaxJump = function(value) {
    if (window.sendCommand) {
        window.sendCommand('eyetracking::setP4MaxJump ' + value);
    }
};

window.etApplyP4MinIntensity = function(value) {
    if (window.sendCommand) {
        window.sendCommand('eyetracking::setP4MinIntensity ' + value);
    }
};

window.etApplyP4PredictionError = function(value) {
    if (window.sendCommand) {
        window.sendCommand('eyetracking::setP4MaxPredictionError ' + value);
    }
};

window.etApplyDebugLevel = function(value) {
    if (window.sendCommand) {
        window.sendCommand('eyetracking::setDebugLevel ' + value);
    }
};

window.etToggleProfiling = function(enabled) {
    if (window.sendCommand) {
        window.sendCommand('eyetracking::enableProfiling ' + (enabled ? 1 : 0));
    }
};

window.etResetTracking = function() {
    if (confirm('Reset eye tracking state? This will clear P1/P4 validators.')) {
        if (window.sendCommand) {
            window.sendCommand('eyetracking::resetTrackingState');
        }
        if (window.showInfo) {
            window.showInfo('Eye tracking state reset');
        }
    }
};

window.etCalibrateP4 = function() {
    if (window.sendCommand) {
        window.sendCommand('eyetracking::calibrateP4Model');
    }
    if (window.showInfo) {
        window.showInfo('P4 calibration initiated');
    }
};

window.etResetP4Model = function() {
    if (confirm('Reset P4 calibration model? This will clear all calibration data.')) {
        if (window.sendCommand) {
            window.sendCommand('eyetracking::resetP4Model');
        }
        const status = document.getElementById('et-p4-status');
        if (status) status.textContent = 'Model reset';
        if (window.showInfo) {
            window.showInfo('P4 model reset');
        }
    }
};

window.etDebugNextFrame = function() {
    if (window.sendCommand) {
        window.sendCommand('eyetracking::debugNextFrame');
    }
    if (window.showInfo) {
        window.showInfo('Next frame will output verbose debug to terminal');
    }
};

// ============================================================================
// EVENT HANDLERS
// ============================================================================

// Listen for vstream-event (dispatched by interface.html)
window.addEventListener('vstream-event', (e) => {
    const { type, data } = e.detail;
    
    console.log('Received vstream-event:', type, data);
    
    if (!type.startsWith('eyetracking/')) return;
    
    const blinkIndicator = document.getElementById('et-blink-indicator');
    const blinkText = document.getElementById('et-blink-text');
    const p1Indicator = document.getElementById('et-p1-indicator');
    const p1Text = document.getElementById('et-p1-text');
    const p4Indicator = document.getElementById('et-p4-indicator');
    const p4Text = document.getElementById('et-p4-text');
    const p4Status = document.getElementById('et-p4-status');
    
    switch(type) {
        case 'eyetracking/settings':
            if (data && data.name && data.value) {
                window.etSyncSettingToUI(data.name, data.value);
            }
            break;
            
        case 'eyetracking/blink_start':
            if (blinkIndicator) blinkIndicator.classList.add('recording');
            if (blinkText) blinkText.textContent = 'BLINK detected';
            break;
            
        case 'eyetracking/blink_end':
            if (blinkIndicator) blinkIndicator.classList.remove('recording');
            if (blinkText) blinkText.textContent = 'Tracking active';
            break;
            
        case 'eyetracking/p1_lost':
            if (p1Indicator) p1Indicator.classList.remove('active');
            if (p1Text) p1Text.textContent = 'P1 lost';
            break;
            
        case 'eyetracking/p1_recovered':
            if (p1Indicator) p1Indicator.classList.add('active');
            if (p1Text) p1Text.textContent = 'P1 tracking';
            break;
            
        case 'eyetracking/p4_calibrated':
            if (p4Indicator) p4Indicator.classList.add('active');
            if (p4Text) p4Text.textContent = 'P4 calibrated';
            if (p4Status && data) {
                const samples = data.samples || '?';
                p4Status.textContent = '✓ Calibrated (' + samples + ' samples)';
                p4Status.style.color = '#4ec9b0';
            }
            break;
    }
});

// Listen for plugin loaded event
window.addEventListener('vstream-plugin-loaded', (e) => {
    console.log('vstream-plugin-loaded event received:', e.detail);
    
    if (e.detail.name === 'eye_tracking') {
        console.log('Eye tracking plugin loaded, initializing...');
        console.log('WebSocket exists?', !!window.ws);
        console.log('WebSocket readyState:', window.ws ? window.ws.readyState : 'N/A');
        
        if (window.ws && window.ws.readyState === WebSocket.OPEN) {
            console.log('Subscribing to eyetracking/*...');
            window.ws.send(JSON.stringify({
                cmd: 'subscribe',
                topics: ['eyetracking/*']
            }));
            
            // Load settings after short delay to ensure subscription is registered
            setTimeout(() => {
                console.log('Loading current settings...');
                window.etLoadCurrentSettings();
            }, 200);
        } else {
            console.error('WebSocket not ready when plugin loaded!');
        }
        
        etSettingsInitialized = true;
    }
});

// Listen for tab activation to refresh settings
window.addEventListener('vstream-plugin-tab-active', () => {
    console.log('Plugins tab activated');
    if (etSettingsInitialized) {
        console.log('Refreshing eyetracking settings...');
        window.etLoadCurrentSettings();
    }
});

console.log('Eye tracking UI script loaded');
)EYETRACK_JS";
}

std::string getUIStyle() const override {
    return R"EYETRACK_CSS(
.control-section {
    margin-bottom: 20px;
    padding: 12px;
    background: #2d2d30;
    border-radius: 4px;
}

.control-section h3 {
    font-size: 13px;
    margin: 0 0 12px 0;
    color: #e0e0e0;
    font-weight: 600;
}

.control-row {
    display: flex;
    gap: 12px;
    align-items: flex-start;
    margin-bottom: 8px;
}

.control-row:last-child {
    margin-bottom: 0;
}

/* Three Column Grid for Detection Parameters */
.three-column-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 16px;
}

.param-column {
    background: #252526;
    padding: 12px;
    border-radius: 4px;
    border: 1px solid #3c3c3c;
}

.param-column h4 {
    font-size: 12px;
    margin: 0 0 12px 0;
    color: #4ec9b0;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}

.param-group {
    margin-bottom: 12px;
}

.param-group:last-child {
    margin-bottom: 0;
}

.param-group label {
    display: block;
    margin-bottom: 4px;
    font-size: 11px;
    color: #cccccc;
}

.param-group input[type="range"] {
    width: 100%;
}

/* Status Row */
.status-row {
    display: flex;
    gap: 20px;
    flex-wrap: wrap;
}

.input-group {
    flex: 1;
    min-width: 0;
}

.input-group label {
    display: block;
    margin-bottom: 4px;
    font-size: 12px;
    color: #cccccc;
}

.input-group input[type="range"] {
    width: 100%;
}

.input-group select {
    width: 100%;
    padding: 4px 8px;
    background: #3c3c3c;
    border: 1px solid #555;
    color: #cccccc;
    border-radius: 3px;
}

.button-group {
    display: flex;
    gap: 8px;
}

button {
    padding: 6px 12px;
    background: #0e639c;
    color: white;
    border: none;
    border-radius: 3px;
    cursor: pointer;
    font-size: 12px;
}

button:hover {
    background: #1177bb;
}

button.secondary {
    background: #3c3c3c;
}

button.secondary:hover {
    background: #505050;
}

.status-indicator {
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 12px;
}

.status-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: #666;
}

.status-dot.active {
    background: #4ec9b0;
}

.status-dot.recording {
    background: #f48771;
    animation: pulse 1s ease-in-out infinite;
}

@keyframes pulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.5; }
}

#et-p4-status {
    font-family: 'SF Mono', Monaco, monospace;
}

/* Responsive: Stack columns on narrow screens */
@media (max-width: 600px) {
    .three-column-grid {
        grid-template-columns: 1fr;
    }
}

@media (min-width: 601px) and (max-width: 900px) {
    .three-column-grid {
        grid-template-columns: repeat(2, 1fr);
    }
}
)EYETRACK_CSS";
}  

  // Format results to be saved with VideoStream metadata
  std::string serializeResults(int frame_idx) override {
    std::lock_guard<std::mutex> lock(results_mutex_);
    if (!latest_results_.valid || latest_results_.frame_idx != frame_idx) {
      return "{}";
    }
    
    json_t* root = json_object();
    
    // Frame info
    json_object_set_new(root, "frame_idx", json_integer(frame_idx));
    json_object_set_new(root, "in_blink", json_boolean(latest_results_.in_blink));
    
    // Pupil data
    if (latest_results_.pupil.detected) {
      json_t* pupil = json_object();
      json_object_set_new(pupil, "x", json_real(latest_results_.pupil.center.x));
      json_object_set_new(pupil, "y", json_real(latest_results_.pupil.center.y));
      json_object_set_new(pupil, "radius", json_real(latest_results_.pupil.radius));
      if (latest_results_.pupil.has_ellipse) {
        json_object_set_new(pupil, "a", json_real(latest_results_.pupil.ellipse_a));
        json_object_set_new(pupil, "b", json_real(latest_results_.pupil.ellipse_b));
        json_object_set_new(pupil, "angle", json_real(latest_results_.pupil.ellipse_angle));
      }
      json_object_set_new(root, "pupil", pupil);
    }
    
    // P1 data
    if (latest_results_.purkinje.p1_detected) {
        json_t* p1 = json_object();
        json_object_set_new(p1, "x", json_real(latest_results_.purkinje.p1_center.x));
        json_object_set_new(p1, "y", json_real(latest_results_.purkinje.p1_center.y));
        json_object_set_new(root, "p1", p1);
    }
    
    // P4 data
    if (latest_results_.purkinje.p4_detected) {
        json_t* p4 = json_object();
        json_object_set_new(p4, "x", json_real(latest_results_.purkinje.p4_center.x));
        json_object_set_new(p4, "y", json_real(latest_results_.purkinje.p4_center.y));
        json_object_set_new(root, "p4", p4);
    }
    
    // Model status (optional, but useful for debugging)
    if (p4_model_.isInitialized()) {
        json_t* model = json_object();
        json_object_set_new(model, "initialized", json_true());
        json_object_set_new(model, "frozen", json_boolean(p4_model_.isFrozen()));
        json_object_set_new(model, "magnitude_ratio", 
                           json_real(p4_model_.getMagnitudeRatio()));
        json_object_set_new(model, "angle_offset_deg", 
                           json_real(p4_model_.getAngleOffset() * 180.0 / M_PI));
        json_object_set_new(root, "p4_model", model);
    }
    
    char* json_str = json_dumps(root, JSON_COMPACT);
    std::string result(json_str);
    free(json_str);
    json_decref(root);
    
    return result;
}


  bool usesStructuredStorage() const override {
    return true;
  }

  // Host calls this once per recording, before closing the db — release the
  // cached INSERT statement so the db can close cleanly.
  void endStorageBatch(sqlite3* db) override {
    if (store_stmt_) {
      sqlite3_finalize(store_stmt_);
      store_stmt_ = nullptr;
      store_stmt_db_ = nullptr;
    }
  }
  
  std::string getTableSchema() const override {
    return R"(
        CREATE TABLE IF NOT EXISTS eyetracking_frames (
            frame_number INTEGER PRIMARY KEY,
            obs_id INTEGER,
            in_blink INTEGER NOT NULL,
            pupil_x REAL,
            pupil_y REAL,
            pupil_radius REAL,
            pupil_a REAL,
            pupil_b REAL,
            pupil_angle REAL,
            p1_x REAL,
            p1_y REAL,
            p4_x REAL,
            p4_y REAL,
            p4_model_initialized INTEGER,
            p4_model_frozen INTEGER,
            p4_magnitude_ratio REAL,
            p4_angle_offset_deg REAL,
            p4_reject_reason INTEGER,
            p4_pred_x REAL,
            p4_pred_y REAL,
            p4_candidate INTEGER,
            p1_reject_reason INTEGER,
            tracking_lost INTEGER
        );
        
       CREATE INDEX IF NOT EXISTS idx_eyetracking_obs
            ON eyetracking_frames(obs_id);
        
        CREATE INDEX IF NOT EXISTS idx_eyetracking_blink 
            ON eyetracking_frames(in_blink);
            
        CREATE INDEX IF NOT EXISTS idx_eyetracking_pupil 
            ON eyetracking_frames(pupil_x, pupil_y) 
            WHERE pupil_x IS NOT NULL;
    )";
  }
  bool storeFrameData(sqlite3* db, int frame_number, int obs_id) override {
    std::lock_guard<std::mutex> lock(results_mutex_);
    
    // ALWAYS store a row - even without valid results
    // This keeps eyetracking_frames synchronized with video frames
    
    const char* sql = R"(
        INSERT INTO eyetracking_frames (
            frame_number, obs_id, in_blink,
            pupil_x, pupil_y, pupil_radius,
            pupil_a, pupil_b, pupil_angle,
            p1_x, p1_y,
            p4_x, p4_y,
            p4_model_initialized, p4_model_frozen,
            p4_magnitude_ratio, p4_angle_offset_deg,
            p4_reject_reason, p4_pred_x, p4_pred_y, p4_candidate,
            p1_reject_reason, tracking_lost
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    )";
    
    // Prepare once per recording (or on db change); re-preparing this INSERT
    // for every frame at 250 Hz was measurable parse overhead.
    if (store_stmt_ == nullptr || store_stmt_db_ != db) {
      if (store_stmt_) {
        sqlite3_finalize(store_stmt_);
        store_stmt_ = nullptr;
      }
      if (sqlite3_prepare_v2(db, sql, -1, &store_stmt_, nullptr) != SQLITE_OK) {
        if (debug_level_ >= DEBUG_CRITICAL) {
          std::cerr << "Failed to prepare eyetracking insert: "
                    << sqlite3_errmsg(db) << std::endl;
        }
        store_stmt_db_ = nullptr;
        return false;
      }
      store_stmt_db_ = db;
    }
    sqlite3_stmt* stmt = store_stmt_;
    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);

    int col = 1;
    
    // Always bind frame_number
    sqlite3_bind_int(stmt, col++, frame_number);
    
    // Bind obs id (or null if outside obs period)
    if (obs_id >= 0) {
   	 sqlite3_bind_int(stmt, col++, obs_id);
	} else {
     sqlite3_bind_null(stmt, col++);
    }
    
    // If we have valid results, store them; otherwise store NULLs
    if (latest_results_.valid) {
      // in_blink
      sqlite3_bind_int(stmt, col++, latest_results_.in_blink ? 1 : 0);
      
      // Pupil data (nullable)
      if (latest_results_.pupil.detected) {
	sqlite3_bind_double(stmt, col++, latest_results_.pupil.center.x);
	sqlite3_bind_double(stmt, col++, latest_results_.pupil.center.y);
	sqlite3_bind_double(stmt, col++, latest_results_.pupil.radius);
      } else {
	sqlite3_bind_null(stmt, col++);
	sqlite3_bind_null(stmt, col++);
	sqlite3_bind_null(stmt, col++);
      }

      // Pupil ellipse (nullable; pupillometry)
      if (latest_results_.pupil.detected && latest_results_.pupil.has_ellipse) {
	sqlite3_bind_double(stmt, col++, latest_results_.pupil.ellipse_a);
	sqlite3_bind_double(stmt, col++, latest_results_.pupil.ellipse_b);
	sqlite3_bind_double(stmt, col++, latest_results_.pupil.ellipse_angle);
      } else {
	sqlite3_bind_null(stmt, col++);
	sqlite3_bind_null(stmt, col++);
	sqlite3_bind_null(stmt, col++);
      }
      
      // P1 data (nullable)
      if (latest_results_.purkinje.p1_detected) {
	sqlite3_bind_double(stmt, col++, latest_results_.purkinje.p1_center.x);
	sqlite3_bind_double(stmt, col++, latest_results_.purkinje.p1_center.y);
      } else {
	sqlite3_bind_null(stmt, col++);
	sqlite3_bind_null(stmt, col++);
      }
      
      // P4 data (nullable)
      if (latest_results_.purkinje.p4_detected) {
	sqlite3_bind_double(stmt, col++, latest_results_.purkinje.p4_center.x);
	sqlite3_bind_double(stmt, col++, latest_results_.purkinje.p4_center.y);
      } else {
	sqlite3_bind_null(stmt, col++);
	sqlite3_bind_null(stmt, col++);
      }
      
      // P4 model status
      sqlite3_bind_int(stmt, col++, p4_model_.isInitialized() ? 1 : 0);
      sqlite3_bind_int(stmt, col++, p4_model_.isFrozen() ? 1 : 0);
      
      // P4 model parameters (nullable if not initialized)
      if (p4_model_.isInitialized()) {
	sqlite3_bind_double(stmt, col++, p4_model_.getMagnitudeRatio());
	sqlite3_bind_double(stmt, col++, p4_model_.getAngleOffset() * 180.0 / M_PI);
      } else {
	sqlite3_bind_null(stmt, col++);
	sqlite3_bind_null(stmt, col++);
      }

      // Reprocess diagnostics: why P4 was/wasn't reported, model prediction,
      // and whether any candidate reflection was found this frame.
      sqlite3_bind_int(stmt, col++, latest_results_.p4_reject_reason);
      if (latest_results_.p4_predicted.x >= 0) {
	sqlite3_bind_double(stmt, col++, latest_results_.p4_predicted.x);
	sqlite3_bind_double(stmt, col++, latest_results_.p4_predicted.y);
      } else {
	sqlite3_bind_null(stmt, col++);
	sqlite3_bind_null(stmt, col++);
      }
      sqlite3_bind_int(stmt, col++, latest_results_.p4_candidate_found ? 1 : 0);
      sqlite3_bind_int(stmt, col++, latest_results_.p1_reject_reason);
      sqlite3_bind_int(stmt, col++, latest_results_.tracking_lost ? 1 : 0);
    } else {
      // No valid results - insert NULL for everything except frame_number
      sqlite3_bind_int(stmt, col++, 0);  // in_blink = 0 (not blinking)

      // All other columns are nullable - set to NULL
      for (int i = 0; i < 20; i++) {  // 14 detection + 5 diagnostic + tracking_lost
	sqlite3_bind_null(stmt, col++);
      }
    }
    
    int result = sqlite3_step(stmt);
    // (statement is cached — reset happens on the next call, finalize in
    // endStorageBatch before the host closes the db)
    
    if (result != SQLITE_DONE) {
      if (debug_level_ >= DEBUG_CRITICAL) {
	std::cerr << "Failed to insert eyetracking data: " 
		  << sqlite3_errmsg(db) << std::endl;
      }
      return false;
    }
    
    return true;
  }  

  void forwardResults(int frame_idx, const FrameMetadata& metadata,
		      const PupilData& pupil, const PurkinjeData& purkinje) {
    if (first_frameID_ < 0) {
        first_frameID_ = metadata.frameID;
        first_timestamp_ = metadata.timestamp;

        // Forward reference values after a reset
        int64_t ref_data[2] = {first_frameID_, first_timestamp_};
        ds_forward_queue.push_back(DataPoint("eyetracking/file_reference",
                                             DataserverForwarder::INT64,
                                             ref_data, sizeof(ref_data)));
    }
        

    // Extended format with frame metadata
    // [frame_id, frame_timestamp_us, pupil_x, pupil_y, pupil_r, 
    //  p1_x, p1_y, p4_x, p4_y, blink, p1_detected, p4_detected]
    float batch[12];

    // Send a frame count reference corresponding to open file frame id
    batch[0] = static_cast<float>(metadata.frameID - first_frameID_);

    // relative to first frame
    batch[1] = (metadata.timestamp - first_timestamp_) / 1e9f;  // nanoseconds to seconds
    
    
    // Pupil data (use -1 for not detected)
    batch[2] = pupil.detected ? pupil.center.x : -1.0f;
    batch[3] = pupil.detected ? pupil.center.y : -1.0f;
    batch[4] = pupil.detected ? pupil.radius : -1.0f;
    
    // P1 data
    batch[5] = purkinje.p1_detected ? purkinje.p1_center.x : -1.0f;
    batch[6] = purkinje.p1_detected ? purkinje.p1_center.y : -1.0f;
    
    // P4 data
    batch[7] = purkinje.p4_detected ? purkinje.p4_center.x : -1.0f;
    batch[8] = purkinje.p4_detected ? purkinje.p4_center.y : -1.0f;
    
    // Status flags
    batch[9] = blink_detector_.isInBlink() ? 1.0f : 0.0f;
    batch[10] = purkinje.p1_detected ? 1.0f : 0.0f;
    batch[11] = purkinje.p4_detected ? 1.0f : 0.0f;
    
    // Send as float array
    ds_forward_queue.push_back(DataPoint("eyetracking/results", 
                                         DataserverForwarder::FLOAT, 
                                         batch, 
                                         sizeof(batch)));
  }

void drawMagnifiedInset(cv::Mat& frame, const cv::Mat& gray_roi, 
                       cv::Point2f center, const std::string& label,
                       cv::Point2f display_pos, int zoom_factor = 10) {
    // Size of the region to magnify (in original pixels)
    const int region_size = 11;  // 11x11 pixel region
    const int half_size = region_size / 2;
    
    // Calculate the inset size
    const int inset_size = region_size * zoom_factor;
    
    // Extract region around the feature (handling boundaries)
    cv::Point2f local_center = center;
    if (roi_enabled_) {
        local_center.x -= current_roi_.x;
        local_center.y -= current_roi_.y;
    }
    
    int x_start = (int)floor(local_center.x) - half_size;
    int y_start = (int)floor(local_center.y) - half_size;
    
    // Clamp to image boundaries
    x_start = std::max(0, std::min(x_start, gray_roi.cols - region_size));
    y_start = std::max(0, std::min(y_start, gray_roi.rows - region_size));
    
    cv::Rect extract_roi(x_start, y_start, region_size, region_size);
    cv::Mat region = gray_roi(extract_roi);
    
    // Convert to color for overlay
    cv::Mat region_color;
    cv::cvtColor(region, region_color, cv::COLOR_GRAY2BGR);
    
    // Resize with nearest neighbor to see actual pixels
    cv::Mat magnified;
    cv::resize(region_color, magnified, cv::Size(inset_size, inset_size), 
               0, 0, cv::INTER_NEAREST);
    
    // Draw pixel grid on magnified view
    for (int i = 0; i <= region_size; i++) {
        int pos = i * zoom_factor;
        cv::line(magnified, cv::Point(pos, 0), cv::Point(pos, inset_size),
                cv::Scalar(80, 80, 80), 1);
        cv::line(magnified, cv::Point(0, pos), cv::Point(inset_size, pos),
                cv::Scalar(80, 80, 80), 1);
    }
    
    // Calculate sub-pixel position within the extracted region
    float rel_x = local_center.x - x_start;
    float rel_y = local_center.y - y_start;
    
    // Draw crosshair at sub-pixel location
    cv::Point2f crosshair_pos(rel_x * zoom_factor, rel_y * zoom_factor);
    
    // Bright cyan crosshair
    cv::drawMarker(magnified, crosshair_pos, cv::Scalar(255, 255, 0),
                  cv::MARKER_CROSS, zoom_factor * 2, 2);
    
    // Draw circle showing detection radius (4 pixels)
    cv::circle(magnified, crosshair_pos, 4 * zoom_factor, 
              cv::Scalar(0, 0, 255), 2);
    
    // Add border and label
    cv::rectangle(magnified, cv::Point(0, 0), 
                 cv::Point(inset_size-1, inset_size-1),
                 cv::Scalar(255, 255, 255), 2);
    
    // Add label and coordinates
    char coord_text[64];
    snprintf(coord_text, sizeof(coord_text), "%s: (%.2f, %.2f)", 
             label.c_str(), center.x, center.y);
    cv::putText(magnified, coord_text, cv::Point(5, 15),
               cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
    
    // Show fractional part
    char frac_text[64];
    snprintf(frac_text, sizeof(frac_text), "Frac: (%.3f, %.3f)",
             center.x - floor(center.x), center.y - floor(center.y));
    cv::putText(magnified, frac_text, cv::Point(5, inset_size - 8),
               cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 255), 1);
    
    // Overlay intensity at center pixel
    int center_px_x = cvRound(local_center.x);
    int center_px_y = cvRound(local_center.y);
    if (center_px_x >= 0 && center_px_x < gray_roi.cols &&
        center_px_y >= 0 && center_px_y < gray_roi.rows) {
        uchar intensity = gray_roi.at<uchar>(center_px_y, center_px_x);
        char int_text[32];
        snprintf(int_text, sizeof(int_text), "I=%d", intensity);
        cv::putText(magnified, int_text, cv::Point(5, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
    }
    
    // Place inset on frame
    cv::Rect inset_roi(display_pos.x, display_pos.y, inset_size, inset_size);
    
    // Make sure it fits on screen
    if (inset_roi.x + inset_roi.width > frame.cols) {
        inset_roi.x = frame.cols - inset_roi.width;
    }
    if (inset_roi.y + inset_roi.height > frame.rows) {
        inset_roi.y = frame.rows - inset_roi.height;
    }
    
    // Copy inset to frame
    magnified.copyTo(frame(inset_roi));
}

// Large magnified view of the P4 region to assist camera focusing. Anchored on
// the P4-model prediction window (the small box that tracks P4 ~centered), with
// a fallback chain so it still works before the model is initialized / P4 is
// detected — exactly the situation during focus setup. Uses INTER_NEAREST so no
// resampling blur is added to confound the focus judgment, and overlays a
// variance-of-Laplacian sharpness score (+ history sparkline) the experimenter
// can maximize while turning the focus ring. Drawn last, over a dimmed frame.
//
// full_gray is the FULL-frame grayscale; all anchor positions (predicted P4,
// p4_center, last-known) are in full-frame coordinates, so no ROI offset math.
// annotate=false draws just the magnified image (for "pure" focusing, no
// reticle/text/sparkline obscuring the view).
void drawFocusPanel(cv::Mat& frame, const cv::Mat& full_gray,
		    bool use_roi, const cv::Rect& roi, bool annotate) {
    if (full_gray.empty() || frame.empty()) return;

    // Source window size: the P4 search ROI plus a little context. Square.
    int win = std::max(p4_search_roi_size_.width, p4_search_roi_size_.height);
    win = (int)std::round(win * 1.4f);
    win = std::max(24, win);
    win = std::min({win, full_gray.cols, full_gray.rows});

    // Anchor (full-frame coords): prefer the P4-model prediction window, then
    // the detected P4, then last-known P4, then ROI/frame center.
    cv::Point2f anchor(-1, -1);
    bool from_prediction = false;
    if (p4_model_.isInitialized() && latest_results_.valid &&
	latest_results_.pupil.detected && latest_results_.purkinje.p1_detected) {
	cv::Point2f pred = p4_model_.predict(latest_results_.pupil.center,
					     latest_results_.purkinje.p1_center);
	if (pred.x > 0) { anchor = pred; from_prediction = true; }
    }
    if (anchor.x < 0 && latest_results_.valid &&
	latest_results_.purkinje.p4_detected) {
	anchor = latest_results_.purkinje.p4_center;
    }
    if (anchor.x < 0 && p4_last_known_position_.x > 0) {
	anchor = p4_last_known_position_;
    }
    if (anchor.x < 0 && use_roi) {
	anchor = cv::Point2f(roi.x + roi.width / 2.0f, roi.y + roi.height / 2.0f);
    }
    if (anchor.x < 0) {
	anchor = cv::Point2f(full_gray.cols / 2.0f, full_gray.rows / 2.0f);
    }

    // Source rect: window centered on anchor, clamped to frame bounds.
    int sx = (int)std::round(anchor.x) - win / 2;
    int sy = (int)std::round(anchor.y) - win / 2;
    sx = std::max(0, std::min(sx, full_gray.cols - win));
    sy = std::max(0, std::min(sy, full_gray.rows - win));
    cv::Mat src = full_gray(cv::Rect(sx, sy, win, win));
    if (src.cols < 2 || src.rows < 2) return;

    // Auto-fit zoom so the panel fills ~60% of the smaller frame dimension.
    const double target = 0.60 * std::min(frame.cols, frame.rows);
    int zoom = (int)std::round(target / win);
    zoom = std::max(3, std::min(zoom, 24));
    const int panel_w = src.cols * zoom;
    const int panel_h = src.rows * zoom;

    // Dim the whole frame so the panel reads as a dedicated mode.
    frame.convertTo(frame, -1, 0.35, 0);

    // Magnify (nearest-neighbor) and colorize.
    cv::Mat mag_gray, panel;
    cv::resize(src, mag_gray, cv::Size(panel_w, panel_h), 0, 0, cv::INTER_NEAREST);
    cv::cvtColor(mag_gray, panel, cv::COLOR_GRAY2BGR);

    if (annotate) {
	// Sharpness metric: variance of the Laplacian over the source region.
	cv::Mat lap;
	cv::Laplacian(src, lap, CV_64F);
	cv::Scalar mu, sigma;
	cv::meanStdDev(lap, mu, sigma);
	float focus = (float)(sigma[0] * sigma[0]);
	focus_best_ = std::max(focus_best_, focus);
	focus_history_.push_back(focus);
	const size_t kHistMax = 160;
	if (focus_history_.size() > kHistMax) {
	    focus_history_.erase(focus_history_.begin(),
				 focus_history_.begin() +
				 (focus_history_.size() - kHistMax));
	}

	// Reticle at the anchor location (P4 sits ~here), in panel coords.
	cv::Point ctr((int)((anchor.x - sx) * zoom), (int)((anchor.y - sy) * zoom));
	cv::Scalar reticle = from_prediction ? cv::Scalar(0, 255, 0)   // model active
					     : cv::Scalar(0, 165, 255); // amber: no model
	cv::line(panel, cv::Point(ctr.x, 0), cv::Point(ctr.x, panel_h), reticle, 1);
	cv::line(panel, cv::Point(0, ctr.y), cv::Point(panel_w, ctr.y), reticle, 1);
	cv::circle(panel, ctr, 4 * zoom, reticle, 1);

	// Mark the detected P4, if any, mapped into panel coordinates.
	if (latest_results_.valid && latest_results_.purkinje.p4_detected) {
	    cv::Point2f p4 = latest_results_.purkinje.p4_center;
	    float lx = p4.x - sx;
	    float ly = p4.y - sy;
	    if (lx >= 0 && lx < src.cols && ly >= 0 && ly < src.rows) {
		cv::circle(panel, cv::Point((int)(lx * zoom), (int)(ly * zoom)),
			   3 * zoom, cv::Scalar(0, 0, 255), 2);
	    }
	}

	// Border + title.
	cv::rectangle(panel, cv::Point(0, 0), cv::Point(panel_w - 1, panel_h - 1),
		      cv::Scalar(255, 255, 255), 2);
	cv::putText(panel, "FOCUS MODE - maximize sharpness", cv::Point(8, 22),
		    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

	// Sharpness readout.
	char sbuf[80];
	snprintf(sbuf, sizeof(sbuf), "Sharpness: %.0f  (best %.0f)", focus, focus_best_);
	cv::putText(panel, sbuf, cv::Point(8, 42),
		    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);

	// Sparkline of the sharpness history along the bottom of the panel.
	if (focus_history_.size() >= 2 && focus_best_ > 0.0f) {
	    const int spark_h = std::min(40, panel_h / 5);
	    const int base_y = panel_h - 8;
	    const int top_y = base_y - spark_h;
	    const float xstep = (float)(panel_w - 16) / (focus_history_.size() - 1);
	    cv::Point prev;
	    for (size_t i = 0; i < focus_history_.size(); ++i) {
		int x = 8 + (int)(i * xstep);
		int y = base_y - (int)((focus_history_[i] / focus_best_) * spark_h);
		y = std::max(top_y, std::min(base_y, y));
		cv::Point cur(x, y);
		if (i > 0) cv::line(panel, prev, cur, cv::Scalar(0, 255, 255), 1);
		prev = cur;
	    }
	}
    }

    // Center the panel on the frame (clamped).
    int px = (frame.cols - panel_w) / 2;
    int py = (frame.rows - panel_h) / 2;
    px = std::max(0, std::min(px, frame.cols - panel_w));
    py = std::max(0, std::min(py, frame.rows - panel_h));
    if (panel_w <= frame.cols && panel_h <= frame.rows) {
	panel.copyTo(frame(cv::Rect(px, py, panel_w, panel_h)));
    }
}

bool drawOverlay(cv::Mat& frame, int frame_idx) override {
    std::lock_guard<std::mutex> lock(results_mutex_);

    // Focus-assist takes over the whole frame and must work even before any
    // detection succeeds (results may be invalid while the experimenter is
    // still focusing), so it runs ahead of the normal overlay path.
    if (focus_mode_ != FOCUS_OFF) {
	cv::Rect froi;
	bool fuse_roi;
	{
	    std::lock_guard<std::mutex> roi_lock(roi_mutex_);
	    froi = current_roi_;
	    fuse_roi = roi_enabled_;
	}
	if (fuse_roi) {
	    if (froi.x < 0 || froi.y < 0 ||
		froi.x + froi.width > frame.cols ||
		froi.y + froi.height > frame.rows ||
		froi.width <= 0 || froi.height <= 0) {
		fuse_roi = false;
	    }
	}
	// Pass the FULL-frame grayscale; the panel anchors a small window on the
	// P4 prediction (full-frame coords), and falls back to the ROI center.
	cv::Mat fgray;
	if (frame.channels() > 1) {
	    cv::cvtColor(frame, fgray, cv::COLOR_BGR2GRAY);
	} else {
	    fgray = frame.clone();
	}
	drawFocusPanel(frame, fgray, fuse_roi, froi,
		       focus_mode_ == FOCUS_ANNOTATED);
	return true;
    }

    if (!latest_results_.valid) return false;

 cv::Rect roi;
    bool use_roi;
    {
        std::lock_guard<std::mutex> roi_lock(roi_mutex_);
        roi = current_roi_;
        use_roi = roi_enabled_;
    }
    
    // Validate ROI against current frame
    if (use_roi) {
        if (roi.x < 0 || roi.y < 0 || 
            roi.x + roi.width > frame.cols || 
            roi.y + roi.height > frame.rows ||
            roi.width <= 0 || roi.height <= 0) {
            use_roi = false;
        }
    }
    
    // Extract gray FIRST, before any drawing on overlay
    cv::Mat roi_frame = use_roi ? frame(roi) : frame;
    cv::Mat gray_for_inset;
    if (roi_frame.channels() > 1) {
        cv::cvtColor(roi_frame, gray_for_inset, cv::COLOR_BGR2GRAY);
    } else {
        gray_for_inset = roi_frame.clone();
    }
    
    // Create overlay for semi-transparent drawing
    cv::Mat overlay = frame.clone();
    double alpha = 0.6;  // Transparency factor (0.0 = fully transparent, 1.0 = opaque)
    
    if (use_roi) {
      cv::rectangle(overlay, roi, cv::Scalar(255, 255, 0), 2);
      cv::putText(overlay, "ROI", 
		  cv::Point(roi.x + 5, roi.y + 20),
		  cv::FONT_HERSHEY_SIMPLEX, 0.5, 
		  cv::Scalar(255, 255, 0), 1);
    }
    
    if (latest_results_.pupil.detected) {
      // Draw pupil with transparency
      cv::circle(overlay, latest_results_.pupil.center, 
		 (int)latest_results_.pupil.radius, 
		 cv::Scalar(0, 255, 0), 2);
      cv::drawMarker(overlay, latest_results_.pupil.center, 
		     cv::Scalar(0, 255, 0), 
		     cv::MARKER_CROSS, 10, 1);
      cv::putText(overlay, "Pupil", 
		  cv::Point(latest_results_.pupil.center.x + 
			    latest_results_.pupil.radius + 5, 
			    latest_results_.pupil.center.y),
		  cv::FONT_HERSHEY_SIMPLEX, 0.5, 
		  cv::Scalar(0, 255, 0), 1);
    }

    if (latest_results_.purkinje.p1_detected) {
      // Blue marker with yellow text - visible against dark pupil
      cv::circle(overlay, latest_results_.purkinje.p1_center, 
		 4, cv::Scalar(255, 0, 0), 1);
      if (last_p1_intensity_ > 0) {
	std::string intensity_text = "I=" + std::to_string((int)last_p1_intensity_);
	cv::putText(overlay, intensity_text,
		    cv::Point(latest_results_.purkinje.p1_center.x + 10,
			      latest_results_.purkinje.p1_center.y - 10),
		    cv::FONT_HERSHEY_SIMPLEX, 0.4,
		    cv::Scalar(255, 255, 0), 1);
      }
      cv::drawMarker(overlay, latest_results_.purkinje.p1_center,
		     cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 6, 1);
      cv::putText(overlay, "P1", 
		  cv::Point(latest_results_.purkinje.p1_center.x + 8, 
			    latest_results_.purkinje.p1_center.y),
		  cv::FONT_HERSHEY_SIMPLEX, 0.4, 
		  cv::Scalar(0, 255, 255), 1);
    }
    
    if (latest_results_.purkinje.p4_detected) {
      // Red circle ONLY - no crosshair to obscure the tiny P4 spot
      cv::circle(overlay, latest_results_.purkinje.p4_center, 
		 4, cv::Scalar(0, 0, 255), 1);
      cv::putText(overlay, "P4", 
		  cv::Point(latest_results_.purkinje.p4_center.x + 8, 
			    latest_results_.purkinje.p4_center.y),
		  cv::FONT_HERSHEY_SIMPLEX, 0.4, 
		  cv::Scalar(0, 0, 255), 1);
    }
    
    // Reference ("ghost") overlay: what the loaded session db stored for this
    // same frame — orange, distinct marker shapes (circle/diamond/square) so
    // live (green/blue/red) and reference never blur together. A missing
    // orange square where the red P4 circle sits IS the improvement, visible.
    if (show_reference_ && reference_loaded_.load(std::memory_order_relaxed)) {
      std::lock_guard<std::mutex> ref_lock(reference_mutex_);
      {
        auto it = reference_.find(latest_results_.abs_frame_id);
        if (it != reference_.end()) {
          const RefFrame& rf = it->second;
          const cv::Scalar ghost(0, 165, 255);  // orange
          if (rf.has & REF_HAS_PUPIL) {
            cv::circle(overlay, cv::Point(cvRound(rf.px), cvRound(rf.py)),
                       (int)rf.pr, ghost, 1);
          }
          if (rf.has & REF_HAS_P1) {
            cv::drawMarker(overlay, cv::Point(cvRound(rf.p1x), cvRound(rf.p1y)),
                           ghost, cv::MARKER_DIAMOND, 10, 1);
          }
          if (rf.has & REF_HAS_P4) {
            cv::drawMarker(overlay, cv::Point(cvRound(rf.p4x), cvRound(rf.p4y)),
                           ghost, cv::MARKER_SQUARE, 10, 1);
          }
        }
      }
    }

  // P4 Manual Calibration Visualization
    if (p4_pending_sample_active_) {
      // PENDING: Yellow circle (awaiting confirmation) - NO crosshair
      cv::circle(overlay, p4_pending_sample_position_, 6, cv::Scalar(0, 255, 255), 1);
      cv::putText(overlay, "P4 Sample (Enter to accept)", 
		  cv::Point(p4_pending_sample_position_.x + 12, 
			    p4_pending_sample_position_.y - 8),
		  cv::FONT_HERSHEY_SIMPLEX, 0.4, 
		  cv::Scalar(0, 255, 255), 1);
    } else if (p4_model_.getCalibrationSampleCount() > 0 && 
	       !p4_model_.isInitialized()) {
      // COLLECTING: Show sample count - NO crosshair
      int sample_count = p4_model_.getCalibrationSampleCount();
      
      cv::circle(overlay, p4_last_known_position_, 4, cv::Scalar(255, 165, 0), 1);
      
      std::string status = "P4 Samples: " + std::to_string(sample_count);
      cv::putText(overlay, status, 
		  cv::Point(p4_last_known_position_.x + 10, 
			    p4_last_known_position_.y),
		  cv::FONT_HERSHEY_SIMPLEX, 0.4, 
		  cv::Scalar(255, 165, 0), 1);
    } else if (p4_model_.isInitialized() && latest_results_.pupil.detected && 
	       latest_results_.purkinje.p1_detected &&
	       latest_results_.purkinje.p4_detected &&
	       !blink_detector_.isRecovering() &&
	       !blink_detector_.isInBlink()) {
      // MODEL ACTIVE: Show prediction ROI only (no center circle)
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
	// Just draw the ROI box - no circle in center that obscures P4
	cv::rectangle(overlay, pred_roi, cv::Scalar(0, 255, 0), 1);
      }
    }


    // Blend overlay with original frame
    cv::addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame);

    // LOST banner drawn opaque on top: the operator should see at a glance
    // that this is an extended signal loss, not an ordinary blink.
    if (latest_results_.tracking_lost) {
      cv::putText(frame, "TRACKING LOST", cv::Point(12, 28),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }

    // Reference scoreboard: running P4 detection, stored session vs the code
    // running now, over the frames seen since the reference was loaded.
    if (show_reference_ && reference_loaded_.load(std::memory_order_relaxed)) {
      std::lock_guard<std::mutex> ref_lock(reference_mutex_);
      if (ref_frames_matched_ > 0) {
        char buf[128];
        snprintf(buf, sizeof(buf),
                 "P4  ref %.1f%%  live %.1f%%  (n=%lld)",
                 100.0 * ref_p4_ref_ / ref_frames_matched_,
                 100.0 * ref_p4_live_ / ref_frames_matched_,
                 ref_frames_matched_);
        cv::putText(frame, buf, cv::Point(12, frame.rows - 12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 165, 255), 1);
      }
    }

    if (show_insets_) {    
      // Draw P1 inset in top-right corner
      if (latest_results_.purkinje.p1_detected) {
	drawMagnifiedInset(frame, gray_for_inset,
			   latest_results_.purkinje.p1_center,
			   "P1",
			   cv::Point2f(frame.cols - 120, 35),
			   10);
      }
      
      // Draw P4 inset below P1 inset
      if (latest_results_.purkinje.p4_detected) {
	drawMagnifiedInset(frame, gray_for_inset,
			   latest_results_.purkinje.p4_center,
			   "P4",
			   cv::Point2f(frame.cols - 120, 155),
			   10);
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

	plugin->updateTimingThresholds();
	
	Tcl_PkgProvide(interp, "Eyetracking", "1.0");
        
        return TCL_OK;
    }
}
