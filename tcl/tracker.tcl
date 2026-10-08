#
# NAME
#  eyetracker.tcl
#

source [file join [file dirname [info script]] et_keys.tcl]

# Camera backend for live mode: flir (Spinnaker) or lucid (Arena SDK).
# Override before sourcing this file, or from the console.
if {![info exists ::camera_type]} { set ::camera_type flir }
source [file join [file dirname [info script]] et_camera.tcl]
# Settings changed in the browser viewer are kept by the server, here.
source [file join [file dirname [info script]] viewer_settings.tcl]
# The dserv connection (forwarding + ess/in_obs, ess/datafile subscription),
# also chosen from the viewer.
source [file join [file dirname [info script]] dserv_link.tcl]

# Live-camera settings per backend, applied once by go_live (see et_camera.tcl).
# Lucid: 430 us exposure at 250 Hz (4001 us frame time), Line1 drives the IR
# source for the duration of the exposure (ExposureActive, inverted for the
# active-low driver input) so illumination and shutter are synced, and the
# camera clock follows the LAN's PTP grandmaster (the dserv host) so frame
# timestamps are comparable with dserv datapoint timestamps.
set ::camera_live_settings {
    flir  {exposure_us 700.0 gain_db 8.5 orientation {1 0} binning {2 2} fps 250.0
           ptp {slave_only 1 wait_s 20}}
    lucid {exposure_us 430.0 gain_db 8.5 orientation {1 0} binning {2 2} frame_time_us 4001
           strobe {line 1 mode Output source ExposureActive inverter 1}
           ttl_line 0
           ptp {slave_only 1 wait_s 20}}
}

# Where obs on/off comes from (vstream::obsSource):
#   line       the camera's TTL input, latched per frame (hardware; the FLIR rig)
#   timestamp  dserv's ess/in_obs datapoints matched to frames by timestamp --
#              needs the camera on PTP with the dserv host (the Lucid rig)
#   dserv      ess/in_obs applied on arrival (no wire, no PTP; ~ms late)
# The TTL bit is recorded per frame whichever source is used, so
# scripts/obs_compare.py can check timestamp mode against the wire.
if {![info exists ::obs_source]} {
    set ::obs_source [expr {$::camera_type eq "lucid" ? "timestamp" : "line"}]
}

namespace eval ::Registry {
    variable widgets
    set widgets [dict create]
    variable recording_state "idle"
    variable current_metadata_base ""
    variable blink_indicator -1
    variable p1_lost_indicator -1
    variable in_obs_indicator -1
    variable paused 0
    variable camera_initialized 0
    variable datafile {}
    variable datafile_indicator -1
    variable video_folder /home/lab/Videos
}

# proc to clear all
proc clearRegistry {} {
    set ::Registry::widgets [dict create]
}

proc onMouseClick {x y modifier} {
    set mode [eyetracking::setDetectionMode]
    
    switch $modifier {
        "shift" {
            # Mark P4 sample position
            if {$mode eq "pupil_p1" || $mode eq "full"} {
                eyetracking::markP4Sample $x $y
                puts "P4 sample marked at ($x,$y) - press Enter to add"
            } else {
                puts "P4 calibration requires at least pupil_p1 mode"
            }
        }
        "alt" {
            # Sample pixel intensity and set pupil threshold just above
            set intensity [vstream::getPixelIntensity $x $y]
            if {$intensity >= 0} {
                eyetracking::setPupilThreshold [expr {$intensity+10}]
                puts "Pupil threshold set to $intensity (clicked at $x,$y)"
            } else {
                puts " Could not sample pixel at ($x,$y)"
            }
        }
        default {
	    
        }
    }
}


# Helper to safely get value from dict
proc get_dict_value {dict_var key {default ""}} {
    if {[dict exists $dict_var $key]} {
        return [dict get $dict_var $key]
    }
    return $default
}

proc obs_indicator { status } {
    if { $status } { 
	if {$::Registry::in_obs_indicator == -1} {
	    set ::Registry::in_obs_indicator \
		[add_circle -16 16 7 {240 10 10} -1]
	}
    } else {
	if {$::Registry::in_obs_indicator != -1} {
	    remove_widget $::Registry::in_obs_indicator
	    set ::Registry::in_obs_indicator -1
	}
    }
}

proc datafile_indicator { filename } {
    if { $filename != "" } { 
	if {$::Registry::datafile_indicator == -1} {
	    set ::Registry::datafile_indicator \
		[add_text 200 20 $filename {255 255 255} 0.6 1]
	}
    } else {
	if {$::Registry::datafile_indicator != -1} {
	    remove_widget $::Registry::datafile_indicator
	    set ::Registry::datafile_indicator -1
	}
    }
}

proc open_datafile { filename } {
    set folder $::Registry::video_folder

    vstream::fourcc AVC1
    
    if {[file extension $filename] eq ""} {
        append filename ".mp4"
    }

    set fullpath [file join $folder $filename]

    # can also store only metadata
    #  ::vstream::fileOpenMetadata $filename $source

    ::vstream::fileOpen $fullpath
    datafile_indicator $fullpath
}

proc close_datafile {} {
    ::vstream::fileClose
    datafile_indicator {}
}

# The ESS datafile changed (subscription push, or the reconcile poll below).
#
# The analysis plugins' per-session state -- for eyetracking, the anchor
# that makes eyetracking/results frame_id/time relative to the datafile --
# used to reset ONLY inside vstream::fileOpen, i.e. only if the video
# recording actually opened.  Anything that kept the video from opening
# (recording off, folder missing, a stale Registry::datafile after a missed
# close) left the anchor aging since some earlier session.  Now every
# transition to a NEW datafile name resets the plugins first,
# unconditionally; opening the video file is a separate, optional
# consequence (its own fileOpen reset is then redundant but harmless).
#
# A repeat of the same name (dserv re-delivery, a reconcile that finds
# nothing changed) is a no-op: a mid-session reset would split the anchor.
proc datafile_changed { data } {
    set current $::Registry::datafile
    if { $data eq $current } { return }

    if { $data ne "" } {
	if { $current ne "" } {
	    # the previous file's close never reached us (dead subscription,
	    # dserv restart between sessions): finish it before the new one
	    ds_log "datafile '$current' closed without notice; closing it before '$data'"
	    set ::Registry::datafile {}
	    close_datafile
	}
	set ::Registry::datafile $data
	vstream::setViewerDatafile $data
	vstream::resetPlugins
	ds_log "datafile open: $data (plugins reset)"
	open_datafile $data
    } else {
	set ::Registry::datafile {}
	vstream::setViewerDatafile {}
	close_datafile
	ds_log "datafile closed"
    }
}

proc handle_dpoint {event_name event_data} {
    set dict_data [jsonToTclDict $event_data]
    set name [dict get $dict_data name]
    set data [dict get $dict_data data]

    switch -glob $name {
        "ess/in_obs" {
	    # this allows dataserver to set obs status
	    # FLIR will use DIO, but other sources can use this
	    #set ::vstream::dsInObs $data
	}
	"ess/datafile" {
	    datafile_changed $data
	}
    }
}

# handles native Tcl types
proc onEvent {type data} {

    switch -glob $type {
	"ds/*" {
	    handle_dpoint $type $data
	}

	"vstream/begin_obs" { obs_indicator 1 }
	"vstream/end_obs"   { obs_indicator 0 }
	
        "vstream/video_source_eof" {
            puts "🎬 End of video reached"
            if {$::Registry::recording_state eq "recording"} {
                puts "Auto-saving recording..."
                stop_metadata_recording
            }
        }
        
        "vstream/video_source_rewind" {
            puts "⏮ Video rewound - resetting tracking state"
            eyetracking::resetTrackingState
        }
        
        "vstream/sampling_progress" {
            # data is a dict with keys: sampled, total
            set sampled [get_dict_value $data sampled 0]
            set total [get_dict_value $data total 0]
            
            # Update progress indicator widget
            if {[dict exists $::Registry::widgets sampling_progress]} {
                set widget [dict get $::Registry::widgets sampling_progress]
                update_widget_text $widget "Sampling: $sampled/$total"
            }
        }
        
        "eyetracking/blink_start" {
            # data is an integer (frame number)
            puts "Blink at frame $data"
            
            # Add visual indicator
            if {$::Registry::blink_indicator == -1} {
                set ::Registry::blink_indicator \
                    [add_text 10 40 "BLINK" {255 255 100} 1.5 3]
            }
        }
        
        "eyetracking/blink_end" {
            # Remove visual indicator
            if {$::Registry::blink_indicator != -1} {
                remove_widget $::Registry::blink_indicator
                set ::Registry::blink_indicator -1
            }
        }
        
        "eyetracking/p1_lost" {
            # data is an integer (frame number)
            puts "P1 lost at frame $data"
            
            # Add persistent warning
            if {$::Registry::p1_lost_indicator == -1} {
                set ::Registry::p1_lost_indicator \
                    [add_text -150 80 "P1 LOST" {255 80 80} 1.2 2]
            }
        }
        
        "eyetracking/p1_recovered" {
            # data is an integer (frame number)
            puts "P1 recovered at frame $data"
            
            # Remove warning
            if {$::Registry::p1_lost_indicator != -1} {
                remove_widget $::Registry::p1_lost_indicator
                set ::Registry::p1_lost_indicator -1
            }
        }
        
        "eyetracking/p4_calibrated" {
            # data is a dict with keys: samples, magnitude, angle
            set samples [get_dict_value $data samples 0]
            set magnitude [get_dict_value $data magnitude 0.0]
            set angle [get_dict_value $data angle 0.0]
            
            puts "P4 Calibrated:"
            puts "   Samples: $samples"
            puts "   Magnitude ratio: [format %.3f $magnitude]"
            puts "   Angle offset: [format %.1f $angle]°"
                 
            # Auto-switch to full mode
            eyetracking::setDetectionMode full
        }
        
	"eyetracking/settings" {
	    set setting_name [get_dict_value $data name ""]
	    set setting_value [get_dict_value $data value ""]

	    switch $setting_name {
		"pupil_threshold" {
		    if {[dict exists $::Registry::widgets pupil_threshold_slider]} {
			set slider [dict get $::Registry::widgets pupil_threshold_slider]
			update_slider_value $slider $setting_value
		    }
		}
		"p1_max_jump" {
		    if {[dict exists $::Registry::widgets p1_max_jump_slider]} {
			set slider [dict get $::Registry::widgets p1_max_jump_slider]
			update_slider_value $slider $setting_value
		    }
		}
		"p4_max_jump" {
		    if {[dict exists $::Registry::widgets p4_max_jump_slider]} {
			set slider [dict get $::Registry::widgets p4_max_jump_slider]
			update_slider_value $slider $setting_value
		    }
		}
		"p4_min_intensity" {
		    if {[dict exists $::Registry::widgets p4_threshold_slider]} {
			set slider [dict get $::Registry::widgets p4_threshold_slider]
			update_slider_value $slider $setting_value
		    }
		}
		"detection_mode" {
		    puts "Mode changed to: $setting_value"
		}
	    }
	} 
    }
}

proc do_exit { args } { ::vstream::exit }

proc toggle_pause { args } {
    set ::Registry::paused [expr {!$::Registry::paused}]
    
    if {$::Registry::paused} {
        vstream::pause 1
        puts "⏸️ PAUSED - Use arrow keys to step frame-by-frame"
        
        # Add visual indicator
        if {![dict exists $::Registry::widgets pause_indicator]} {
            set indicator [add_text 20 120 "PAUSED" {255 255 100} 1.0 2]
            dict set ::Registry::widgets pause_indicator $indicator
        }
    } else {
        vstream::pause 0
        puts "▶️ Playing"
        
        # Remove indicator
        if {[dict exists $::Registry::widgets pause_indicator]} {
            remove_widget [dict get $::Registry::widgets pause_indicator]
            dict unset ::Registry::widgets pause_indicator
        }
    }
}

# Step forward
proc step_forward { code } {
    if {!$::Registry::paused} {
        puts "Pause first (press Space)"
        return
    }
    
    vstream::step 1
    set frame [vstream::getCurrentFrame]
    puts "→ Frame $frame"
}

# Step backward
proc step_backward { code } {
    if {!$::Registry::paused} {
        puts "Pause first (press Space)"
        return
    }
    
    vstream::step -1
    set frame [vstream::getCurrentFrame]
    puts "← Frame $frame"
}

# Show current frame info
proc show_frame_info { code } {
    set frame [vstream::getCurrentFrame]
    set total [vstream::getTotalFrames]
    set results [eyetracking::getResults]
    
    puts "\n═══════════════════════════════════════════"
    puts "Frame $frame / $total"
    puts "═══════════════════════════════════════════"
    
    if {$results eq "no results"} {
        puts "No tracking results available"
        return
    }
    
    # Pupil
    if {[dict exists $results pupil]} {
        set pupil [dict get $results pupil]
        puts "Pupil: ([format %.1f [dict get $pupil x]], [format %.1f [dict get $pupil y]]) r=[format %.1f [dict get $pupil radius]]"
    } else {
        puts "Pupil: NOT DETECTED"
    }
    
    # P1
    if {[dict exists $results p1]} {
        set p1 [dict get $results p1]
        puts "P1: ([format %.1f [dict get $p1 x]], [format %.1f [dict get $p1 y]])"
    } else {
        puts "P1: NOT DETECTED"
    }
    
    # P4
    if {[dict exists $results p4]} {
        set p4 [dict get $results p4]
        puts "P4: ([format %.1f [dict get $p4 x]], [format %.1f [dict get $p4 y]])"
    } else {
        puts "P4: NOT DETECTED"
    }
    
    # Blink
    if {[dict exists $results in_blink]} {
        puts "Blink: [expr {[dict get $results in_blink] ? 1 : 0 }]"
    }
    
    eyetracking::debugNextFrame
    
    puts "═══════════════════════════════════════════\n"
}

proc reset_tracking { code } {
    eyetracking::resetP4Model
    eyetracking::setDetectionMode pupil_only
    eyetracking::setDetectionMode pupil_p1
}

proc accept_p4_sample { args } {
    if {[catch {eyetracking::acceptP4Sample} result]} {
        puts "$result"
    } else {
        puts "Sample $result added"
    }
}

proc reset_p4_model { args } {
    eyetracking::resetP4Model
    eyetracking::setDetectionMode pupil_p1
}

proc calibrate_p4_model { args } {
    set status [eyetracking::getP4ModelStatus]
    set count [dict get $status samples]
    set initialized [dict get $status initialized]
    
    # Check if already calibrated
    if {$initialized} {
        puts "Model already calibrated"
        eyetracking::setDetectionMode full
        return
    }
    
    # Check if we have samples
    if {$count < 1} {
        puts "Need at least 1 sample to calibrate"
        puts "  Shift-click P4 on one or more frames"
        return
    }
    
    # Calibrate the model
    puts "Calibrating P4 model with $count sample(s)..."
    if {[catch {eyetracking::calibrateP4Model} result]} {
        puts "Calibration failed: $result"
        return
    }
    
    puts "$result"
    
    # Show model parameters
    set status [eyetracking::getP4ModelStatus]
    if {[dict get $status initialized]} {
        set mag [dict get $status magnitude_ratio]
        set angle [dict get $status angle_offset_deg]
        puts "  Magnitude ratio: [format %.3f $mag]"
        puts "  Angle offset: [format %.1f $angle]°"
        
        # Now switch to full mode to use the model
        eyetracking::setDetectionMode full
        puts "Switched to full tracking mode"
    }
}

proc review_next { code } {
    ::vstream::reviewNext
    eyetracking::clearP4Sample
    slider_update
}

proc review_previous { code } {
    ::vstream::reviewPrevious
    eyetracking::clearP4Sample
    slider_update
}

proc review_goto_frame { frame } {
    ::vstream::reviewJumpTo [expr {$frame-1}]
    eyetracking::clearP4Sample
    review_update
    slider_update
}

proc review_update {} {
    set_variable total_frames [::vstream::reviewCount]
    set_variable current_frame [expr {[::vstream::reviewIndex]+1}]
}

proc slider_update {} {
    set s [dict get $::Registry::widgets frame_slider]
    update_slider_vals $s 1 [vstream::reviewCount] [expr {[vstream::reviewIndex]+1}]
}

# ============================================================================
# METADATA RECORDING FUNCTIONS
# ============================================================================

proc start_metadata_recording {} {
    set base_name [file rootname [file tail $::source_file]]
    set timestamp [clock format [clock seconds] -format "%Y%m%d_%H%M%S"]
    set metadata_name "${base_name}_${timestamp}"
    
    set ::Registry::current_metadata_base $metadata_name
    
    if {[catch {
        vstream::fileUseSQLite 1
        vstream::fileOpenMetadata $metadata_name $::source_file
    } err]} {
        puts "Failed to start metadata recording: $err"
        set ::Registry::recording_state "idle"
        return
    }
    
    set ::Registry::recording_state "recording"
    puts "Started metadata recording: ${metadata_name}.db"
    puts "   Source: $::source_file"
    
    # Rewind video to beginning
    vstream::stopSource
    
    # Update button text
    if {[dict exists $::Registry::widgets save_button]} {
        set btn [dict get $::Registry::widgets save_button]
        update_widget_text $btn "Close"
    }
    
    # Add recording indicator
    if {![dict exists $::Registry::widgets recording_indicator]} {
        set indicator [add_text 20 80 "REC" {255 80 80} 0.8 2]
        dict set ::Registry::widgets recording_indicator $indicator
    }

    vstream::fileStartRecording
    vstream::startSource playback file $::source_file speed 1.0 loop 0
}

proc stop_metadata_recording {} {
    if {[catch {vstream::fileClose} err]} {
        puts "Failed to close recording: $err"
    } else {
        puts "Saved: ${::Registry::current_metadata_base}.db"
    }
    
    set ::Registry::recording_state "idle"
    
    # Update button text
    if {[dict exists $::Registry::widgets save_button]} {
        set btn [dict get $::Registry::widgets save_button]
        update_widget_text $btn "Save Run"
    }
    
    # Remove recording indicator
    if {[dict exists $::Registry::widgets recording_indicator]} {
        remove_widget [dict get $::Registry::widgets recording_indicator]
        dict unset ::Registry::widgets recording_indicator
    }
}

proc toggle_recording {} {
    if {$::Registry::recording_state eq "idle"} {
        start_metadata_recording
    } else {
        stop_metadata_recording
    }
}

proc rewind_playback {} {
    vstream::stopSource
    vstream::startSource playback file $::source_file speed 1.0 loop 0
    puts "⏮ Rewound to beginning"
}

# ============================================================================
# LIVE / ROI Control
# ============================================================================

namespace eval ::ROI {
    variable step -1  ;# Will be set from camera constraints
    
    proc init {} {
        variable step
        
        # Get increment from camera
        if {[catch {camera::getROIConstraints} c]} {
            puts "Warning: Could not get ROI constraints, using default step=8"
            set step 8
            return
        }
        
        # Use offset increment (usually same for x and y)
        set step [dict get $c offset_x_inc]
        puts "ROI nudge step set to $step (from camera constraints)"
    }
    
    proc get_step {} {
        variable step
        
        # Lazy initialization
        if {$step == -1} {
            init
        }
        
        return $step
    }

    proc nudge {dx dy} {
        set step [get_step]
        set dx [expr {$dx * $step}]
        set dy [expr {$dy * $step}]
        
        set roi [camera::getROI]
        set x [dict get $roi offset_x]
        set y [dict get $roi offset_y]
        
        set new_x [expr {$x + $dx}]
        set new_y [expr {$y + $dy}]
        
        # Use the offset-only command (safe during streaming)
        if {[catch {
            camera::setROIOffset $new_x $new_y
        } err]} {
            # Silently fail
        }
    }

    proc nudgeLeft  { args } { nudge -1 0 }
    proc nudgeRight { args } { nudge 1 0 }
    proc nudgeUp  { args } { nudge 0 1 }
    proc nudgeDown  { args } { nudge 0 -1 }

    proc center_on_pupil { args } {
        set step [get_step]
        
        # Get current ROI
        set roi [camera::getROI]
        set w [dict get $roi width]
        set h [dict get $roi height]
        set current_offset_x [dict get $roi offset_x]
        set current_offset_y [dict get $roi offset_y]
        
        # Get latest results
        set results [eyetracking::getResults]
        
        if {$results eq "no results"} {
            puts "No tracking results available"
            return
        }
        
        if {![dict exists $results pupil]} {
            puts "No valid pupil detected"
            return
        }
        
        set pupil [dict get $results pupil]
        set px [dict get $pupil x]
        set py [dict get $pupil y]
        
        # Pupil position is in ROI-local coordinates!
        # Convert to sensor coordinates:
        set pupil_sensor_x [expr {$current_offset_x + $px}]
        set pupil_sensor_y [expr {$current_offset_y + $py}]
        
        puts "Pupil in ROI coords: ($px, $py)"
        puts "Pupil in sensor coords: ($pupil_sensor_x, $pupil_sensor_y)"
        
        # Calculate new offset to center pupil in ROI
        set center_x [expr {$w / 2}]
        set center_y [expr {$h / 2}]
        
        set new_offset_x [expr {int($pupil_sensor_x - $center_x)}]
        set new_offset_y [expr {int($pupil_sensor_y - $center_y)}]
        
        puts "Centering pupil..."
        
        if {[catch {
            camera::setROIOffset $new_offset_x $new_offset_y
        } err]} {
            puts "ROI center failed: $err"
        } else {
            puts "ROI centered at offset ($new_offset_x, $new_offset_y)"
        }
    }
}

# What the viewer's "reset camera" returns to: the scripted rig settings.
proc ::vs::camera_defaults {} {
    ::et_camera::apply_live_settings \
        [::et_camera::settings_for_backend $::camera_live_settings]
}

proc go_live {} {
    set initialized $::Registry::camera_initialized
    
    if { !$initialized } {
        ::et_camera::apply_live_settings \
            [::et_camera::settings_for_backend $::camera_live_settings]
        # exposure / gain / frame rate / binning saved from the viewer win over
        # the rig values above; strobe, TTL line and PTP stay as scripted
        ::vs::apply_camera
        set ::Registry::camera_initialized 1
    }
    
    camera::startAcquisition

    # ROI control buttons (compact arrows)
    set use_roi 0
    if { $use_roi } {
        camera::configureROI 720 450 24 24; # width, height, offsetx, offsety
	
	add_button -320 -50 30 30 "v" {::ROI::nudgeDown}
	add_button -320 -85 30 30 "^" {::ROI::nudgeUp}
	add_button -350 -67 30 30 "<" {::ROI::nudgeLeft}
	add_button -290 -67 30 30 ">" {::ROI::nudgeRight}
	
	bind_key $::keys::DOWN {::ROI::nudgeDown}
	bind_key $::keys::UP {::ROI::nudgeUp}
	bind_key $::keys::LEFT {::ROI::nudgeLeft}
	bind_key $::keys::RIGHT {::ROI::nudgeRight}
    }
}

# Browser viewer (web/) source controls: the same commands serve.tcl
# provides (camera/file picker, playback speed, Stop and Play-after-Stop).
# A live camera is (re)opened with this rig's settings (strobe, exposure,
# PTP), since a reopened device is set up from scratch.  While an ESS
# datafile is open the live recording is protected: no switching or stopping.
set ::viewer_last_kind camera     ;# camera | playback
set ::viewer_live_id 0
set ::viewer_live_serial ""
set ::viewer_playback_speed 1.0

proc viewer_check_not_recording {} {
    if {$::Registry::datafile ne ""} {
        return -code error "recording to $::Registry::datafile; close the datafile first"
    }
}

proc viewer_start_live {} {
    if {$::camera_type eq "webcam"} {
        vstream::startSource webcam id $::viewer_live_id
    } elseif {$::viewer_live_serial ne ""} {
        vstream::startSource $::camera_type id $::viewer_live_id serial $::viewer_live_serial
    } else {
        vstream::startSource $::camera_type id $::viewer_live_id
    }
    if {$::camera_type ne "webcam"} {
        set ::Registry::camera_initialized 0
        go_live
    }
    eyetracking::resetTrackingState
    vstream::pause 0
    set ::Registry::paused 0
}

proc switch_to_camera {vendor id {serial ""}} {
    viewer_check_not_recording
    if {$vendor ni {webcam flir lucid}} {
        return -code error "unknown camera vendor: $vendor"
    }
    set ::camera_type $vendor
    set ::viewer_live_id $id
    set ::viewer_live_serial $serial
    set ::viewer_last_kind camera
    viewer_start_live
}

proc switch_to_playback {path} {
    viewer_check_not_recording
    set ::source_file $path
    set ::viewer_last_kind playback
    vstream::startSource playback file $path speed $::viewer_playback_speed loop 1 rate_limited 1
    eyetracking::resetTrackingState
    vstream::pause 0
    set ::Registry::paused 0
}

proc set_playback_speed {spd} {
    set ::viewer_playback_speed $spd
    if {[vstream::getSourceType] eq "playback"} {
        vstream::setPlaybackSpeed $spd
    }
}

proc stop_active_source {} {
    viewer_check_not_recording
    if {[vstream::getSourceType] ne ""} {
        vstream::stopSource
    }
    return stopped
}

proc resume_stopped_source {} {
    if {[vstream::getSourceType] ne ""} {
        vstream::pause 0
        set ::Registry::paused 0
        return running
    }
    if {$::viewer_last_kind eq "playback"} {
        switch_to_playback $::source_file
    } else {
        viewer_start_live
    }
    return running
}

# ============================================================================
# LIVE MODE
# ============================================================================

proc live_mode { } {
    # Stop any existing recording when switching modes
    if {$::Registry::recording_state ne "idle"} {
        stop_metadata_recording
    }

    vstream::startSource $::camera_type
    
    set ::Registry::paused 0  ;# Reset pause state
    
#    clear_widgets
#    clearRegistry
#    clear_key_bindings

    # Button row
    add_button -340 -50 80 40 Accept accept_p4_sample
    add_button -260 -50 80 40 Model calibrate_p4_model
    add_button -180 -50 80 40 Reset reset_p4_model
    add_button -100 -50 80 40 Pause toggle_pause
    
#    set save_btn [add_button -180 -50 80 40 "Save Run" toggle_recording]
#    dict set ::Registry::widgets save_button $save_btn
    
    # Parameter sliders
    set s [add_int_slider 20 -50 150 40 \
	       {Pupil Threshold} 1 255 [eyetracking::setPupilThreshold] eyetracking::setPupilThreshold]
    dict set ::Registry::widgets pupil_threshold_slider $s
    
    set s [add_int_slider 20 -95 150 40 \
	       {P4 Threshold} 1 255 [eyetracking::setP4MinIntensity] eyetracking::setP4MinIntensity]
    dict set ::Registry::widgets p4_threshold_slider $s
    
    set s [add_int_slider 20 -140 150 40 \
	       {P1 Min Intensity} 1 255 [eyetracking::setP1MinIntensity] eyetracking::setP1MinIntensity]
    dict set ::Registry::widgets p1_threshold_slider $s
    
    set s [add_float_slider 20 -185 150 40 \
	       {P1 Max Jump} 5 100 [eyetracking::setP1MaxJump] eyetracking::setP1MaxJump]
    dict set ::Registry::widgets p1_max_jump_slider $s

    # P1 blob area gate (px^2); depends on the camera's focal length
    set s [add_float_slider 20 -230 150 40 \
	       {P1 Min Area} 5 200 [eyetracking::setP1MinArea] eyetracking::setP1MinArea]
    dict set ::Registry::widgets p1_min_area_slider $s
    
    # Key bindings
    bind_key "s" toggle_recording
    bind_key "r" reset_tracking
    bind_key "m" calibrate_p4_model
    bind_key " " toggle_pause              ;# SPACE to pause/resume
    bind_key $::keys::RIGHT step_forward   ;# Arrow to step when paused
    bind_key $::keys::LEFT step_backward   ;# Arrow to step when paused
    bind_key "I" show_frame_info           ;# 'i' for info
    ::et::bind_overlay_keys               ;# i = insets, f = focus (shared)
    bind_key $::keys::ENTER accept_p4_sample
    eyetracking::resetP4Model
    
    go_live
}


# ============================================================================
# DATA SERVER CONNECTION
# ============================================================================

#
# Connecting, subscribing (ess/in_obs, ess/datafile) and the dead-subscription
# watchdog are in dserv_link.tcl (dserv_connect / dserv_startup), shared with
# serve.tcl and driven from the viewer too. What is specific to the tracker:
# after every (re)registration, ask dserv for the current datafile, since a
# push that happened while the subscription was dead never arrives.

proc ds_log { msg } { ::dsl::log $msg }

# Ask dserv for the datafile right now and act on any difference from what
# we believe -- catches an open (or close) that happened while the
# subscription was dead, and a tracker started mid-session.
proc ds_reconcile_datafile {} {
    if { $::dsl::host eq "" } { return }
    if { [catch { vstream::dsGet $::dsl::host ess/datafile $::dsl::port } reply] } {
	return
    }
    if { [catch { ds_get_value $reply ess/datafile } df] } {
	ds_log "unexpected %get reply for ess/datafile: '$reply'"
	return
    }
    datafile_changed $df
}

# Value from a dserv %get reply.  dsGet returns the raw reply line,
#   "<status> <name> <dtype> <timestamp> <length> {<data>}"
# (e.g. "1 ess/datafile 1 1786666050447124 0 {}" for a closed datafile), so
# the data is the last field -- NOT the whole line.  Passing the whole line
# to datafile_changed opened a bogus recording named after it on every start
# with no datafile open (2026-09-23).  Status 0 means the datapoint does not
# exist; treat that as empty.
proc ds_get_value { reply name } {
    if { ![string is list -strict $reply] || [llength $reply] < 6 } {
	error "not a %get reply"
    }
    lassign $reply status dpname
    if { $dpname ne $name } {
	error "reply is for '$dpname', not '$name'"
    }
    if { $status == 0 } { return "" }
    return [lindex $reply 5]
}

# dserv_link.tcl hook: after each (re)registration
proc dserv_on_subscribed {} { ds_reconcile_datafile }

# ============================================================================
# INITIALIZATION
# ============================================================================

load [file dir [info nameofexecutable]]/plugins/eyetracking[info sharedlibextension]

# Where the viewer's file picker looks for eye-tracking recordings (as in
# serve.tcl): the lab data share, and under WSL each Windows user's
# Videos/eye_tracking.
set places [list "Eye tracking videos" /mnt/analysis/data/eye_tracking]
foreach d [glob -nocomplain -types d /mnt/c/Users/*/Videos/eye_tracking] {
    lappend places "Eye tracking videos" $d
}
vstream::mediaPlaces $places
unset -nocomplain places d

# Default detector parameters, in code. The browser's "Reset to defaults"
# comes back here (apply_default_tuning, below, adds the rig-local file).
proc apply_code_defaults {} {
    eyetracking::setP1MaxJump 24
    eyetracking::setP1MinIntensity 145
    eyetracking::setP4MaxJump 100
    eyetracking::setP4MinIntensity 24
    eyetracking::setPupilThreshold 32
    eyetracking::setDetectionMode pupil_p1
    eyetracking::setP4MaxPredictionError 40

    # Per-backend detector defaults: the Lucid rig's shorter focal length makes
    # P1 smaller than on the FLIR rig (plugin default 40 px^2).
    if { $::camera_type eq "lucid" } {
        eyetracking::setP1MinArea 25
    }
}

proc apply_default_tuning {} {
    apply_code_defaults
    if { $::tracker_local_file ne "" && [file exists $::tracker_local_file] } {
        source $::tracker_local_file
    }
}

eyetracking::setROI 160 80 430 365
set ::vs::default_roi {160 80 430 365}
eyetracking::resetP4Model
apply_code_defaults

# Rig-local overrides, kept out of git: tracker_local.tcl is sourced after the
# defaults above, so per-rig values (thresholds, areas, ROI) survive updates.
# Looked for next to this script, then in ~/.config/videostream/.
# save_detector_settings writes the current values to the first of those
# that is writable (an installed /usr/local/videostream/tcl is root-owned, so
# that usually means the per-user file).
set ::tracker_local_candidates \
    [list [file join [file dirname [info script]] tracker_local.tcl] \
	  [file join $::env(HOME) .config videostream tracker_local.tcl]]
set ::tracker_local_file ""
foreach f $::tracker_local_candidates {
    if { [file exists $f] } {
	puts "Loading rig-local settings from $f"
	source $f
	set ::tracker_local_file $f
	break
    }
}
if { $::tracker_local_file eq "" } {
    foreach f $::tracker_local_candidates {
	if { [file writable [file dirname $f]] || ![file exists [file dirname $f]] } {
	    set ::tracker_local_file $f
	    break
	}
    }
}

# Keep the detector parameters currently in effect (sliders included) as the
# saved settings, so they are restored at the next start. This is what the old
# display UI's save button does; the browser viewer saves every change as it
# is made. (tracker_local.tcl above is still read, as a lower layer.)
proc save_detector_settings {} {
    dict for {name setter} $::vs::detector_setter {
        if { ![catch {eyetracking::$setter} v] } {
            ::vs::put det.$name $v
        }
    }
    ::vs::flush
    puts "Saved detector settings to [::vs::path]"
}

# Saved browser-viewer settings go on top of the defaults above.
::vs::load
::vs::apply_saved
vstream::addShutdownCmd ::vs::flush

vstream::onlySaveInObs 0
vstream::obsSource $::obs_source

# --ds-host if given, else the dserv last chosen in the viewer
dserv_startup

live_mode

