namespace eval ::Registry {
    variable widgets
    set widgets [dict create]
    variable recording_state "idle"
    variable current_metadata_base ""
    variable blink_indicator -1
    variable p1_lost_indicator -1
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
            set roi_x [expr {$x - 200}]
            set roi_y [expr {$y - 200}]
            eyetracking::setROI $roi_x $roi_y 400 400
            puts "ROI set around $x,$y"
        }
        default {
            puts "Click at pixel ($x, $y)"
        }
    }
}

proc get_event_data {info key {default ""}} {
    if {[dict exists $info $key]} {
        return [dict get $info $key]
    }
    return $default
}

proc onEvent {event data} {
    # Parse data
    set info [dict create]
    foreach {key val} $data {
        dict set info $key $val
    }

    switch $event {
	"video_source_eof" {
            puts "📹 End of video reached"
            if {$::Registry::recording_state eq "recording"} {
                puts "💾 Auto-saving recording..."
                stop_metadata_recording
            }
	}
	
	"video_source_rewind" {
            puts "⏮  Video rewound - resetting tracking state"
            eyetracking::resetTrackingState
	}
	
	       "sampling_progress" {
            set sampled [get_event_data $info sampled 0]
            set total [get_event_data $info total 0]
            
            # Update a progress indicator widget if you have one
            if {[dict exists $::Registry::widgets sampling_progress]} {
                set widget [dict get $::Registry::widgets sampling_progress]
                update_widget_text $widget "Sampling: $sampled/$total"
            }
        }
        
        "sampling_complete" {
            set sampled [get_event_data $info sampled 0]
            set total [get_event_data $info total 0]
            puts "✅ Sampling complete: $sampled/$total frames"
            
            # Remove progress indicator if present
            if {[dict exists $::Registry::widgets sampling_progress]} {
                remove_widget [dict get $::Registry::widgets sampling_progress]
                dict unset ::Registry::widgets sampling_progress
            }
            
            # Switch to review mode
            review_mode
        }
        
        "eyetracking_blink_start" {
            set frame [dict get $info frame]
            
            # Add visual indicator
            if {$::Registry::blink_indicator == -1} {
                set ::Registry::blink_indicator \
                    [add_text -150 40 "BLINK" {255 255 100} 1.5 3]
            }
        }
        
        "eyetracking_blink_end" {
            # Remove visual indicator
            if {$::Registry::blink_indicator != -1} {
                remove_widget $::Registry::blink_indicator
                set ::Registry::blink_indicator -1
            }
        }
        
        "eyetracking_p1_lost" {
            set frame [dict get $info frame]
            puts "⚠️  P1 lost at frame $frame"
            
            # Add persistent warning
            if {$::Registry::p1_lost_indicator == -1} {
                set ::Registry::p1_lost_indicator \
                    [add_text -150 80 "P1 LOST" {255 80 80} 1.2 2]
            }
        }
        
        "eyetracking_p1_recovered" {
            set frame [dict get $info frame]
            puts "✅ P1 recovered at frame $frame"
            
            # Remove warning
            if {$::Registry::p1_lost_indicator != -1} {
                remove_widget $::Registry::p1_lost_indicator
                set ::Registry::p1_lost_indicator -1
            }
        }
        
        "eyetracking_p4_calibrated" {
            set samples [get_event_data $info samples 0]
            set magnitude [get_event_data $info magnitude 0.0]
            set angle [get_event_data $info angle 0.0]
            
            puts "✅ P4 Calibrated:"
            puts "   Samples: $samples"
            puts "   Magnitude ratio: [format %.3f $magnitude]"
            puts "   Angle offset: [format %.1f $angle]°"
                 
            # Auto-switch to full mode
            eyetracking::setDetectionMode full
        }
    }
}

proc accept_p4_sample { code } {
    if {[catch {eyetracking::acceptP4Sample} result]} {
        puts "❌ $result"
    } else {
        puts "✅ Sample $result added"
    }
}

proc calibrate_p4_model {} {
    set status [eyetracking::getP4ModelStatus]
    set count [dict get $status samples]
    set initialized [dict get $status initialized]
    
    # Check if already calibrated
    if {$initialized} {
        puts "✅ Model already calibrated"
        eyetracking::setDetectionMode full
        return
    }
    
    # Check if we have samples
    if {$count < 1} {
        puts "⚠️  Need at least 1 sample to calibrate"
        puts "  Shift-click P4 on one or more frames"
        return
    }
    
    # Calibrate the model
    puts "Calibrating P4 model with $count sample(s)..."
    if {[catch {eyetracking::calibrateP4Model} result]} {
        puts "❌ Calibration failed: $result"
        return
    }
    
    puts "✅ $result"
    
    # Show model parameters
    set status [eyetracking::getP4ModelStatus]
    if {[dict get $status initialized]} {
        set mag [dict get $status magnitude_ratio]
        set angle [dict get $status angle_offset_deg]
        puts "  Magnitude ratio: [format %.3f $mag]"
        puts "  Angle offset: [format %.1f $angle]°"
        
        # Now switch to full mode to use the model
        eyetracking::setDetectionMode full
        puts "✅ Switched to full tracking mode"
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

proc review_gui {} {
    clear_widgets
    clearRegistry
    
    clear_key_bindings
    review_update

    # Widgets
    add_text 20 40 "Review" {240 140 180} 1.0 2

    # Top row buttons
    add_button -260 -50 80 40 Live run_mode
    add_button -180 -50 80 40 Model calibrate_p4_model
    add_button -100 -50 80 40 Setup collect_samples
    
    set s \
        [add_int_slider -200 25 150 40 Frame 1 \
             [vstream::reviewCount] [expr {[vstream::reviewIndex]+1}] review_goto_frame]
    dict set ::Registry::widgets frame_slider $s

    set s \
        [add_int_slider 20 -50 150 40 \
             {Pupil Threshold} 1 255 [eyetracking::setPupilThreshold] eyetracking::setPupilThreshold]
    dict set ::Registry::widgets pupil_threshold_slider $s
    
    # Key Bindings
    bind_key $::keys::RIGHT review_next
    bind_key $::keys::LEFT review_previous
    bind_key $::keys::ENTER accept_p4_sample
}

proc review_mode {} {
	eyetracking::resetP4Model
	eyetracking::setDetectionMode pupil_p1
    vstream::startSource review
    review_gui
}

proc collect_samples { { n 16 } { interval_ms 700 } } {
    set random 1
    run_mode
    vstream::reviewClear
    
    # Add progress indicator
    set progress_widget [add_text 20 120 "Sampling: 0/$n" {100 255 100} 1.0 2]
    dict set ::Registry::widgets sampling_progress $progress_widget

    ::vstream::reviewSampleMultiple $n $interval_ms $random
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
        puts "❌ Failed to start metadata recording: $err"
        set ::Registry::recording_state "idle"
        return
    }
    
    set ::Registry::recording_state "recording"
    puts "✅ Started metadata recording: ${metadata_name}.db"
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
    vstream::startSource flir
}

proc stop_metadata_recording {} {
    if {[catch {vstream::fileClose} err]} {
        puts "❌ Failed to close recording: $err"
    } else {
        puts "✅ Saved: ${::Registry::current_metadata_base}.db"
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
    vstream::startSource flir
    puts "⏮  Rewound to beginning"
}

# ============================================================================
# PLAYBACK MODE
# ============================================================================

proc playback_mode { { filename {} } } {
    if { $filename == "" } {
        set filename $::source_file
    } else {
        set ::source_file $filename
    }
    
    # Stop any existing recording when switching modes
    if {$::Registry::recording_state ne "idle"} {
        stop_metadata_recording
    }
    
    clear_widgets
    clearRegistry
    clear_key_bindings

    vstream::startSource flir
    
    # Button row
    add_button -100 -50 80 40 Setup collect_samples
    add_button -260 -50 80 40 Rewind rewind_playback
    
    # Save button - main feature
    set save_btn [add_button -180 -50 80 40 "Save Run" toggle_recording]
    dict set ::Registry::widgets save_button $save_btn
    
    # Parameter sliders
    set s [add_int_slider 20 -50 150 40 \
         {Pupil Threshold} 1 255 [eyetracking::setPupilThreshold] eyetracking::setPupilThreshold]
    dict set ::Registry::widgets pupil_threshold_slider $s
    
    add_int_slider 20 -95 150 40 \
         {P4 Threshold} 1 255 [eyetracking::setP4MinIntensity] eyetracking::setP4MinIntensity
    
    add_float_slider 20 -140 150 40 \
         {P1 Max Jump} 5 100 [eyetracking::setP1MaxJump] eyetracking::setP1MaxJump
    
    add_float_slider 20 -190 150 40 \
         {P4 Max Jump} 5 100 [eyetracking::setP4MaxJump] eyetracking::setP4MaxJump
    
    # Key bindings
    bind_key "s" toggle_recording
    bind_key "r" rewind_playback
    
    # Show instructions
    puts ""
    puts "============================================"
    puts "Metadata Recording Mode"
    puts "============================================"
    puts "1. Adjust parameters with sliders"
    puts "2. Click 'Save Run' (or press 's')"
    puts "3. Video rewinds and plays through"
    puts "4. Click 'Save Run' again when done"
    puts "5. Adjust parameters for next run"
    puts "6. Repeat!"
    puts ""
    puts "Files saved as: [file tail $filename]_YYYYMMDD_HHMMSS.db"
    puts "============================================"
    puts ""
}

# ============================================================================
# INITIALIZATION
# ============================================================================

load [file dir [info nameofexecutable]]/plugins/eyetracking[info sharedlibextension]

# Default parameters
eyetracking::setP1MaxJump 40
eyetracking::setP4MaxJump 40
eyetracking::setP4MinIntensity 42
eyetracking::setPupilThreshold 45
eyetracking::setDetectionMode pupil_p1
eyetracking::resetP4Model

# Start with first video
run_mode

# We have already calibrated this P4 model
eyetracking::setP4Model .421 169.5
eyetracking::setDetectionMode full

# For now, save metadata/analysis for all frames
vstream::onlySaveInObs 0

vstream::configureROI 720 450 200 200
vstream::configureExposure 2000
vstream::configureGain 0.9
vstream::frameRate 60
