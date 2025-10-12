namespace eval ::Registry {
    variable widgets
    set widgets [dict create]
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

proc accept_p4_sample { code } {
    if {[catch {eyetracking::acceptP4Sample} result]} {
        puts "❌ $result"
    } else {
        puts "✓ $result"
    }
}

proc calibrate_p4_model {} {
    set status [eyetracking::getP4ModelStatus]
    set count [dict get $status samples]
    set initialized [dict get $status initialized]
    
    # Check if already calibrated
    if {$initialized} {
        puts "✓ Model already calibrated"
        eyetracking::setDetectionMode full
        return
    }
    
    # Check if we have samples
    if {$count < 1} {
        puts "⚠ Need at least 1 sample to calibrate"
        puts "  Shift-click P4 on one or more frames"
        return
    }
    
    # Calibrate the model
    puts "Calibrating P4 model with $count sample(s)..."
    if {[catch {eyetracking::calibrateP4Model} result]} {
        puts "❌ Calibration failed: $result"
        return
    }
    
    puts "✓ $result"
    
    # Show model parameters
    set status [eyetracking::getP4ModelStatus]
    if {[dict get $status initialized]} {
        set mag [dict get $status magnitude_ratio]
        set angle [dict get $status angle_offset_deg]
        puts "  Magnitude ratio: [format %.3f $mag]"
        puts "  Angle offset: [format %.1f $angle]°"
        
        # Now switch to full mode to use the model
        eyetracking::setDetectionMode full
        puts "✓ Switched to full tracking mode"
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

proc detect_p4 { v } {
    if { $v } {
	eyetracking::setDetectionMode full
    } else {
	eyetracking::setDetectionMode pupil_p1
    }
}

proc review_gui {} {
    clear_widgets
    clearRegistry
    
    clear_key_bindings
    review_update

    # Widgets
    add_text 20 40 "Review" {240 140 180} 1.0 2

    add_button -100 -50 80 40 Live playback_mode
    add_button -180 -50 80 40 Model calibrate_p4_model
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
    vstream::startSource review
    review_gui
}

proc collect_samples { { n 16 } { interval_ms 700 } } {
    set random 1
    playback_mode
    vstream::reviewClear
    ::vstream::reviewSampleCallback review_mode
    ::vstream::reviewSampleMultiple $n $interval_ms $random
}

proc playback_mode { { filename {} } } {
    if { $filename == "" } {
	set filename $::source_file
    } else {
	set ::source_file $filename
    }
    clear_widgets
    clearRegistry
    clear_key_bindings

    vstream::startSource playback file $filename speed 1
    
    add_button -100 -50 80 40 Setup collect_samples
    add_int_slider 20 -50 150 40 \
         {Pupil Threshold} 1 255 [eyetracking::setPupilThreshold] eyetracking::setPupilThreshold
    add_int_slider 20 -90 150 40 \
         {P4 Threshold} 1 255 [eyetracking::setP4MinIntensity] eyetracking::setP4MinIntensity
}

load [file dir [info nameofexecutable]]/plugins/eyetracking[info sharedlibextension]

set files [list \
	       {/Users/sheinb/Desktop/trial-videos/OpenIris-2025Jun23-131340-Right.mp4} \
	       {/Users/sheinb/Desktop/trial-videos/OpenIris-2025Oct03-143614-Right.mkv} \
	       {/Users/sheinb/Desktop/trial-videos/glen_dual_purkinje.mp4} \	       
	      ]

eyetracking::setP1MaxJump 40
eyetracking::setP4MaxJump 40
eyetracking::setP4MinIntensity 42
eyetracking::setDetectionMode pupil_p1
eyetracking::resetP4Model

playback_mode [lindex $files 1]
