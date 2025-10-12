namespace eval ::Registry {
    variable widgets
    set widgets [dict create]
}

# proc to clear all
proc clearRegistry {} {
    set ::Registry::widgets [dict create]
}

proc onMouseClick {x y modifier} {
    puts "Mouse clicked at ($x, $y) with modifier: $modifier"
    
    switch $modifier {
        "shift" {
            # Add P4 calibration sample
            if {[info commands ::eyetracking::addP4CalibrationSample] ne ""} {
                ::eyetracking::addP4CalibrationSample $x $y
                puts "P4 calibration sample added at $x,$y"
            }
        }	
        "ctrl" {
            # Set P1 seed ROI (30x30 around click)
            if {[info commands ::eyetracking::setP1SeedROI] ne ""} {
                ::eyetracking::setP1SeedROI $x $y 30 30
                puts "P1 seed ROI set at $x,$y"
            }
        }
        "alt" {
            # Set main ROI (around click)
            if {[info commands ::eyetracking::setROI] ne ""} {
                set roi_x [expr {$x - 200}]
                set roi_y [expr {$y - 200}]
                ::eyetracking::setROI $roi_x $roi_y 400 400
                puts "ROI set around $x,$y"
            }
        }
        default {
            # Regular click - could show pixel value or coordinates
            puts "Click at pixel ($x, $y)"
        }
    }
}

# not currently used...
proc review_update {} {
    set_variable total_frames [::vstream::reviewCount]
    set_variable current_frame [expr {[::vstream::reviewIndex]+1}]
}

proc slider_update {} {
    set s [dict get $::Registry::widgets frame_slider]
    update_slider_vals $s 1 [vstream::reviewCount] [expr {[vstream::reviewIndex]+1}]
}

proc review_next { code } {
    ::vstream::reviewNext
    slider_update
}

proc review_previous { code } {
    ::vstream::reviewPrevious
    slider_update
}

proc review_goto_frame { frame } {
    ::vstream::reviewJumpTo [expr {$frame-1}]
    review_update
    slider_update
}

proc review_gui {} {
    clear_widgets
    clearRegistry
    
    clear_key_bindings
    review_update

    # Widgets
    add_text 20 40 "Review" {240 140 180} 1.0 2

    add_button -100 -50 80 40 Play playback_mode

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
}

proc review_mode {} {
    vstream::startSource review
    review_gui
}

proc collect_samples { { n 6 } { interval_ms 500 } } {
    set random 1
    playback_mode
    vstream::reviewClear
    ::vstream::reviewSampleCallback review_mode
    ::vstream::reviewSampleMultiple $n $interval_ms $random
}

set files [list \
	       {/Users/sheinb/Desktop/trial-videos/OpenIris-2025Oct03-143614-Right.mkv} \
	      ]
	   

proc playback_mode { { index 0 } } {
    clear_widgets
    clearRegistry
    clear_key_bindings

    vstream::startSource playback file [lindex $::files $index] speed 0.5
    add_button -100 -50 80 40 Review collect_samples
    add_int_slider 20 -50 150 40 \
	     {Pupil Threshold} 1 255 [eyetracking::setPupilThreshold] eyetracking::setPupilThreshold
}    

load [file dir [info nameofexecutable]]/plugins/eyetracking[info sharedlibextension]
eyetracking::setP4MinIntensity 100
playback_mode
