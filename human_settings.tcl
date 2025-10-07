load build/plugins/eyetracking.dylib
::eyetracking::resetValidator
::eyetracking::setValidatorThresholds 40 50
::eyetracking::setPupilThreshold 45
::eyetracking::setP4DetectionParams 0.9 0.25 140 1.5
::eyetracking::setP4AreaConstraints 1 150

# Example Tcl mouse handlers
proc onMouseClick {x y modifier} {
    puts "Mouse clicked at ($x, $y) with modifier: $modifier"
    
    switch $modifier {
        "shift" {
            # Capture P4 template
            if {[info commands ::eyetracking::setP4Template] ne ""} {
                ::eyetracking::setP4Template $x $y
                puts "P4 template captured at $x,$y"
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
            # Set main ROI (500x500 around click)
            if {[info commands ::eyetracking::setROI] ne ""} {
                set roi_x [expr {$x - 250}]
                set roi_y [expr {$y - 250}]
                ::eyetracking::setROI $roi_x $roi_y 500 500
                puts "ROI set around $x,$y"
            }
        }
        default {
            # Regular click - could show pixel value or coordinates
            puts "Click at pixel ($x, $y)"
        }
    }
}

proc onMouseRightClick {x y} {
    puts "Right click at ($x, $y)"
    # Clear templates/ROIs
    if {[info commands ::eyetracking::clearP4Template] ne ""} {
        ::eyetracking::clearP4Template
    }
    if {[info commands ::eyetracking::disableROI] ne ""} {
        ::eyetracking::disableROI
    }
    puts "Cleared templates and ROIs"
}

proc onMouseMiddleClick {x y} {
    # Could toggle something
    puts "Middle click at ($x, $y)"
}
