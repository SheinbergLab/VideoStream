proc onMouseClick {x y modifier} {
    puts "Mouse clicked at ($x, $y) with modifier: $modifier"
    
    switch $modifier {
        "shift" {
            # Capture P4 template
            if {[info commands ::eyetracking::setP4Template] ne ""} {
		::eyetracking::resetValidator
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
            # Set main ROI (around click)
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
