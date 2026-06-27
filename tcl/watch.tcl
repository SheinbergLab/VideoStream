#
# watch.tcl - interactive playback of a recorded mp4 through the eye-tracking
# plugin, with the live overlay, so you can SEE P1/P4 detection (and spurious
# /missed detections) frame-by-frame and tune settings. Plays "as if live".
#
#   ./build/VideoStream -d -f tcl/watch.tcl -- <mp4_path> [mag angle] [speed]
#
# Defaults to the coherence-sweep pilot file + its P4 model. Controls:
#   SPACE pause/resume   <-/-> step (paused)   i insets   I frame info
#   shift-click = mark P4 sample   m calibrate model   r reset tracking
#
namespace eval ::R { variable w [dict create]; variable paused 0; variable insets 0 }

set DEF_MP4   /Users/sheinb/src/VideoStream/data/pilot_2026-06-23/human_pursuit_coherence-sweep_260626135852.mp4
set DEF_MAG 0.5023
set DEF_ANG -179.32

set mp4   [expr {[llength $argv] >= 1 ? [lindex $argv 0] : $DEF_MP4}]
set ::mag [expr {[llength $argv] >= 3 ? [lindex $argv 1] : $DEF_MAG}]
set ::ang [expr {[llength $argv] >= 3 ? [lindex $argv 2] : $DEF_ANG}]
set ::spd [expr {[llength $argv] >= 4 ? [lindex $argv 3] : 0.25}]
set ::source_file $mp4

load [file dir [info nameofexecutable]]/plugins/eyetracking[info sharedlibextension]

# defaults (same as the headless reprocess path)
eyetracking::setROI 160 80 430 365
eyetracking::setP1MaxJump 24
eyetracking::setP1MinIntensity 130
eyetracking::setP4MaxJump 100
eyetracking::setP4MinIntensity 22
eyetracking::setPupilThreshold 45
eyetracking::setP4MaxPredictionError 40
eyetracking::resetP4Model
eyetracking::setP4Model $::mag $::ang
eyetracking::setDetectionMode full
eyetracking::setDebugLevel 1

proc onMouseClick {x y mod} {
    if {$mod eq "shift"} { eyetracking::markP4Sample $x $y; puts "P4 sample @ $x,$y (Enter to add)" }
}
proc onEvent {type data} {
    switch -glob $type {
        "vstream/video_source_eof"    { puts "EOF (looping)" }
        "vstream/video_source_rewind" { eyetracking::resetTrackingState }
        "eyetracking/p4_lost"     { puts "P4 lost  @frame $data" }
        "eyetracking/p4_recovered"{ puts "P4 recovered @frame $data" }
        "eyetracking/p1_lost"     { puts "P1 lost  @frame $data" }
    }
}
proc toggle_pause {args} {
    set ::R::paused [expr {!$::R::paused}]; vstream::pause $::R::paused
    puts [expr {$::R::paused ? "PAUSED (<-/-> to step)" : "PLAYING"}]
}
proc step_fwd {args} { if {$::R::paused} { vstream::step 1;  puts "frame [vstream::getCurrentFrame]" } }
proc step_bwd {args} { if {$::R::paused} { vstream::step -1; puts "frame [vstream::getCurrentFrame]" } }
proc toggle_insets {args} { set ::R::insets [expr {!$::R::insets}]; eyetracking::toggleInsets $::R::insets }
proc accept_p4 {args}  { catch {eyetracking::acceptP4Sample} r; puts $r }
proc reset_trk {args}  { eyetracking::resetTrackingState; puts "tracking reset" }
proc calib_p4 {args}   { catch {eyetracking::calibrateP4Model} r; puts $r; eyetracking::setDetectionMode full }
proc frame_info {args} { puts "frame [vstream::getCurrentFrame]/[vstream::getTotalFrames]: [eyetracking::getResults]" }

clear_widgets; clear_key_bindings
vstream::startSource playback file $mp4 speed $::spd loop 1
eyetracking::toggleInsets 1; set ::R::insets 1

# parameter sliders
dict set ::R::w pup [add_int_slider 20 -50 150 40 {Pupil Threshold} 1 255 [eyetracking::setPupilThreshold] eyetracking::setPupilThreshold]
dict set ::R::w p4t [add_int_slider 20 -95 150 40 {P4 Threshold} 1 255 [eyetracking::setP4MinIntensity] eyetracking::setP4MinIntensity]
dict set ::R::w p4j [add_float_slider 20 -140 150 40 {P4 Max Jump} 5 200 [eyetracking::setP4MaxJump] eyetracking::setP4MaxJump]
dict set ::R::w p4e [add_float_slider 20 -185 150 40 {P4 Max PredErr} 5 100 [eyetracking::setP4MaxPredictionError] eyetracking::setP4MaxPredictionError]

bind_key " " toggle_pause
bind_key $::keys::RIGHT step_fwd
bind_key $::keys::LEFT  step_bwd
bind_key "i" toggle_insets
bind_key "I" frame_info
bind_key "r" reset_trk
bind_key "m" calib_p4
bind_key $::keys::ENTER accept_p4

puts "=============================================="
puts " watch.tcl: $mp4"
puts "   model mag=$::mag angle=$::ang  speed=$::spd  full mode"
puts "   SPACE pause | <-/-> step | i insets | I info | shift-click mark P4 | m calibrate | r reset"
puts "=============================================="
