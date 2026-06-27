# Headless batch reprocess: play an mp4 through the eyetracking plugin and
# write a fresh metadata .db, then exit. No GUI, no display.
#
#   VideoStream -f tcl/reprocess_headless.tcl <mp4_path> <out_db_base> [speed] [mag angle]
#
# If mag+angle are given, loads that P4 model and runs in 'full' mode.
# Otherwise stays in pupil_p1 (no P4) — useful for a pure determinism check.

load [file dir [info nameofexecutable]]/plugins/eyetracking[info sharedlibextension]

if {[llength $argv] < 2} {
    puts "usage: reprocess_headless.tcl <mp4_path> <out_db_base> \[speed\] \[mag angle\]"
    catch {vstream::exit}; return
}
set mp4   [lindex $argv 0]
set outbase [lindex $argv 1]
# Default to a high speed multiplier: reprocess is offline, run as fast as the
# serial pipeline allows (the per-frame barrier paces it, not wall-clock).
set speed [expr {[llength $argv] >= 3 ? [lindex $argv 2] : 8.0}]

if {![file exists $mp4]} { puts "NOT_FOUND: $mp4"; catch {vstream::exit}; return }

# Deterministic, frame-complete reprocess: inline analysis + one-frame-in-flight.
eyetracking::setSynchronous 1
vstream::setReprocessMode 1
eyetracking::setDebugLevel 0   ;# quiet: per-frame reasons go to the db, not stdout

# Fixed, explicit parameters (so reprocess is reproducible)
eyetracking::setROI 160 80 430 365
eyetracking::setP1MaxJump 24
eyetracking::setP1MinIntensity 130
eyetracking::setP4MaxJump 100
eyetracking::setP4MinIntensity 22
eyetracking::setPupilThreshold 45
eyetracking::setP4MaxPredictionError 40
eyetracking::resetP4Model
eyetracking::setDetectionMode pupil_p1

if {[llength $argv] >= 5} {
    set mag   [lindex $argv 3]
    set angle [lindex $argv 4]
    eyetracking::setP4Model $mag $angle
    # "online" anywhere in argv => leave the model adaptive (tests the live-safe
    # conservative-learning path). Otherwise freeze for a fixed-model baseline.
    set online [expr {[lsearch $argv "online"] >= 0}]
    if { !$online } { eyetracking::freezeP4Model 1 }
    eyetracking::setDetectionMode full
    puts "P4 model set: mag=$mag angle=$angle -> full mode ([expr {$online ? {online/adaptive} : {frozen}}])"
}

vstream::onlySaveInObs 0

proc onEvent {type data} {
    switch -glob $type {
        "vstream/video_source_eof" -
        "vstream/source_eof" {
            puts "EOF reached -> closing"
            catch {vstream::fileClose}
            puts "REPROCESS_DONE"
            catch {vstream::exit}
        }
    }
}
# stubs so plugin event callbacks don't error
proc onMouseClick {x y modifier} {}

vstream::fileUseSQLite 1
vstream::fileOpenMetadata $outbase $mp4
vstream::fileStartRecording
puts "REPROCESS_START mp4=$mp4 out=$outbase speed=$speed"
vstream::startSource playback file $mp4 speed $speed loop 0
