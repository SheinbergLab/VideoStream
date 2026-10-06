#
# serve.tcl - headless playback of a recorded mp4 through the eye-tracking
# plugin, for the browser viewer. No display window or widgets: open
# http://<host>:8080/app/ (or your --ws-port) to watch the video and overlay.
#
#   ./build/VideoStream -f tcl/serve.tcl -- [fallback_mp4] [mag angle] [speed]
#
# On startup, the first detected camera is used. If none is found, an optional
# fallback_mp4 is played; otherwise no source is started until you pick one in
# the viewer.
#
# With a P4 model (mag angle) the plugin runs in full mode; without one it
# runs pupil + P1 only. Playback loops.
#
# Speed defaults to 1.0 (real time). Pass a lower value (e.g. 0.25) if analysis
# cannot keep up and the frame queue grows.
#
namespace eval ::S {}

set ::S::arg_idx 0
set ::S::fallback_mp4 ""
if {[llength $argv] >= 1} {
    set maybe [lindex $argv 0]
    if {[file exists $maybe] && [file isfile $maybe]} {
        set ::S::fallback_mp4 $maybe
        set ::S::arg_idx 1
    }
}

set ::S::have_model [expr {[llength $argv] - $::S::arg_idx >= 2}]
set ::S::mag [expr {$::S::have_model ? [lindex $argv $::S::arg_idx] : ""}]
set ::S::ang [expr {$::S::have_model ? [lindex $argv [expr {$::S::arg_idx + 1}]] : ""}]
set ::S::spd [expr {[llength $argv] - $::S::arg_idx >= 3 ? [lindex $argv end] : 1.0}]
set ::source_file $::S::fallback_mp4
set ::S::last_kind ""
set ::S::live_vendor ""
set ::S::live_id ""
set ::S::live_serial ""

load [file dir [info nameofexecutable]]/plugins/eyetracking[info sharedlibextension]

# Where the viewer's file picker looks for eye-tracking recordings: the lab
# data share, and under WSL each Windows user's Videos/eye_tracking.
set ::S::places [list "Eye tracking videos" /mnt/analysis/data/eye_tracking]
foreach d [glob -nocomplain -types d /mnt/c/Users/*/Videos/eye_tracking] {
    lappend ::S::places "Eye tracking videos" $d
}
vstream::mediaPlaces $::S::places

# Settings changed in the browser viewer are kept by the server, here.
source [file join [file dirname [info script]] viewer_settings.tcl]

# Detector tuning defaults (same as watch.tcl and the headless reprocess
# path). The viewer's tuning panel calls this for "Reset to defaults".
proc apply_default_tuning {} {
    eyetracking::setP1MaxJump 24
    eyetracking::setP1MinIntensity 130
    eyetracking::setP1MinArea 40
    eyetracking::setP1MaxArea 600
    eyetracking::setP1PupilRadiusMax 1.5
    eyetracking::setP4MaxJump 100
    eyetracking::setP4MinIntensity 22
    eyetracking::setPupilThreshold 45
    eyetracking::setP4MaxPredictionError 40
    if {$::S::have_model} {
        eyetracking::setDetectionMode full
    } else {
        eyetracking::setDetectionMode pupil_p1
    }
}

eyetracking::setROI 160 80 430 365
set ::vs::default_roi {160 80 430 365}
eyetracking::resetP4Model
if {$::S::have_model} {
    eyetracking::setP4Model $::S::mag $::S::ang
}
apply_default_tuning

proc onMouseClick {x y mod} {}
proc onEvent {type data} {
    switch -glob $type {
        "vstream/video_source_rewind" { eyetracking::resetTrackingState }
        "vstream/video_source_eof" { run_on_eof }
        "vstream/metadata_recording_started" { run_on_opened }
        "vstream/metadata_recording_closed" { run_on_closed }
    }
}

# ---------------------------------------------------------------------------
# Reference .db files for the current video.
#
# The rig writes <base>.db next to <base>.mp4. Runs created from the viewer
# are <base>_1.db, <base>_2.db, ... in the same directory. A run in progress
# writes <tail>_pending.db in a local scratch dir and is copied there on
# save: SQLite cannot lock files on the CIFS data share (no nobrl), so it
# only ever sees local files while writing.
# ---------------------------------------------------------------------------
set ::S::scratch_dir [expr {[info exists ::env(VS_REPROCESS_TMP)] ?
                            $::env(VS_REPROCESS_TMP) : "/tmp/videostream-reprocess"}]
set ::S::ref_db ""
set ::S::run_state idle         ;# idle | running | closing | done
set ::S::run_after_close ""     ;# done | restart | discard
set ::S::pending_db ""
set ::S::run_total 0
set ::S::run_prev_ref ""
set ::S::run_saved_obs 1
set ::S::run_waiting_open 0
set ::S::run_timer ""
set ::S::run_roi {}
set ::S::run_keep_roi 0

proc ref_video_base {} {
    return [file rootname $::source_file]
}

# Numbered run index of a .db for this video, or -1 if it isn't one.
proc ref_run_index {base path} {
    set prefix "[file tail $base]_"
    set tail [file rootname [file tail $path]]
    if {[string first $prefix $tail] != 0} { return -1 }
    set n [string range $tail [string length $prefix] end]
    if {$n eq "" || ![string is digit -strict $n] || [string length $n] > 4} {
        return -1
    }
    return [scan $n %d]
}

proc ref_files {} {
    set base [ref_video_base]
    set out {}
    if {[file exists $base.db]} { lappend out [file normalize $base.db] }
    set runs {}
    foreach f [glob -nocomplain -types f -- "${base}_*.db"] {
        set n [ref_run_index $base $f]
        if {$n >= 0} { lappend runs [list $n [file normalize $f]] }
    }
    foreach r [lsort -integer -index 0 $runs] { lappend out [lindex $r 1] }
    return $out
}

proc ref_next_index {} {
    set base [ref_video_base]
    set max 0
    foreach f [glob -nocomplain -types f -- "${base}_*.db"] {
        set n [ref_run_index $base $f]
        if {$n > $max} { set max $n }
    }
    return [expr {$max + 1}]
}

proc ref_list {} {
    if {$::source_file eq "" ||
        ([vstream::getSourceType] ne "playback" && $::S::run_state eq "idle")} {
        return {}
    }
    return [dict create \
        video [file normalize $::source_file] \
        current $::S::ref_db \
        files [ref_files] \
        state $::S::run_state \
        pending $::S::pending_db \
        next [file tail "[ref_video_base]_[ref_next_index].db"]]
}

proc ref_select {path} {
    if {$path eq "none" || $path eq ""} {
        eyetracking::clearReference
        set ::S::ref_db ""
        return ""
    }
    set n [eyetracking::loadReference $path]
    set ::S::ref_db [file normalize $path]
    puts "reference overlay: [file tail $path] ($n frames)"
    return $n
}

proc load_reference_for {path} {
    set refdb [file rootname $path].db
    set ::S::ref_db ""
    eyetracking::clearReference
    if {[file exists $refdb]} {
        catch {ref_select $refdb}
    }
}

proc run_delete_db {path} {
    foreach f [list $path $path-wal $path-shm $path-journal] {
        if {[file exists $f]} { file delete -- $f }
    }
}

proc run_busy_error {} {
    switch -- $::S::run_state {
        running - closing {
            return -code error "a re-process run is in progress; discard it first"
        }
        done {
            return -code error "an unsaved re-process run exists; save or discard it first"
        }
    }
}

proc run_play_looping {} {
    vstream::startSource playback file $::source_file speed $::S::spd loop 1 rate_limited 1
    eyetracking::resetTrackingState
    vstream::pause 0
}

# Re-process: run the whole video from the start through the current
# settings, writing <base>_pending.db. Deterministic (synchronous analysis,
# serial storage) and fast; pause still works and paused frames are not stored.
proc run_start {} {
    if {[vstream::getSourceType] ne "playback" && $::S::run_state ne "done"} {
        return -code error "re-process needs a pre-recorded video"
    }
    switch -- $::S::run_state {
        running {
            set ::S::run_after_close restart
            run_close
            return closing
        }
        closing {
            set ::S::run_after_close restart
            return closing
        }
    }
    if {$::S::run_state eq "idle"} {
        set ::S::run_prev_ref $::S::ref_db
    }
    # ROI auto-follow keeps moving the ROI during looping playback; pin the
    # run's starting ROI at the press and reuse it when the run is restarted.
    if {!$::S::run_keep_roi} {
        set ::S::run_roi [eyetracking::setROI]
    }
    set ::S::run_keep_roi 0
    file mkdir $::S::scratch_dir
    set ::S::pending_db [file normalize [file join $::S::scratch_dir \
        "[file tail [ref_video_base]]_pending.db"]]
    run_delete_db $::S::pending_db

    catch {vstream::stopSource}
    set ::S::run_saved_obs [vstream::onlySaveInObs 0]
    vstream::fileUseSQLite 1
    eyetracking::setSynchronous 1
    vstream::setReprocessMode 1
    set ::S::run_total 0
    set ::S::run_state running
    set ::S::run_after_close done
    # The process thread opens the file asynchronously and drops frames that
    # arrive before it is open, so playback starts from run_on_opened.
    set ::S::run_waiting_open 1
    vstream::fileOpenMetadata [file rootname $::S::pending_db] $::source_file
    set ::S::run_timer [after 10000 run_open_timeout]
    return running
}

proc run_on_opened {} {
    if {!$::S::run_waiting_open} return
    set ::S::run_waiting_open 0
    after cancel $::S::run_timer
    vstream::fileStartRecording
    eyetracking::setROI {*}$::S::run_roi
    vstream::startSource playback file $::source_file speed 8.0 loop 0 rate_limited 1
    eyetracking::resetTrackingState
    vstream::pause 0
    set ::S::run_total [vstream::getTotalFrames]
    puts "re-process: writing [file tail $::S::pending_db] ($::S::run_total frames)"
}

proc run_open_timeout {} {
    if {!$::S::run_waiting_open || $::S::run_state ne "running"} return
    set ::S::run_waiting_open 0
    puts "re-process: could not open [file tail $::S::pending_db]"
    set ::S::run_after_close discard
    set ::S::run_state closing
    run_on_closed
}

proc run_close {} {
    set ::S::run_waiting_open 0
    set ::S::run_state closing
    catch {vstream::stopSource}
    after cancel $::S::run_timer
    vstream::fileClose
    # If the file never finished opening there is no close event.
    set ::S::run_timer [after 10000 {if {$::S::run_state eq "closing"} run_on_closed}]
}

proc run_on_eof {} {
    if {$::S::run_state ne "running"} return
    run_close
}

proc run_on_closed {} {
    if {$::S::run_state ne "closing"} return
    after cancel $::S::run_timer
    eyetracking::setSynchronous 0
    vstream::setReprocessMode 0
    vstream::onlySaveInObs $::S::run_saved_obs
    switch -- $::S::run_after_close {
        restart {
            # The close event fires just before the host marks the file
            # closed; give it a moment before reopening.
            set ::S::run_state done
            set ::S::run_keep_roi 1
            after 200 {
                if {[catch {run_start} err]} {
                    puts "re-process restart failed: $err"
                    set ::S::run_state done
                }
            }
            return
        }
        discard {
            run_delete_db $::S::pending_db
            set ::S::pending_db ""
            set ::S::run_state idle
            run_play_looping
            catch {ref_select $::S::run_prev_ref}
            return
        }
    }
    set ::S::run_state done
    run_play_looping
    if {[catch {ref_select $::S::pending_db} err]} {
        puts "re-process: could not load result: $err"
    }
    puts "re-process: finished [file tail $::S::pending_db]"
}

proc run_status {} {
    set frame 0
    catch {set frame [vstream::getCurrentFrame]}
    return [dict create state $::S::run_state frame $frame total $::S::run_total \
                pending $::S::pending_db]
}

proc run_save {} {
    if {$::S::run_state ne "done"} {
        return -code error "no finished re-process run to save"
    }
    set base [ref_video_base]
    set dest [file normalize "${base}_[ref_next_index].db"]
    if {[file exists $::S::pending_db-wal] && [file size $::S::pending_db-wal] > 0} {
        return -code error "re-process file was not closed cleanly (WAL present)"
    }
    file copy -- $::S::pending_db $dest.part
    file rename -- $dest.part $dest
    run_delete_db $::S::pending_db
    catch {ref_select $dest}
    set ::S::ref_db $dest
    set ::S::pending_db ""
    set ::S::run_state idle
    puts "re-process: saved [file tail $dest]"
    return $dest
}

# next_ref: reference to load once the run is gone ("keep" = what was loaded
# before the run started, "none" = no reference).
proc run_discard {{next_ref keep}} {
    if {$next_ref ne "keep"} {
        set ::S::run_prev_ref [expr {$next_ref eq "none" ? "" : $next_ref}]
    }
    switch -- $::S::run_state {
        running {
            set ::S::run_after_close discard
            run_close
        }
        closing {
            set ::S::run_after_close discard
        }
        done {
            run_delete_db $::S::pending_db
            set ::S::pending_db ""
            set ::S::run_state idle
            catch {ref_select $::S::run_prev_ref}
        }
    }
    return $::S::run_state
}

# Browser viewer: switch sources without restarting VideoStream.
proc switch_to_playback {path} {
    run_busy_error
    set ::source_file $path
    set ::S::last_kind playback
    vstream::startSource playback file $path speed $::S::spd loop 1 rate_limited 1
    eyetracking::resetTrackingState
    load_reference_for $path
    vstream::pause 0
}

proc set_playback_speed {spd} {
    set ::S::spd $spd
    if {[vstream::getSourceType] eq "playback"} {
        vstream::setPlaybackSpeed $spd
    }
}

proc switch_to_camera {vendor id {serial ""}} {
    run_busy_error
    set ::S::last_kind camera
    set ::S::live_vendor $vendor
    set ::S::live_id $id
    set ::S::live_serial $serial
    switch -exact $vendor {
        webcam {
            vstream::startSource webcam id $id
        }
        flir {
            vstream::startSource flir id $id width 1440 height 1080
            camera::startAcquisition
        }
        lucid {
            if {$serial ne ""} {
                vstream::startSource lucid id $id serial $serial
            } else {
                vstream::startSource lucid id $id
            }
            camera::startAcquisition
        }
        default {
            return -code error "unknown camera vendor: $vendor"
        }
    }
    # exposure / gain / frame rate / binning saved from the viewer
    ::vs::apply_camera
    eyetracking::resetTrackingState
    vstream::pause 0
    puts "serve.tcl: switched to live $vendor (headless - no et_camera tuning)"
}

# Close the camera or the video file. The last source is remembered so
# resume_stopped_source can open it again. Pause is unchanged: it keeps
# holding one frame.
proc stop_active_source {} {
    run_busy_error
    if {[vstream::getSourceType] eq ""} {
        return stopped
    }
    vstream::stopSource
    return stopped
}

proc resume_stopped_source {} {
    if {[vstream::getSourceType] ne ""} {
        vstream::pause 0
        return running
    }
    switch -- $::S::last_kind {
        playback {
            if {$::source_file eq ""} {
                return -code error "no video to open"
            }
            switch_to_playback $::source_file
        }
        camera {
            if {$::S::live_vendor eq ""} {
                return -code error "no camera to open"
            }
            switch_to_camera $::S::live_vendor $::S::live_id $::S::live_serial
        }
        default {
            return -code error "nothing to open"
        }
    }
    return running
}

proc start_default_source {} {
    set cam [vstream::probeFirstCamera]
    if {[llength $cam] >= 2} {
        set vendor [lindex $cam 0]
        set id [lindex $cam 1]
        if {[llength $cam] >= 3} {
            switch_to_camera $vendor $id [lindex $cam 2]
        } else {
            switch_to_camera $vendor $id
        }
        return camera
    }
    if {$::S::fallback_mp4 ne "" && [file exists $::S::fallback_mp4]} {
        switch_to_playback $::S::fallback_mp4
        return playback
    }
    puts "serve.tcl: no camera detected; no fallback video - pick a source in the viewer"
    return none
}

# Saved browser-viewer settings go on top of the defaults above. Loaded here,
# after the procs they call (set_playback_speed) exist and before the first
# source opens (so the saved camera values apply to it).
::vs::load
::vs::apply_saved
vstream::addShutdownCmd ::vs::flush

set ::S::started_as [start_default_source]

puts "=============================================="
if {$::S::started_as eq "camera"} {
    puts " serve.tcl: live camera (first detected)"
} elseif {$::S::started_as eq "playback"} {
    puts " serve.tcl: $::S::fallback_mp4"
} else {
    puts " serve.tcl: idle (no camera, no fallback file)"
}
if {$::S::have_model} {
    puts "   model mag=$::S::mag angle=$::S::ang  speed=$::S::spd  full mode"
} else {
    puts "   no P4 model  speed=$::S::spd  pupil_p1 mode"
}
puts "   viewer: http://localhost:[expr {[info exists ::vstream::wsPort] ? $::vstream::wsPort : 8080}]/app/"
puts "=============================================="
