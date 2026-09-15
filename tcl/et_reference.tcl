#
# et_reference.tcl - work with a previously recorded metadata .db.
#
# A recorded run (.db) holds, per frame, the detections that run produced and
# the P4 model that was in force. Reprocessing the same video is much easier
# when you can see the old detections under the new ones ("ghosts") and start
# from the old run's P4 model instead of recalibrating by hand.
#
#   ::et::use_run <db>        settings + ghosts + P4 model: reproduce the run
#   ::et::load_reference <db> ghosts only
#   ::et::adopt_model <db>    P4 model only
#   ::et::apply_settings <db> detector parameters only
#   ::et::clear_reference     drop the ghosts
#   ::et::ref_stats           live-vs-reference agreement so far
#   ::et::db_source <db>      the mp4 the run was recorded from
#   ::et::db_settings <db>    the parameters in force, as a dict
#   ::et::db_info <db>        one-line summary of a .db
#
# Ghost drawing is toggled with "g" (see et_keys.tcl); loading turns it on.
#
# Parameters come from the eyetracking_settings table, which the plugin writes
# on the first recorded frame and again whenever a parameter changes mid-run.
# Runs recorded before that table existed have none - apply_settings says so
# and leaves the current parameters alone rather than guessing.
#
# p4_search_width/height are recorded but have no Tcl setter, so they are
# reported and not applied.
#
package require sqlite3

namespace eval ::et {
    variable ref_db {}
}

# Run one SQL statement against a .db read-only and return the rows as a list.
proc ::et::_query {db sql} {
    if { ![file exists $db] } {
        error "no such metadata db: $db"
    }
    set handle ::et::_dbh[incr ::et::_dbn]
    sqlite3 $handle $db -readonly true
    set rows {}
    try {
        $handle eval $sql values {
            set row {}
            foreach c $values(*) { lappend row $values($c) }
            lappend rows $row
        }
    } finally {
        $handle close
    }
    return $rows
}

# The mp4 this run was recorded from, as stored at record time.
proc ::et::db_source {db} {
    set rows [::et::_query $db {SELECT filename FROM recording_metadata LIMIT 1}]
    if { ![llength $rows] } { return {} }
    return [lindex $rows 0 0]
}

# Recover the P4 model from a run: the magnitude/angle pair the model held for
# the most frames. The model adapts online, so this is the longest-lived state
# rather than a single "the" value - the frame count says how settled it was.
# Returns {mag angle frames initialized_frames} or {} if the run never had one.
proc ::et::db_p4_model {db} {
    set rows [::et::_query $db {
        SELECT p4_magnitude_ratio, p4_angle_offset_deg, COUNT(*) AS n
          FROM eyetracking_frames
         WHERE p4_model_initialized = 1
           AND p4_magnitude_ratio IS NOT NULL
      GROUP BY p4_magnitude_ratio, p4_angle_offset_deg
      ORDER BY n DESC
         LIMIT 1
    }]
    if { ![llength $rows] } { return {} }
    lassign [lindex $rows 0] mag angle n
    set tot [lindex [::et::_query $db {
        SELECT COUNT(*) FROM eyetracking_frames WHERE p4_model_initialized = 1
    }] 0 0]
    return [list $mag $angle $n $tot]
}

# Does this db predate the settings table?
proc ::et::db_has_settings {db} {
    set rows [::et::_query $db {
        SELECT name FROM sqlite_master
         WHERE type='table' AND name='eyetracking_settings'
    }]
    return [llength $rows]
}

# The detector parameters in force at a given frame, as a dict. The plugin
# writes a row on the first recorded frame and again on every change, so the
# row that applies is the latest one at or before the frame asked for.
# Default frame 0 gives the parameters the run started with.
proc ::et::db_settings {db {frame 0}} {
    if { ![::et::db_has_settings $db] } { return {} }
    set rows [::et::_query $db "
        SELECT * FROM eyetracking_settings
         WHERE frame_number <= $frame
      ORDER BY frame_number DESC
         LIMIT 1
    "]
    if { ![llength $rows] } {
        # Asked before the first row: fall back to the earliest one.
        set rows [::et::_query $db {
            SELECT * FROM eyetracking_settings ORDER BY frame_number LIMIT 1
        }]
        if { ![llength $rows] } { return {} }
    }
    set cols [::et::_query $db {
        SELECT name FROM pragma_table_info('eyetracking_settings')
    }]
    set d [dict create]
    foreach c $cols v [lindex $rows 0] {
        dict set d [lindex $c 0] $v
    }
    return $d
}

# Every frame at which the parameters changed during a run.
proc ::et::db_setting_changes {db} {
    if { ![::et::db_has_settings $db] } { return {} }
    set out {}
    foreach r [::et::_query $db {
        SELECT frame_number FROM eyetracking_settings ORDER BY frame_number
    }] { lappend out [lindex $r 0] }
    return $out
}

# Put the plugin back into the state a recorded run was in. Pass a frame number
# to pick up the parameters in force at that point instead of the run's start.
proc ::et::apply_settings {db {frame 0}} {
    set s [::et::db_settings $db $frame]
    if { ![dict size $s] } {
        puts "no eyetracking_settings in [file tail $db] - parameters unchanged"
        puts "  (recorded before the settings table existed)"
        # The thresholds are gone, but the detections still say where the eye
        # was, and a stale ROI that clips the pupil breaks tracking outright.
        ::et::roi_from_run $db
        return 0
    }

    eyetracking::setPupilThreshold      [dict get $s pupil_threshold]
    eyetracking::setPupilGate           [dict get $s pupil_min_area] \
                                        [dict get $s pupil_max_area] \
                                        [dict get $s pupil_min_extent] \
                                        [dict get $s pupil_max_aspect]
    eyetracking::setP1MinIntensity      [dict get $s p1_min_intensity]
    eyetracking::setP1MaxJump           [dict get $s p1_max_jump]
    eyetracking::setP1MinArea           [dict get $s p1_min_area]
    eyetracking::setP1MaxArea           [dict get $s p1_max_area]
    eyetracking::setP1PupilRadiusMax    [dict get $s p1_pupil_radius_max]
    eyetracking::setP4MinIntensity      [dict get $s p4_min_intensity]
    eyetracking::setP4MaxJump           [dict get $s p4_max_jump]
    eyetracking::setP4MaxPredictionError [dict get $s p4_max_prediction_error]
    eyetracking::setP4PupilMargin       [dict get $s p4_pupil_margin]

    if { [dict get $s roi_enabled] } {
        eyetracking::setROI [dict get $s roi_x] [dict get $s roi_y] \
                            [dict get $s roi_width] [dict get $s roi_height]
    } else {
        eyetracking::disableROI
    }
    eyetracking::setDetectionMode [dict get $s detection_mode]

    puts "settings from [file tail $db] @ frame [dict get $s frame_number]:"
    puts [format "  pupil_threshold=%s  p1_min_intensity=%s  p4_min_intensity=%s" \
              [dict get $s pupil_threshold] [dict get $s p1_min_intensity] \
              [dict get $s p4_min_intensity]]
    puts [format "  p1_max_jump=%s  p4_max_jump=%s  p4_max_prediction_error=%s  margin=%s" \
              [dict get $s p1_max_jump] [dict get $s p4_max_jump] \
              [dict get $s p4_max_prediction_error] [dict get $s p4_pupil_margin]]
    if { [dict get $s roi_enabled] } {
        puts "  roi=[dict get $s roi_x] [dict get $s roi_y] [dict get $s roi_width] [dict get $s roi_height]  mode=[dict get $s detection_mode]"
    } else {
        puts "  roi=disabled  mode=[dict get $s detection_mode]"
    }
    puts "  (p4 search window [dict get $s p4_search_width]x[dict get $s p4_search_height], recorded at [dict get $s frame_rate] Hz - not settable from Tcl)"

    set changes [::et::db_setting_changes $db]
    if { [llength $changes] > 1 } {
        puts "  NOTE: parameters changed mid-run at frames [join [lrange $changes 1 end] {, }]"
        puts "        ::et::apply_settings [file tail $db] <frame> picks a later set"
    }
    return 1
}

# One-line summary: frames, detection yield, blinks, model.
proc ::et::db_info {db} {
    lassign [lindex [::et::_query $db {
        SELECT COUNT(*),
               SUM(pupil_x IS NOT NULL),
               SUM(p1_x IS NOT NULL),
               SUM(p4_x IS NOT NULL),
               SUM(in_blink)
          FROM eyetracking_frames
    }] 0] n pupil p1 p4 blink
    if { $n == 0 } { puts "[file tail $db]: empty"; return }

    puts "[file tail $db]"
    puts "  source : [::et::db_source $db]"
    puts "  frames : $n"
    puts [format "  pupil  : %d (%.1f%%)   P1: %d (%.1f%%)   P4: %d (%.1f%%)" \
              $pupil [expr {100.0*$pupil/$n}] \
              $p1    [expr {100.0*$p1/$n}] \
              $p4    [expr {100.0*$p4/$n}]]
    puts "  blink  : $blink frames"

    set model [::et::db_p4_model $db]
    if { [llength $model] } {
        lassign $model mag angle held tot
        puts [format "  P4 model: mag=%.3f angle=%.1f deg (held %d/%d model frames)" \
                  $mag $angle $held $tot]
    } else {
        puts "  P4 model: none (run had no calibrated model)"
    }

    set changes [::et::db_setting_changes $db]
    if { ![llength $changes] } {
        puts "  settings: not recorded (predates the eyetracking_settings table)"
    } elseif { [llength $changes] == 1 } {
        puts "  settings: recorded, unchanged through the run"
    } else {
        puts "  settings: recorded, [llength $changes] sets (changed at frames [join [lrange $changes 1 end] {, }])"
    }
}

# Set the ROI to cover everywhere the eye actually went in a recorded run:
# the bounding box of its pupil centres, grown by the largest pupil radius seen
# and a margin, clamped to the frame. A run whose parameters were never
# recorded still tells you this much, and a hardcoded ROI that clips the pupil
# is the usual reason a reprocess loses tracking that the original run held.
proc ::et::roi_from_run {db {margin 20}} {
    lassign [lindex [::et::_query $db {
        SELECT MIN(pupil_x), MAX(pupil_x), MIN(pupil_y), MAX(pupil_y),
               MAX(pupil_radius), COUNT(*)
          FROM eyetracking_frames WHERE pupil_x IS NOT NULL
    }] 0] minx maxx miny maxy maxr n

    if { $n == 0 } {
        puts "[file tail $db]: no pupil detections - cannot derive an ROI"
        return {}
    }

    lassign [lindex [::et::_query $db {
        SELECT width, height FROM recording_metadata LIMIT 1
    }] 0] fw fh
    if { $fw eq "" || $fw <= 0 } { set fw 720 }
    if { $fh eq "" || $fh <= 0 } { set fh 540 }

    set pad [expr {$maxr + $margin}]
    set x0 [expr {int(max(0, $minx - $pad))}]
    set y0 [expr {int(max(0, $miny - $pad))}]
    set x1 [expr {int(min($fw, $maxx + $pad))}]
    set y1 [expr {int(min($fh, $maxy + $pad))}]

    eyetracking::setROI $x0 $y0 [expr {$x1-$x0}] [expr {$y1-$y0}]
    puts [format "ROI from %s: %d %d %d %d" \
              [file tail $db] $x0 $y0 [expr {$x1-$x0}] [expr {$y1-$y0}]]
    puts [format "  (pupil centres spanned x %.0f-%.0f, y %.0f-%.0f; max radius %.0f, margin %d)" \
              $minx $maxx $miny $maxy $maxr $margin]
    return [list $x0 $y0 [expr {$x1-$x0}] [expr {$y1-$y0}]]
}

# How well the P4 model aimed: the offset between where P4 was found and where
# the model predicted it. The predictive search window is centred on the
# prediction, so a large mean offset means the true P4 sat off-centre in the
# box - and a large *bias* (mean dx/dy, rather than scatter) means the model
# itself is off, which recalibrating fixes. Offsets approaching the half-width
# of the search window mean P4 was near the edge and at risk of being missed.
proc ::et::db_p4_centering {db} {
    lassign [lindex [::et::_query $db {
        SELECT COUNT(*),
               AVG(SQRT((p4_x-p4_pred_x)*(p4_x-p4_pred_x) +
                        (p4_y-p4_pred_y)*(p4_y-p4_pred_y))),
               AVG(p4_x-p4_pred_x),
               AVG(p4_y-p4_pred_y),
               MAX(SQRT((p4_x-p4_pred_x)*(p4_x-p4_pred_x) +
                        (p4_y-p4_pred_y)*(p4_y-p4_pred_y)))
          FROM eyetracking_frames
         WHERE p4_x IS NOT NULL AND p4_pred_x IS NOT NULL
    }] 0] n mean dx dy max

    if { $n == 0 } {
        puts "[file tail $db]: no P4 predictions to compare"
        return {}
    }

    # The search window is recorded per run once the settings table exists.
    set half 25
    set s [::et::db_settings $db]
    if { [dict size $s] } {
        set half [expr {[dict get $s p4_search_width] / 2.0}]
    }

    puts "P4 centering in [file tail $db] ($n frames):"
    puts [format "  mean offset from prediction : %.2f px  (max %.2f)" $mean $max]
    puts [format "  bias                        : dx=%+.2f  dy=%+.2f px" $dx $dy]
    puts [format "  search half-width           : %.0f px" $half]
    if { $mean > $half * 0.25 } {
        puts "  -> P4 sits well off-centre; recalibrating the model would recentre it"
    }
    if { $max > $half } {
        puts "  -> some frames put P4 at or past the window edge"
    }
    return [dict create frames $n mean $mean dx $dx dy $dy max $max half $half]
}

# Load a run's detections as the reference overlay. Drawing is enabled by the
# load; "g" toggles it afterwards.
proc ::et::load_reference {db} {
    variable ref_db
    set n [eyetracking::loadReference [file normalize $db]]
    set ref_db [file normalize $db]
    puts "ghosts: loaded $n frames from [file tail $db] (g toggles)"
    return $n
}

proc ::et::clear_reference {args} {
    variable ref_db
    eyetracking::clearReference
    set ref_db {}
    puts "ghosts: cleared"
}

proc ::et::ref_stats {args} {
    if { [catch {eyetracking::referenceStats} s] } { puts "ghosts: $s"; return }
    puts "ghosts: $s"
    return $s
}

# Start from the P4 model a previous run converged on, instead of recalibrating.
# The model is left adaptive; pass -freeze to pin it for an A/B comparison.
proc ::et::adopt_model {db args} {
    set model [::et::db_p4_model $db]
    if { ![llength $model] } {
        puts "no P4 model stored in [file tail $db] - nothing to adopt"
        return 0
    }
    lassign $model mag angle held tot
    eyetracking::setP4Model $mag $angle
    if { [lsearch -exact $args -freeze] >= 0 } {
        eyetracking::freezeP4Model 1
        set how "frozen"
    } else {
        set how "adaptive"
    }
    eyetracking::setDetectionMode full
    puts [format "P4 model from [file tail $db]: mag=%.3f angle=%.1f deg (%s, held %d/%d)" \
              $mag $angle $how $held $tot]
    return 1
}

# The usual thing: show me what the last run got, and start where it left off.
# Settings first, then ghosts, then the model - adopt_model switches to full
# mode, so it has to come after apply_settings restores the recorded mode.
#
# -nomodel keeps the ghosts and the parameters but leaves the P4 model cleared,
# for when the run's model is the thing you want to redo.
proc ::et::use_run {db args} {
    set db [file normalize $db]
    ::et::db_info $db
    ::et::apply_settings $db
    ::et::load_reference $db

    set i [lsearch -exact $args -nomodel]
    if { $i < 0 } {
        ::et::adopt_model $db {*}$args
        return
    }

    eyetracking::resetP4Model
    eyetracking::setDetectionMode pupil_p1
    puts "P4 model: cleared (-nomodel), mode pupil_p1"
    ::et::db_p4_centering $db
    puts "  to recalibrate: shift-click the real P4, Enter to accept"
    puts "  (repeat on a few frames), then m"
}
