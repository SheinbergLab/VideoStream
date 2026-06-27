#
# dpi_reproject.tcl - reproject reprocessed P1/P4 tracking into gaze degrees and
# write an augmented trials.dgz with the best-possible eye position.
#
# Usage:
#   dlsh dpi_reproject.tcl <obs.dgz> <trials.dgz> <reproc.csv> <out_trials.dgz>
#
# reproc.csv columns: frame_number,p1_x,p1_y,p4_x,p4_y,pupil_radius,in_blink (-1 = not detected)
# raw signal convention (matches realtime eyetracking/raw): h = p4_x-p1_x, v = p1_y-p4_y
#
package require dlsh
package require em

lassign $argv obs_path trials_path csv_path out_path
set MAXDEG 30   ;# reject gaze beyond this many degrees as implausible/spurious

set og [dg_read $obs_path]
set tg [dg_read $trials_path]

# --- calibration coeffs (existing fit; reprocessed emcalib shown equivalent) ---
set calib  [em::extract_calibration_from_dg $og]
set coeffs [em::calibration_coeffs $calib]
if {$coeffs eq ""} { puts "NO CALIBRATION in $obs_path"; exit 1 }
puts "calibration source: [dict get $calib source]"

# --- load reprocessed per-frame tracking, keyed by frame_number ---
array set P1X {}; array set P1Y {}; array set P4X {}; array set P4Y {}; array set PR {}
set fh [open $csv_path]
while {[gets $fh line] >= 0} {
    lassign [split $line ,] fn p1x p1y p4x p4y r bl
    set P1X($fn) $p1x; set P1Y($fn) $p1y; set P4X($fn) $p4x; set P4Y($fn) $p4y; set PR($fn) $r
}
close $fh
puts "loaded [array size P1X] reprocessed frames"

set obsids [dl_tcllist $tg:obsid]

# nested per-trial result columns
dl_local h_all   [dl_llist]
dl_local v_all   [dl_llist]
dl_local valid_all [dl_llist]
dl_local pr_all  [dl_llist]
set comp_orig {}; set comp_rep {}

foreach k $obsids {
    dl_local fids [dl_choose $og:<ds>em/frame_id [dl_ilist $k]]
    set fidl [dl_tcllist $fids:0]

    # original per-obs P4 detected flag for completeness comparison
    set odet [dl_tcllist [dl_choose $og:<ds>em/p4_detected [dl_ilist $k]]:0]
    set norig [llength $odet]; set sorig 0
    foreach d $odet { if {$d > 0} { incr sorig } }
    lappend comp_orig [expr {$norig ? double($sorig)/$norig : 0}]

    # build reprocessed raw + validity from the db, in frame order
    set hs {}; set vs {}; set vm {}; set prs {}; set nrep 0; set srep 0
    foreach fid $fidl {
        incr nrep
        if {[info exists P4X($fid)] && $P4X($fid) >= 0 && $P1X($fid) >= 0} {
            lappend hs [expr {$P4X($fid) - $P1X($fid)}]
            lappend vs [expr {$P1Y($fid) - $P4Y($fid)}]
            lappend vm 1; lappend prs $PR($fid); incr srep
        } else {
            lappend hs 0.0; lappend vs 0.0; lappend vm 0; lappend prs -1
        }
    }
    lappend comp_rep [expr {$nrep ? double($srep)/$nrep : 0}]

    # raw -> degrees via the calibration
    dl_local rh [dl_flist {*}$hs]
    dl_local rv [dl_flist {*}$vs]
    dl_local cal [em::apply_calibration $coeffs $rh $rv]

    # plausibility filter: a "detected" P4 can still be spurious and the
    # biquadratic extrapolates wildly outside the calibration range. Mark gaze
    # invalid (and recount completeness) when it lands beyond plausible visual
    # angle (MAXDEG). 99.9th pct of real pursuit gaze here is ~12 deg.
    set hdeg [dl_tcllist $cal:0]; set vdeg [dl_tcllist $cal:1]
    set vm2 {}; set srep 0
    foreach m $vm hd $hdeg vd $vdeg {
        if {$m && abs($hd) <= $::MAXDEG && abs($vd) <= $::MAXDEG} {
            lappend vm2 1; incr srep
        } else {
            lappend vm2 0
        }
    }
    set comp_rep [lreplace $comp_rep end end [expr {$nrep ? double($srep)/$nrep : 0}]]

    dl_append $h_all $cal:0
    dl_append $v_all $cal:1
    dl_append $valid_all [dl_ilist {*}$vm2]
    dl_append $pr_all [dl_flist {*}$prs]
}

# attach improved columns to the trials dg and write
dl_set $tg:em_h_deg_reproc $h_all
dl_set $tg:em_v_deg_reproc $v_all
dl_set $tg:em_valid_reproc $valid_all
dl_set $tg:pupil_r_reproc  $pr_all
dg_write $tg $out_path
puts "wrote $out_path with em_h_deg_reproc/em_v_deg_reproc/em_valid_reproc/pupil_r_reproc"

# completeness summary (fraction of in-obs frames with valid gaze)
proc mean {l} { set s 0; foreach x $l { set s [expr {$s+$x}] }; return [expr {[llength $l]?$s/[llength $l]:0}] }
puts [format "GAZE COMPLETENESS (valid samples / in-obs frame): original=%.1f%%  reprocessed=%.1f%%" \
        [expr {100*[mean $comp_orig]}] [expr {100*[mean $comp_rep]}]]
