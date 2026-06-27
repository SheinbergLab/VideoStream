#
# dpi_replace.tcl - replace the canonical eye columns of a trials.dgz with the
# reprocessed (best-possible) tracking, then re-derive movement events so the
# file is drop-in for existing visualization tools.
#
#   dlsh dpi_replace.tcl <obs.dgz> <trials.dgz> <reproc.csv> <out_trials.dgz>
#
# Replaces per-sample: pupil_x/y p1_x/y p4_x/y eye_raw_h/v em_h_deg em_v_deg
#   pupil_r in_blink eye_raw  (+ adds em_valid). Keeps timing (em_time/seconds/
#   frame_id) and all stim/metadata. Re-runs em::extract_events to refresh
#   sac_*/pur_*/fix_*/em_state. Invalid samples (no P4 or |deg|>MAXDEG) are
#   marked in_blink=1 and the gaze is held at the last valid value.
#
package require dlsh
package require em

lassign $argv obs_path trials_path csv_path out_path
set MAXDEG 30

set og [dg_read $obs_path]
set tg [dg_read $trials_path]
set coeffs [em::calibration_coeffs [em::extract_calibration_from_dg $og]]
if {$coeffs eq ""} { puts "NO CALIBRATION"; exit 1 }

# reprocessed per-frame tracking
array set P1X {}; array set P1Y {}; array set P4X {}; array set P4Y {}; array set PR {}
set fh [open $csv_path]
while {[gets $fh line] >= 0} {
    lassign [split $line ,] fn a b c d r bl
    set P1X($fn) $a; set P1Y($fn) $b; set P4X($fn) $c; set P4Y($fn) $d; set PR($fn) $r
}
close $fh

set ntr [dl_length $tg:obsid]
foreach c {PUX PUY A1X A1Y A4X A4Y RAWH RAWV HDEG VDEG PRR BLK RAW VALID} { dl_local $c [dl_llist] }
set tot 0; set good 0

for {set t 0} {$t < $ntr} {incr t} {
    set fids [dl_tcllist $tg:frame_id:$t]
    set obl  [dl_tcllist $tg:in_blink:$t]
    set pux {}; set puy {}; set a1x {}; set a1y {}; set a4x {}; set a4y {}
    set rh {}; set rv {}; set prr {}; set vmask {}
    foreach fid $fids ob $obl {
        set det [expr {[info exists P4X($fid)] && $P4X($fid) >= 0 && $P1X($fid) >= 0}]
        if {$det} {
            lappend pux $P1X($fid); # placeholder; pupil not needed for deg
            lappend a1x $P1X($fid); lappend a1y $P1Y($fid)
            lappend a4x $P4X($fid); lappend a4y $P4Y($fid)
            lappend rh [expr {$P4X($fid)-$P1X($fid)}]
            lappend rv [expr {$P1Y($fid)-$P4Y($fid)}]
            lappend prr $PR($fid); lappend vmask 1
        } else {
            lappend a1x -1; lappend a1y -1; lappend a4x -1; lappend a4y -1
            lappend rh 0.0; lappend rv 0.0; lappend prr -1; lappend vmask 0
        }
    }
    # raw -> degrees
    dl_local rhd [dl_flist {*}$rh]; dl_local rvd [dl_flist {*}$rv]
    dl_local cal [em::apply_calibration $coeffs $rhd $rvd]
    set hd [dl_tcllist $cal:0]; set vd [dl_tcllist $cal:1]
    # plausibility + hold-last-valid; build final per-sample lists
    set fh2 {}; set fv2 {}; set blk {}; set raw {}; set lasth 0.0; set lastv 0.0
    foreach v $vmask h $hd vv $vd ob $obl rhh $rh rvv $rv {
        incr tot
        set ok [expr {$v && abs($h) <= $::MAXDEG && abs($vv) <= $::MAXDEG}]
        if {$ok} {
            set lasth $h; set lastv $vv; incr good
            lappend fh2 $h; lappend fv2 $vv; lappend blk $ob
            lappend raw $rhh; lappend raw $rvv
        } else {
            lappend fh2 $lasth; lappend fv2 $lastv; lappend blk 1
            lappend raw 0.0; lappend raw 0.0
        }
    }
    dl_append $A1X [dl_flist {*}$a1x]; dl_append $A1Y [dl_flist {*}$a1y]
    dl_append $A4X [dl_flist {*}$a4x]; dl_append $A4Y [dl_flist {*}$a4y]
    dl_append $RAWH [dl_flist {*}$rh]; dl_append $RAWV [dl_flist {*}$rv]
    dl_append $HDEG [dl_flist {*}$fh2]; dl_append $VDEG [dl_flist {*}$fv2]
    dl_append $PRR [dl_flist {*}$prr]; dl_append $BLK [dl_ilist {*}$blk]
    dl_append $RAW [dl_flist {*}$raw]; dl_append $VALID [dl_ilist {*}$vmask]
}

# overwrite canonical eye columns (keep em_time/em_seconds/frame_id/pupil_x/y as-is
# except p1/p4/raw/deg/pupil_r/in_blink which we replace)
dl_set $tg:p1_x $A1X; dl_set $tg:p1_y $A1Y
dl_set $tg:p4_x $A4X; dl_set $tg:p4_y $A4Y
dl_set $tg:eye_raw_h $RAWH; dl_set $tg:eye_raw_v $RAWV
dl_set $tg:em_h_deg $HDEG; dl_set $tg:em_v_deg $VDEG
dl_set $tg:pupil_r $PRR; dl_set $tg:in_blink $BLK
dl_set $tg:eye_raw $RAW; dl_set $tg:em_valid $VALID

# re-derive saccade/pursuit/fixation/em_state from the new gaze
em::extract_events $tg

dg_write $tg $out_path
puts [format "REPLACED eye data + re-derived events. valid samples %d/%d = %.1f%%  -> %s" \
        $good $tot [expr {100.0*$good/$tot}] $out_path]
