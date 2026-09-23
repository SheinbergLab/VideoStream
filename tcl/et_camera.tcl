#
# et_camera.tcl - live-camera settings shared by the launchers.
#
# Each launcher keeps its own per-backend settings dict (the FLIR rig and the
# Lucid rig want different exposures, and the Lucid drives the IR source from
# an output line); this applies one of them through the camera::* commands.
#
#   ::et_camera::apply_live_settings $settings
#
# settings keys (all optional):
#   exposure_us     camera::configureExposure
#   gain_db         camera::configureGain
#   orientation     {reverseX reverseY} -> camera::configureImageOrientation
#   binning         {h v}               -> camera::configureBinning
#   fps             camera::configureFrameRate
#   frame_time_us   AcquisitionFrameTime node (Lucid); falls back to fps = 1e6/t
#   strobe          {line N mode Output source ExposureActive inverter 0|1}
#                   -> camera::configureLine, i.e. an output that follows the
#                   exposure so a light source is synced to the shutter
#   ttl_line        N: the INPUT line whose state is recorded per frame as
#                   line_status (obs sync wire) -> camera::ttlLine. Set it
#                   explicitly when a strobe is configured: the default is
#                   whatever line the camera's LineSelector was left on.
#   nodes           {NodeName value ...} any further GenICam features, set last
#   ptp             {slave_only 1 wait_s 20}: enable IEEE 1588 on the camera
#                   (Lucid) so frame timestamps are on the LAN grandmaster's
#                   clock; waits up to wait_s for PtpStatus Slave, printing the
#                   offset from master. Do this before acquisition starts: the
#                   clock steps when it locks.
#
# Values are applied in that order (geometry before frame rate, so the rate
# limits are those of the final ROI/binning; PTP last).

namespace eval ::et_camera {

    proc apply_live_settings {settings} {
        if {[dict exists $settings exposure_us]} {
            camera::configureExposure [dict get $settings exposure_us]
        }
        if {[dict exists $settings gain_db]} {
            camera::configureGain [dict get $settings gain_db]
        }
        if {[dict exists $settings orientation]} {
            camera::configureImageOrientation {*}[dict get $settings orientation]
        }
        if {[dict exists $settings binning]} {
            camera::configureBinning {*}[dict get $settings binning]
        }
        if {[dict exists $settings frame_time_us]} {
            set t [dict get $settings frame_time_us]
            if {[catch {camera::node AcquisitionFrameTime $t} err]} {
                puts "AcquisitionFrameTime not settable ($err); using frame rate instead"
                camera::configureFrameRate [expr {1.0e6 / $t}]
            }
        } elseif {[dict exists $settings fps]} {
            camera::configureFrameRate [dict get $settings fps]
        }
        if {[dict exists $settings strobe]} {
            set s [dict get $settings strobe]
            foreach k {mode source inverter} { if {![dict exists $s $k]} { dict set s $k - } }
            set r [camera::configureLine [dict get $s line] \
                       [dict get $s mode] [dict get $s source] [dict get $s inverter]]
            puts "Strobe line: $r"
        }
        if {[dict exists $settings ttl_line]} {
            camera::ttlLine [dict get $settings ttl_line]
            puts "TTL input line: Line[camera::ttlLine] (lineStatusAll [camera::lineStatusAll])"
        }
        if {[dict exists $settings nodes]} {
            foreach {name value} [dict get $settings nodes] {
                camera::node $name $value
            }
        }
        if {[dict exists $settings ptp]} {
            ptp_enable [dict get $settings ptp]
        }
    }

    # Enable PTP (slave-only unless told otherwise) and wait for the clock to
    # lock to the grandmaster.  Returns the final status dict.
    proc ptp_enable {{opts {}}} {
        set slave_only [expr {[dict exists $opts slave_only] ? [dict get $opts slave_only] : 1}]
        set wait_s     [expr {[dict exists $opts wait_s] ? [dict get $opts wait_s] : 20}]
        if {[catch {
            camera::node PtpSlaveOnly $slave_only
            camera::node PtpEnable 1
        } err]} {
            puts "PTP: cannot enable on this camera ($err)"
            return {}
        }
        set deadline [expr {[clock milliseconds] + int($wait_s * 1000)}]
        while {1} {
            set st [ptp_status]
            set status [dict get $st status]
            if {$status eq "Slave" || [clock milliseconds] > $deadline} { break }
            after 500
        }
        if {$status eq "Slave"} {
            puts "PTP: locked to grandmaster, offset [dict get $st offset_ns] ns"
        } else {
            puts "PTP: status $status after ${wait_s}s (no grandmaster on the LAN?); timestamps stay on the camera's free-running clock"
        }
        return $st
    }

    # Latched PTP state: status (Disabled/Listening/Uncalibrated/Slave/...),
    # servo, offset_ns from the master, clock/parent/grandmaster IDs.
    proc ptp_status {} {
        set d [dict create enabled [camera::node PtpEnable]]
        catch { camera::node PtpDataSetLatch 1 }
        foreach {key node} {status PtpStatus servo PtpServoStatus offset_ns PtpOffsetFromMaster
                            clock_id PtpClockID parent_id PtpParentClockID grandmaster_id PtpGrandmasterClockID} {
            if {![catch {camera::node $node} v]} { dict set d $key $v }
        }
        return $d
    }

    # Settings for the current backend from a dict keyed by camera type
    # (flir / lucid); empty if there is none.
    proc settings_for_backend {by_type {type ""}} {
        if {$type eq ""} { set type $::camera_type }
        if {[dict exists $by_type $type]} { return [dict get $by_type $type] }
        puts "et_camera: no live settings for camera type '$type'"
        return {}
    }
}
