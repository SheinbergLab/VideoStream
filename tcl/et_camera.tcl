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
#   nodes           {NodeName value ...} any further GenICam features, set last
#
# Values are applied in that order (geometry before frame rate, so the rate
# limits are those of the final ROI/binning).

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
        if {[dict exists $settings nodes]} {
            foreach {name value} [dict get $settings nodes] {
                camera::node $name $value
            }
        }
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
