#
# viewer_settings.tcl - the browser viewer's settings, held by the server.
#
# Every setting a person can change from the viewer (detector thresholds, ROI,
# camera gain/exposure/frame rate/binning, and page layout such as panel state
# and overlay layers) lives here, in one file, and every browser that connects
# adopts it. The browser stores nothing of its own.
#
#   ~/.config/videostream/viewer_settings.dict    (env VS_SETTINGS_FILE overrides)
#
# The file is a plain Tcl dict, one "key value" per line, safe to edit while the
# server is stopped. It is written automatically ~0.5 s after the last change.
#
# Launcher scripts (tracker.tcl, serve.tcl) source this file, then:
#   ::vs::load                       read the file
#   ::vs::apply_saved                apply saved detector/ROI/playback values
#   ::vs::apply_camera               apply saved camera values (when a camera opens)
#   proc apply_default_tuning {}     (launcher) restore the code defaults
#   set ::vs::default_roi {x y w h}  (launcher) ROI that "reset roi" returns to
#   proc ::vs::camera_defaults {}    (launcher, optional) re-apply the script's camera settings
#
# Commands the browser calls (through eval):
#   ::vs::all                        dict of every saved key and value
#   ::vs::put key value              validate, apply, remember, save, tell all browsers
#   ::vs::drop key ...               forget saved keys (the value in effect is left alone)
#   ::vs::reset detector|roi|camera|ui|all
#                                    forget a group of saved keys and put the
#                                    script's own defaults back
#
# Keys:
#   det.<name>            detector parameter (names as eyetracking::getSettings) and det.mode
#   roi                   "x y w h"        roi.follow   0|1
#   cam.<vendor>.gain|exposure_us|fps|binning     per camera backend, e.g. cam.lucid.gain
#   ui.layers ui.layersOpen ui.tuningOpen ui.history.open ui.history.window
#   ui.pickerSort playback.speed
#   dserv                 "host port" of the dserv to connect to, "" for none
#                         (applied by dserv_connect / dserv_disconnect from
#                         dserv_link.tcl; reconnected at start by dserv_startup)
#
# Events (to every browser, via vstream/*):
#   vstream/settings        {key value}
#   vstream/settings_unset  {key ...}

namespace eval ::vs {
    variable values [dict create]
    variable loaded 0
    variable save_timer ""
    variable save_delay_ms 500
    variable default_roi {}

    # detector key -> eyetracking setter
    variable detector_setter [dict create \
        pupil_threshold          setPupilThreshold \
        p1_min_intensity         setP1MinIntensity \
        p1_max_jump              setP1MaxJump \
        p1_min_area              setP1MinArea \
        p1_max_area              setP1MaxArea \
        p1_pupil_radius_max      setP1PupilRadiusMax \
        p4_min_intensity         setP4MinIntensity \
        p4_max_jump              setP4MaxJump \
        p4_max_prediction_error  setP4MaxPredictionError]

    variable detection_modes {full pupil_p1 pupil_only}
    variable camera_vendors {flir lucid webcam}
    variable camera_fields {binning fps exposure_us gain}   ;# the order they are applied in
    variable ui_flags {ui.layersOpen ui.tuningOpen ui.history.open}
    variable max_value_length 4000

    proc path {} {
        if {[info exists ::env(VS_SETTINGS_FILE)] && $::env(VS_SETTINGS_FILE) ne ""} {
            return $::env(VS_SETTINGS_FILE)
        }
        set home [expr {[info exists ::env(HOME)] ? $::env(HOME) : "~"}]
        return [file join $home .config videostream viewer_settings.dict]
    }

    proc is_number {v} { return [string is double -strict $v] }
    proc is_flag {v}   { return [expr {$v eq "0" || $v eq "1"}] }

    # Check one key/value and return the value to keep; an error if it is not
    # a setting the viewer owns.
    proc validate {key value} {
        variable detector_setter
        variable detection_modes
        variable camera_vendors
        variable ui_flags
        variable max_value_length
        if {[string first "\n" $value] >= 0 || [string length $value] > $max_value_length} {
            return -code error "bad value for $key"
        }
        set value [string trim $value]
        if {[string match det.* $key]} {
            set name [string range $key 4 end]
            if {$name eq "mode"} {
                if {$value ni $detection_modes} { return -code error "det.mode must be one of $detection_modes" }
                return $value
            }
            if {![dict exists $detector_setter $name]} { return -code error "unknown setting $key" }
            if {![is_number $value]} { return -code error "$key must be a number" }
            return $value
        }
        switch -exact -- $key {
            roi {
                if {[llength $value] != 4} { return -code error "roi must be: x y w h" }
                foreach n $value { if {![string is integer -strict $n]} { return -code error "roi must be integers" } }
                if {[lindex $value 2] < 1 || [lindex $value 3] < 1} { return -code error "roi width and height must be positive" }
                return [join $value " "]
            }
            roi.follow - ui.layersOpen - ui.tuningOpen - ui.history.open {
                if {![is_flag $value]} { return -code error "$key must be 0 or 1" }
                return $value
            }
            ui.history.window {
                if {![string is integer -strict $value] || $value < 1 || $value > 600} { return -code error "$key must be a number of seconds" }
                return $value
            }
            playback.speed {
                if {![is_number $value] || $value < 0.25 || $value > 2.0} { return -code error "playback.speed must be 0.25 to 2" }
                return $value
            }
            ui.layers - ui.pickerSort { return $value }
            dserv {
                if {$value eq ""} { return "" }
                if {[llength $value] != 2 || [lindex $value 0] eq ""
                    || ![string is integer -strict [lindex $value 1]]
                    || [lindex $value 1] < 1 || [lindex $value 1] > 65535} {
                    return -code error "dserv must be: host port (or empty for none)"
                }
                return [join $value " "]
            }
        }
        if {[regexp {^cam\.([a-z0-9]+)\.([a-z_]+)$} $key -> vendor field]} {
            if {$vendor ni $camera_vendors} { return -code error "unknown camera backend $vendor" }
            switch -exact -- $field {
                gain {
                    if {![is_number $value]} { return -code error "$key must be a number" }
                }
                exposure_us - fps {
                    if {![is_number $value] || $value <= 0} { return -code error "$key must be a positive number" }
                }
                binning {
                    if {[llength $value] != 2 || ![string is integer -strict [lindex $value 0]] || ![string is integer -strict [lindex $value 1]]} {
                        return -code error "$key must be: horizontal vertical"
                    }
                    return [join $value " "]
                }
                default { return -code error "unknown setting $key" }
            }
            return $value
        }
        return -code error "unknown setting $key"
    }

    # ---- applying ------------------------------------------------------------

    proc active_camera {} {
        if {![llength [info commands ::camera::vendor]]} { return "" }
        if {[catch {::camera::vendor} v]} { return "" }
        return $v
    }

    # Put one value into effect. Returns the value to remember (for the camera,
    # what the camera ended up with).
    proc apply {key value} {
        variable detector_setter
        if {[string match det.* $key]} {
            set name [string range $key 4 end]
            if {$name eq "mode"} {
                eyetracking::setDetectionMode $value
            } else {
                eyetracking::[dict get $detector_setter $name] $value
            }
            return $value
        }
        switch -exact -- $key {
            roi        { eyetracking::setROI {*}$value }
            roi.follow { eyetracking::roiFollow $value }
            playback.speed {
                if {[llength [info commands ::set_playback_speed]]} { ::set_playback_speed $value }
            }
            dserv {
                if {[llength [info commands ::dserv_connect]]} {
                    if {$value eq ""} { ::dserv_disconnect } else { ::dserv_connect {*}$value }
                }
            }
        }
        if {[regexp {^cam\.([a-z0-9]+)\.([a-z_]+)$} $key -> vendor field]} {
            if {[active_camera] eq $vendor} { return [apply_camera_field $field $value] }
        }
        return $value
    }

    proc apply_camera_field {field value} {
        switch -exact -- $field {
            gain        { camera::configureGain $value;      return [format %.3g [camera::configureGain]] }
            fps         { camera::configureFrameRate $value; return [format %.6g [camera::configureFrameRate]] }
            exposure_us { camera::configureExposure $value;  return [format %.6g [camera::node ExposureTime]] }
            binning     { camera::configureBinning {*}$value; return $value }
        }
        return $value
    }

    # Apply the saved camera values for the camera that is open now. Errors are
    # reported but do not stop the rest.
    proc apply_camera {} {
        variable values
        variable camera_fields
        set vendor [active_camera]
        if {$vendor eq ""} { return }
        foreach field $camera_fields {
            set key cam.$vendor.$field
            if {![dict exists $values $key]} continue
            if {[catch {apply_camera_field $field [dict get $values $key]} err]} {
                puts "viewer settings: could not apply $key ([dict get $values $key]): $err"
            }
        }
    }

    # Apply the saved detector, ROI and playback values (after the script's defaults).
    proc apply_saved {} {
        variable values
        foreach key [lsort [dict keys $values]] {
            if {[string match det.* $key] || $key eq "roi" || $key eq "roi.follow" || $key eq "playback.speed"} {
                if {[catch {apply $key [dict get $values $key]} err]} {
                    puts "viewer settings: could not apply $key ([dict get $values $key]): $err"
                }
            }
        }
    }

    # ---- the commands the browser uses ------------------------------------------

    proc all {} {
        variable values
        return $values
    }

    proc announce {event data} {
        if {[llength [info commands ::vstream::fireEvent]]} {
            catch {::vstream::fireEvent $event $data}
        }
    }

    proc put {key value} {
        variable values
        set value [validate $key $value]
        set kept [apply $key $value]
        dict set values $key $kept
        schedule_save
        announce vstream/settings [list $key $kept]
        return $kept
    }

    proc drop {args} {
        variable values
        set gone {}
        foreach key $args {
            if {[dict exists $values $key]} {
                dict unset values $key
                lappend gone $key
            }
        }
        if {[llength $gone]} {
            schedule_save
            announce vstream/settings_unset $gone
        }
        return $gone
    }

    proc keys_in_group {group} {
        variable values
        set out {}
        foreach key [dict keys $values] {
            switch -exact -- $group {
                detector { if {[string match det.* $key]} { lappend out $key } }
                roi      { if {$key eq "roi" || $key eq "roi.follow"} { lappend out $key } }
                camera   { if {[string match cam.* $key]} { lappend out $key } }
                ui       { if {[string match ui.* $key] || $key eq "playback.speed"} { lappend out $key } }
            }
        }
        return $out
    }

    # Forget a group of saved values and put the script's own defaults back.
    proc reset {group} {
        variable default_roi
        if {$group eq "all"} {
            foreach g {detector roi camera ui} { reset $g }
            return all
        }
        if {$group ni {detector roi camera ui}} { return -code error "unknown group $group" }
        drop {*}[keys_in_group $group]
        switch -exact -- $group {
            detector {
                if {[llength [info commands ::apply_default_tuning]]} { ::apply_default_tuning }
            }
            roi {
                if {[llength $default_roi] == 4} {
                    catch {eyetracking::setROI {*}$default_roi}
                    catch {eyetracking::roiFollow 0}
                }
            }
            camera {
                if {[active_camera] ne "" && [llength [info commands ::vs::camera_defaults]]} {
                    catch ::vs::camera_defaults
                }
            }
            ui {
                if {[llength [info commands ::set_playback_speed]]} { catch {::set_playback_speed 1.0} }
            }
        }
        return $group
    }

    # ---- the file ---------------------------------------------------------------

    proc load {} {
        variable values
        variable loaded
        set loaded 1
        set values [dict create]
        set file [path]
        if {![file exists $file]} { return 0 }
        if {[catch {
            set f [open $file r]
            set text [read $f]
            close $f
            set lines {}
            foreach line [split $text "\n"] {
                if {[string match "#*" [string trim $line]]} continue
                lappend lines $line
            }
            set loaded_dict [dict create {*}[join $lines "\n"]]
        } err]} {
            catch {close $f}
            set bad $file.bad
            catch {file rename -force $file $bad}
            puts "viewer settings: $file is not readable ($err); moved to $bad and starting empty"
            return 0
        }
        dict for {key value} $loaded_dict {
            if {[catch {validate $key $value} kept]} {
                puts "viewer settings: ignoring $key in $file ($kept)"
            } else {
                dict set values $key $kept
            }
        }
        return [dict size $values]
    }

    proc save {} {
        variable values
        variable save_timer
        set save_timer ""
        set file [path]
        set tmp $file.tmp[pid]
        if {[catch {
            file mkdir [file dirname $file]
            set f [open $tmp w]
            puts $f "# VideoStream viewer settings. Written by the server when a setting changes;"
            puts $f "# safe to edit while the server is stopped. Delete a line to go back to the default."
            foreach key [lsort [dict keys $values]] {
                puts $f [list $key [dict get $values $key]]
            }
            close $f
            file rename -force $tmp $file
        } err]} {
            catch {close $f}
            catch {file delete $tmp}
            puts "viewer settings: could not save $file: $err"
        }
    }

    proc schedule_save {} {
        variable save_timer
        variable save_delay_ms
        if {$save_timer ne ""} { after cancel $save_timer }
        set save_timer [after $save_delay_ms ::vs::save]
    }

    # Write now if a save is waiting (called at shutdown).
    proc flush {} {
        variable save_timer
        if {$save_timer ne ""} {
            after cancel $save_timer
            save
        }
    }
}
