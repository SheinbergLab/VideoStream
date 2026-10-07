#
# dserv_link.tcl - the connection to a dserv, shared by the launcher scripts
# (serve.tcl, tracker.tcl).
#
# "Connected to a dserv" is two links:
#   forwarding    our datapoints (eyetracking/results, ...) go to the dserv
#                 (vstream::dsForward; reconnects by itself)
#   subscription  the dserv pushes ess/in_obs and ess/datafile to us
#                 (vstream::dsRegister + dsAddMatch), arriving as ds/* events
#
# dserv delivers subscriptions over ONE persistent connect-back socket per
# registration, opened by %reg and closed if dserv reaps us (send stall,
# hard error) or restarts.  It never reconnects on its own and nothing
# tells the client.  On 2026-09-10 a tracker ran ~10 days with a dead
# subscription, never heard about a datafile open, and four sessions of eye
# timing were logged against a 10-day-old anchor.  vstream::dsConnections
# counts the live connect-back sockets; a registered client with zero of
# them has lost its subscriptions, so the watchdog re-registers and calls
# the launcher's ::dserv_on_subscribed (tracker.tcl: ask dserv for the
# current datafile, what the push would have said).
#
# Commands:
#   dserv_connect host ?port?    connect both links (port: the dserv's
#                                datapoint port, 4620 unless it advertises
#                                another); replaces any current dserv
#   dserv_disconnect             drop both links
#   dserv_status                 dict: host port forwarding subscribed
#                                discovery found
#   dserv_startup                at launch: --ds-host if given, else the
#                                dserv saved in the viewer settings
#
# The browser connects by saving the `dserv` setting (::vs::put dserv
# "host port", or "" to disconnect), which calls dserv_connect /
# dserv_disconnect and is remembered for the next start.
#
# Optional launcher hooks:
#   ::dserv_on_subscribed        after each (re)registration
#   ::dserv_matches              datapoints to subscribe to (default below)
#

namespace eval ::dsl {
    variable host ""
    variable port 4620
    variable subscribed 0
    variable lost_logged 0
    variable watch_ms 2000           ;# subscription liveness check period
    variable retry_max_ms 30000      ;# backoff ceiling while dserv is unreachable
    variable retry_ms 2000           ;# current delay before the next check
    variable watch_id ""
    variable default_matches {ess/in_obs ess/datafile}
}

proc ::dsl::log { msg } {
    puts "\[[clock format [clock seconds] -format %H:%M:%S]\] dserv: $msg"
}

proc ::dsl::announce {} {
    if {[llength [info commands ::vstream::fireEvent]]} {
        catch { ::vstream::fireEvent vstream/dserv [dserv_status] }
    }
}

proc ::dsl::subscribe {} {
    variable host
    variable port
    variable subscribed
    variable default_matches
    if { [catch { vstream::dsRegister $host $port } ok] || !$ok } {
        set subscribed 0
        return 0
    }
    set matches $default_matches
    if { [info exists ::dserv_matches] } { set matches $::dserv_matches }
    foreach m $matches {
        vstream::dsAddMatch $host $m $port
    }
    set subscribed 1
    if { [llength [info commands ::dserv_on_subscribed]] } {
        if { [catch ::dserv_on_subscribed err] } {
            log "dserv_on_subscribed: $err"
        }
    }
    return 1
}

# Registering runs on the Tcl (main loop) thread and can take up to
# DservSocket's 2 s connect bound when the dserv is unreachable, so the next
# check is scheduled only after this one finishes, and failed attempts back
# off (2, 4, 8 ... 30 s). Rescheduling up front at a fixed 2 s let failed
# attempts run back to back and kept the main loop blocked nearly all the time.
proc ::dsl::watch {} {
    variable host
    variable subscribed
    variable lost_logged
    variable watch_ms
    variable retry_ms
    variable retry_max_ms
    variable watch_id
    set watch_id ""
    if { $host eq "" } { return }

    if { $subscribed && [vstream::dsConnections] > 0 } {
        set lost_logged 0
        set retry_ms $watch_ms
    } else {
        if { !$lost_logged } {
            log "subscription to $host lost (no live connect-back); re-registering"
            set lost_logged 1
        }
        if { [subscribe] } {
            log "re-registered with $host"
            set lost_logged 0
            set retry_ms $watch_ms
            announce
        } else {
            set retry_ms [expr { min($retry_ms * 2, $retry_max_ms) }]
        }
    }
    set watch_id [after $retry_ms ::dsl::watch]
}

proc dserv_connect { host {port 4620} } {
    if { $host eq "" } { return [dserv_disconnect] }
    if { ![string is integer -strict $port] || $port < 1 || $port > 65535 } {
        error "dserv port must be 1-65535"
    }
    if { $host eq $::dsl::host && $port == $::dsl::port } { return [dserv_status] }
    if { $::dsl::host ne "" } { dserv_disconnect }

    set ::dsl::host $host
    set ::dsl::port $port
    set ::dsl::lost_logged 0
    set ::dsl::retry_ms $::dsl::watch_ms
    vstream::dsForward $host $port
    if { [::dsl::subscribe] } {
        ::dsl::log "connected to $host:$port"
    } else {
        ::dsl::log "registration with $host:$port failed; will keep retrying"
        set ::dsl::lost_logged 1
    }
    if { $::dsl::watch_id eq "" } {
        set ::dsl::watch_id [after $::dsl::retry_ms ::dsl::watch]
    }
    ::dsl::announce
    return [dserv_status]
}

proc dserv_disconnect {} {
    if { $::dsl::host ne "" } {
        catch { vstream::dsUnregister }
        ::dsl::log "disconnected from $::dsl::host"
    }
    vstream::dsForward off
    set ::dsl::host ""
    set ::dsl::subscribed 0
    if { $::dsl::watch_id ne "" } {
        after cancel $::dsl::watch_id
        set ::dsl::watch_id ""
    }
    ::dsl::announce
    return [dserv_status]
}

proc dserv_status {} {
    set fwd [vstream::dsForward]
    set subscribed [expr { $::dsl::subscribed && [vstream::dsConnections] > 0 }]
    return [dict create \
                host $::dsl::host \
                port [expr { $::dsl::host eq "" ? "" : $::dsl::port }] \
                forwarding [dict get $fwd connected] \
                subscribed $subscribed \
                discovery [vstream::dservDiscovery] \
                found [vstream::dservList]]
}

proc dserv_startup {} {
    if { $vstream::dsHost ne "" } {
        set port 4620
        if { [info exists vstream::dsPortDefault] } { set port $vstream::dsPortDefault }
        dserv_connect $vstream::dsHost $port
        return
    }
    if { [llength [info commands ::vs::all]] && [dict exists [::vs::all] dserv] } {
        set saved [dict get [::vs::all] dserv]
        if { [llength $saved] == 2 } { dserv_connect {*}$saved }
    }
}
