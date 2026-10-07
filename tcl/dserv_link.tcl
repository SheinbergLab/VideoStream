#
# dserv_link.tcl - the connection to a dserv, shared by the launcher scripts
# (serve.tcl, tracker.tcl).
#
# "Connected to a dserv" is two links:
#   forwarding    our datapoints (eyetracking/results, ...) go to the dserv
#                 (vstream::dsForward; reconnects by itself)
#   subscription  the dserv pushes ess/in_obs and ess/datafile to us
#                 (%reg + %match), arriving as ds/* events
#
# Subscribing never blocks this (main-loop) thread: vstream::dsSubscribe
# queues the %reg/%match on a worker and returns, and ::dsl::poll picks up
# the outcome. Done here instead, an unreachable dserv stalled the main loop
# up to 2 s per attempt -- the whole frame buffer at 250 Hz.
#
# dserv delivers subscriptions over ONE persistent connect-back socket per
# registration, opened by %reg and closed if dserv reaps us (send stall,
# hard error) or restarts.  It never reconnects on its own and nothing
# tells the client.  On 2026-09-10 a tracker ran ~10 days with a dead
# subscription, never heard about a datafile open, and four sessions of eye
# timing were logged against a 10-day-old anchor.  vstream::dsConnections
# counts the live connect-back sockets; a registered client with zero of
# them has lost its subscriptions, so the watchdog re-subscribes (backing
# off 2..30 s while the dserv is unreachable) and then calls the launcher's
# ::dserv_on_subscribed (tracker.tcl: ask dserv for the current datafile,
# what the push would have said).
#
# Commands:
#   dserv_connect host ?port?    connect both links (port: the dserv's
#                                datapoint port, 4620 unless it advertises
#                                another); replaces any current dserv.
#                                Returns at once; the subscription follows.
#   dserv_disconnect             drop both links
#   dserv_status                 dict: host port forwarding subscribed
#                                subscribing error discovery found
#   dserv_startup                at launch: --ds-host if given, else the
#                                dserv saved in the viewer settings
#
# The browser connects by saving the `dserv` setting (::vs::put dserv
# "host port", or "" to disconnect), which calls dserv_connect /
# dserv_disconnect and is remembered for the next start.
#
# Optional launcher hooks:
#   ::dserv_on_subscribed        after each successful (re)subscription
#   ::dserv_matches              datapoints to subscribe to (default below)
#

namespace eval ::dsl {
    variable host ""
    variable port 4620
    variable subscribed 0
    variable ever_subscribed 0       ;# for "connected" vs "re-registered" in the log
    variable lost_logged 0
    variable error ""                ;# why the last subscribe failed
    variable watch_ms 2000           ;# subscription liveness check period
    variable retry_max_ms 30000      ;# backoff ceiling while dserv is unreachable
    variable retry_ms 2000           ;# current delay before the next check
    variable watch_id ""
    variable sub_seq 0               ;# the subscribe job we are waiting for
    variable poll_id ""
    variable poll_ms 100
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

# Queue a subscribe to the current dserv and start polling for its outcome.
proc ::dsl::request_subscribe {} {
    variable host
    variable port
    variable sub_seq
    variable poll_id
    variable poll_ms
    variable default_matches
    set matches $default_matches
    if { [info exists ::dserv_matches] } { set matches $::dserv_matches }
    set sub_seq [vstream::dsSubscribe $host $port $matches]
    if { $poll_id eq "" } { set poll_id [after $poll_ms ::dsl::poll] }
}

proc ::dsl::poll {} {
    variable host
    variable subscribed
    variable ever_subscribed
    variable lost_logged
    variable error
    variable sub_seq
    variable poll_id
    variable poll_ms
    variable retry_ms
    variable retry_max_ms
    variable watch_ms
    set poll_id ""
    if { $host eq "" } { return }
    set st [vstream::dsSubscribeStatus]
    if { [dict get $st seq] != $sub_seq || [dict get $st state] eq "pending" } {
        set poll_id [after $poll_ms ::dsl::poll]
        return
    }
    if { [dict get $st state] eq "ok" } {
        set subscribed 1
        set error ""
        set retry_ms $watch_ms
        log [expr { $ever_subscribed ? "re-registered with $host" : "connected to $host" }]
        set ever_subscribed 1
        set lost_logged 0
        if { [llength [info commands ::dserv_on_subscribed]] } {
            if { [catch ::dserv_on_subscribed err] } { log "dserv_on_subscribed: $err" }
        }
    } else {
        set subscribed 0
        set error [dict get $st error]
        if { !$lost_logged } {
            log "$error; will keep retrying"
            set lost_logged 1
        }
        set retry_ms [expr { min($retry_ms * 2, $retry_max_ms) }]
    }
    announce
}

# Liveness check. Runs every retry_ms; queues a re-subscribe when the
# subscription is gone and none is already in flight.
proc ::dsl::watch {} {
    variable host
    variable subscribed
    variable lost_logged
    variable retry_ms
    variable watch_id
    variable poll_id
    set watch_id ""
    if { $host eq "" } { return }
    if { $poll_id eq "" } {
        if { $subscribed && [vstream::dsConnections] == 0 } {
            set subscribed 0
            log "subscription to $host lost (no live connect-back); re-registering"
            set lost_logged 1
            announce
        }
        if { !$subscribed } { request_subscribe }
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
    set ::dsl::subscribed 0
    set ::dsl::ever_subscribed 0
    set ::dsl::lost_logged 0
    set ::dsl::error ""
    set ::dsl::retry_ms $::dsl::watch_ms
    vstream::dsForward $host $port
    ::dsl::log "connecting to $host:$port"
    ::dsl::request_subscribe
    if { $::dsl::watch_id eq "" } {
        set ::dsl::watch_id [after $::dsl::retry_ms ::dsl::watch]
    }
    ::dsl::announce
    return [dserv_status]
}

proc dserv_disconnect {} {
    if { $::dsl::host ne "" } {
        # Only a dserv that ever accepted our %reg has anything to drop; one
        # that never answered would just cost the worker another 2 s timeout
        # ahead of the next connect.
        if { $::dsl::ever_subscribed } {
            vstream::dsUnsubscribe $::dsl::host $::dsl::port
        }
        ::dsl::log "disconnected from $::dsl::host"
    }
    vstream::dsForward off
    set ::dsl::host ""
    set ::dsl::subscribed 0
    set ::dsl::error ""
    foreach id {watch_id poll_id} {
        if { [set ::dsl::$id] ne "" } {
            after cancel [set ::dsl::$id]
            set ::dsl::$id ""
        }
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
                subscribing [expr { $::dsl::poll_id ne "" }] \
                error $::dsl::error \
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
