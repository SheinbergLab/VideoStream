// Tcl Code to be evaluated directly

const char *ds_str = R"V0G0N(
  proc vstream::dsRegister { server { port 4620 } } {
	set s [socket $server $port]
	fconfigure $s -buffering line
	set our_ip [lindex [fconfigure $s -sockname] 0]
	puts $s "%reg $our_ip $::vstream::dsPort"
	catch {gets $s status}
	close $s
        if { $status } { 
           set ::vstream::dsServerIP $server 
           set ::vstream::dsServerPort $port
        }
	return $status
    }

    proc vstream::dsUnregister { } {
        if { ![info exists ::vstream::dsServerIP] } { 
             return 0 
        } else {
             set server $::vstream::dsServerIP
        }
        if { ![info exists ::vstream::dsServerPort] } { 
             return 0 
        } else {
             set port $::vstream::dsServerPort
        }
	set s [socket $server $port]
	fconfigure $s -buffering line
	set our_ip [lindex [fconfigure $s -sockname] 0]
	puts $s "%unreg $our_ip $::vstream::dsPort"
	catch {gets $s status}
	close $s
        unset ::vstream::dsServerIP ::vstream::dsServerPort
	return $status
    }

    proc vstream::dsAddMatch { server pattern { port 4620 } } {
	set s [socket $server $port]
	fconfigure $s -buffering line
	set our_ip [lindex [fconfigure $s -sockname] 0]
	puts $s "%match $our_ip $::vstream::dsPort $pattern 1"
	catch {gets $s status}
	close $s
	return $status
    }

    proc vstream::dsRemoveMatch { server pattern { port 4620 } } {
	set s [socket $server $port]
	fconfigure $s -buffering line
	set our_ip [lindex [fconfigure $s -sockname] 0]
	puts $s "%unmatch $our_ip $::vstream::dsPort $pattern"
	catch {gets $s status}
	close $s
	return $status
    }
)V0G0N";
