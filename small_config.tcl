puts "Configuring camera"
# Setup camera
if { !$vstream::useWebcam } { 
#    vstream::configureExposure 9500.0
#    vstream::configureGain 6.0
#    vstream::configureROI 960 780 280 100; # width, height, shift left, shift up
    vstream::configureROI 400 400 100 100
    
    vstream::configureFrameRate 100.0
}

# Can override output format of videos using FourCC codes
#vstream::fourcc XVID

# set dataserver

proc ds_setup { server } {

    set ::ds_server $server
    set ::ds_shutdown_added 0
    set ::ds_connected 0
    
    namespace inscope :: {
	set ::vstream::ds_tick_2 -2
	set ::vstream::ds_tick_1 -1
	set ::vstream::ds_tick 0

	# every tick
	#   update tick counts
	#   check if current count hasn't changed over last two
	#   if they are the same, try to reconnect to dataserver
	proc onWatchdog {} {
	    set ::vstream::ds_tick_2 $::vstream::ds_tick_1
	    set ::vstream::ds_tick_1 $::vstream::ds_tick
	    if { $::vstream::ds_tick_2 == $::vstream::ds_tick } {
		ds_connect
	    }
	}

	# update dataserver tick variable using callback
	proc watchdog { v } {
	    set ::vstream::ds_tick [lindex $::dsVals($v) 4]
	}
	
	proc beginobs { v } { vstream::inObs 1 }
	proc endobs { v } { vstream::inObs 0 }
	
	proc openfile { v } {
	    set filename [lindex $::dsVals($v) 4]
	    set folder ./videos
	    set outfile [file join $folder $filename.avi]
	    puts "fileopen $outfile"
	    vstream::fileOpen $outfile
	}
	
	proc closefile { v } {
	    set filename [lindex $::dsVals($v) 4]
	    puts "fileclose $filename"
	    vstream::fileClose
	}

	set dsCmds(ds:tick) watchdog
	
	set dsCmds(ess:obs:begin) beginobs
	set dsCmds(ess:obs:end) endobs
	
	set dsCmds(grasp:file:open) openfile
	set dsCmds(grasp:file:close) closefile
	
	proc ds_connect {} {
	    if { $::ds_connected } {
		vstream::dsUnregister
		set ::ds_connected 0
	    }
	    # Setup dataserver connections
	    if [vstream::dsRegister $::ds_server] {
		set ::ds_connected 1
	    } else {
		return 0
	    }

	    # Watchdog 1sec timer tick
	    vstream::dsAddMatch $::ds_server ds:tick
	    
	    # Obs period controls
	    vstream::dsAddMatch $::ds_server ess:obs:*

	    # Datafile controls
	    vstream::dsAddMatch $::ds_server grasp:file:*

	    if { !$::ds_shutdown_added } {
		::vstream::addShutdownCmd "vstream::dsUnregister"
		set ::ds_shutdown_added 1
	    }
	    return 1
	}
    }
}


#ds_setup 192.168.1.143
#ds_setup 128.148.110.17
#ds_connect

