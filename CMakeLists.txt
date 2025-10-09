cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release)

project( VideoStream )

if(WIN32)
set (OpenCV_DIR c:/usr/local/lib/opencv/build/x64/vc15/lib)
endif()

find_package( OpenCV REQUIRED )

find_library(LZ4_LIBRARY NAMES lz4)

if(WIN32)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -DUSE_FLIR=1")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	set(TCLLIB tcl90t_64)  # Updated to Tcl 9
	include_directories("c:/usr/local/include" "C:/Program Files/Point Grey Research/Spinnaker/include")
	link_directories("c:/usr/local/lib64" "c:/usr/local/lib" "C:/Program Files/Point Grey Research/Spinnaker/lib64/vs2015")
	set(DGLIB dg)
	set(FLIRLIB Spinnaker_v140)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
	set(TCLLIB tcl90t)  # Updated to Tcl 9
	set(LABLIB lablib)
endif()
elseif(APPLE)
	# Try to find Tcl 9 first, fall back to 8.6
	find_library(TCL_LIBRARY 
	    NAMES tcl9.0 tcl90 tcl9 tcl8.6
	    HINTS /usr/local/lib /opt/homebrew/lib
	)
	if(TCL_LIBRARY)
	    get_filename_component(TCLLIB ${TCL_LIBRARY} NAME_WE)
	else()
	    set(TCLLIB tcl8.6)  # Fallback
	endif()
	set(DGLIB dg)
	set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}")
	include_directories(/usr/local/include /opt/homebrew/include)
	link_directories(/usr/local/lib /opt/homebrew/lib)
else()  # Linux
	find_library(LIBDL dl)
	find_library(LIBPTHREAD pthread)

        find_library(FLIR_LIBRARY
          NAMES Spinnaker
          PATHS /opt/spinnaker/lib
          REQUIRED
        )
	
	# Try to find Tcl 9 first
	find_path(TCL_INCLUDE_DIR tcl.h
	    PATHS /usr/include/tcl9.0 /usr/include/tcl9 /usr/include/tcl8.6 /usr/local/include
	    PATH_SUFFIXES tcl9.0 tcl9 tcl8.6
	)
	
	find_library(TCL_LIBRARY
	    NAMES tcl9.0 tcl90 tcl9 tcl8.6
	    PATHS /usr/lib /usr/local/lib
	)
	
	if(TCL_LIBRARY)
	    get_filename_component(TCLLIB ${TCL_LIBRARY} NAME_WE)
	else()
	    set(TCLLIB tcl8.6)  # Fallback
	endif()
	
	# REMOVE THIS LINE FROM HERE
	# link_directories(/opt/spinnaker/lib)
	
	set(DGLIB dg)
	set(FLIR_INCLUDE_DIR /opt/spinnaker/include)
	set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-conversion-null -DUSE_FLIR=1")
	add_definitions(-DUSE_FLIR=1)
endif()

include_directories( ${OpenCV_INCLUDE_DIRS} ${TCL_INCLUDE_DIR} ${FLIR_INCLUDE_DIR} )

add_executable( VideoStream VideoStream.cpp FrameBufferManager.cpp
		SourceManager.cpp Widget.cpp SamplingManager.cpp ReviewModeSource.cpp WebcamSource.cpp
		FlirCameraSource.cpp VideoFileSource.cpp AnalysisPluginRegistry.cpp tclproc.cpp )

# Link with found libraries
target_link_libraries( VideoStream 
    ${OpenCV_LIBS} 
    ${TCL_LIBRARY}    # Use the found library instead of hardcoded name
    ${LIBPTHREAD} 
    ${FLIR_LIBRARY} 
    ${LIBDL} 
    ${DGLIB} 
    ${LZ4_LIBRARY}    # Use the found lz4 library
    z 
)

# For the host executable on Linux
if(UNIX AND NOT APPLE)
    target_link_options(VideoStream PRIVATE -rdynamic)
endif()

# For static linking of lz4, you might need to set this
if(LZ4_USE_STATIC_LIBS)
    set_target_properties(VideoStream PROPERTIES LINK_SEARCH_START_STATIC ON)
    set_target_properties(VideoStream PROPERTIES LINK_SEARCH_END_STATIC OFF)
endif()

######
##### Plugins
######

# Build eye tracking plugin
add_library(eyetracking SHARED 
    plugins/EyeTrackingPlugin.cpp
)

target_include_directories(eyetracking PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${TCL_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}  # For IAnalysisPlugin.h, etc.
)

target_link_libraries(eyetracking
    ${OpenCV_LIBS}
    ${TCL_LIBRARY}
)

# Set output directory for plugin
set_target_properties(eyetracking PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins"
    PREFIX ""  # Don't add 'lib' prefix on Unix
)

if(APPLE)
    set_target_properties(eyetracking PROPERTIES
        LINK_FLAGS "-undefined dynamic_lookup"
    )
endif()
