cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Release)

project( VideoStream )

if(WIN32)
set (OpenCV_DIR c:/usr/local/lib/opencv/build/x64/vc15/lib)
endif()

find_package( OpenCV REQUIRED )


# Add uWebSockets as external project
include(FetchContent)

# Set shorter timeouts for FetchContent
set(FETCHCONTENT_QUIET OFF)  # Show progress
set(CMAKE_TIMEOUT 120)       # 2 minute timeout

FetchContent_Declare(
  uwebsockets
  GIT_REPOSITORY https://github.com/uNetworking/uWebSockets.git
  GIT_TAG        v20.62.0
  GIT_SHALLOW    TRUE
  GIT_SUBMODULES ""
  TIMEOUT        120
)

FetchContent_Declare(
  usockets
  GIT_REPOSITORY https://github.com/uNetworking/uSockets.git
  GIT_TAG        v0.8.8  
  GIT_SHALLOW    TRUE
  GIT_SUBMODULES ""  # This prevents boringssl/lsquic from being cloned
  TIMEOUT        120
)

# Make both available
FetchContent_MakeAvailable(usockets uwebsockets)

# Add uWebSockets include directories
include_directories(${uwebsockets_SOURCE_DIR}/src)
include_directories(${usockets_SOURCE_DIR}/src)

# Also add the parent directory for proper includes
include_directories(${uwebsockets_SOURCE_DIR})

# find sqlite header
include_directories(external/sqlite)

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
	find_library(LIBJANSSON NAMES "libjansson.a")
	set(DGLIB dg)
	set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}")
	include_directories(/usr/local/include /opt/homebrew/include)
	link_directories(/usr/local/lib /opt/homebrew/lib)

   # macOS requires libuv for uSockets
    find_package(PkgConfig)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBUV libuv)
        if(NOT LIBUV_FOUND)
            find_library(LIBUV_LIBRARIES NAMES uv libuv)
            if(LIBUV_LIBRARIES)
                set(LIBUV_FOUND TRUE)
            endif()
        endif()
    else()
        find_library(LIBUV_LIBRARIES NAMES uv libuv)
        if(LIBUV_LIBRARIES)
            set(LIBUV_FOUND TRUE)
        endif()
    endif()
    
    if(NOT LIBUV_FOUND)
        message(FATAL_ERROR "libuv is required on macOS. Install with: brew install libuv")
    endif()
    
    set(USOCKETS_LIBS ${LIBUV_LIBRARIES})
    set(WITH_LIBUV 1)

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
	
        find_library(LIBJANSSON NAMES "libjansson.a")
	
	set(DGLIB dg)
	set(FLIR_INCLUDE_DIR /opt/spinnaker/include)
	set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-conversion-null -DUSE_FLIR=1")
	add_definitions(-DUSE_FLIR=1)
endif()


# Build uSockets using its own build system approach
# Collect the core source files
set(USOCKETS_SOURCES
    ${usockets_SOURCE_DIR}/src/bsd.c
    ${usockets_SOURCE_DIR}/src/context.c
    ${usockets_SOURCE_DIR}/src/loop.c
    ${usockets_SOURCE_DIR}/src/socket.c
)

# Add event loop implementation based on platform
if(WITH_LIBUV)
    list(APPEND USOCKETS_SOURCES ${usockets_SOURCE_DIR}/src/eventing/libuv.c)
else()
    list(APPEND USOCKETS_SOURCES ${usockets_SOURCE_DIR}/src/eventing/epoll_kqueue.c)
endif()

# Check for additional files that might exist in different versions
if(EXISTS ${usockets_SOURCE_DIR}/src/eventing/gcd.c)
    list(APPEND USOCKETS_SOURCES ${usockets_SOURCE_DIR}/src/eventing/gcd.c)
endif()

# Check for loop_data.c which contains the missing functions
if(EXISTS ${usockets_SOURCE_DIR}/src/loop_data.c)
    list(APPEND USOCKETS_SOURCES ${usockets_SOURCE_DIR}/src/loop_data.c)
endif()

# Check for internal directory
if(EXISTS ${usockets_SOURCE_DIR}/src/internal/internal.c)
    list(APPEND USOCKETS_SOURCES ${usockets_SOURCE_DIR}/src/internal/internal.c)
endif()

# Check for other potential source files
file(GLOB USOCKETS_INTERNAL_SOURCES ${usockets_SOURCE_DIR}/src/internal/*.c)
if(USOCKETS_INTERNAL_SOURCES)
    list(APPEND USOCKETS_SOURCES ${USOCKETS_INTERNAL_SOURCES})
endif()

# Print what we're building
message(STATUS "Building uSockets with sources: ${USOCKETS_SOURCES}")

# Create the library
add_library(usockets STATIC ${USOCKETS_SOURCES})

# Set uSockets compile definitions (no SSL)
target_compile_definitions(usockets PRIVATE LIBUS_NO_SSL)

# Platform-specific event loop selection
if(WIN32)
    # Windows uses libuv by default
    target_compile_definitions(usockets PRIVATE LIBUS_USE_LIBUV)
elseif(APPLE)
    # macOS uses libuv
    target_compile_definitions(usockets PRIVATE LIBUS_USE_LIBUV)
    target_include_directories(usockets PRIVATE ${LIBUV_INCLUDE_DIRS})
else()
    # Linux uses epoll
    target_compile_definitions(usockets PRIVATE LIBUS_USE_EPOLL)
endif()

# Add compile flags that uSockets expects
if(NOT WIN32)
    target_compile_options(usockets PRIVATE -std=c11 -flto)
endif()

# Include libuv headers if found
if(LIBUV_FOUND)
    target_include_directories(usockets PRIVATE ${LIBUV_INCLUDE_DIRS})
endif()

# Create directory for web assets
set(WEB_ASSETS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/www)
file(MAKE_DIRECTORY ${WEB_ASSETS_DIR})


# Function to embed files as C++ headers
function(embed_resource input_file output_file variable_name)
    add_custom_command(
        OUTPUT ${output_file}
        COMMAND ${CMAKE_COMMAND} -E echo "// Auto-generated file. Do not edit!" > ${output_file}
        COMMAND ${CMAKE_COMMAND} -E echo "#pragma once" >> ${output_file}
        COMMAND ${CMAKE_COMMAND} -E echo "" >> ${output_file}
        COMMAND ${CMAKE_COMMAND} -E echo "namespace embedded {" >> ${output_file}
        COMMAND ${CMAKE_COMMAND} -E echo "const char* ${variable_name} = R\"EMBEDDED(" >> ${output_file}
        COMMAND ${CMAKE_COMMAND} -E cat ${input_file} >> ${output_file}
        COMMAND ${CMAKE_COMMAND} -E echo ")EMBEDDED\";" >> ${output_file}
        COMMAND ${CMAKE_COMMAND} -E echo "} // namespace embedded" >> ${output_file}
        DEPENDS ${input_file}
        COMMENT "Embedding ${input_file} as ${variable_name}"
        VERBATIM
    )
endfunction()

message(STATUS "Checking for web assets...")
message(STATUS "WEB_ASSETS_DIR: ${WEB_ASSETS_DIR}")

# Check if files exist before embedding
if(EXISTS ${WEB_ASSETS_DIR}/terminal.html)
    message(STATUS "Found terminal.html")
else()
    message(WARNING "terminal.html NOT FOUND at ${WEB_ASSETS_DIR}/terminal.html")
endif()

if(EXISTS ${WEB_ASSETS_DIR}/interface.html)
    message(STATUS "Found interface.html")
else()
    message(WARNING "interface.html NOT FOUND at ${WEB_ASSETS_DIR}/interface.html")
endif()

embed_resource(
    ${WEB_ASSETS_DIR}/terminal.html
    ${CMAKE_CURRENT_BINARY_DIR}/embedded_terminal.h
    terminal_html
)

embed_resource(
    ${WEB_ASSETS_DIR}/interface.html
    ${CMAKE_CURRENT_BINARY_DIR}/embedded_interface.h
    interface_html
)

# Create list of all embedded headers that will be generated
set(EMBEDDED_HEADERS
    ${CMAKE_CURRENT_BINARY_DIR}/embedded_terminal.h
    ${CMAKE_CURRENT_BINARY_DIR}/embedded_interface.h
)

# Debug: Print what headers we expect to generate
message(STATUS "Will generate these embedded headers:")
foreach(header ${EMBEDDED_HEADERS})
    get_filename_component(header_name "${header}" NAME)
    message(STATUS "  ${header_name}")
endforeach()

# Create target for generating embedded headers
add_custom_target(generate_embedded_headers 
    DEPENDS ${EMBEDDED_HEADERS}
    COMMENT "Generating embedded resource headers"
)


include_directories( ${OpenCV_INCLUDE_DIRS} ${TCL_INCLUDE_DIR} ${FLIR_INCLUDE_DIR} )

add_executable( VideoStream VideoStream.cpp FrameBufferManager.cpp DservSocket.cpp
		SourceManager.cpp Widget.cpp WidgetManager.cpp KeyboardCallbackRegistry.cpp
		SamplingManager.cpp ReviewModeSource.cpp WebcamSource.cpp
		FlirCameraSource.cpp VideoFileSource.cpp StorageManager.cpp
		AnalysisPluginRegistry.cpp tclproc.cpp
		external/sqlite/sqlite3.c)


# Make VideoStream depend on header generation (AFTER add_executable)
add_dependencies(VideoStream generate_embedded_headers)

# Create list of embedded source files for VideoStream target
set(VSTREAM_EMBEDDED_SOURCES
    ${CMAKE_CURRENT_BINARY_DIR}/embedded_terminal.h 
    ${CMAKE_CURRENT_BINARY_DIR}/embedded_interface.h
)

target_sources(VideoStream PRIVATE ${VSTREAM_EMBEDDED_SOURCES})
target_include_directories(VideoStream PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Link with found libraries
target_link_libraries( VideoStream 
    ${OpenCV_LIBS} 
    ${TCL_LIBRARY}    # Use the found library instead of hardcoded name
    ${LIBPTHREAD} 
    ${FLIR_LIBRARY} 
    ${LIBDL} 
    ${LIBJANSSON} 
    ${DGLIB}
    usockets
    ${USOCKETS_LIBS}    
    ${LZ4_LIBRARY}    # Use the found lz4 library
    z 
)

# For the host executable on Linux
# Enable dynamic symbol export for plugins
if(APPLE)
    # macOS needs this to export symbols to dylibs
    set_target_properties(VideoStream PROPERTIES
        ENABLE_EXPORTS ON
    )
    target_link_options(VideoStream PRIVATE 
        -Wl,-export_dynamic
    )
elseif(UNIX)
    # Linux
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
