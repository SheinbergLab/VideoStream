cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE Release)

# Version is injected from the release tag in CI (-D PROJECT_VERSION=<tag>); 0.0.0 locally.
# project(VERSION) demands a strict numeric X.Y.Z, but tags may carry a leading 'v'
# and/or a suffix (e.g. v0.0.1-test), so extract the numeric core for project()/CPack.
if(NOT DEFINED PROJECT_VERSION)
    set(PROJECT_VERSION 0.0.0)
endif()
string(REGEX MATCH "[0-9]+(\\.[0-9]+)+" _vs_ver "${PROJECT_VERSION}")
if(NOT _vs_ver)
    set(_vs_ver 0.0.0)
endif()
set(PROJECT_VERSION "${_vs_ver}")

project( VideoStream VERSION ${PROJECT_VERSION} )

# FLIR Spinnaker is a proprietary SDK and is NOT redistributable, so it is OFF by
# default: the standard build (and all CI release artifacts) supports webcam/UVC,
# mp4 playback, review and headless reprocess. Enable -DWITH_FLIR=ON on a machine
# that has the Spinnaker SDK installed to build the camera-acquisition path.
option(WITH_FLIR "Build with FLIR Spinnaker camera support (requires Spinnaker SDK; not redistributable)" OFF)

# macOS only: build a self-contained VideoStream.app / .dmg (dylibbundler + DragNDrop).
# OFF by default so local dev keeps the plain ./build/VideoStream binary; CI/packaging
# turns it ON.
option(MACOS_APP_BUNDLE "Build a self-contained macOS .app bundle (for .dmg packaging)" OFF)

if(WIN32)
set (OpenCV_DIR c:/usr/local/lib/opencv/build/x64/vc15/lib)
endif()

if(APPLE)
    # Homebrew's `opencv` formula moved to OpenCV 5.0 (2026-07), which
    # reorganized imgproc so the shape-analysis API this code uses (moments,
    # contourArea, fitEllipse, ...) no longer resolves via the 4.x headers.
    # We build against the OpenCV 4 API: prefer an opencv@4 keg when present
    # (keg-only, so it needs an explicit prefix). CMAKE_PREFIX_PATH is searched
    # ahead of the system prefixes, so 4.x wins even with 5.x also installed.
    list(APPEND CMAKE_PREFIX_PATH /opt/homebrew/opt/opencv@4 /usr/local/opt/opencv@4)
endif()

# Only the OpenCV modules actually used (drops dnn/calib3d/features2d/photo/ml/...)
find_package( OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui )
message(STATUS "OpenCV ${OpenCV_VERSION} (${OpenCV_INCLUDE_DIRS})")
if(OpenCV_VERSION VERSION_GREATER_EQUAL 5)
    message(WARNING "OpenCV ${OpenCV_VERSION} found, but this codebase targets "
                    "the OpenCV 4 API (imgproc shape functions moved in 5.x). "
                    "Install OpenCV 4 (macOS: brew install opencv@4).")
endif()


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
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	set(TCLLIB tcl90t_64)  # Updated to Tcl 9
	include_directories("c:/usr/local/include")
	link_directories("c:/usr/local/lib64" "c:/usr/local/lib")
	set(DGLIB dg)
	if(WITH_FLIR)
		include_directories("C:/Program Files/Point Grey Research/Spinnaker/include")
		link_directories("C:/Program Files/Point Grey Research/Spinnaker/lib64/vs2015")
		set(FLIRLIB Spinnaker_v140)
	endif()
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
	set(TCLLIB tcl90t)  # Updated to Tcl 9
	set(LABLIB lablib)
endif()
elseif(APPLE)
	# Prefer Homebrew's tcl-tk@9: it ships a bundle-friendly libtcl (no appended
	# zipfs, so install_name_tool/dylibbundler/codesign work) plus an on-disk
	# stdlib (lib/tcl9.0) we can bundle into the .app — matching stim2's macOS
	# build. (The dlsh /usr/local Tcl embeds the stdlib as a zipfs appended to
	# the dylib, which the macOS relocation tools can't process.)
	# brew puts Tcl 9 headers in include/tcl-tk (NOT include/) and needs
	# -DTCL_WITH_EXTERNAL_TOMMATH, so resolve compile flags via pkg-config rather
	# than guessing — otherwise the compiler silently falls back to the macOS SDK's
	# Tcl 8.6 headers (Tcl_Size unknown, Tcl_LinkVar signature mismatch).
	find_package(PkgConfig REQUIRED)
	execute_process(COMMAND brew --prefix tcl-tk@9
	    OUTPUT_VARIABLE BREW_TCL OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
	if(BREW_TCL AND EXISTS "${BREW_TCL}/lib/pkgconfig/tcl.pc")
	    set(ENV{PKG_CONFIG_PATH} "${BREW_TCL}/lib/pkgconfig")
	    pkg_check_modules(TCL REQUIRED tcl)
	    set(TCL_INCLUDE_DIR "${TCL_INCLUDE_DIRS}")
	    set(TCL_LIBRARY "${BREW_TCL}/lib/libtcl9.0.dylib")
	    include_directories(${TCL_INCLUDE_DIRS})
	    link_directories(${TCL_LIBRARY_DIRS})
	    add_compile_options(${TCL_CFLAGS_OTHER})   # -DTCL_WITH_EXTERNAL_TOMMATH
	    message(STATUS "Using Homebrew tcl-tk@9 (pkg-config): ${TCL_INCLUDE_DIRS}")
	else()
	    find_library(TCL_LIBRARY
		NAMES tcl9.0 tcl90 tcl9 tcl8.6
		HINTS /usr/local/lib /opt/homebrew/lib)
	    message(STATUS "Homebrew tcl-tk@9 not found; using ${TCL_LIBRARY}")
	endif()
	if(TCL_LIBRARY)
	    get_filename_component(TCLLIB ${TCL_LIBRARY} NAME_WE)
	    # Dir holding libtcl + the tcl9.0/ stdlib (resolve symlinks for bundling).
	    get_filename_component(_tcl_real "${TCL_LIBRARY}" REALPATH)
	    get_filename_component(LIBTCL_DIR "${_tcl_real}" DIRECTORY)
	else()
	    set(TCLLIB tcl8.6)  # Fallback
	endif()
	find_library(LIBJANSSON NAMES "libjansson.a")
	set(DGLIB dg)
	set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}")
	include_directories(/usr/local/include /opt/homebrew/include)
	link_directories(/usr/local/lib /opt/homebrew/lib)

    # macOS uses uSockets' native kqueue backend (its default on Darwin), so no
    # libuv dependency — leaner deps and one fewer dylib in the .app bundle
    # (cf. stim2's no-libuv guard). WITH_LIBUV stays unset here.

else()  # Linux
	find_library(LIBDL dl)
	find_library(LIBPTHREAD pthread)

	if(WITH_FLIR)
	    find_library(FLIR_LIBRARY
	      NAMES Spinnaker
	      PATHS /opt/spinnaker/lib
	      REQUIRED
	    )
	    set(FLIR_INCLUDE_DIR /opt/spinnaker/include)
	endif()

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
	set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-conversion-null")
endif()

# Unified FLIR define across platforms (source is guarded by #ifdef USE_FLIR).
if(WITH_FLIR)
    add_definitions(-DUSE_FLIR=1)
    message(STATUS "FLIR Spinnaker camera support: ENABLED")
else()
    message(STATUS "FLIR Spinnaker camera support: disabled (default). Use -DWITH_FLIR=ON to enable.")
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
    # macOS uses the native kqueue backend (no libuv)
    target_compile_definitions(usockets PRIVATE LIBUS_USE_KQUEUE)
else()
    # Linux uses epoll
    target_compile_definitions(usockets PRIVATE LIBUS_USE_EPOLL)
endif()

# Add compile flags that uSockets expects
if(NOT WIN32)
    target_compile_options(usockets PRIVATE -std=c11 -flto)
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

# add options to sqlite compilation
set_source_files_properties(external/sqlite/sqlite3.c 
    PROPERTIES COMPILE_DEFINITIONS "SQLITE_ENABLE_COLUMN_METADATA=1"
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

######
##### Install / packaging
######
#
# Layout matches the SheinbergLab convention (cf. dserv): everything under
# <prefix>/videostream/, with the plugin in plugins/ next to the binary so the
# runtime loader (`[file dir [info nameofexecutable]]/plugins/...`) finds it.
#
# A.2a covers the generic/Linux path (DEB). The macOS .app bundle + dylibbundler
# + signing is A.2b (pairs with release_macos.yml).
#
# ---- Common package metadata ----
set(CPACK_PACKAGE_NAME videostream)
set(CPACK_PACKAGE_VENDOR SheinbergLab)
set(CPACK_PACKAGE_CONTACT SheinbergLab)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Camera/video streaming + Purkinje eye-tracking host")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_COMPONENTS_ALL videostream)

if(APPLE AND MACOS_APP_BUNDLE)
    # ============ Self-contained macOS .app -> .dmg (A.2b) ============
    set_target_properties(VideoStream PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_BUNDLE_NAME VideoStream
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
        MACOSX_BUNDLE_GUI_IDENTIFIER org.sheinberglab.videostream
        MACOSX_BUNDLE_ICON_FILE VideoStream
        # Custom Info.plist carries NSCameraUsageDescription + CFBundleIconFile.
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_SOURCE_DIR}/macos/Info.plist.in)
    set(CMAKE_SKIP_RPATH TRUE)

    # Ship the app icon in Contents/Resources/VideoStream.icns.
    target_sources(VideoStream PRIVATE ${CMAKE_SOURCE_DIR}/macos/VideoStream.icns)
    set_source_files_properties(${CMAKE_SOURCE_DIR}/macos/VideoStream.icns
        PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")

    # Built-bundle path differs by generator (Xcode is multi-config -> Release/).
    if(CMAKE_GENERATOR MATCHES "Xcode")
        set(APP_BUNDLE ${CMAKE_BINARY_DIR}/Release/VideoStream.app)
    else()
        set(APP_BUNDLE ${CMAKE_BINARY_DIR}/VideoStream.app)
    endif()

    # Assemble everything INTO the built .app via absolute-path installs, then
    # copy the finished bundle into the package staging at the end. The plugin
    # sits next to the binary (Contents/MacOS/plugins) so the runtime loader
    # `[file dir [info nameofexecutable]]/plugins/...` resolves inside the bundle.
    install(TARGETS eyetracking
            LIBRARY DESTINATION ${APP_BUNDLE}/Contents/MacOS/plugins COMPONENT videostream)
    install(DIRECTORY tcl DESTINATION ${APP_BUNDLE}/Contents/Resources COMPONENT videostream
            REGEX "~$"                                 EXCLUDE
            REGEX "\\.(NEW|thresholds|BAK|bak|orig|swp)$" EXCLUDE
            REGEX "\\.DS_Store$"                       EXCLUDE
            REGEX "history\\.txt$"                     EXCLUDE)

    # Bundle the Tcl stdlib so the relocated libtcl finds it at ../lib/tcl9.0
    # (relative to Contents/Frameworks). tcl9.0/ = packages, tcl9/ = Tcl modules.
    # Trailing slash on the source + full destination path => install the
    # CONTENTS (dereferencing brew's keg symlinks). Without it, a symlinked
    # lib/tcl9.0 on the CI runner copies as an empty dir entry. (Matches stim2.)
    if(LIBTCL_DIR AND EXISTS "${LIBTCL_DIR}/tcl9.0")
        install(DIRECTORY ${LIBTCL_DIR}/tcl9.0/
                DESTINATION ${APP_BUNDLE}/Contents/lib/tcl9.0 COMPONENT videostream)
        install(DIRECTORY ${LIBTCL_DIR}/tcl9/
                DESTINATION ${APP_BUNDLE}/Contents/lib/tcl9 COMPONENT videostream OPTIONAL)
    else()
        message(FATAL_ERROR "Tcl stdlib (tcl9.0/) not found under ${LIBTCL_DIR}; cannot build a self-contained .app")
    endif()

    # Fail the build loudly if the stdlib didn't actually land in the bundle
    # (e.g. an empty/symlinked copy) rather than shipping a .app that can't
    # find init.tcl at runtime.
    install(CODE "
        if(NOT EXISTS \"${APP_BUNDLE}/Contents/lib/tcl9.0/init.tcl\")
            message(FATAL_ERROR \"Bundled Tcl stdlib is missing Contents/lib/tcl9.0/init.tcl\")
        endif()
    " COMPONENT videostream)

    # Identity for the post-bundling re-sign. Empty -> ad-hoc ("-"), which is
    # enough to RUN locally (arm64 SIGKILLs binaries whose signature was
    # invalidated by editing load commands). CI sets a Developer ID for A.2b-2.
    set(MACOS_CODESIGN_IDENTITY "" CACHE STRING "codesign identity for the .app (empty = ad-hoc)")

    # Relocate non-system dylibs (OpenCV + its media tree + libtcl9.0; jansson/dg
    # are static, no libuv) into Contents/Frameworks with install names -> @rpath,
    # then repair rpaths and re-sign. dylibbundler's --install-path @rpath leaves
    # a bogus, DUPLICATED '@rpath/' LC_RPATH that dyld rejects, so we replace it:
    # bundled dylibs resolve siblings via @loader_path; the executable + plugin
    # reach Frameworks via @executable_path/../Frameworks.
    set(_DYLIBBUNDLER "dylibbundler --bundle-deps --overwrite-files --create-dir --search-path /opt/homebrew/lib --search-path ${LIBTCL_DIR} --dest-dir ${APP_BUNDLE}/Contents/Frameworks --install-path @rpath")
    install(CODE "
        set(_int \"${CMAKE_INSTALL_NAME_TOOL}\")
        set(_bin \"${APP_BUNDLE}/Contents/MacOS/VideoStream\")
        set(_plug \"${APP_BUNDLE}/Contents/MacOS/plugins/eyetracking.dylib\")
        set(_id \"${MACOS_CODESIGN_IDENTITY}\")
        if(NOT _id)
            set(_id \"-\")
        endif()

        execute_process(COMMAND_ECHO STDOUT
            COMMAND ${_DYLIBBUNDLER} -x \"\${_bin}\" -x \"\${_plug}\"
            RESULT_VARIABLE _rc)
        if(NOT _rc EQUAL 0)
            message(FATAL_ERROR \"dylibbundler failed (rc=\${_rc})\")
        endif()

        file(GLOB _fw \"${APP_BUNDLE}/Contents/Frameworks/*.dylib\")
        # bundled dylibs find their siblings via @loader_path
        foreach(_l \${_fw})
            execute_process(COMMAND \${_int} -add_rpath @loader_path \"\${_l}\" ERROR_QUIET)
        endforeach()
        # strip every bogus/duplicate literal '@rpath/' rpath (dyld rejects dups)
        foreach(_f \${_fw} \"\${_bin}\" \"\${_plug}\")
            set(_more TRUE)
            while(_more)
                execute_process(COMMAND \${_int} -delete_rpath @rpath/ \"\${_f}\"
                    RESULT_VARIABLE _d OUTPUT_QUIET ERROR_QUIET)
                if(NOT _d EQUAL 0)
                    set(_more FALSE)
                endif()
            endwhile()
        endforeach()
        # executable + plugin reach Frameworks (plugin's @executable_path is the host)
        execute_process(COMMAND \${_int} -add_rpath @executable_path/../Frameworks \"\${_bin}\" ERROR_QUIET)
        execute_process(COMMAND \${_int} -add_rpath @executable_path/../Frameworks \"\${_plug}\" ERROR_QUIET)

        # re-sign (editing load commands invalidated signatures; arm64 SIGKILLs
        # edited binaries). A real Developer ID gets hardened runtime + the
        # entitlements + a secure timestamp (all required for notarization);
        # otherwise ad-hoc, which is enough to run locally. Sign inner-out:
        # dylibs -> plugin -> binary -> bundle.
        set(_ent \"${CMAKE_SOURCE_DIR}/macos/VideoStream.entitlements\")
        if(_id STREQUAL \"-\")
            set(_opts \"\")
        else()
            set(_opts --options runtime --timestamp --entitlements \"\${_ent}\")
        endif()
        foreach(_l \${_fw})
            execute_process(COMMAND codesign --force \${_opts} -s \"\${_id}\" \"\${_l}\")
        endforeach()
        execute_process(COMMAND codesign --force \${_opts} -s \"\${_id}\" \"\${_plug}\")
        execute_process(COMMAND codesign --force \${_opts} -s \"\${_id}\" \"\${_bin}\")
        execute_process(COMMAND codesign --force \${_opts} -s \"\${_id}\" \"${APP_BUNDLE}\")
    " COMPONENT videostream)

    # Stage the finished bundle into /Applications (pkg installs there).
    install(TARGETS VideoStream BUNDLE DESTINATION Applications COMPONENT videostream)

    # Ship a signed .pkg (like dlsh-interp) rather than a .dmg: its postinstall
    # drops a /usr/local/bin/videostream CLI launcher so args work from the
    # terminal. cpack produces an UNSIGNED .pkg; the workflow then runs
    # `productsign` (Developer ID Installer) + notarize + staple.
    set(CPACK_GENERATOR "productbuild")
    set(CPACK_PACKAGING_INSTALL_PREFIX "/")
    set(CPACK_PRODUCTBUILD_COMPONENT_INSTALL ON)
    set(CPACK_POSTFLIGHT_VIDEOSTREAM_SCRIPT ${CMAKE_SOURCE_DIR}/macos/postinstall.sh)
    set(CPACK_PACKAGE_FILE_NAME "videostream-${PROJECT_VERSION}-Darwin")

elseif(NOT APPLE)
    # ============ Linux: /usr/local/videostream + DEB (A.2a) ============
    install(TARGETS VideoStream  DESTINATION videostream                 COMPONENT videostream)
    install(TARGETS eyetracking  LIBRARY DESTINATION videostream/plugins COMPONENT videostream)
    install(DIRECTORY tcl DESTINATION videostream COMPONENT videostream
            REGEX "~$"                                 EXCLUDE
            REGEX "\\.(NEW|thresholds|BAK|bak|orig|swp)$" EXCLUDE
            REGEX "\\.DS_Store$"                       EXCLUDE
            REGEX "history\\.txt$"                     EXCLUDE)
    install(DIRECTORY www DESTINATION videostream COMPONENT videostream
            REGEX "\\.DS_Store$" EXCLUDE)

    set(CPACK_GENERATOR "DEB")
    set(CPACK_PACKAGING_INSTALL_PREFIX "/usr/local")  # -> /usr/local/videostream

    # Hand-list runtime deps (NOT dpkg-shlibdeps): Tcl 9 is built from source into
    # /usr/local, which has no dpkg metadata and would make shlibdeps error.
    # jansson + dg are static (no dep). libtcl9.0 ships in apt only from Debian
    # Trixie / newer Ubuntu onward; elsewhere the admin provides Tcl 9.
    set(_vs_deb_deps "libc6, libstdc++6, zlib1g, liblz4-1, libopencv-dev, dlsh-dg")
    if(NOT DEFINED DISTRO_SUFFIX)
        set(DISTRO_SUFFIX "")
    endif()
    set(_vs_tcl9_distros "trixie" "noble" "")
    list(FIND _vs_tcl9_distros "${DISTRO_SUFFIX}" _vs_tcl9_idx)
    if(_vs_tcl9_idx GREATER_EQUAL 0)
        set(_vs_deb_deps "${_vs_deb_deps}, libtcl9.0")
    endif()
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "${_vs_deb_deps}")

    execute_process(COMMAND dpkg --print-architecture
        OUTPUT_VARIABLE PACKAGE_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(NOT PACKAGE_ARCH)
        set(PACKAGE_ARCH amd64)
    endif()
    if(DISTRO_SUFFIX)
        set(CPACK_DEBIAN_FILE_NAME "videostream_${PROJECT_VERSION}_${PACKAGE_ARCH}_${DISTRO_SUFFIX}.deb")
    else()
        set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
    endif()
endif()

include(CPack)
