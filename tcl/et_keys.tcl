#
# et_keys.tcl - shared eye-tracking overlay key bindings.
#
# These are the display-overlay toggles that every launcher (watch / review /
# run / tracker / eyetracker) wants identically. Centralized here so a new
# overlay toggle is a ONE-LINE edit instead of touching five scripts.
#
# State lives in the plugin (the no-arg commands toggle/cycle), so these are
# thin wrappers that just invoke the command and echo the new state.
#
# Usage from a launcher (after the plugin is loaded and AFTER any
# clear_key_bindings, since that wipes the registry):
#
#   source [file join [file dirname [info script]] et_keys.tcl]
#   ::et::bind_overlay_keys
#
# Script-specific keys (sliders, ROI nudge, recording, pause/step, ...) stay in
# the individual launcher - only the genuinely-shared overlay toggles live here.
#
namespace eval ::et {}

# Bind to procs (not inline script blocks): the host appends an event argument
# to key/mouse callbacks, so an inline "puts \"...\"" would receive that extra
# token and mis-parse it as a channel name. Procs with {args} swallow it.
proc ::et::toggle_insets {args} { puts "insets: [eyetracking::toggleInsets]" }
proc ::et::cycle_focus  {args} { puts "focus:  [eyetracking::focusMode]" }  ;# off->annotated->clean
proc ::et::center_roi   {args} {
    # Recenter the ROI on the current pupil (size unchanged, clamped to frame).
    # Safe mid-session: tracking state is full-frame, only the crop moves.
    if {[catch {eyetracking::centerROI} r]} { puts "center ROI: $r" } else { puts "ROI -> $r" }
}

proc ::et::bind_overlay_keys {} {
    bind_key "i" ::et::toggle_insets
    bind_key "f" ::et::cycle_focus
    bind_key "c" ::et::center_roi
    # New overlay toggles: add a proc above + a bind_key here, once.
}
