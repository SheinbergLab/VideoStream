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

proc ::et::bind_overlay_keys {} {
    bind_key "i" { puts "insets: [eyetracking::toggleInsets]" }
    bind_key "f" { puts "focus:  [eyetracking::focusMode]" }  ;# off->annotated->clean
    # New overlay toggles go HERE, once.
}
