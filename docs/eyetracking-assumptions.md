# EyeTrackingPlugin: built-in assumptions and where they live

Every constant below encodes an assumption about the *current* rig: IR source
placement, optics/binning, subject population, or frame rate. All defaults are
validated against the 2026-06 pilot sessions (FLIR, 2x2 binning, ~250 Hz,
IR source below the eye). When reconfiguring the rig — or debugging a setup
where tracking misbehaves — start here: find the symptom, check whether the
assumption still holds, and use the listed knob.

"Tcl" means adjustable at runtime; "recompile" means a constant in
`plugins/EyeTrackingPlugin.cpp`.

## Illumination & source placement

| Assumption | Where | Adjust | Symptom when violated |
|---|---|---|---|
| P1 sits BELOW the pupil center (candidates below get a 1.2x score bonus) | `vertical_bias` in `detectP1` | recompile | With the source elsewhere, a spurious upper reflection can outscore the real P1 when both are present |
| One IR source, one P4; P4 relates to P1 by a fixed magnitude ratio + angle around the pupil center | `P1P4RotationalModel` | per-session calibration handles placement; the *form* (single source) is structural | Multiple sources / extended sources break the prediction model entirely |
| Glint brightness floors match IR power + exposure + gain | `p1_min_intensity` (140), `p4_min_intensity` (140; rig scripts use 130/22) | Tcl `setP1MinIntensity`, `setP4MinIntensity` | P1/P4 `NO_CANDIDATE` on dimmer setups; iris texture wins on brighter ones |
| P1 glint area range | `p1_min_area`/`p1_max_area` (40–600 px²), hard clamps 20/800 in `detectP1` | Tcl `setP1MinArea`/`setP1MaxArea`; clamps recompile | Bigger/smaller source image → real P1 rejected by area |
| Saturated P1 handling (max ≥ 254 → use mean x 1.2) | `effective_intensity` in `detectP1` | recompile | Mis-ranking under different exposure regimes |

## Pupil & subject conditions

| Assumption | Where | Adjust | Symptom when violated |
|---|---|---|---|
| Pupil is the darkest region at threshold 45 | `pupil_threshold` | Tcl `setPupilThreshold` | Pupil merges with shadows/lashes/iris under different lighting |
| Pupil-like blob: area 200–150k px², extent ≥ 0.4, bbox aspect ≤ 3 | pupil plausibility gate in `detectPupil` | Tcl `setPupilGate min max extent aspect` | Legit pupil rejected (e.g. chronic partial lid droop → low extent) |
| P1 lies within 1.5x pupil radius of the pupil center | `p1_pupil_radius_max` | Tcl `setP1PupilRadiusMax` | Constricted pupil + eccentric gaze → real P1 outside the search disk (no absolute floor yet — parked) |
| P4 lies inside the pupil disk (0.95r, floored at predicted orbit + 6 px, HARD-CAPPED at 1.15r) | `p4_pupil_search_margin_`, `P4_DISK_PRED_SLACK`, `P4_DISK_MAX_FRAC` | Tcl `setP4PupilMargin`; slack/cap recompile | The cap is what stops a wrong model from hunting spectacle/iris glints outside the pupil (2026-07-13 rig runaway); a critical-log warning fires when the prediction orbit exceeds r |
| Model-ratio learning requires \|P1−pupil\| ≥ 3x max prediction error | `LEARN_MIN_CONDITIONING` in `detectPurkinje` | recompile | Near-axis gaze (small P1–pupil baseline) makes the learned ratio ill-conditioned; without this gate it staircases upward (mag 0.5→3.3 on 2026-07-13) |
| P4 is not within 0.3x pupil radius of P1 (exclusion hole) | `p1_exclusion_radius` in `detectP4` | recompile | Large pupil with P4 optically near P1 → real P4 carved out of the search (parked: size by P1's own extent) |
| "Well-dilated" for model learning = ≥ 85% of the *blink baseline* radius | `LEARN_MIN_DILATION` in `detectPurkinje` | recompile | Baseline tracks a sustained constriction down in ~1–2 s, eroding the guard; parked fix = anchor to calibration-time radius |
| Blink = radius < 0.6x baseline (exit at 0.85x) | `BLINK_ENTER_RATIO`/`BLINK_EXIT_RATIO` | recompile | Ptosis / unusual lid behavior → false or stuck blinks; saccade-induced radius dips still read as blinks (discrimination upgrade parked) |
| P4 brightness beats iris texture within the search box at penalty 1.5 px⁻¹ | `proximity_weight_` | recompile | Different iris pigmentation/texture contrast → wrong spot wins the proximity-weighted score |

## If you move the illuminator

Three things care about illuminator geometry, in increasing order of automatic
adaptation: (1) the P1 **vertical bias** (1.2x score bonus for candidates below
the pupil center) is hard-coded to a below-the-eye source — moving the source
above/side may require flipping or removing it (recompile); (2) the **P4 model
angle** re-derives per session calibration — no code change, just recalibrate;
(3) **dust visibility**: specks on the window light up according to the
illumination path, so a direction change alters which contaminants compete
with P4 (the 2026-07-13 dust map won't transfer). Re-run the median-stack
check (see below) after any illuminator change.

Dust check: median-stack ~300 frames of any session's mp4 — bright specks at
fixed image coordinates inside the pupil zone are on the optics, not the eye.
Measured 2026-07-13: 74 specks; 5.6% of that session's P4 picks landed on
mapped specks, and the model-runaway lock target was the speck band itself.
Parked countermeasure if cleaning isn't enough: tightness-weighted P4 scoring
(dust is defocused: weighted sigma 2.95px vs 0.81px for true P4 — task #11).

## Pixel scale & optics

All px-denominated constants (areas, jump limits, search sizes, sub-pixel
windows 19x19 / 5x5, P4 box 50x50) assume the current optics + **2x2 binning**.
Changing binning or magnification rescales every one of them (areas go with
the square). There is no global px-scale knob — retune the Tcl-settable values
and recalibrate; treat the recompile-only ones as suspect until re-derived.

## Frame rate

Time-based thresholds (loss/recovery/retry/blink/LOST/desperation windows,
long-blink self-heal) are specified in **milliseconds** and recomputed from the
source frame rate (`updateTimingThresholds`). Per-frame **jump limits**
(`p1_max_jump`, `p4_max_jump`) are user-specified in *px per frame at 250 Hz*
and scaled by `250/fps` internally, since inter-frame displacement grows as
frame rate drops. With no source available the fallback rate is 250 Hz.
Caveat: timing recomputes on init, `resetTrackingState`, and `calibrateP4Model`
— not automatically on source switch; call `resetTrackingState` after changing
sources with a different rate.

## Not assumptions, but load-bearing validated choices

- The pupil-disk constraint on the P4 search is *protective*, not cosmetic:
  without it an iris spot outscores the true P4 on ~1/3 of coherence-sweep
  frames (2026-07-10 A/B).
- Model learning is deliberately conservative (`LEARN_MAX_ERR_FRAC` 0.5 +
  dilation gate): loosening it caused runaway drift and P4-loss cascades
  (2026-06-26).
