#!/usr/bin/env bash
#
# regenerate_session.sh - rebuild a session's trials.dgz with reprocessed
# (best-possible P4) eye data, in one command.
#
#   scripts/regenerate_session.sh <session_prefix> [mag angle]
#
# e.g.  scripts/regenerate_session.sh human_pursuit_coherence-sweep_260626141534
#
# Pipeline:
#   1. derive the P4 model from the original .db (or use the mag/angle you pass)
#   2. reprocess the .mp4 through the fixed plugin (deterministic, no crash)
#   3. export per-frame tracking to CSV
#   4. replace the eye columns in trials.dgz + re-derive saccade/pursuit events
#      (dlsh scripts/dpi_replace.tcl, reusing the em:: calibration from obs.dgz)
#   5. back up the original trials.dgz and install the regenerated one
#
# Data dir defaults to data/pilot_2026-06-23; override with DPI_DATA=/path.
#
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DATA="${DPI_DATA:-$ROOT/data/pilot_2026-06-23}"
BIN="$ROOT/build/VideoStream"
TCLS="$ROOT/tcl/reprocess_headless.tcl"

[ $# -ge 1 ] || { echo "usage: $0 <session_prefix> [mag angle]"; exit 1; }
SESS="$1"
MP4="$DATA/$SESS.mp4"
ODB="$DATA/$SESS.db"
OBS="$DATA/obs/$SESS.obs.dgz"
TR="$DATA/trials/$SESS.trials.dgz"
for f in "$MP4" "$OBS" "$TR"; do
  [ -f "$f" ] || { echo "ERROR: missing $f"; exit 1; }
done
WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT

# 1. P4 model
if [ $# -ge 3 ]; then
  MAG="$2"; ANG="$3"
elif [ -f "$ODB" ]; then
  read -r MAG ANG < <(python3 "$ROOT/scripts/dpi_derive_model.py" "$ODB")
else
  echo "ERROR: no original .db to derive the P4 model from; pass 'mag angle'"; exit 1
fi
echo "[1/5] P4 model: mag=$MAG angle=$ANG"

# 2. reprocess (run from $WORK so the .db lands there)
echo "[2/5] reprocessing $SESS (fixed plugin) ..."
( cd "$WORK" && "$BIN" -f "$TCLS" -- "$MP4" repro 8.0 "$MAG" "$ANG" online ) >"$WORK/repro.log" 2>&1
grep -q REPROCESS_DONE "$WORK/repro.log" || { echo "  reprocess did not finish:"; tail -3 "$WORK/repro.log"; exit 1; }
echo "      $(sqlite3 "$WORK/repro.db" 'SELECT COUNT(*) FROM eyetracking_frames') frames"

# 3. export CSV
sqlite3 -csv "$WORK/repro.db" \
  "SELECT frame_number,COALESCE(p1_x,-1),COALESCE(p1_y,-1),COALESCE(p4_x,-1),COALESCE(p4_y,-1),COALESCE(pupil_radius,-1),in_blink \
   FROM eyetracking_frames ORDER BY frame_number" > "$WORK/repro.csv"
echo "[3/5] exported $(wc -l < "$WORK/repro.csv" | tr -d ' ') frames to CSV"

# 4. replace eye columns + re-derive events
echo "[4/5] reprojecting to degrees + re-deriving events ..."
dlsh "$ROOT/scripts/dpi_replace.tcl" "$OBS" "$TR" "$WORK/repro.csv" "$WORK/new.trials.dgz" | sed 's/^/      /'

# 5. back up original (once) + install
mkdir -p "$DATA/trials/backup_orig"
[ -f "$DATA/trials/backup_orig/$SESS.trials.dgz" ] || cp "$TR" "$DATA/trials/backup_orig/$SESS.trials.dgz"
cp "$WORK/new.trials.dgz" "$TR"
echo "[5/5] installed $TR"
echo "      (original preserved in trials/backup_orig/)"
