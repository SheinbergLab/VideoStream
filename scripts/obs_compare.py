#!/usr/bin/env python3
"""Compare obs boundaries in a VideoStream recording .db with the camera's TTL bit.

VideoStream records, per frame, both the obs assignment it used (obs_id, from
vstream::obsSource: line / timestamp / dserv) and the raw TTL input bit
(line_status). When the source is `timestamp` (dserv ess/in_obs matched to
frames on the PTP clock) this reports, per observation, how many frames the
used boundaries differ from the wire's transitions -- the check to run before
unplugging the sync line.

usage: obs_compare.py recording.db [--verbose]
"""
import sqlite3, sys

def main(path, verbose=False):
    db = sqlite3.connect(path)
    cols = [r[1] for r in db.execute("PRAGMA table_info(frames)")]
    has_cam = "camera_time_us" in cols
    meta = db.execute("SELECT * FROM recording_metadata").fetchone()
    mcols = [r[1] for r in db.execute("PRAGMA table_info(recording_metadata)")]
    src = dict(zip(mcols, meta)).get("obs_source", "?") if meta else "?"
    print(f"{path}: obs source '{src}', camera_time column {'present' if has_cam else 'absent'}")

    frames = db.execute("SELECT frame_number, obs_id, line_status" + (", camera_time_us" if has_cam else "")
                        + " FROM frames ORDER BY frame_number").fetchall()
    if not frames:
        print("no frames"); return 1
    # wire transitions
    wire = []  # (start_frame, stop_frame) from line_status
    prev = 0; start = None
    for row in frames:
        fn, obs, ls = row[0], row[1], row[2]
        if ls and not prev: start = fn
        if prev and not ls and start is not None: wire.append((start, fn - 1)); start = None
        prev = ls
    if start is not None: wire.append((start, frames[-1][0]))
    obs = db.execute("SELECT obs_id, start_frame, stop_frame FROM observations ORDER BY obs_id").fetchall()
    print(f"{len(obs)} observations recorded, {len(wire)} TTL periods on the wire")
    if not wire:
        print("TTL never went high: nothing to compare (is the line wired and camera::ttlLine set?)"); return 0

    diffs = []
    for oid, s, e in obs:
        # nearest wire period by start frame
        w = min(wire, key=lambda p: abs(p[0] - s))
        ds, de = s - w[0], (e - w[1]) if (e is not None and w[1] is not None) else None
        diffs.append((oid, s, e, w[0], w[1], ds, de))
        if verbose or abs(ds) > 1 or (de is not None and abs(de) > 1):
            print(f"  obs {oid}: used {s}..{e}  wire {w[0]}..{w[1]}  start {ds:+d} frames"
                  + (f"  stop {de:+d} frames" if de is not None else ""))
    if diffs:
        starts = [d[5] for d in diffs]; stops = [d[6] for d in diffs if d[6] is not None]
        def summ(v): return f"min {min(v):+d} max {max(v):+d} mean {sum(v)/len(v):+.2f}" if v else "n/a"
        print(f"start offset (used - wire), frames: {summ(starts)}")
        print(f"stop  offset (used - wire), frames: {summ(stops)}")
        if has_cam:
            fr = db.execute("SELECT frame_rate FROM recording_metadata").fetchone()
            if fr and fr[0]:
                print(f"(1 frame = {1e6 / fr[0]:.0f} us at {fr[0]:.1f} fps)")
    return 0

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    sys.exit(main(sys.argv[1], "--verbose" in sys.argv))
