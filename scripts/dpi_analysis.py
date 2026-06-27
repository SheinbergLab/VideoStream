#!/usr/bin/env python3
"""
DPI eye-tracker pilot-data analysis.

Baseline scorecard + saccade-locked P4/P1 recovery analysis for the
reprocessed pilot sessions in data/pilot_2026-06-23/.

Four aligned views per session (joined by frame_id == db frame_number):
  .mp4         raw video (sync-pulse = in-obs)
  .db          plugin storeFrameData (live)        <- analyzed here
  .obs.dgz     plugin->dserv live stream (em/*)     <- independent detection record
  .trials.dgz  ESS trial table                      <- known saccade cue/landing times

Usage:
  python3 scripts/dpi_analysis.py                  # scorecard for all sessions
  python3 scripts/dpi_analysis.py --saccades       # + saccade-locked recovery
  python3 scripts/dpi_analysis.py <session_prefix> # one session, verbose
"""
import sqlite3, glob, os, sys, math
import numpy as np

DATA = os.path.join(os.path.dirname(__file__), "..", "data", "pilot_2026-06-23")
DATA = os.path.abspath(DATA)

try:
    import dgread
    HAVE_DG = True
except Exception:
    HAVE_DG = False


# ----------------------------------------------------------------------------
# loading
# ----------------------------------------------------------------------------
def load_db(db_path):
    """Return dict of per-frame arrays from eyetracking_frames joined to frames."""
    con = sqlite3.connect(db_path)
    rows = con.execute("""
        SELECT e.frame_number, e.obs_id, e.in_blink,
               e.pupil_x, e.pupil_y, e.pupil_radius,
               e.p1_x, e.p1_y, e.p4_x, e.p4_y
        FROM eyetracking_frames e
        ORDER BY e.frame_number
    """).fetchall()
    con.close()
    a = {k: [] for k in
         ("frame", "obs", "blink", "px", "py", "pr", "p1x", "p1y", "p4x", "p4y")}
    for r in rows:
        a["frame"].append(r[0]); a["obs"].append(r[1]); a["blink"].append(r[2])
        a["px"].append(r[3]); a["py"].append(r[4]); a["pr"].append(r[5])
        a["p1x"].append(r[6]); a["p1y"].append(r[7])
        a["p4x"].append(r[8]); a["p4y"].append(r[9])
    out = {}
    for k, v in a.items():
        out[k] = np.array([np.nan if x is None else x for x in v], dtype=float)
    out["in_obs"] = ~np.isnan(out["obs"])
    out["p1_ok"] = ~np.isnan(out["p1x"])
    out["p4_ok"] = ~np.isnan(out["p4x"])
    return out


def _cat(dgz, key):
    """Concatenate per-obs-segment arrays from a dgz datapoint key."""
    if key not in dgz or dgz[key] is None:
        return None
    out = []
    for seg in dgz[key]:
        if seg is None:
            continue
        out.append(np.asarray(seg).ravel())
    return np.concatenate(out) if out else np.array([])


def load_obs_stream(obs_path):
    """Return the live em/* detection stream keyed by frame_id (in-obs frames)."""
    if not HAVE_DG:
        return None
    o = dgread.dgread(obs_path)
    fid = _cat(o, "<ds>em/frame_id")
    if fid is None:
        return None
    return {
        "frame_id": fid.astype(int),
        "p4_det": _cat(o, "<ds>em/p4_detected"),
        "p1_det": _cat(o, "<ds>em/p1_detected"),
        "blink": _cat(o, "<ds>em/blink"),
    }


# ----------------------------------------------------------------------------
# loss-run statistics
# ----------------------------------------------------------------------------
def run_lengths(mask_lost):
    """Lengths of consecutive-True runs in a boolean array."""
    if mask_lost.size == 0:
        return np.array([], dtype=int)
    d = np.diff(np.concatenate(([0], mask_lost.astype(int), [0])))
    starts = np.where(d == 1)[0]
    ends = np.where(d == -1)[0]
    return ends - starts


def pct(x):
    return f"{100*x:5.1f}%"


# ----------------------------------------------------------------------------
# saccade detection (from pupil-center step) + recovery measurement
# ----------------------------------------------------------------------------
def detect_saccades(db, fps=250.0, vel_thresh_px=3.0, min_gap=15):
    """
    Detect saccades from frame-to-frame pupil-center displacement.
    Returns list of frame indices (into db arrays) where a saccade starts.
    vel_thresh_px: per-frame pupil step (px) that counts as saccadic.
    min_gap: min frames between distinct saccades.
    """
    px, py = db["px"], db["py"]
    dx = np.diff(px); dy = np.diff(py)
    step = np.sqrt(dx*dx + dy*dy)
    step[np.isnan(step)] = 0.0
    cand = np.where(step > vel_thresh_px)[0]
    sacc = []
    last = -10**9
    for c in cand:
        if c - last >= min_gap:
            sacc.append(c)
            last = c
    return sacc


def recovery_after(idx, ok_mask, blink_mask, horizon=200):
    """
    Starting at frame idx, characterize a (possible) loss + recovery of `ok_mask`.
    Returns (lost_frames, recovered_within_horizon, coincident_blink).
    """
    n = ok_mask.size
    end = min(n, idx + horizon)
    seg = ok_mask[idx:end]
    blk = blink_mask[idx:end]
    # find first loss at/after idx
    lost_idx = np.where(~seg)[0]
    if lost_idx.size == 0:
        return 0, True, False
    start = lost_idx[0]
    # length of the loss run starting there
    run = 0
    for j in range(start, seg.size):
        if not seg[j]:
            run += 1
        else:
            break
    recovered = (start + run) < seg.size
    coincident_blink = bool(np.any(blk[start:start+max(run, 1)] > 0))
    return run, recovered, coincident_blink


# ----------------------------------------------------------------------------
# reports
# ----------------------------------------------------------------------------
def sessions():
    dbs = sorted(glob.glob(os.path.join(DATA, "*.db")))
    return [(os.path.basename(d)[:-3], d) for d in dbs]


def scorecard():
    print(f"{'session':<42} {'frames':>7} {'inobs':>6} {'blink':>6} "
          f"{'P1null':>7} {'P4null':>7} {'P4null_obs':>10} {'P4runs':>7} {'longest':>8} {'dbVSobs':>8}")
    print("-" * 120)
    for prefix, db_path in sessions():
        db = load_db(db_path)
        io = db["in_obs"]
        n = db["frame"].size
        n_obs = int(io.sum())
        blink = db["blink"].mean()
        p1null = (~db["p1_ok"]).mean()
        p4null = (~db["p4_ok"]).mean()
        p4null_obs = (~db["p4_ok"][io]).mean() if n_obs else float("nan")
        p4_lost_runs = run_lengths(~db["p4_ok"][io]) if n_obs else np.array([])
        nruns = p4_lost_runs.size
        longest = int(p4_lost_runs.max()) if nruns else 0

        # db vs obs.dgz agreement on P4 detection (in-obs)
        agree = "n/a"
        obs_path = os.path.join(DATA, "obs", prefix + ".obs.dgz")
        if HAVE_DG and os.path.exists(obs_path):
            st = load_obs_stream(obs_path)
            if st is not None and st["p4_det"] is not None:
                # map db frame->p4_ok, compare on shared frame_ids
                dbmap = {int(f): bool(ok) for f, ok in zip(db["frame"], db["p4_ok"])}
                shared = [(int(f), d) for f, d in zip(st["frame_id"], st["p4_det"])
                          if int(f) in dbmap]
                if shared:
                    same = sum(1 for f, d in shared if dbmap[f] == bool(d))
                    agree = f"{100*same/len(shared):.1f}%"
        print(f"{prefix:<42} {n:>7} {pct(n_obs/n):>6} {pct(blink):>6} "
              f"{pct(p1null):>7} {pct(p4null):>7} {pct(p4null_obs):>10} "
              f"{nruns:>7} {longest:>8} {agree:>8}")


def saccade_report():
    print("\n=== Saccade-locked P4 recovery (in-obs frames; saccade = pupil step > 3px) ===")
    print(f"{'session':<42} {'#sacc':>6} {'P4drop%':>8} {'med_lost':>9} "
          f"{'p90_lost':>9} {'unrec%':>7} {'blink%':>7}")
    print("-" * 100)
    for prefix, db_path in sessions():
        db = load_db(db_path)
        io = db["in_obs"]
        sacc = detect_saccades(db)
        sacc = [s for s in sacc if io[s]]
        if not sacc:
            print(f"{prefix:<42} {0:>6}")
            continue
        losts, unrec, blinks, dropped = [], 0, 0, 0
        for s in sacc:
            run, recovered, blk = recovery_after(s, db["p4_ok"], db["blink"])
            if run > 0:
                dropped += 1
                losts.append(run)
                if not recovered:
                    unrec += 1
                if blk:
                    blinks += 1
        losts = np.array(losts) if losts else np.array([0])
        print(f"{prefix:<42} {len(sacc):>6} {pct(dropped/len(sacc)):>8} "
              f"{np.median(losts):>9.0f} {np.percentile(losts,90):>9.0f} "
              f"{pct(unrec/max(dropped,1)):>7} {pct(blinks/max(dropped,1)):>7}")


def detail(prefix):
    db_path = os.path.join(DATA, prefix + ".db")
    if not os.path.exists(db_path):
        print("no such session:", prefix); return
    db = load_db(db_path)
    io = db["in_obs"]
    print(f"=== {prefix} ===")
    print(f"frames={db['frame'].size}  in-obs={int(io.sum())}")
    for tag, ok in (("P1", db["p1_ok"]), ("P4", db["p4_ok"])):
        runs = run_lengths(~ok[io])
        if runs.size:
            print(f"{tag}: null={pct((~ok[io]).mean())}  runs={runs.size}  "
                  f"median={np.median(runs):.0f}  p90={np.percentile(runs,90):.0f}  "
                  f"max={runs.max()}  total_lost={runs.sum()}")
            # histogram of run lengths
            bins = [1, 2, 3, 5, 10, 25, 50, 100, 10**9]
            h, _ = np.histogram(runs, bins=bins)
            labels = ["1", "2", "3-4", "5-9", "10-24", "25-49", "50-99", "100+"]
            print("   run-length hist: " +
                  "  ".join(f"{l}:{c}" for l, c in zip(labels, h)))
        else:
            print(f"{tag}: null={pct((~ok[io]).mean())}  (no losses in obs)")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:]]
    if args and not args[0].startswith("--"):
        detail(args[0])
    else:
        scorecard()
        if "--saccades" in args:
            saccade_report()
