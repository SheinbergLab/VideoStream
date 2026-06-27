#!/usr/bin/env python3
"""
Compare emcalib biquadratic calibration quality: ORIGINAL (live) tracking vs
REPROCESSED tracking. Re-fits the 9-term biquadratic (raw eye -> degrees) on
per-fixation means computed IDENTICALLY for both, so the only difference is the
P1/P4 source. Reports per-axis RMS (degrees).

raw eye signal = P1 - P4 (consistent definition; biquadratic absorbs signs).
Fit basis (matches biquadratic::evaluate):
  z = a0 + a1 x + a2 y + a3 x^2 + a4 y^2 + a5 xy + a6 x^2 y + a7 x y^2 + a8 x^2 y^2
"""
import sqlite3, sys, re, os
import numpy as np
import dgread

DATA = "/Users/sheinb/src/VideoStream/data/pilot_2026-06-23"
SESS = "human_emcalib_9point_260626135422"
REPRO_DB = sys.argv[1] if len(sys.argv) > 1 else \
    "/private/tmp/claude-501/-Users-sheinb-src-VideoStream/8502ce13-c716-4531-ad0d-7886a1ca84bd/scratchpad/em/emRepro.db"

def basis(x, y):
    x = np.asarray(x, float); y = np.asarray(y, float)
    return np.column_stack([np.ones_like(x), x, y, x*x, y*y, x*y, x*x*y, x*y*y, x*x*y*y])

def fit(eye_x, eye_y, cal):
    A = basis(eye_x, eye_y)
    coef, *_ = np.linalg.lstsq(A, np.asarray(cal, float), rcond=None)
    return coef

def predict(coef, eye_x, eye_y):
    return basis(eye_x, eye_y) @ coef

def rms(pred, target):
    return float(np.sqrt(np.mean((np.asarray(pred) - np.asarray(target))**2)))

# ---- load original obs (per-obs em streams) + trials ----
obs = dgread.dgread(f"{DATA}/obs/{SESS}.obs.dgz")
tr  = dgread.dgread(f"{DATA}/trials/{SESS}.trials.dgz")

obsid   = np.asarray(tr["obsid"]).astype(int)
calib_x = np.asarray(tr["calib_x"], float)
calib_y = np.asarray(tr["calib_y"], float)
refix   = np.asarray(tr["refixate"], float)   # ms from obs start
dur     = np.asarray(tr["duration"], float)   # ms
emx_live= np.asarray(tr["eye_mean_x"], float)
emy_live= np.asarray(tr["eye_mean_y"], float)

# ---- reprocessed per-frame P1/P4 ----
con = sqlite3.connect(REPRO_DB)
rep = {fn:(p1x,p1y,p4x,p4y) for fn,p1x,p1y,p4x,p4y in con.execute(
    "SELECT frame_number,p1_x,p1_y,p4_x,p4_y FROM eyetracking_frames")}
con.close()

def seg(key, i):
    return np.asarray(obs[key][i]).ravel()

def window_mean_raw(i, refix_ms, dur_ms, source):
    """Mean (p1-p4) over the post-refixation window for obs segment i.
    source='orig' uses obs em/p1,em/p4; 'repro' looks up the reprocess db."""
    fid  = seg("<ds>em/frame_id", i).astype(int)
    t    = seg("<ds>em/time", i)                  # seconds, session clock
    n = min(len(fid), len(t)); fid=fid[:n]; t=t[:n]
    t0 = t[0]
    lo = t0 + refix_ms/1000.0
    hi = t0 + (dur_ms - 50)/1000.0                # small end margin
    win = (t >= lo) & (t <= hi)
    hs, vs = [], []
    if source == "orig":
        p1 = seg("<ds>em/p1", i); p4 = seg("<ds>em/p4", i)
        p1x,p1y,p4x,p4y = p1[0:2*n:2],p1[1:2*n:2],p4[0:2*n:2],p4[1:2*n:2]
        for j in np.where(win)[0]:
            if p1x[j] >= 0 and p4x[j] >= 0:
                hs.append(p1x[j]-p4x[j]); vs.append(p1y[j]-p4y[j])
    else:
        for j in np.where(win)[0]:
            r = rep.get(int(fid[j]))
            if r and r[0] is not None and r[2] is not None:
                hs.append(r[0]-r[2]); vs.append(r[1]-r[3])
    if len(hs) < 3:
        return None
    return np.mean(hs), np.mean(vs)

# ---- per-trial fixation means ----
ox, oy, rx, ry, cx, cy = [], [], [], [], [], []
for k, i in enumerate(obsid):
    o = window_mean_raw(i, refix[k], dur[k], "orig")
    r = window_mean_raw(i, refix[k], dur[k], "repro")
    if o is None or r is None:
        continue
    ox.append(o[0]); oy.append(o[1]); rx.append(r[0]); ry.append(r[1])
    cx.append(calib_x[k]); cy.append(calib_y[k])
print(f"usable calibration points: {len(cx)} of {len(obsid)} trials")

def report(label, ex, ey, cx, cy):
    if len(cx) < 9:
        print(f"  {label}: only {len(cx)} pts, need >=9"); return
    bx = fit(ex, ey, cx); by = fit(ex, ey, cy)
    rx_ = rms(predict(bx, ex, ey), cx); ry_ = rms(predict(by, ex, ey), cy)
    comb = np.sqrt((rx_**2 + ry_**2)/2)
    print(f"  {label:28s} RMS_x={rx_:.3f}  RMS_y={ry_:.3f}  combined={comb:.3f} deg")

print("\n=== calibration fit RMS (degrees) — lower is better ===")
report("ORIGINAL (live tracking)", ox, oy, cx, cy)
report("REPROCESSED tracking",     rx, ry, cx, cy)

# ---- validation: do the STORED coeffs reproduce calib from live eye_mean? ----
m = re.search(r"bq_h_coeffs \{([^}]*)\}.*bq_v_coeffs \{([^}]*)\}",
              str(obs["<session>em/settings"][0]))
if m:
    chx = np.array(m.group(1).split(), float); chy = np.array(m.group(2).split(), float)
    valid = (emx_live > -900) & (emy_live > -900)
    px = basis(emx_live[valid], emy_live[valid]) @ chx
    py = basis(emx_live[valid], emy_live[valid]) @ chy
    print("\n=== validation: STORED coeffs applied to LIVE eye_mean ===")
    print(f"  RMS_x={rms(px, calib_x[valid]):.3f}  RMS_y={rms(py, calib_y[valid]):.3f} deg "
          f"(should be small if my biquadratic matches theirs)")
