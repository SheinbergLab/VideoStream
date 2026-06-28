#!/usr/bin/env python3
"""
Derive the P4 model (magnitude_ratio, angle_offset_deg) from a tracking .db's
good frames -- the geometric P1->P4 relationship the plugin uses to predict P4.

Usage:  dpi_derive_model.py <tracking.db>      # prints:  <mag> <angle_deg>
"""
import sqlite3, sys, math, statistics as st

if len(sys.argv) < 2:
    sys.exit("usage: dpi_derive_model.py <tracking.db>")

con = sqlite3.connect(sys.argv[1])
rows = con.execute(
    "SELECT pupil_x,pupil_y,p1_x,p1_y,p4_x,p4_y FROM eyetracking_frames "
    "WHERE pupil_x IS NOT NULL AND p1_x IS NOT NULL AND p4_x IS NOT NULL "
    "AND in_blink=0").fetchall()
con.close()

if len(rows) < 200:
    sys.exit(f"only {len(rows)} good frames; not enough to derive a P4 model")

mags, sin_, cos_ = [], [], []
for pux, puy, p1x, p1y, p4x, p4y in rows:
    v1 = (p1x - pux, p1y - puy)
    v4 = (p4x - pux, p4y - puy)
    d1 = math.hypot(*v1)
    if d1 < 1e-6:
        continue
    mags.append(math.hypot(*v4) / d1)
    a = math.atan2(v4[1], v4[0]) - math.atan2(v1[1], v1[0])
    sin_.append(math.sin(a)); cos_.append(math.cos(a))

mag = st.mean(mags)
ang = math.degrees(math.atan2(st.mean(sin_), st.mean(cos_)))
print(f"{mag:.4f} {ang:.2f}")
