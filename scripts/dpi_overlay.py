#!/usr/bin/env python3
"""
Overlay tracker output (pupil/P1/P4 from the .db) onto the actual video frames,
with a zoomed inset, so true-vs-false P4 loss can be judged by eye.

Requires: ffmpeg on PATH, PIL, numpy. (No cv2 needed.)

Usage:
  python3 scripts/dpi_overlay.py <session_prefix> <frame> [frame ...]
  python3 scripts/dpi_overlay.py <session_prefix> --fail N      # N evenly-spaced TRUE-p4-fail frames
  python3 scripts/dpi_overlay.py <session_prefix> --context F W  # frames F-W .. F+W (a loss event)

Writes PNGs to scratchpad and prints their paths.
"""
import sqlite3, glob, os, sys, subprocess, tempfile
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DATA = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "data", "pilot_2026-06-23"))
# Annotated frames land here by default (visible in the repo). Override with DPI_OUT.
OUT = os.environ.get("DPI_OUT") or os.path.join(DATA, "review")
os.makedirs(OUT, exist_ok=True)

def db_rows(db_path, frames):
    con = sqlite3.connect(db_path)
    q = ("SELECT frame_number,obs_id,in_blink,pupil_x,pupil_y,pupil_radius,"
         "p1_x,p1_y,p4_x,p4_y FROM eyetracking_frames WHERE frame_number IN (%s)"
         % ",".join("?"*len(frames)))
    out = {}
    for r in con.execute(q, frames):
        out[r[0]] = dict(zip(
            ("frame","obs","blink","px","py","pr","p1x","p1y","p4x","p4y"), r))
    con.close()
    return out

def extract_frames(mp4, frames):
    """Frame-accurate extract via ffmpeg select. Returns {frame: PIL.Image}."""
    sel = "+".join(f"eq(n\\,{f})" for f in frames)
    tmp = tempfile.mkdtemp(dir=OUT)
    pat = os.path.join(tmp, "f%04d.png")
    subprocess.run(["ffmpeg","-hide_banner","-loglevel","error","-i",mp4,
                    "-vf",f"select='{sel}'","-vsync","0",pat], check=True)
    pngs = sorted(glob.glob(os.path.join(tmp,"*.png")))
    return {f: Image.open(p).convert("RGB") for f, p in zip(sorted(frames), pngs)}

def draw_markers(img, row, zoom_box=70, scale=4):
    """Return a side-by-side: full frame (annotated) + zoom around pupil."""
    d = ImageDraw.Draw(img)
    px, py, pr = row.get("px"), row.get("py"), row.get("pr")
    # pupil circle (cyan)
    if px is not None:
        r = pr or 25
        d.ellipse([px-r,py-r,px+r,py+r], outline=(0,255,255), width=2)
        d.line([px-4,py,px+4,py], fill=(0,255,255)); d.line([px,py-4,px,py+4], fill=(0,255,255))
    # P1 (green +)
    if row.get("p1x") is not None:
        x,y=row["p1x"],row["p1y"]
        d.line([x-7,y,x+7,y],fill=(0,255,0),width=2); d.line([x,y-7,x,y+7],fill=(0,255,0),width=2)
    # P4 (red x) or LOST
    p4_lost = row.get("p4x") is None
    if not p4_lost:
        x,y=row["p4x"],row["p4y"]
        d.line([x-6,y-6,x+6,y+6],fill=(255,40,40),width=2); d.line([x-6,y+6,x+6,y-6],fill=(255,40,40),width=2)
    # label
    tag = f"f{row['frame']} {'BLINK ' if row.get('blink') else ''}{'P4-LOST' if p4_lost else 'P4-ok'}"
    d.text((4,4), tag, fill=(255,255,0))
    # zoom inset around pupil: raw + contrast-stretched (to reveal faint P4)
    if px is not None:
        cx,cy=int(px),int(py)
        box=(max(0,cx-zoom_box),max(0,cy-zoom_box),
             min(img.width,cx+zoom_box),min(img.height,cy+zoom_box))
        crop=img.crop(box)
        raw=crop.resize((crop.width*scale,crop.height*scale),Image.NEAREST)
        # percentile contrast stretch on luminance to pull out dim reflections
        g=np.asarray(crop.convert("L")).astype(float)
        lo,hi=np.percentile(g,2),np.percentile(g,99.5)
        gs=np.clip((g-lo)/max(hi-lo,1)*255,0,255).astype(np.uint8)
        enh=Image.fromarray(gs).convert("RGB").resize((crop.width*scale,crop.height*scale),Image.NEAREST)
        ed=ImageDraw.Draw(enh); ed.text((2,2),"stretched",fill=(255,255,0))
        rd=ImageDraw.Draw(raw); rd.text((2,2),"raw zoom",fill=(255,255,0))
        H=max(img.height,raw.height)
        canvas=Image.new("RGB",(img.width+raw.width+enh.width+20,H),(20,20,20))
        canvas.paste(img,(0,0)); canvas.paste(raw,(img.width+10,0))
        canvas.paste(enh,(img.width+raw.width+20,0))
        return canvas
    return img

def session_paths(prefix):
    # DPI_DB env var overrides the db (e.g. point at a reprocess output) while
    # still reading frames from the matching data/ mp4.
    db = os.environ.get("DPI_DB") or os.path.join(DATA, prefix+".db")
    return os.path.join(DATA, prefix+".mp4"), db

def pick_fail(db_path, n):
    con=sqlite3.connect(db_path)
    rows=[r[0] for r in con.execute(
        "SELECT frame_number FROM eyetracking_frames WHERE obs_id IS NOT NULL "
        "AND p4_x IS NULL AND in_blink=0 AND pupil_x IS NOT NULL AND p1_x IS NOT NULL "
        "ORDER BY frame_number")]
    con.close()
    if not rows: return []
    idx=np.linspace(0,len(rows)-1,min(n,len(rows))).astype(int)
    return [rows[i] for i in idx]

def pick_recovered(db, orig_db, n):
    """Frames where orig_db has no P4 (in-obs, not blink) but db (reprocess) does."""
    con=sqlite3.connect(db); con.execute("ATTACH ? AS o",(orig_db,))
    rows=[r[0] for r in con.execute(
        "SELECT a.frame_number FROM eyetracking_frames a "
        "JOIN o.eyetracking_frames b ON a.frame_number=b.frame_number "
        "WHERE a.p4_x IS NOT NULL AND b.p4_x IS NULL AND b.obs_id IS NOT NULL "
        "AND b.in_blink=0 ORDER BY a.frame_number")]
    con.close()
    if not rows: return []
    idx=np.linspace(0,len(rows)-1,min(n,len(rows))).astype(int)
    return [rows[i] for i in idx]

def main():
    a=sys.argv[1:]
    prefix=a[0]; mp4,db=session_paths(prefix)
    if "--fail" in a:
        frames=pick_fail(db,int(a[a.index("--fail")+1]))
    elif "--recovered" in a:
        i=a.index("--recovered"); orig=a[i+1]; n=int(a[i+2]) if len(a)>i+2 and a[i+2].isdigit() else 8
        frames=pick_recovered(db,orig,n)
    elif "--context" in a:
        i=a.index("--context"); F,W=int(a[i+1]),int(a[i+2])
        frames=list(range(F-W,F+W+1))
    elif "--frames" in a:
        i=a.index("--frames"); frames=[int(x) for x in a[i+1].split(",")]
    else:
        frames=[int(x) for x in a[1:]]
    if not frames:
        print("no frames"); return
    rows=db_rows(db,frames)
    imgs=extract_frames(mp4,frames)
    for f in frames:
        if f not in imgs: continue
        row=rows.get(f,{"frame":f})
        canvas=draw_markers(imgs[f],row)
        outp=os.path.join(OUT,f"ovl_{prefix}_{f:06d}.png")
        canvas.save(outp); print(outp)

if __name__=="__main__":
    main()
