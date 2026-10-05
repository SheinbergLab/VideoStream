#!/usr/bin/env python3
"""Print VideoStream RSS every interval until dead or duration."""
import subprocess
import sys
import time

interval = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

t0 = time.time()
while time.time() - t0 < duration:
    pid = subprocess.run(
        ["pgrep", "-x", "VideoStream"],
        capture_output=True,
        text=True,
    )
    if pid.returncode != 0:
        print(f"DIED t={time.time()-t0:.1f}s (no process)")
        subprocess.run(["tail", "-30", "/tmp/serve.log"])
        subprocess.run(["dmesg"], capture_output=True)
        # last oom lines
        d = subprocess.run(["dmesg"], capture_output=True, text=True)
        for line in d.stdout.splitlines()[-15:]:
            if "oom" in line.lower() or "killed process" in line.lower():
                print(line)
        sys.exit(1)
    p = pid.stdout.strip().split()[0]
    rss = subprocess.run(
        ["ps", "-o", "rss=", "-p", p],
        capture_output=True,
        text=True,
    )
    mb = int(rss.stdout.strip()) / 1024
    print(f"t={time.time()-t0:.0f}s rss={mb:.0f} MB pid={p}", flush=True)
    time.sleep(interval)
print("OK still alive at end of monitor window")
