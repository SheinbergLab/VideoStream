#!/bin/bash
# Start VideoStream headless in WSL serving the browser viewer (dev test).
#   wsl-serve-test.sh [mp4] [extra serve.tcl args: mag angle speed]
REPO=/mnt/c/Users/ryan/Documents/videostream
MP4=${1:-/mnt/c/Users/ryan/Videos/eye_tracking/human_planko_drop_hazard_120_260925100259.mp4}
shift
pkill -x VideoStream 2>/dev/null
sleep 0.5
cd ~/vs-build
nohup ./VideoStream --www-dir "$REPO/web/dist" -f "$REPO/tcl/serve.tcl" -- "$MP4" "$@" > /tmp/serve.log 2>&1 &
echo "pid $!"
sleep 3
cat /tmp/serve.log
