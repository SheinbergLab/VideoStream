#!/bin/bash
set -euo pipefail
pkill -9 -x VideoStream 2>/dev/null || true
sleep 1
: > /tmp/serve.log
cd ~/vs-build
./VideoStream \
  --www-dir /mnt/c/Users/ryan/Documents/videostream/web/dist \
  -f /mnt/c/Users/ryan/Documents/videostream/tcl/serve.tcl \
  -- /mnt/c/Users/ryan/Videos/eye_tracking/human_planko_drop_hazard_120_260925100259.mp4 \
  >> /tmp/serve.log 2>&1 &
VS=$!
echo "started pid=$VS"
for i in $(seq 1 90); do
  sleep 1
  if kill -0 "$VS" 2>/dev/null; then
    rss=$(awk '/^VmRSS:/ {print $2}' "/proc/$VS/status" 2>/dev/null || echo "?")
    echo "t=${i}s alive rss=${rss}kB"
  else
    echo "t=${i}s DEAD"
    tail -30 /tmp/serve.log
    dmesg 2>/dev/null | grep -iE 'oom|killed process' | tail -8 || true
    exit 1
  fi
done
echo "still alive after 90s"
