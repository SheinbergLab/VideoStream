#!/bin/bash
# Start VideoStream detached (survives WSL one-shot invocations).
pkill -9 -x VideoStream 2>/dev/null || true
sleep 1
: > /tmp/serve.log
cd ~/vs-build
nohup ./VideoStream \
  --www-dir /mnt/c/Users/ryan/Documents/videostream/web/dist \
  -f /mnt/c/Users/ryan/Documents/videostream/tcl/serve.tcl \
  -- /mnt/c/Users/ryan/Videos/eye_tracking/human_planko_drop_hazard_120_260925100259.mp4 \
  >> /tmp/serve.log 2>&1 &
disown
sleep 2
if pgrep -x VideoStream >/dev/null; then
  echo "VideoStream running pid=$(pgrep -x VideoStream)"
  head -5 /tmp/serve.log
else
  echo "FAILED to start"
  cat /tmp/serve.log
  exit 1
fi
