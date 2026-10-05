#!/bin/bash
pkill -x VideoStream 2>/dev/null
sleep 1
rm -f /tmp/serve.log /tmp/mon.log
cd ~/vs-build || exit 1
./VideoStream --www-dir /mnt/c/Users/ryan/Documents/videostream/web/dist \
  -f /mnt/c/Users/ryan/Documents/videostream/tcl/serve.tcl -- \
  /mnt/c/Users/ryan/Videos/eye_tracking/human_planko_drop_hazard_120_260925100259.mp4 \
  >>/tmp/serve.log 2>&1 &
VS=$!
echo "started pid=$VS"
for i in $(seq 1 50); do
  sleep 3
  if kill -0 "$VS" 2>/dev/null; then
    rss=$(ps -o rss= -p "$VS" | tr -d ' ')
    echo "t=$((i * 3))s rss=${rss}KB" | tee -a /tmp/mon.log
  else
    echo "DIED at t=$((i * 3))s" | tee -a /tmp/mon.log
    echo "--- serve.log tail ---"
    tail -30 /tmp/serve.log
    exit 1
  fi
done
echo "still alive after 150s"
tail -5 /tmp/mon.log
