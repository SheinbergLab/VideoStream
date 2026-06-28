#!/bin/sh
# Runs after the .pkg installs VideoStream.app into /Applications.
# Creates a CLI launcher so `videostream <args>` works from the terminal — the
# self-contained .app resolves its bundled Frameworks via the real executable
# path, so a symlink works fine.
set -e
mkdir -p /usr/local/bin
ln -sf /Applications/VideoStream.app/Contents/MacOS/VideoStream /usr/local/bin/videostream
exit 0
