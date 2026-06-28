#!/bin/sh
# Runs after the .pkg installs VideoStream.app into /Applications.
# Installs a CLI launcher so `videostream <args>` works from the terminal.
#
# NOTE: a *symlink* doesn't work here. When launched through a symlink, Tcl
# resolves its executable path to the symlink's directory (/usr/local/bin), so it
# looks for its stdlib at /usr/local/lib/tcl9.0 instead of inside the bundle and
# fails with "can't find a usable init.tcl". A wrapper that exec's the real
# in-bundle binary makes the executable path resolve inside the .app, so Tcl finds
# Contents/lib/tcl9.0.
set -e
mkdir -p /usr/local/bin
cat > /usr/local/bin/videostream <<'EOF'
#!/bin/sh
exec "/Applications/VideoStream.app/Contents/MacOS/VideoStream" "$@"
EOF
chmod +x /usr/local/bin/videostream
exit 0
