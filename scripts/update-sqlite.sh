#!/usr/bin/env bash
#
# Update the vendored SQLite amalgamation (external/sqlite/sqlite3.{c,h}) to the
# current release published on sqlite.org.
#
# Usage:  scripts/update-sqlite.sh
#
# After it runs, rebuild and commit the two files:
#   cmake --build build -j
#   git add external/sqlite/sqlite3.c external/sqlite/sqlite3.h
#   git commit -m "Update vendored SQLite to <version>"
#
set -euo pipefail

cd "$(dirname "$0")/.."          # repo root
DEST="external/sqlite"

[ -d "$DEST" ] || { echo "error: $DEST not found (run from the VideoStream repo)" >&2; exit 1; }

# sqlite.org/download.html embeds a machine-readable product list; the
# amalgamation relpath looks like "2026/sqlite-amalgamation-3530300.zip"
# (the year prefix is why we scrape it rather than guess the URL).
echo "Looking up the current SQLite amalgamation on sqlite.org ..."
relpath=$(curl -fsSL https://sqlite.org/download.html \
          | grep -oE '20[0-9][0-9]/sqlite-amalgamation-[0-9]+\.zip' \
          | head -1)
[ -n "$relpath" ] || { echo "error: could not find the amalgamation URL on sqlite.org/download.html" >&2; exit 1; }

url="https://sqlite.org/${relpath}"
echo "Downloading ${url}"

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
curl -fsSL -o "$tmp/amalg.zip" "$url"
unzip -q "$tmp/amalg.zip" -d "$tmp"
src=$(find "$tmp" -maxdepth 1 -type d -name 'sqlite-amalgamation-*' | head -1)
[ -n "$src" ] || { echo "error: unexpected archive layout" >&2; exit 1; }

cp "$src/sqlite3.c" "$DEST/sqlite3.c"
cp "$src/sqlite3.h" "$DEST/sqlite3.h"

ver=$(grep -m1 '#define SQLITE_VERSION ' "$DEST/sqlite3.h" | sed -E 's/.*"([0-9.]+)".*/\1/')
echo "Updated vendored SQLite to ${ver}  (${DEST}/sqlite3.{c,h})"
echo "Next: rebuild, then  git add ${DEST}/sqlite3.c ${DEST}/sqlite3.h && git commit -m \"Update vendored SQLite to ${ver}\""
