#!/bin/sh
# Build the minimal Arena SDK bundle that CI uses to produce the Lucid-enabled
# VideoStream package: headers to compile against plus the unmodified runtime
# libraries VideoStream links or loads. Everything else in the 1 GB SDK
# (examples, docs, Save API, ffmpeg, Metavision HAL, .cti producers) is left out.
#
# The result is for a PRIVATE release asset (the Arena EULA restricts the SDK
# itself to your organisation; only the runtime libraries may be redistributed,
# and only in original form -- so nothing here is stripped). Upload the tarball
# to the repo named by the ARENA_SDK_REPO CI variable and give CI a read token
# in the ARENA_SDK_TOKEN secret; see .github/workflows/release_linux.yml.
#
# usage: scripts/make-arena-runtime-bundle.sh /path/to/ArenaSDK_Linux_x64 [outdir]
set -eu

SDK=${1:?usage: $0 /path/to/ArenaSDK_Linux_x64 [outdir]}
OUT=${2:-.}
[ -f "$SDK/include/Arena/ArenaApi.h" ] || { echo "$SDK is not an Arena SDK tree (no include/Arena/ArenaApi.h)" >&2; exit 1; }

# version from the real libarena file name, e.g. libarena.so.1.0.10 -> 1.0.10
VER=$(ls "$SDK"/lib64/libarena.so.[0-9]* | sed -E 's/.*libarena\.so\.//' | sort -V | tail -1)
NAME="arena-sdk-runtime-${VER}-linux-x64"
STAGE=$(mktemp -d)
B="$STAGE/$NAME"
mkdir -p "$B/include" "$B/lib64" "$B/GenICam/library/lib/Linux64_x64" "$B/GenICam/library/CPP" "$B/Metavision/lib"

# headers (same layout as the SDK so ARENA_SDK_DIR can point at the bundle)
cp -a "$SDK/include/Arena" "$SDK/include/GenTL" "$B/include/"
cp -a "$SDK/GenICam/library/CPP/include" "$B/GenICam/library/CPP/"

# runtime closure (keep the .so.N symlinks)
for lib in libarena libgentl liblucidlog; do
    cp -a "$SDK"/lib64/$lib.so* "$B/lib64/"
done
cp -a "$SDK"/GenICam/library/lib/Linux64_x64/*_LUCID.so "$B/GenICam/library/lib/Linux64_x64/"
for lib in libmetavision_sdk_core libmetavision_sdk_base; do
    cp -a "$SDK"/Metavision/lib/$lib.so* "$B/Metavision/lib/"
done

cp -a "$SDK/licenses" "$B/"
[ -f "$SDK/../README" ] && cp "$SDK/../README" "$B/README.arena" || true
cat > "$B/README.bundle" <<EOF
Minimal Arena SDK $VER bundle for building VideoStream with -D WITH_LUCID=ON
-D ARENA_SDK_DIR=<this directory>. Produced by scripts/make-arena-runtime-bundle.sh.
Runtime libraries are unmodified copies from the Lucid Vision Labs Arena SDK; see
licenses/ for their terms. Not for redistribution outside the organisation.
EOF

mkdir -p "$OUT"
TARBALL="$OUT/$NAME.tar.gz"
tar -C "$STAGE" -czf "$TARBALL" "$NAME"
rm -rf "$STAGE"
echo "wrote $TARBALL ($(du -h "$TARBALL" | cut -f1))"
