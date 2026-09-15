#!/bin/sh
# Build the minimal Spinnaker SDK bundle that CI uses to produce the FLIR-enabled
# VideoStream package (videostream-flir): headers to compile against plus the
# unmodified runtime libraries VideoStream links (libSpinnaker and its GenICam
# and OpenMP dependencies). SpinVideo, SpinUpdate, SpinView, the C API and the
# GenTL producer are left out.
#
# LICENSE: the FLIR Spinnaker SDK License Agreement does not allow providing the
# SDK to third parties. This bundle, and the videostream-flir packages built from
# it, are for the organisation's own machines and cameras only -- keep them in a
# PRIVATE repository, never on a public release. Nothing here is stripped or
# otherwise modified.
#
# The SDK's libstdc++ requirement follows the toolchain FLIR built it with: the
# Ubuntu 24.04 package (lib*_gcc13_*) runs on noble/trixie, the 22.04 package
# (lib*_gcc11_*) on jammy/bookworm. Make one bundle per ABI; the name records it.
#
# usage: scripts/make-spinnaker-runtime-bundle.sh <spinnaker root> [outdir]
#   <spinnaker root> is /opt/spinnaker on a machine with the SDK installed, or
#   <dir>/opt/spinnaker after `dpkg-deb -x` of libgentl, libspinnaker and
#   libspinnaker-dev from the SDK tarball.
set -eu

SDK=${1:?usage: $0 <spinnaker root, e.g. /opt/spinnaker> [outdir]}
OUT=${2:-.}
[ -f "$SDK/include/Spinnaker.h" ] || { echo "$SDK has no include/Spinnaker.h (install libspinnaker-dev)" >&2; exit 1; }

VER=$(ls "$SDK"/lib/libSpinnaker.so.[0-9]*.[0-9]* | sed -E 's/.*libSpinnaker\.so\.//' | sort -V | tail -1)
ABI=$(ls "$SDK"/lib/libGenApi_gcc*_v3_0.so | sed -E 's/.*libGenApi_(gcc[0-9]+)_v3_0\.so/\1/' | head -1)
NAME="spinnaker-sdk-runtime-${VER}-${ABI}-linux-x64"
STAGE=$(mktemp -d)
B="$STAGE/$NAME"
mkdir -p "$B/lib"

cp -a "$SDK/include" "$B/"
cp -a "$SDK"/lib/libSpinnaker.so* "$B/lib/"
cp -a "$SDK"/lib/lib*_${ABI}_v3_0.so "$B/lib/"
cp -a "$SDK"/lib/libiomp5.so "$B/lib/"
[ -d "$SDK/licenses" ] && cp -a "$SDK/licenses" "$B/" || true

# the license text ships as the Debian copyright file of libspinnaker
for c in "$SDK/../../usr/share/doc/libspinnaker/copyright" /usr/share/doc/libspinnaker/copyright; do
    if [ -f "$c" ]; then cp "$c" "$B/copyright"; break; fi
done

cat > "$B/README.bundle" <<EOF
Minimal Spinnaker SDK $VER ($ABI) bundle for building VideoStream with
-D WITH_FLIR=ON -D SPINNAKER_SDK_DIR=<this directory>. Produced by
scripts/make-spinnaker-runtime-bundle.sh. Unmodified copies from the Teledyne FLIR
Spinnaker SDK; see copyright and licenses/. Organisation-internal only: the SDK
license does not permit distribution to third parties.
EOF

mkdir -p "$OUT"
TARBALL="$OUT/$NAME.tar.gz"
tar -C "$STAGE" -czf "$TARBALL" "$NAME"
rm -rf "$STAGE"
echo "wrote $TARBALL ($(du -h "$TARBALL" | cut -f1))"
