# VideoStream

Standalone program to acquire video frames (from a standard webcam/UVC device, a
FLIR Spinnaker camera, or an mp4 file), display them, run analysis plugins
(e.g. Purkinje eye tracking), and store frames to disk. It is a general tool for
**video-based characterization of behavior**, eye movements being one case.

For control, the program opens a TCP/IP port and receives Tcl scripts. Metadata
about frames is stored alongside the video in the `dgz` format.

Core dependencies: [Tcl 9](https://tcl.tk), [OpenCV](https://opencv.org),
[libdg](https://github.com/SheinbergLab/dlsh) (from the dlsh release), plus
[uWebSockets/uSockets](https://github.com/uNetworking/uWebSockets) (fetched
automatically via CMake), `jansson` and `lz4` (system packages), and `sqlite`
(vendored in `external/`). FLIR Spinnaker is optional (see below).

## Releases

Prebuilt, signed artifacts are published on the
[Releases page](https://github.com/SheinbergLab/VideoStream/releases) for each
tagged version. **These builds are FLIR-free** — they support webcam/UVC capture,
mp4 playback, and review/reprocess. For FLIR camera acquisition, build from source
with `-D WITH_FLIR=ON` (see [Building with FLIR support](#building-with-flir-support)).

### macOS

A signed and **notarized** `.pkg` (Apple Silicon, macOS 14 Sonoma or newer).
Double-click to install; it places `VideoStream.app` in `/Applications` and a
`videostream` command-line launcher in `/usr/local/bin`. The app is
self-contained (OpenCV and Tcl are bundled) — nothing else needs to be installed.

```sh
videostream --help
videostream -f /path/to/script.tcl
```

On first camera use macOS will prompt for camera permission.

### Linux

A `.deb` for Debian (Bookworm/Trixie) and Ubuntu (Jammy/Noble), amd64 and arm64.
It installs under `/usr/local/videostream/`.

```sh
sudo apt install ./videostream_<version>_<arch>_<distro>.deb
/usr/local/videostream/VideoStream --help
```

Note: the package depends on `libtcl9.0`, which is only available from apt on
Debian Trixie / newer Ubuntu. On older releases you must provide Tcl 9 yourself
(or build from source).

CI (GitHub Actions) builds these artifacts on every tag; see
[docs/build-and-release-plan.md](docs/build-and-release-plan.md) for details.

## Building from source

The default build is FLIR-free (webcam/UVC + mp4 + review/reprocess).

### Linux

```sh
sudo apt install build-essential cmake pkg-config
sudo apt install libopencv-dev zlib1g-dev liblz4-dev libjansson-dev
# Tcl 9: build from the deps/tcl submodule, or install libtcl9.0/tcl9.0-dev where available
# libdg: install the dlsh-dg .deb from https://github.com/SheinbergLab/dlsh/releases

cmake -B build
cmake --build build -j
./build/VideoStream --help
```

### macOS

```sh
brew install cmake pkg-config opencv tcl-tk@9 lz4 jansson
# libdg: install the dlsh .pkg from https://github.com/SheinbergLab/dlsh/releases

cmake -B build
cmake --build build -j
./build/VideoStream --help
```

To build the self-contained, signed `.app`/`.dmg` locally, configure with
`-D MACOS_APP_BUNDLE=ON` (and `-D MACOS_CODESIGN_IDENTITY="Developer ID Application: ..."`
to sign); see the CMake `APPLE` branch and `release_macos.yml` for the full flow.

## Building with FLIR support

FLIR Spinnaker is a proprietary SDK that is **not redistributable**, so it is
**off by default** and never included in the published artifacts. To build a
camera-acquisition binary on a machine that has the SDK installed:

1. Install the **Spinnaker SDK** from Teledyne FLIR (the SpinView app is not
   required — only the SDK libraries/headers). On Linux the build expects it under
   `/opt/spinnaker` (`/opt/spinnaker/include`, `/opt/spinnaker/lib`).
2. Configure with FLIR enabled and build:

   ```sh
   cmake -B build -D WITH_FLIR=ON
   cmake --build build -j
   ```

   CMake prints `FLIR Spinnaker camera support: ENABLED` when it is on.

3. Run with a FLIR source, e.g. `./build/VideoStream --flir`.

Without `-D WITH_FLIR=ON`, passing `--flir` prints a message and exits — use
`--webcam` (or a `playback`/file source) instead.

FLIR support is currently Linux/Windows only; macOS builds are webcam/file only.

## General functions
```
 vstream::fileOpen
 vstream::fileClose
 vstream::domainSocketOpen
 vstream::domainSocketClose
 vstream::domainSocketSendN (-1: continuous; 0: stop; n send "n")
 vstream::inObs
 vstream::fourcc
 vstream::addShutdownCmd
 vstream::displayOpen
 vstream::displayClose
```

## FLIR camera functions
```
vstream::configureExposure
vstream::configureGain
vstream::configureFrameRate
vstream::configureROI
```
