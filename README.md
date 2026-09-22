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

### FLIR-enabled .deb (`videostream-flir`, org-internal)

The Spinnaker license does **not** allow passing the SDK to third parties (only
OEMs may redistribute the libraries, and only if they prevent any further
redistribution), so there is no public FLIR package. For the lab's own rigs, CI
builds a self-contained `videostream-flir_<version>_amd64_<distro>.deb` and
uploads it to a **private** repo release instead (default
`SheinbergLab/videostream-private`, release named after the VideoStream tag).
Its postinst creates the `flirimaging` udev group, installs the USB udev rule
and sets the usbfs memory limit that USB3 cameras need, and adds the installing
user to the group; other camera users need `sudo usermod -aG flirimaging USER`.

```sh
sudo apt install ./videostream-flir_<version>_amd64_<distro>.deb   # log out/in once
/usr/local/videostream/VideoStream --flir -f /usr/local/videostream/tcl/tracker.tcl
```

Setup, once: make one bundle per libstdc++ ABI with
`scripts/make-spinnaker-runtime-bundle.sh` (FLIR's Ubuntu 24.04 SDK build,
`gcc13`, serves noble/trixie; the 22.04 build, `gcc11`, serves jammy/bookworm),
attach both to a release tagged `spinnaker-sdk-runtime` in the private repo,
and add a fine-grained token with contents read/write on that repo as the
`VIDEOSTREAM_PRIVATE_TOKEN` secret. `-D SPINNAKER_SDK_DIR=` points a local build
at a bundle instead of `/opt/spinnaker`; `-D FLIR_BUNDLE_RUNTIME=OFF` installs
without the libraries.

## Building with Lucid (Arena SDK) support

Lucid Vision Labs GigE cameras (e.g. Triton) are driven through the **Arena
SDK**, which is also proprietary and **not redistributable**, so it is off by
default. Linux only.

1. Extract the Arena SDK tarball somewhere, e.g. `~/ArenaSDK_Linux_x64` or
   `/opt/ArenaSDK_Linux_x64`. The SDK's `Arena_SDK_Linux_x64.conf` (ld.so.conf)
   step is **not** required: the binary carries an rpath to the SDK directory.
   `libibverbs1` and `librdmacm1` must be installed (`apt install`).
2. Configure with Lucid enabled and build:

   ```sh
   cmake -B build -D WITH_LUCID=ON -D ARENA_SDK_DIR=$HOME/ArenaSDK_Linux_x64
   cmake --build build -j
   ```

   `ARENA_SDK_DIR` may be omitted if the SDK is in one of the two locations
   above. FLIR and Lucid can be enabled in the same build.

3. Run with a Lucid source, e.g. `./build/VideoStream --lucid -f tcl/tracker.tcl`,
   or from Tcl `vstream::startSource lucid ?id N? ?serial S?` (device index in
   the Arena device list, or a serial number). `--flir` / `--lucid` also set
   `::camera_type`, which the tracker scripts use when they (re)start the live
   source.

### Lucid-enabled .deb (`videostream-lucid`)

CI also publishes `videostream-lucid_<version>_amd64_<distro>.deb` (Debian
Bookworm/Trixie, Ubuntu Jammy/Noble). It is self-contained: the unmodified Arena
runtime libraries are installed under `/usr/local/videostream/lib/arena` and the
binary finds them by rpath, so no SDK install is needed on the target machine.
It conflicts with the plain `videostream` package (same install location).

```sh
sudo apt install ./videostream-lucid_<version>_amd64_<distro>.deb
/usr/local/videostream/VideoStream --lucid -f /usr/local/videostream/tcl/tracker.tcl
```

Because the SDK is not public, the release job pulls a private headers+runtime
bundle instead of the SDK itself. To set that up once:

1. On a machine with the SDK, run
   `scripts/make-arena-runtime-bundle.sh /path/to/ArenaSDK_Linux_x64` (produces
   `arena-sdk-runtime-<ver>-linux-x64.tar.gz`, ~80 MB).
2. Upload it as a release asset in a **private** repo (default
   `SheinbergLab/arena-sdk-runtime`; override with the `ARENA_SDK_REPO` and
   `ARENA_SDK_TAG` repository variables).
3. Add a fine-grained token with read access to that repo as the
   `ARENA_SDK_TOKEN` secret. Without it the Lucid job skips itself.

For a GigE camera the receiving interface should allow jumbo frames (MTU 9000)
and the kernel receive buffers should be raised (`net.core.rmem_max` /
`net.core.rmem_default`, e.g. 32 MB); the SDK's `.conf` script sets the sysctls.
The stream is configured to auto-negotiate the packet size and request packet
resends, so it also works at MTU 1500 with a small amount of resend traffic.

## Camera functions

Both camera backends expose the same commands under `camera::`; `flir::` and
`lucid::` are the same commands under the vendor name (the tracker scripts use
`camera::`, and pick the backend with `set ::camera_type flir|lucid`).
```
camera::isAvailable            camera::vendor
camera::startAcquisition       camera::stopAcquisition       camera::isStreaming
camera::configureExposure ?us? camera::configureGain ?dB?
camera::configureFrameRate ?hz? camera::getFrameRateRange
camera::configureBinning ?h v? camera::configureImageOrientation reverseX reverseY
camera::configureROI ?w h x y? camera::getROI  camera::setROIOffset ?x y?  camera::getROIConstraints
camera::ttlLine ?line?         camera::lineStatusAll
camera::getSettings            camera::refreshSettings
camera::node name ?value?      camera::nodeInfo name         camera::nodes ?pattern?
camera::configureLine line ?mode? ?source? ?inverter?
```

`camera::node` reads or writes any GenICam feature by name (integers and
floats as numbers, booleans as 1/0, enumerations by entry name; commands
execute), `camera::nodeInfo` returns its type, access, range and enumeration
entries, and `camera::nodes Line*` lists feature names. Features the camera
locks while streaming are written with a brief stream pause.
`camera::configureLine` selects an I/O line and sets its mode, source and
inverter, e.g. a strobe output that follows the exposure so a light source is
synced to the shutter (what the Lucid rig does on Line1):

```tcl
camera::configureLine 1 Output ExposureActive 1   ;# LineSelector Line1, LineMode Output,
                                                  ;# LineSource ExposureActive, LineInverter 1
camera::configureExposure 430
camera::node AcquisitionFrameTime 4001            ;# or camera::configureFrameRate 250
```

The tracker scripts keep such settings per backend in `::camera_live_settings`
(see `tcl/et_camera.tcl`), applied once when going live.

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

