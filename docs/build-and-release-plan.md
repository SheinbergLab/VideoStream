# VideoStream — Build, Dependency & Release Plan

Status: **proposal for review** (no code changed yet)
Goal: reduce dependencies, simplify the default build, and ship pre-built release
artifacts via GitHub Actions, aligned with the `SheinbergLab/{stim2,dserv,dlsh}`
conventions.

---

## 1. Guiding decisions (agreed)

1. **FLIR is opt-in source only.** The Spinnaker SDK cannot be redistributed in a
   pre-built binary, and hosted CI can't install it. Since the code is MIT-track,
   an optional FLIR build path in source is legal — the user supplies the SDK and
   builds with `-D WITH_FLIR=ON`. **The shipped artifact is always FLIR-free.**
2. **The distributable artifact targets standard cameras + playback:** webcam/UVC
   (OpenCV `VideoCapture`, already implemented in `WebcamSource`), mp4 file
   playback, review mode, and the headless reprocess path. All MIT-clean and
   fully redistributable.
3. **Follow the stim2 convention** for dependencies (build-from-source deps as
   `deps/` git submodules) and CI (tag-triggered `release_linux.yml` /
   `release_macos.yml`, consume `dlsh` as a published artifact, `cpack` package,
   `ncipollo/release-action` upload, plus a `check` install job).
4. **Platforms: Linux + macOS only** (Windows deferred).
5. **Per-platform packaging & layout:**
   - **Linux:** `.deb` (cpack DEB), installs to `/usr/local/videostream/`; big
     standard libs (OpenCV) are **declared apt dependencies**, resolved on install.
   - **macOS:** self-contained **`VideoStream.app`** in a signed/notarized **`.dmg`**
     (`cpack -G DragNDrop`), dragged to `/Applications`. All non-system dylibs
     (incl. OpenCV **and the eyetracking plugin's** OpenCV refs) are bundled into
     `Contents/Frameworks` via **`dylibbundler`** — **no brew needed at runtime**.
6. **OpenCV is shared + bundled, not static** (see §4.2 for rationale).

### Non-goals
- Shipping a camera-acquisition binary for FLIR hardware (build locally instead).
- Reworking the embedded scripting language (Tcl stays — see prior discussion).
- Windows support (deferred).
- Statically linking OpenCV.

---

## 2. Licensing

- Add a top-level `LICENSE` (**MIT**, matching stim2/dlsh).
- Dependency licenses are all redistribution-friendly: OpenCV (Apache-2.0), Tcl
  (BSD-style), jansson (MIT), uSockets/uWebSockets (Apache-2.0), SQLite (public
  domain), libuv (MIT), lz4/zlib (BSD/zlib), dlsh `dg` (MIT).
- **FLIR Spinnaker** (proprietary) never enters the artifact; document it as an
  external prerequisite for `WITH_FLIR=ON` builds.

---

## 3. Dependency plan (full stim2 convention)

| Dependency | Today | Plan |
|---|---|---|
| **dlsh `dg` / `df.h`** | system `dg` | Consume dlsh **release artifact**: `dlsh-dg_${V}_${arch}.deb` (+ `dlsh-dlsh`) on Linux via `dpkg`; `dlsh-*-Darwin-*.pkg` on macOS. (Mirrors stim2.) |
| **Tcl 9** | `find_library` w/ 9.0→9→8.6 fallback | **`deps/tcl` submodule**, configure+make+install in CI. Removes version drift and the fallback logic. |
| **jansson** | `find_library` static | **`deps/jansson` submodule**, static lib (as stim2/dserv do). |
| **OpenCV** | `find_package(OpenCV REQUIRED)`, links `${OpenCV_LIBS}` | apt `libopencv-dev` / brew `opencv`; **link only needed modules** (core, imgproc, videoio, imgcodecs — verify against call sites). |
| **uSockets / uWebSockets** | FetchContent (pinned) | Keep FetchContent (reproducible) **or** submodule for full consistency. *Low priority.* |
| **SQLite** | vendored in `external/sqlite` | No change. |
| **lz4 / zlib / libuv** | system | apt/brew. |
| **FLIR Spinnaker** | linked on Linux/Win (`USE_FLIR`) | Behind `option(WITH_FLIR OFF)`; never in CI artifact. |

Net effect on the **default** build: OpenCV + Tcl(submodule) + jansson(submodule)
+ dlsh(artifact) + vendored sqlite + FetchContent sockets + system lz4/zlib/uv.
No proprietary deps.

---

## 4. CMake changes

1. **`option(WITH_FLIR "Build FLIR Spinnaker camera source" OFF)`**
   - Compile `FlirCameraSource.cpp` and the `SourceManager` FLIR branch only when
     set; link `${FLIR_LIBRARY}` only then. (Source is already `#ifdef USE_FLIR`;
     wire the option to that define and to the source list / link.)
2. **OpenCV: shared libs, trimmed modules, bundled on macOS (not static).**
   - Link only the modules actually used (verify: core, imgproc, videoio,
     imgcodecs; drop highgui/dnn/etc. if unused).
   - **Do not static-link.** Reasons: (a) avoids a 20–40 min OpenCV-from-source
     build and its static 3rd-party chain; (b) avoids tens-of-MB binary bloat;
     (c) **critical** — `eyetracking.dylib` also links OpenCV, so a static build
     would embed two separate OpenCV copies (host + plugin) across the dlopen
     boundary, duplicating globals/allocators. Shared = one copy both load.
   - **Linux:** OpenCV from apt (`libopencv-dev` at build, runtime libs declared
     as DEB `Depends:`). apt resolves it on install.
   - **macOS:** build against brew OpenCV, then `dylibbundler` copies OpenCV (and
     all non-system dylibs) into `VideoStream.app/Contents/Frameworks` and rewrites
     install names. The plugin `eyetracking.dylib` must also be processed so its
     OpenCV refs point at the bundled Frameworks (stim2 does this for `stimdlls`).
     Verify with `otool -L` that no `/usr/local` or `/opt/homebrew` refs remain.
3. **Tcl/jansson from `deps/` submodules** — replace the `find_library` fallbacks
   with the submodule build outputs (consistent with stim2).
4. **CPack packaging** (currently absent):
   - `cpack -G DEB` on Linux with `PROJECT_VERSION` from the tag and a
     `DISTRO_SUFFIX` (e.g. `bookworm`/`noble`), matching stim2's naming.
   - Install rules: binary → `/usr/local/videostream/`, plus `plugins/`, `tcl/`,
     `www/`. macOS: `.pkg` (productbuild) or a versioned `.zip`.
   - Declare runtime deps in the DEB control (opencv, tcl9, dlsh-dg, libuv…).
5. **Version injection**: `-D PROJECT_VERSION=${github.ref_name}` like stim2.

---

## 5. Repo cleanup (reduce footprint & noise)

These are not build inputs (not referenced by CMake) and add clutter / confusion:

- **Dead Python scrap in root** → move to `contrib/` or delete:
  `pyqt_video.py`, `pyqt5_video.py`, `receive_frames.py`, `receive_video.py`,
  `tkframe_viewer.py`, `uds_server.py`, `run_camera`.
- **Stray artifacts committed / loose in tree**: `build/`, `_deps/`, `*.db`,
  `*.mp4`, `*.bak`, `*~`, `.DS_Store`, `*.dylib`. Add a real **`.gitignore`**
  (the current `git status` is dominated by these).
- Keep `external/sqlite` (intentional vendoring).
- Consolidate the many `tcl/*.tcl~` / `*.NEW` / `*.thresholds` backups.

---

## 6. GitHub Actions workflows (mirror stim2)

### `build_and_test.yml` (on: push) — fast feedback
- Matrix: linux (debian/ubuntu × amd64/arm64) + macOS (14/15), **FLIR-free**.
- Steps: install deps → checkout `submodules: true` → build Tcl9 submodule →
  install dlsh artifact → `cmake -B build -D WITH_FLIR=OFF` → `cmake --build` →
  `ctest` (reprocess smoke test, §7).

### `release_linux.yml` (on: push tag `*`)
- Matrix over distro containers × arch (copy stim2's matrix).
- Steps: apt deps (incl. `libopencv-dev`) → checkout submodules → build+install
  Tcl9 (and jansson) from `deps/` → `dpkg --install` dlsh `dg`/`dlsh` `.deb` →
  `cmake -D PROJECT_VERSION=... -D DISTRO_SUFFIX=... -D WITH_FLIR=OFF -B build` →
  `cmake --build` → `cpack -G DEB` → upload via `ncipollo/release-action@v1`.
- **`check` job**: install the produced `.deb` on a clean container and sanity-run
  (`videostream --help` and/or a one-frame headless reprocess).

### `release_macos.yml` (on: push tag `*`) — mirrors stim2 exactly
- Runner: `macos-14` (build on not-latest for modest dep versions).
- Steps:
  - `brew install cmake dylibbundler opencv lz4 libuv jq` (+ tcl via submodule or
    `brew tcl-tk@9`).
  - Checkout `submodules: true`.
  - Build Tcl9 (and jansson) from `deps/` submodules.
  - Install dlsh `.pkg`: `installer -pkg dlsh-${V}-Darwin-arm64-signed.pkg`.
  - `cmake -B build -G Xcode -D PROJECT_VERSION=${tag} -D WITH_FLIR=OFF` → build.
  - **Bundle**: `dylibbundler` over the app binary **and** `eyetracking.dylib`
    into `Contents/Frameworks`; `cpack -G DragNDrop` → `.dmg`.
  - **Self-contained guard** (copy stim2): `otool -L` checks asserting no
    `/usr/local` or `/opt/homebrew` deps in the binary, bundled dylibs, or plugin.
  - **Sign + notarize** (Apple Developer secrets, as stim2): keychain import,
    `codesign`, `xcrun notarytool submit --wait`, `xcrun stapler staple`.
  - Upload `*.dmg` via `ncipollo/release-action@v1`.
- **`check` job** on `macos-15` (clean, no brew dlsh): mount the `.dmg`, copy
  `VideoStream.app` to `/Applications`, run `--help` + a short headless reprocess
  to prove the bundle is self-contained.

> If Apple Developer secrets aren't available to this repo initially, ship an
> **unsigned `.dmg`** (Gatekeeper will warn) and add signing/notarization later.

---

## 7. CI smoke test (reuse what exists)

The deterministic headless reprocess harness (`tcl/reprocess_headless.tcl`,
`scripts/regenerate_session.sh`) is an ideal `ctest`:

1. Commit a **tiny fixture** (a few-frame mp4 + expected tracking output) under
   `tests/fixtures/`.
2. `ctest` runs `VideoStream` headless over the fixture and diffs output against
   the golden file (tolerant compare for floats).
3. Wire into `build_and_test.yml` and the release `check` jobs.

This validates the FLIR-free path end-to-end on every push — mirrors dserv's
`ctest` step.

---

## 8. Suggested sequencing

1. **Track A — CMake/deps (local, no CI yet)**
   - `LICENSE` (MIT), `.gitignore`, repo cleanup.
   - `option(WITH_FLIR OFF)`; verify FLIR-free build still runs (webcam + mp4 +
     reprocess).
   - OpenCV module trim.
   - Add `deps/tcl`, `deps/jansson` submodules; switch CMake to use them.
   - Add CPack config; produce a local `.deb`/`.zip` to validate packaging.
2. **Track B — smoke test**
   - Add fixture + `ctest`; confirm green locally.
3. **Track C — workflows**
   - `build_and_test.yml` first (catch issues cheaply).
   - Then `release_linux.yml` + `release_macos.yml`; cut a `v0.x` test tag.

Each track is independently reviewable/commit-able.

---

## 8a. Local testing & secrets

**Develop both pipelines locally before pushing tags.**
- **macOS:** native on the Mac — `brew install` deps, cmake/Xcode build,
  `dylibbundler`, `cpack -G DragNDrop`, `otool -L` self-contained checks, and
  `codesign`/`notarytool` (cert in keychain). Full fidelity without CI.
- **Linux:** Docker using the **same container images CI uses**
  (`debian:12|13`, `ubuntu:22.04|24.04`):
  `docker run --rm -it -v "$PWD":/src -w /src debian:12 bash`. On Apple Silicon,
  arm64 legs run native; amd64 via `--platform linux/amd64` (qemu). `act` can run
  the Linux workflow YAML locally (no macOS runner support). Parallels only if you
  need to interactively run the **GUI** app (Docker is headless — fine for
  build/package/headless-reprocess).
- **Anti-drift:** factor build steps into `scripts/ci-build-linux.sh` (and a mac
  equivalent) called by both local runs and the workflow.

**Secrets (Apple signing):**
- Currently **repo-level on `stim2`** (9 secrets); `VideoStream` has none. GitHub
  secrets are **write-only** — cannot be copied between repos.
- **Recommended:** promote to **org-level** secrets (SheinbergLab) shared with
  stim2/dserv/dlsh/VideoStream. Alternative: re-add the needed subset to
  VideoStream. Either needs the original material (re-export `.p12`, regenerate the
  app-specific password if lost).
- For a `.dmg` only the **Application** cert set is needed (skip `APPLE_INSTALLER_*`).
- **Not a blocker:** Track A and the Linux pipeline need no Apple secrets; ship an
  unsigned `.dmg` until signing is wired.

## 9. Decisions & remaining questions

**Resolved:**
- **Platforms:** Linux + macOS; **Windows deferred.**
- **Linux layout:** `/usr/local/videostream/`.
- **macOS packaging:** self-contained `.app` in a `.dmg` (`cpack -G DragNDrop`),
  dylibs bundled via `dylibbundler` — matches stim2.
- **OpenCV:** shared + bundled (macOS) / apt dependency (Linux); **not static.**

**Still open:**
1. **macOS signing** — are the Apple Developer secrets stim2 uses
   (`APPLE_ID`, `APPLE_TEAM_ID`, certificate/keychain passwords) available to this
   repo? If yes → sign + notarize like stim2. If not → start with unsigned `.dmg`.
2. **Runtime asset location** — how should the installed app locate `plugins/`,
   `tcl/`, `www/`? Today the loader uses `[file dir [info nameofexecutable]]/...`.
   On Linux that's `/usr/local/videostream/`; on macOS it must resolve inside the
   `.app` bundle (`Contents/Resources` or `Contents/MacOS`). May need a small
   path-resolution tweak + the dlsh.zip VFS handling stim2 uses.
3. **OpenCV provenance** — accept per-distro apt versions (variance like stim2's
   ffmpeg matrix), or pin? apt is simplest; recommend accepting variance.
4. **uSockets/uWebSockets** — keep FetchContent or convert to `deps/` submodules
   for full convention parity? (Low priority; FetchContent already reproducible.)
5. **dlsh macOS consumption** — confirm the `.pkg` installs `df.h` + `libdg` where
   CMake finds them (add a hint path if needed), plus the `dlsh.zip` VFS at
   `/usr/local/dlsh/dlsh.zip` like stim2.
