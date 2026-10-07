import { Connection } from "./connection";
import { cameraFpsReader } from "./cameraFps";
import { Renderer } from "./renderer";
import { Stats } from "./stats";
import {
  LAYERS,
  drawEyeTracking,
  drawTuneProposal,
  eyeTrackingBadges,
  pupilViolatesRoi,
  type LayerId,
  type LayerState,
  type TuneProposal,
} from "./overlay/eyetracking";
import { attachRoiEditor, type RoiRect } from "./roiEditor";
import { attachFilePicker } from "./filePicker";
import {
  PLAYBACK_SPEEDS,
  formatPlaybackSpeed,
  nearestPlaybackSpeed,
} from "./playbackSpeed";
import { attachRefPicker } from "./refPicker";
import { attachSourceMenu } from "./sourceMenu";
import { attachDservMenu } from "./dservMenu";
import { attachConsolePanel, type ConsolePanel } from "./consolePanel";
import { settings } from "./settings";
import { attachTuningPanel } from "./tuningPanel";
import { HistoryPlot } from "./history";
import type { EyeTrackingOverlay, PreviewSource } from "./protocol";

const params = new URLSearchParams(location.search);
const fps = Number(params.get("fps")) || 30;
const quality = Number(params.get("quality")) || undefined;

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const connEl = $("conn");
const badgesEl = $("badges");
const fileTag = $("file-tag");
const layersPanel = $("layers-panel");
const layersToggle = $("layers-toggle") as HTMLButtonElement;
const layersBody = $("layers-body");
const sourcePickerWrap = $("source-picker-wrap");
const sourcePickerBtn = $("source-picker") as HTMLButtonElement;
const sourcePickerLabel = sourcePickerBtn.querySelector(".source-picker-label") as HTMLSpanElement;
const speedSelect = $("playback-speed") as HTMLSelectElement;
const pauseBtn = $("transport-pause") as HTMLButtonElement;
const pauseIcon = pauseBtn.querySelector(".transport-pause-icon") as HTMLSpanElement;
const stopBtn = $("transport-stop") as HTMLButtonElement;
const statsEl = $("stats");
const placeholder = $("placeholder");
const overlayCanvas = $("overlay") as HTMLCanvasElement;

const NO_CAMERA_HINT =
  "Camera not found. Connect one or choose a video for playback above.";

let playbackSpeed: number = nearestPlaybackSpeed(settings.num("playback.speed", 1));
// Speed the user just picked; frames still report the old speed until the
// server applies it, so they must not override the selection meanwhile.
let pendingSpeed: number | null = null;
let pendingSpeedTimer: number | undefined;

function baseName(path: string): string {
  const i = Math.max(path.lastIndexOf("/"), path.lastIndexOf("\\"));
  return i >= 0 ? path.slice(i + 1) : path;
}

function playbackFileFromTooltip(tip: string | undefined): string | undefined {
  if (!tip) return undefined;
  const line = tip.split("\n").find((l) => l.startsWith("Recorded file: "));
  const path = line?.slice("Recorded file: ".length).trim();
  return path && path !== "?" ? path : undefined;
}

function withPlaybackFile(src: PreviewSource | undefined): PreviewSource | undefined {
  if (!src || src.type !== "playback" || src.file) return src;
  const file = playbackFileFromTooltip(src.tooltip);
  return file ? { ...src, file } : src;
}

function sourceDisplayLabel(src: PreviewSource | undefined): string {
  if (!src || !src.type) return "Choose source\u2026";
  if (src.type === "playback") return src.file ? baseName(src.file) : "Review";
  return src.label;
}

function sourcePickerTooltip(src: PreviewSource | undefined): string {
  const hint = "Click here to choose another source.";
  if (!src?.type) return `Choose a camera or video file.\n${hint}`;
  let tip = (src.tooltip ?? "Change video source").replace(/\nSpeed:[^\n]*/g, "").trim();
  return `${tip}\n${hint}`;
}

function syncSpeedSelect(): void {
  speedSelect.value = String(playbackSpeed);
}

function setPlaybackSpeedPref(speed: number): void {
  playbackSpeed = nearestPlaybackSpeed(speed);
  settings.set("playback.speed", playbackSpeed); // the server applies it to a playing file too
  pendingSpeed = playbackSpeed;
  window.clearTimeout(pendingSpeedTimer);
  pendingSpeedTimer = window.setTimeout(() => (pendingSpeed = null), 3000);
  syncSpeedSelect();
  updateSourcePickerUi();
}

function adoptServerSpeed(speed: number): void {
  const s = nearestPlaybackSpeed(speed);
  if (pendingSpeed !== null) {
    if (s !== pendingSpeed) return;
    pendingSpeed = null;
    window.clearTimeout(pendingSpeedTimer);
  }
  if (s === playbackSpeed) return;
  playbackSpeed = s;
  syncSpeedSelect();
}

// Another browser changed the speed (or this one just loaded the server's).
settings.on("playback.speed", () => {
  const s = nearestPlaybackSpeed(settings.num("playback.speed", 1));
  if (s === playbackSpeed) return;
  playbackSpeed = s;
  syncSpeedSelect();
  updateSourcePickerUi();
});

// ---- layer toggles (kept by the server, the same in every browser) ----
const LAYER_SETTING = "ui.layers";

function layerDefaultOn(id: LayerId): boolean {
  return id !== "pupil_ellipse" && id !== "reference_ellipse";
}

function migrateLayerPrefs(raw: Record<string, unknown>): Partial<LayerState> {
  const out = { ...raw };
  if ("reference" in out && !("ref_pupil" in out)) {
    const v = out.reference;
    out.ref_pupil = v;
    out.ref_p1 = v;
    out.ref_p4 = v;
    delete out.reference;
  }
  return out as Partial<LayerState>;
}

const layers = Object.fromEntries(LAYERS.map((l) => [l.id, layerDefaultOn(l.id)])) as LayerState;

const layerBoxes = new Map<LayerId, HTMLInputElement>();
const groupBoxes = new Map<"live" | "stored", HTMLInputElement>();
function saveLayers() {
  settings.set(LAYER_SETTING, JSON.stringify(layers));
  renderer.invalidate();
}

// Adopt the server's layer toggles, unless a tune proposal has them switched off for now.
function layersFromSettings(): void {
  if (savedLayers) return;
  const saved = migrateLayerPrefs(settings.json<Record<string, unknown>>(LAYER_SETTING, {}));
  for (const l of LAYERS) {
    const v = saved[l.id];
    const on = typeof v === "boolean" ? v : layerDefaultOn(l.id);
    layers[l.id] = on;
    const box = layerBoxes.get(l.id);
    if (box) box.checked = on;
  }
  for (const group of groupBoxes.keys()) syncGroupCheck(group);
  renderer.invalidate();
}
settings.on(LAYER_SETTING, layersFromSettings);

function groupLayerIds(group: "live" | "stored"): LayerId[] {
  return LAYERS.filter((l) => l.group === group).map((l) => l.id);
}

function syncGroupCheck(group: "live" | "stored"): void {
  const box = groupBoxes.get(group);
  if (!box) return;
  const ids = groupLayerIds(group);
  const on = ids.filter((id) => layers[id]).length;
  box.indeterminate = on > 0 && on < ids.length;
  box.checked = on === ids.length;
}

function setGroupLayers(group: "live" | "stored", on: boolean): void {
  for (const id of groupLayerIds(group)) {
    layers[id] = on;
    const box = layerBoxes.get(id);
    if (box) box.checked = on;
  }
  syncGroupCheck(group);
  saveLayers();
}

function beginTuneProposal(p: TuneProposal): void {
  if (!savedLayers) {
    savedLayers = { ...layers };
    for (const id of Object.keys(layers) as LayerId[]) {
      layers[id] = false;
      const box = layerBoxes.get(id);
      if (box) box.checked = false;
    }
    layersPanel.classList.add("proposal-lock");
    for (const box of layerBoxes.values()) box.disabled = true;
    for (const [group, box] of groupBoxes) {
      box.disabled = true;
      syncGroupCheck(group);
    }
  }
  tuneProposal = p;
  renderer.invalidate();
}

function endTuneProposal(): TuneProposal | null {
  const last = tuneProposal;
  tuneProposal = null;
  if (savedLayers) {
    for (const id of Object.keys(savedLayers) as LayerId[]) {
      layers[id] = savedLayers[id];
      const box = layerBoxes.get(id);
      if (box) {
        box.checked = savedLayers[id];
        box.disabled = false;
      }
    }
    for (const [group, box] of groupBoxes) {
      box.disabled = false;
      syncGroupCheck(group);
    }
    layersPanel.classList.remove("proposal-lock");
    savedLayers = null;
    saveLayers();
  }
  renderer.invalidate();
  return last;
}

function getTuneProposal(): TuneProposal | null {
  return tuneProposal;
}

function appendLayerRow(parent: HTMLElement, l: (typeof LAYERS)[number]) {
  const label = document.createElement("label");
  label.title = l.hint;
  const box = document.createElement("input");
  box.type = "checkbox";
  box.title = l.hint;
  box.checked = layers[l.id];
  layerBoxes.set(l.id, box);
  box.addEventListener("change", () => {
    layers[l.id] = box.checked;
    if (l.group === "live" || l.group === "stored") syncGroupCheck(l.group);
    saveLayers();
  });
  const swatch = document.createElement("span");
  swatch.className = "swatch";
  swatch.style.background = l.color;
  label.append(box, swatch, l.label);
  parent.append(label);
}

function appendGroupHead(
  parent: HTMLElement,
  title: string,
  hint: string,
  group: "live" | "stored",
): void {
  const head = document.createElement("div");
  head.className = "layers-group-head";
  head.title = hint;
  const box = document.createElement("input");
  box.type = "checkbox";
  box.title = hint;
  box.addEventListener("change", () => setGroupLayers(group, box.checked));
  groupBoxes.set(group, box);
  const text = document.createElement("span");
  text.textContent = title;
  head.append(box, text);
  parent.append(head);
  syncGroupCheck(group);
}

appendGroupHead(layersBody, "Live", "Marks from the tracker on the current frame.", "live");
for (const l of LAYERS.filter((x) => x.group === "live")) appendLayerRow(layersBody, l);

const storedGroupEl = document.createElement("div");
storedGroupEl.className = "layers-group layers-group-stored";
appendGroupHead(
  storedGroupEl,
  "Saved tracking",
  "Marks from the loaded saved tracking. Shown during playback.",
  "stored",
);
for (const l of LAYERS.filter((x) => x.group === "stored")) appendLayerRow(storedGroupEl, l);
layersBody.append(storedGroupEl);

const layersDivider = document.createElement("div");
layersDivider.className = "layers-divider";
layersBody.append(layersDivider);
for (const l of LAYERS.filter((x) => x.group === "general")) appendLayerRow(layersBody, l);

function showLayersOpen(open: boolean) {
  layersPanel.classList.toggle("collapsed", !open);
  layersToggle.setAttribute("aria-expanded", String(open));
  layersToggle.title = open ? "Hide layers" : "Show layers";
}
showLayersOpen(settings.flag("ui.layersOpen", true));
settings.on("ui.layersOpen", () => showLayersOpen(settings.flag("ui.layersOpen", true)));
layersToggle.addEventListener("click", () => {
  const open = layersPanel.classList.contains("collapsed");
  showLayersOpen(open);
  settings.set("ui.layersOpen", open);
});

let pendingRoi: RoiRect | null = null;
let tuneProposal: TuneProposal | null = null;
let savedLayers: LayerState | null = null;
let wsConnected = false;
let sourcePaused = false;
let syncLoupe = (): void => {};
let sourceStopped = false;
let pauseEvalPending = false;
let stepEvalPending = false;
let syncHistorySeek: () => void = () => {};
let currentSource: PreviewSource | undefined;

const PLAYBACK_STEP_HINT =
  " \u2190/\u2192 step one frame; Shift \u2190/\u2192 step 100.";

function isPlaybackSource(): boolean {
  return currentSource?.type === "playback";
}

function pauseButtonTitle(): string {
  if (!isPlaybackSource()) {
    return sourcePaused
      ? "Resume playback and capture (Space)"
      : "Pause playback and capture (Space)";
  }
  if (sourcePaused) {
    return `Resume playback (Space).${PLAYBACK_STEP_HINT}`;
  }
  return `Pause playback (Space). \u2190/\u2192 pause and step one frame; Shift \u2190/\u2192 step 100.`;
}

function keyboardShortcutBlocked(ev: KeyboardEvent): boolean {
  if (sourceMenu.isOpen()) return true;
  const tag = (ev.target as HTMLElement | null)?.tagName;
  if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return true;
  if (tag === "BUTTON" && ev.target !== pauseBtn && ev.target !== stopBtn) return true;
  return false;
}

function updateSourcePickerUi(): void {
  const enabled = wsConnected;
  sourcePickerBtn.disabled = !enabled;
  const playback = isPlaybackSource();
  speedSelect.hidden = !playback;
  speedSelect.disabled = !enabled || !playback;
  const hasSource = Boolean(currentSource?.type);
  pauseBtn.disabled = !enabled || (sourceStopped && !hasSource);
  stopBtn.disabled = !enabled || sourceStopped || !hasSource;
  const label = enabled ? sourceDisplayLabel(currentSource) : "—";
  sourcePickerLabel.textContent = label;
  sourcePickerBtn.title = enabled ? sourcePickerTooltip(currentSource) : "Choose a camera or video file";
  const showPlay = sourceStopped || sourcePaused;
  pauseIcon.textContent = showPlay ? "\u25B6" : "\u23F8";
  pauseBtn.dataset.paused = showPlay ? "1" : "0";
  pauseBtn.title = sourceStopped ? "Open this camera or video again (Space)" : pauseButtonTitle();
  pauseBtn.setAttribute("aria-label", showPlay ? "Play" : "Pause");
  stopBtn.title = "Stop the camera, or close the video file";
  tuningPanel.setSourceState(hasSource && !sourceStopped, sourcePaused);
  syncLoupe();
  storedGroupEl.hidden = !isPlaybackSource();
  sourceMenu.syncPlayback();
  syncHistorySeek();
}

function showStoppedBadge(): void {
  const s = document.createElement("span");
  s.className = "badge badge-warn";
  s.textContent = "Stopped";
  s.title = "The camera is off, or the video file is closed. Press play to open it again.";
  badgesEl.replaceChildren(s);
  lastBadges = "";
}

async function stopActive(): Promise<void> {
  if (!wsConnected || sourceStopped || !currentSource?.type) return;
  try {
    await conn.sendEvalAsync("stop_active_source");
    sourceStopped = true;
    sourcePaused = false;
    pauseEvalPending = false;
    showStoppedBadge();
    updateSourcePickerUi();
  } catch (msg) {
    console.warn("stop failed:", msg);
  }
}

async function resumeStopped(): Promise<void> {
  if (!wsConnected || !sourceStopped) return;
  sourceSwitching = true;
  sourcePickerLabel.textContent = "Connecting\u2026";
  placeholder.textContent = "Connecting\u2026";
  placeholder.style.display = "";
  try {
    await conn.sendEvalAsync("resume_stopped_source");
    sourceStopped = false;
    sourcePaused = false;
    sourceSwitching = false;
    lastSourceKey = "";
    updateSourcePickerUi();
  } catch (msg) {
    sourceSwitching = false;
    placeholder.style.display = "none";
    updateSourcePickerUi();
    console.warn("resume failed:", msg);
  }
}

function togglePause(): void {
  if (!wsConnected) return;
  if (sourceStopped) {
    void resumeStopped();
    return;
  }
  const next = !sourcePaused;
  pauseEvalPending = true;
  sourcePaused = next;
  updateSourcePickerUi();
  conn.sendEval(`vstream::pause ${next ? 1 : 0}`);
}

pauseBtn.addEventListener("click", () => togglePause());
stopBtn.addEventListener("click", () => void stopActive());

async function ensurePausedForStep(): Promise<boolean> {
  if (sourcePaused) return true;
  pauseEvalPending = true;
  sourcePaused = true;
  updateSourcePickerUi();
  try {
    await conn.sendEvalAsync("vstream::pause 1");
    pauseEvalPending = false;
    return true;
  } catch {
    sourcePaused = false;
    pauseEvalPending = false;
    updateSourcePickerUi();
    return false;
  }
}

async function stepPlayback(delta: number): Promise<void> {
  if (!wsConnected || sourceStopped || !isPlaybackSource() || stepEvalPending) return;
  // Stored rows are one per video frame in order; stepping would break that.
  if (refPicker.runState() !== "idle") return;
  stepEvalPending = true;
  try {
    if (!(await ensurePausedForStep())) return;
    if (delta < 0 || Math.abs(delta) > 1) {
      await conn.sendEvalAsync("eyetracking::resetTrackingState");
    }
    await conn.sendEvalAsync(`vstream::step ${delta}`);
  } catch (msg) {
    console.warn("step failed:", msg);
  } finally {
    stepEvalPending = false;
  }
}

async function seekPlayback(frame: number, alreadyThere: boolean): Promise<void> {
  if (!wsConnected || sourceStopped || !isPlaybackSource() || stepEvalPending) return;
  if (refPicker.runState() !== "idle") return;
  if (alreadyThere && sourcePaused) return;
  const target = Math.max(0, Math.round(frame));
  stepEvalPending = true;
  try {
    if (!(await ensurePausedForStep())) return;
    if (alreadyThere) return;
    await conn.sendEvalAsync("eyetracking::resetTrackingState");
    await conn.sendEvalAsync(`vstream::seek ${target}`);
  } catch (msg) {
    console.warn("seek failed:", msg);
  } finally {
    stepEvalPending = false;
  }
}

// True after the server says this browser is on the same machine. Latency uses
// both clocks, so it stays hidden until that welcome arrives.
let clientOnServer = false;

// ---- connection (created early for modal) ----
let consolePanel: ConsolePanel | null = null;
const conn = new Connection({
  fps,
  quality,
  onEvent: (event, data) => {
    settings.handleEvent(event, data);
    dservMenu.handleEvent(event);
  },
  onLog: (lines) => consolePanel?.append(lines),
  onLogReset: () => consolePanel?.reset(),
  onFrame: (msg) => {
    stats.frameReceived(msg);
    renderer.submit(msg);
  },
  onWelcome: (local) => {
    clientOnServer = local;
    renderStats();
  },
  onStatus: (up, url) => {
    wsConnected = up;
    if (!up) {
      fileTag.hidden = true;
      clientOnServer = false;
      renderStats();
      sourcePaused = false;
      sourceStopped = false;
      pauseEvalPending = false;
      currentSource = undefined;
      sourceMenu.close();
    }
    updateSourcePickerUi();
    connEl.classList.toggle("conn-up", up);
    connEl.classList.toggle("conn-down", !up);
    connEl.textContent = up ? "VideoStream connected" : "VideoStream not found";
    connEl.title = url;
    if (!up) {
      stats.resetSeq();
      pendingRoi = null;
      placeholder.style.display = "";
      placeholder.textContent = `connecting to ${url}\u2026`;
      lastSourceKey = "";
    } else {
      placeholder.textContent = "waiting for frames\u2026";
      void showNoCameraHintIfNeeded();
      void conn
        .sendEvalAsync("vstream::pause")
        .then((v) => {
          if (pauseEvalPending) return;
          sourcePaused = v.trim() === "1";
          updateSourcePickerUi();
        })
        .catch(() => {});
    }
    tuningPanel.setEnabled(up);
    dservMenu.setEnabled(up);
    if (up) {
      // The server's settings first: every panel below adopts them.
      void settings.load();
      void tuningPanel.onConnected();
      void sourceMenu.refreshCameras();
      void refPicker.refresh();
    }
  },
  onEvalOk: () => {
    pauseEvalPending = false;
  },
  onEvalError: (msg) => {
    if (pauseEvalPending) {
      sourcePaused = !sourcePaused;
      pauseEvalPending = false;
      updateSourcePickerUi();
    }
    console.warn("eval failed:", msg);
  },
});
settings.bind(conn);

for (const speed of PLAYBACK_SPEEDS) {
  const o = document.createElement("option");
  o.value = String(speed);
  o.textContent = formatPlaybackSpeed(speed);
  speedSelect.append(o);
}
syncSpeedSelect();
speedSelect.addEventListener("change", () => {
  const speed = nearestPlaybackSpeed(Number(speedSelect.value));
  setPlaybackSpeedPref(speed);
});

const refErrorEl = $("ref-error");
let sourceMenu!: ReturnType<typeof attachSourceMenu>;
const refPicker = attachRefPicker({
  conn,
  row: $("ref-row"),
  refSection: $("source-menu-saved"),
  refList: $("source-menu-ref-list"),
  choiceEl: $("source-menu-saved-current"),
  reprocessBtn: $("ref-reprocess") as HTMLButtonElement,
  saveBtn: $("ref-save") as HTMLButtonElement,
  showError: (msg) => {
    refErrorEl.textContent = msg;
    refErrorEl.title = msg;
    refErrorEl.hidden = !msg;
  },
  onMenuUpdate: () => refPicker.renderMenu(),
  onRefPicked: () => sourceMenu.close(),
});

const tuningPanel = attachTuningPanel(
  conn,
  $("tuning-panel"),
  $("tuning-toggle") as HTMLButtonElement,
  $("tuning-body"),
  {
    setPaused: (paused) => {
      sourcePaused = paused;
      updateSourcePickerUi();
    },
    setPendingRoi: (roi) => {
      pendingRoi = roi;
      renderer.invalidate();
    },
    beginTuneProposal,
    endTuneProposal,
    getTuneProposal,
  },
);

const dservMenu = attachDservMenu({
  conn,
  settings,
  wrap: $("dserv-wrap"),
  button: $("dserv-picker") as HTMLButtonElement,
  label: $("dserv-label"),
  menu: $("dserv-menu"),
  foundList: $("dserv-menu-found"),
  refreshBtn: $("dserv-menu-refresh") as HTMLButtonElement,
  manualForm: $("dserv-menu-manual") as HTMLFormElement,
  hostInput: $("dserv-menu-host") as HTMLInputElement,
  portInput: $("dserv-menu-port") as HTMLInputElement,
  currentSection: $("dserv-menu-current"),
  statusEl: $("dserv-menu-status"),
  disconnectBtn: $("dserv-menu-disconnect") as HTMLButtonElement,
  errorEl: $("dserv-menu-error"),
});

const filePicker = attachFilePicker(conn);
sourceMenu = attachSourceMenu({
  conn,
  filePicker,
  refPicker,
  wrap: sourcePickerWrap,
  menu: $("source-menu"),
  anchorBtn: sourcePickerBtn,
  cameraList: $("source-menu-cameras"),
  refreshBtn: $("source-menu-refresh") as HTMLButtonElement,
  currentFileBtn: $("source-menu-current-file") as HTMLButtonElement,
  openFileBtn: $("source-menu-open-file") as HTMLButtonElement,
  savedWrap: $("source-menu-saved"),
  savedToggle: $("source-menu-saved-toggle") as HTMLButtonElement,
  savedPanel: $("source-menu-saved-panel"),
  errorEl: $("source-menu-error"),
  getPlaybackSpeed: () => playbackSpeed,
  getCurrentSource: () => currentSource,
  onSwitchStart: () => {
    sourceStopped = false;
    sourceSwitching = true;
    sourcePickerLabel.textContent = "Connecting\u2026";
    placeholder.textContent = "Connecting\u2026";
    placeholder.style.display = "";
  },
  onSwitchDone: (src) => {
    sourceSwitching = false;
    sourceStopped = false;
    sourcePaused = false;
    pendingRoi = null;
    lastSourceKey = "";
    currentSource = src;
    updateSourcePickerUi();
    void refPicker.refresh();
  },
  onSwitchError: () => {
    sourceSwitching = false;
    placeholder.textContent = "waiting for frames\u2026";
    updateSourcePickerUi();
  },
});

sourcePickerBtn.addEventListener("click", (ev) => {
  ev.stopPropagation();
  sourceMenu.toggle();
});

window.addEventListener("keydown", (ev) => {
  if (keyboardShortcutBlocked(ev)) return;

  if (ev.code === "Space" || ev.key === " ") {
    ev.preventDefault();
    togglePause();
    return;
  }

  if (sourceStopped || !isPlaybackSource()) return;
  if (ev.key !== "ArrowLeft" && ev.key !== "ArrowRight") return;
  ev.preventDefault();
  const mag = ev.shiftKey ? 100 : 1;
  const delta = ev.key === "ArrowRight" ? mag : -mag;
  void stepPlayback(delta);
});

// ---- rendering ----
const stats = new Stats();
const history = new HistoryPlot(
  $("history"),
  $("history-toggle") as HTMLButtonElement,
  $("history-canvas") as HTMLCanvasElement,
  $("history-window") as HTMLSelectElement,
  (frame, alreadyThere) => {
    void seekPlayback(frame, alreadyThere);
  },
);
syncHistorySeek = () => {
  history.setSeekEnabled(wsConnected && isPlaybackSource() && !sourceStopped);
};
const renderer = new Renderer($("stage"), $("video"), overlayCanvas, (ctx, header, view) => {
  if (tuneProposal) {
    drawTuneProposal(ctx, tuneProposal, view);
    return;
  }
  const et = header.overlay.eye_tracking;
  if (!et) return;
  const roi = pendingRoi ?? et.roi;
  let draw: EyeTrackingOverlay = et;
  if (roi && (pendingRoi || et.roi)) {
    const localViolation = pupilViolatesRoi(et.pupil, roi);
    draw = { ...et, roi, roi_violation: et.roi_violation || localViolation };
  }
  drawEyeTracking(ctx, draw, layers, view, isPlaybackSource());
});

let lastBadges = "";
let lastSourceKey = "";
let lastCameraKey = "";
let sourceSwitching = false;

renderer.onDisplayed = (header) => {
  stats.frameDisplayed(header);
  // Stop closes the file, then one more preview can arrive with no source.
  // Leave the menu and the Stopped badge on the video Play will reopen.
  if (sourceStopped) return;
  if (!sourceSwitching) placeholder.style.display = "none";

  const src = withPlaybackFile(header.source);
  if (src?.type === "playback" && src.speed !== undefined) adoptServerSpeed(src.speed);
  const displayLabel = sourceDisplayLabel(src);
  const sourceKey = src ? `${displayLabel}\0${src.file ?? ""}\0${src.tooltip ?? ""}` : "";
  if (!sourceSwitching && src?.type && sourceKey !== lastSourceKey) {
    lastSourceKey = sourceKey;
    currentSource = src;
    updateSourcePickerUi();
    void refPicker.refresh();
  }
  const ck = src?.camera_key ?? "";
  if (ck !== lastCameraKey) {
    lastCameraKey = ck;
    void tuningPanel.setCamera(ck || undefined);
  }
  tuningPanel.setSourceInfo({ width: header.width, height: header.height, fps: header.src_fps });
  history.push(header.overlay.eye_tracking, header.src_fps, sourceKey);

  const badgeRoi = pendingRoi ?? header.overlay.eye_tracking?.roi;
  tuningPanel.setRoi(badgeRoi);
  const badges = eyeTrackingBadges(header.overlay.eye_tracking, badgeRoi);
  if (header.in_obs) badges.push({ text: "in obs", kind: "ok" });
  const runBadge = refPicker.badge();
  if (runBadge) badges.unshift(runBadge);
  const datafile = header.datafile ?? "";
  if (fileTag.hidden === (datafile !== "")) {
    fileTag.hidden = datafile === "";
  }
  if (datafile !== "" && fileTag.title !== `Datafile open on the dataserver: ${datafile}`) {
    fileTag.title = `Datafile open on the dataserver: ${datafile}`;
  }
  const key = JSON.stringify(badges);
  if (key !== lastBadges) {
    lastBadges = key;
    badgesEl.replaceChildren(
      ...badges.map((b) => {
        const s = document.createElement("span");
        s.className = `badge badge-${b.kind}`;
        s.textContent = b.text;
        if (b.title) s.title = b.title;
        return s;
      }),
    );
  }

  if (pendingRoi && etRoiMatches(header.overlay.eye_tracking?.roi, pendingRoi)) {
    pendingRoi = null;
  }
};

function etRoiMatches(
  server: { x: number; y: number; w: number; h: number } | undefined,
  local: RoiRect,
): boolean {
  if (!server) return false;
  return (
    Math.abs(server.x - local.x) < 2 &&
    Math.abs(server.y - local.y) < 2 &&
    Math.abs(server.w - local.w) < 2 &&
    Math.abs(server.h - local.h) < 2
  );
}

async function showNoCameraHintIfNeeded(): Promise<void> {
  try {
    const sources = await conn.fetchSources();
    if (sources.source_active) return;
    if (sources.cameras.length === 0) {
      placeholder.textContent = NO_CAMERA_HINT;
      placeholder.style.display = "";
    }
  } catch {
    // ignore; connection may still be settling
  }
}

conn.start();
updateSourcePickerUi();

consolePanel = attachConsolePanel(conn, $("console-toggle") as HTMLButtonElement, statsEl);

attachRoiEditor(
  overlayCanvas,
  () => renderer.getState(),
  () => {
    if (tuneProposal) return null;
    const et = renderer.getState().header?.overlay.eye_tracking;
    return pendingRoi ?? et?.roi ?? null;
  },
  (roi) => {
    pendingRoi = roi;
    renderer.invalidate();
  },
);

const PROPOSAL_HIT_PX = 12;
let proposalDrag: "p1" | "p4" | null = null;

function framePoint(ev: { clientX: number; clientY: number }): { x: number; y: number } | null {
  const { header, view } = renderer.getState();
  if (!header) return null;
  const rect = overlayCanvas.getBoundingClientRect();
  return {
    x: (ev.clientX - rect.left - view.offsetX) / view.scale,
    y: (ev.clientY - rect.top - view.offsetY) / view.scale,
  };
}

function hitProposalMark(ev: { clientX: number; clientY: number }): "p1" | "p4" | null {
  const p = tuneProposal;
  const pt = framePoint(ev);
  const { view } = renderer.getState();
  if (!p || !pt || view.scale <= 0) return null;
  const tol = PROPOSAL_HIT_PX / view.scale;
  const near = (m: { x: number; y: number }) => Math.hypot(pt.x - m.x, pt.y - m.y) <= tol;
  const onP1 = p.p1 ? near(p.p1) : false;
  const onP4 = near(p.p4);
  if (onP1 && onP4 && p.p1) {
    const d1 = Math.hypot(pt.x - p.p1.x, pt.y - p.p1.y);
    const d4 = Math.hypot(pt.x - p.p4.x, pt.y - p.p4.y);
    return d1 <= d4 ? "p1" : "p4";
  }
  if (onP1) return "p1";
  if (onP4) return "p4";
  return null;
}

function clampDragged(which: "p1" | "p4", x: number, y: number): { x: number; y: number } {
  const p = tuneProposal!;
  if (which === "p4") {
    const dx = x - p.pupil.x;
    const dy = y - p.pupil.y;
    const maxR = p.pupil.r;
    const mag = Math.hypot(dx, dy);
    if (mag > maxR && mag > 0) {
      return { x: p.pupil.x + (dx / mag) * maxR, y: p.pupil.y + (dy / mag) * maxR };
    }
    return { x, y };
  }
  const { roi } = p;
  return {
    x: Math.min(roi.x + roi.w, Math.max(roi.x, x)),
    y: Math.min(roi.y + roi.h, Math.max(roi.y, y)),
  };
}

overlayCanvas.addEventListener("pointerdown", (ev) => {
  if (!tuneProposal || ev.button !== 0) return;
  const hit = hitProposalMark(ev);
  if (!hit) return;
  proposalDrag = hit;
  overlayCanvas.setPointerCapture(ev.pointerId);
  if (!sourcePaused) overlayCanvas.style.cursor = "grabbing";
  ev.preventDefault();
});

overlayCanvas.addEventListener("pointermove", (ev) => {
  if (!tuneProposal) return;
  if (proposalDrag) {
    const pt = framePoint(ev);
    if (!pt) return;
    const next = clampDragged(proposalDrag, pt.x, pt.y);
    if (proposalDrag === "p4") tuneProposal.p4 = next;
    else if (tuneProposal.p1) tuneProposal.p1 = next;
    if (!sourcePaused) overlayCanvas.style.cursor = "grabbing";
    renderer.invalidate();
    return;
  }
  if (!sourcePaused) overlayCanvas.style.cursor = hitProposalMark(ev) ? "grab" : "";
});

overlayCanvas.addEventListener("pointerup", (ev) => {
  if (!proposalDrag) return;
  proposalDrag = null;
  if (overlayCanvas.hasPointerCapture(ev.pointerId)) overlayCanvas.releasePointerCapture(ev.pointerId);
  if (!sourcePaused) overlayCanvas.style.cursor = tuneProposal && hitProposalMark(ev) ? "grab" : "";
});

overlayCanvas.addEventListener("pointercancel", () => {
  proposalDrag = null;
});

// Paused feed: the pointer is a magnifier, so the grab hand never covers the
// pixel being placed. Registered after the ROI and proposal handlers so it
// wins the cursor while it is showing.
let loupeClient: { x: number; y: number } | null = null;

// The loupe stays away from the ROI edge (in frame pixels) so dragging the
// ROI's edges and corners is not done through a magnifier.
const LOUPE_ROI_INSET = 20;

function loupeAllowedAt(x: number, y: number): boolean {
  if (tuneProposal) return true; // the ROI is not editable while a proposal is shown
  const roi = pendingRoi ?? renderer.getState().header?.overlay.eye_tracking?.roi;
  if (!roi) return true;
  return (
    x >= roi.x + LOUPE_ROI_INSET &&
    x <= roi.x + roi.w - LOUPE_ROI_INSET &&
    y >= roi.y + LOUPE_ROI_INSET &&
    y <= roi.y + roi.h - LOUPE_ROI_INSET
  );
}

function loupeFramePoint(): { x: number; y: number } | null {
  if (!sourcePaused || !loupeClient) return null;
  const { header, view } = renderer.getState();
  if (!header || view.scale <= 0) return null;
  const rect = overlayCanvas.getBoundingClientRect();
  const x = (loupeClient.x - rect.left - view.offsetX) / view.scale;
  const y = (loupeClient.y - rect.top - view.offsetY) / view.scale;
  if (x < 0 || y < 0 || x >= header.width || y >= header.height) return null;
  if (!loupeAllowedAt(x, y)) return null;
  return { x, y };
}

// Shown above the loupe when a proposed P1/P4 mark is under the pointer.
// P1 is the center of the bright blob and P4 the weighted center of its bright
// spot, not the single brightest pixel (see refineP1SubPixel / refineP4SubPixelWeighted).
const LOUPE_DRAG_HINT = "click+drag to center of mass";

// The cursor is hidden by a class whose rule is !important, not by writing
// style.cursor: the ROI and proposal handlers reset the inline cursor on every
// pointer move, and a loupe that writes it back afterwards leaves the canvas
// showing the arrow for the instants in between.
syncLoupe = () => {
  const frame = loupeFramePoint();
  const canDrag =
    frame !== null &&
    loupeClient !== null &&
    (proposalDrag !== null || hitProposalMark({ clientX: loupeClient.x, clientY: loupeClient.y }) !== null);
  renderer.setLoupe(frame, canDrag ? LOUPE_DRAG_HINT : null);
  overlayCanvas.classList.toggle("loupe-active", frame !== null);
};

function trackLoupePointer(ev: PointerEvent): void {
  loupeClient = { x: ev.clientX, y: ev.clientY };
  syncLoupe();
}

overlayCanvas.addEventListener("pointerdown", trackLoupePointer);
overlayCanvas.addEventListener("pointermove", trackLoupePointer);
overlayCanvas.addEventListener("pointerup", trackLoupePointer);
overlayCanvas.addEventListener("pointerleave", () => {
  loupeClient = null;
  syncLoupe();
});

// ---- status bar ----
// Fields stay in the DOM. Rewriting innerHTML while the pointer is over a
// field destroys the element the browser is waiting on, so the tooltip never
// appears until the pointer leaves and comes back.
function ensureGroup(id: string, label: string, tip: string): HTMLElement {
  let group = statsEl.querySelector(`[data-group="${id}"]`) as HTMLElement | null;
  if (!group) {
    group = document.createElement("span");
    group.className = "stats-group";
    group.dataset.group = id;
    const lab = document.createElement("span");
    lab.className = "stats-group-label";
    group.append(lab);
    statsEl.append(group);
  }
  const lab = group.querySelector(".stats-group-label") as HTMLElement;
  if (lab.textContent !== label) lab.textContent = label;
  if (lab.title !== tip) lab.title = tip;
  return group;
}

function setField(
  group: HTMLElement,
  id: string,
  name: string,
  value: string,
  tip: string,
  visible = true,
): void {
  let el = group.querySelector(`[data-field="${id}"]`) as HTMLElement | null;
  if (!el) {
    el = document.createElement("span");
    el.dataset.field = id;
    el.title = tip;
    const n = document.createElement("span");
    n.textContent = name;
    const b = document.createElement("b");
    el.append(n, document.createTextNode(" "), b);
    group.append(el);
  }
  el.hidden = !visible;
  if (!visible) return;
  const b = el.querySelector("b") as HTMLElement;
  if (b.textContent !== value) b.textContent = value;
}

function formatKb(kb: number): string {
  if (kb >= 1024 * 1024) return `${(kb / (1024 * 1024)).toFixed(1)} GB`;
  if (kb >= 1024) return `${Math.round(kb / 1024)} MB`;
  return `${kb} KB`;
}

let cameraFps: number | null = null;
let linkBps: number | null = null;
let linkMissing = false;
let linkCameraKey = "";
let cameraPollBusy = false;
const statsFps = cameraFpsReader(conn);

async function pollCameraPath(): Promise<void> {
  const key = currentSource?.camera_key;
  if (!key || cameraPollBusy) return;
  if (key !== linkCameraKey) {
    linkCameraKey = key;
    linkMissing = false;
    linkBps = null;
    cameraFps = null;
    statsFps.reset();
  }
  cameraPollBusy = true;
  try {
    try {
      const fps = await statsFps.read();
      if (currentSource?.camera_key === key && Number.isFinite(fps)) cameraFps = fps;
    } catch {
      /* leave the last reading */
    }
    if (!linkMissing && currentSource?.camera_key === key) {
      try {
        const bps = Number(await conn.sendEvalAsync("camera::node DeviceLinkCurrentThroughput"));
        if (currentSource?.camera_key !== key) return;
        if (Number.isFinite(bps)) linkBps = bps;
        else {
          linkMissing = true;
          linkBps = null;
        }
      } catch {
        if (currentSource?.camera_key === key) {
          linkMissing = true;
          linkBps = null;
        }
      }
    }
  } finally {
    cameraPollBusy = false;
    renderStats();
  }
}

function renderStats(): void {
  const h = stats.last;
  const camera = Boolean(currentSource?.camera_key);
  const playback = isPlaybackSource();
  const mbps = linkBps === null ? null : (linkBps * 8) / 1e6;
  const cpu =
    h?.proc_cpu === undefined || h.host_cpu === undefined
      ? "\u2013"
      : `${Math.round(h.proc_cpu)}% · ${Math.round(h.host_cpu)}%`;
  const mem =
    h?.rss_kb === undefined || h.mem_avail_kb === undefined
      ? "\u2013"
      : `${formatKb(h.rss_kb)} · ${formatKb(h.mem_avail_kb)} free`;

  const source = ensureGroup(
    "source",
    "Server",
    "On the server: the camera link, frames kept here, and how loaded this process is.",
  );
  source.hidden = false;
  setField(
    source,
    "rate",
    "rate",
    cameraFps === null ? "\u2013" : `${cameraFps.toFixed(1)} fps`,
    "Pictures per second the camera is delivering to the server.",
    camera,
  );
  setField(
    source,
    "track",
    "track",
    stats.trackMs === null || stats.trackMaxMs === null
      ? "\u2013"
      : `${stats.trackMs.toFixed(1)} ms (max ${stats.trackMaxMs.toFixed(1)})`,
    "Time to find the pupil, P1 and P4 in one frame, averaged over the last second, with the slowest frame in brackets. 1000 divided by this is the highest frame rate tracking can keep up with: 2 ms is about 500 fps.",
    stats.trackMs !== null,
  );
  setField(
    source,
    "link",
    "link",
    mbps === null ? "\u2013" : `${mbps.toFixed(1)} Mbps`,
    "Megabits per second on the camera link.",
    camera && !linkMissing,
  );
  setField(
    source,
    "cam-missed",
    "missed",
    h?.incomplete_frames === undefined ? "\u2013" : String(h.incomplete_frames),
    "Frames the camera sent that arrived incomplete and were discarded before the server stored them. These are the incomplete-image lines in the server log.",
    camera && Boolean(h),
  );
  setField(
    source,
    "file",
    "file",
    h ? String(h.video_frame) : "\u2013",
    "Frame index in the video file.",
    !camera && playback && Boolean(h),
  );
  setField(
    source,
    "cpu",
    "cpu",
    cpu,
    "VideoStream first, then the whole machine. 100% is one core, so capture, tracking, and encoding together can read above 100%. The machine figure is how busy all cores are.",
  );
  setField(
    source,
    "mem",
    "mem",
    mem,
    "Memory held by VideoStream, then memory still available on the machine.",
  );

  const preview = ensureGroup("preview", "Client", "In this browser.");
  setField(
    preview,
    "display",
    "display",
    `${stats.displayFps.toFixed(1)} fps`,
    "Preview pictures painted per second. This is the stream to the page.",
  );
  setField(
    preview,
    "dropped",
    "dropped here",
    String(renderer.skipped),
    "Preview frames discarded in the browser because the previous image was still decoding.",
  );
  setField(
    preview,
    "missed",
    "missed",
    String(stats.serverGaps),
    "Preview frames the server encoded that this page never received.",
  );
  setField(
    preview,
    "encode",
    "encode",
    h ? `${h.encode_ms.toFixed(1)} ms` : "\u2013",
    "Time to compress the preview JPEG, including the wait for this frame's tracking marks.",
    Boolean(h),
  );
  setField(
    preview,
    "lag",
    "overlay lag",
    stats.lag === null ? "\u2013" : `${stats.lag} fr`,
    "Camera frames the tracking marks trail the picture. Zero means the marks belong to the picture on screen.",
    Boolean(h),
  );
  setField(
    preview,
    "latency",
    "latency",
    stats.latencyMs === null ? "\u2013" : `~${Math.round(stats.latencyMs)} ms`,
    "Time from the server handing off the preview until this page shows it.",
    Boolean(h) && clientOnServer,
  );
}

setInterval(() => {
  if (!stats.tick()) return;
  if (currentSource?.camera_key) void pollCameraPath();
  else {
    cameraFps = null;
    linkBps = null;
    linkCameraKey = "";
    linkMissing = false;
  }
  renderStats();
}, 250);
