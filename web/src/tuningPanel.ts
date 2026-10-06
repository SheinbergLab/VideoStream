import type { Connection } from "./connection";
import { cameraFpsReader } from "./cameraFps";
import type { TuneProposal } from "./overlay/eyetracking";
import { settings } from "./settings";
import { parseTclDictNested, parseTclList } from "./tcl";

interface NumControl {
  key: string;
  label: string;
  min: number;
  max: number;
  step: number;
  unit?: string;
  hint?: string;
}

interface TargetSection {
  id: "pupil" | "p1" | "p4";
  title: string;
  controls: NumControl[];
}

const PUPIL_CONTROLS: NumControl[] = [
  {
    key: "pupil_threshold",
    label: "Threshold",
    min: 0,
    max: 255,
    step: 1,
    hint: "Pixels darker than this inside the ROI are pupil candidates.",
  },
];

const P1_CONTROLS: NumControl[] = [
  {
    key: "p1_min_intensity",
    label: "Min intensity",
    min: 0,
    max: 255,
    step: 1,
    hint: "Minimum brightness for the first Purkinje glint.",
  },
  {
    key: "p1_max_jump",
    label: "Max jump",
    min: 1,
    max: 150,
    step: 0.5,
    unit: "px",
    hint: "Largest frame-to-frame P1 movement accepted.",
  },
  {
    key: "p1_min_area",
    label: "Min area",
    min: 1,
    max: 400,
    step: 1,
    unit: "px\u00b2",
  },
  {
    key: "p1_max_area",
    label: "Max area",
    min: 50,
    max: 3000,
    step: 10,
    unit: "px\u00b2",
  },
  {
    key: "p1_pupil_radius_max",
    label: "Pupil radius max",
    min: 0.5,
    max: 3,
    step: 0.05,
    unit: "\u00d7",
    hint: "P1 must lie within this multiple of the pupil radius.",
  },
];

const P4_CONTROLS: NumControl[] = [
  {
    key: "p4_min_intensity",
    label: "Min intensity",
    min: 0,
    max: 255,
    step: 1,
    hint: "Minimum brightness for the fourth Purkinje reflection.",
  },
  {
    key: "p4_max_jump",
    label: "Max jump",
    min: 1,
    max: 150,
    step: 0.5,
    unit: "px",
    hint: "Largest frame-to-frame P4 movement accepted.",
  },
  {
    key: "p4_max_prediction_error",
    label: "Max prediction error",
    min: 1,
    max: 80,
    step: 0.5,
    unit: "px",
    hint: "How far P4 may sit from the model's predicted position.",
  },
];

const TARGET_SECTIONS: TargetSection[] = [
  { id: "pupil", title: "Pupil", controls: PUPIL_CONTROLS },
  { id: "p1", title: "P1", controls: P1_CONTROLS },
  { id: "p4", title: "P4", controls: P4_CONTROLS },
];

const ALL_CONTROLS = TARGET_SECTIONS.flatMap((s) => s.controls);

const MODES = [
  {
    id: "pupil_only",
    label: "Pupil",
    hint: "Find the pupil only.",
  },
  {
    id: "pupil_p1",
    label: "Pupil + P1",
    hint: "Find the pupil and P1, the first Purkinje glint on the cornea.",
  },
  {
    id: "full",
    label: "Pupil + P1 + P4",
    hint: "Find the pupil, P1, and P4, the fourth Purkinje reflection inside the pupil. P4 is searched when P1 is found.",
  },
] as const;

const AUTO_ROI_TITLE =
  "Slowly shift the ROI toward the pupil when the pupil leaves it (moves only; size stays the same).";
const ROI_MIN = 32;

const ROI_W_CONTROL: NumControl = {
  key: "roi_w",
  label: "Width",
  min: ROI_MIN,
  max: 9999,
  step: 1,
  unit: "px",
};

const ROI_H_CONTROL: NumControl = {
  key: "roi_h",
  label: "Height",
  min: ROI_MIN,
  max: 9999,
  step: 1,
  unit: "px",
};

const ROI_FOLLOW = [
  { label: "Fixed", follow: false },
  { label: "Follow pupil", follow: true },
] as const;
const SEND_INTERVAL_MS = 100;

const GAIN_CONTROL: NumControl = {
  key: "camera_gain",
  label: "Gain",
  min: 0,
  max: 48,
  step: 0.1,
  unit: "dB",
  hint: "Camera gain in dB (FLIR and Lucid).",
};

const FPS_CONTROL: NumControl = {
  key: "camera_fps",
  label: "Frame rate",
  min: 1,
  max: 250,
  step: 1,
  unit: "fps",
  hint: "Frames per second the camera is asked to take. It cannot start the next frame until the exposure finishes, so a long exposure delivers fewer pictures than this setting. The upper end is whatever the camera reports, and it rises with a shorter exposure or 2\u00d72 binning.",
};

const BINNING = [
  { label: "1\u00d71", h: 1, v: 1 },
  { label: "2\u00d72", h: 2, v: 2 },
] as const;

const BINNING_HINT =
  "Pixel binning needs less light, because several sensor pixels feed each output pixel. It combines neighboring pixels into one. 2\u00d72 merges each 2\u00d72 block, so the picture is half as wide and half as tall, and the camera can run faster.";

const EXPOSURE_CONTROL: NumControl = {
  key: "camera_exposure",
  label: "Exposure",
  min: 1,
  max: 99,
  step: 0.1,
  unit: "%",
  hint: "Percent of the requested frame spent exposing. At 100 fps, 10% is 1000 \u00b5s. Auto exposure stays off.",
};

// Everything below that changes a tracking or camera setting goes through
// settings.put, so the server applies it, keeps it for the next start, and every
// other browser sees it. Detector values are saved as det.<key>, the ROI as
// roi, and camera values per backend as cam.<vendor>.<field>.
const detKey = (key: string) => `det.${key}`;
const vendorOf = (cameraKey: string) => cameraKey.split(":")[0];

export interface TuningPanel {
  onConnected: () => Promise<void>;
  setEnabled: (enabled: boolean) => void;
  setSourceState: (active: boolean, paused: boolean) => void;
  setCamera: (cameraKey: string | undefined) => Promise<void>;
  setSourceInfo: (info: { width?: number; height?: number; fps?: number }) => void;
  setRoi: (roi: RoiBox | undefined) => void;
}

type RoiBox = { x: number; y: number; w: number; h: number };

interface QualityCheckRow {
  id: string;
  level: string;
  message: string;
}

/** Settings auto-detect may change (tracking limits and P1 radius stay put). */
const AUTO_KEYS = [
  "pupil_threshold",
  "p1_min_intensity",
  "p4_min_intensity",
  "p1_min_area",
  "p1_max_area",
] as const;

const AUTO_LABEL = "Auto-tune from paused frame";
const AUTO_HINT_PLAYING =
  "Have the subject look straight ahead, then pause the video to enable auto-tune.";
const AUTO_HINT_READY =
  "Find the ROI, pupil threshold, P1/P4 intensities and P1 size gate from the paused frame.";

const ACCEPT_TITLE = "Dismiss this report and keep the applied settings.";

/** Below this pupil-to-P1 distance a one-sample P4 model is too ill-conditioned. */
const P4_CALIB_MIN_BASELINE_PX = 10;

function finitePoint(xRaw: string | undefined, yRaw: string | undefined): Pt | null {
  const x = Number(xRaw);
  const y = Number(yRaw);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  return { x, y };
}

/** Placeholder glint, inside the pupil and opposite P1 when P1 is known. */
function defaultP4(pupil: { x: number; y: number; r: number }, p1: Pt | null): Pt {
  const dist = pupil.r * 0.45;
  let dx = 1;
  let dy = 0;
  if (p1) {
    const vx = p1.x - pupil.x;
    const vy = p1.y - pupil.y;
    const mag = Math.hypot(vx, vy);
    if (mag > 1) {
      dx = -vx / mag;
      dy = -vy / mag;
    }
  }
  return { x: pupil.x + dx * dist, y: pupil.y + dy * dist };
}

function clampToPupil(pupil: { x: number; y: number; r: number }, p: Pt): Pt {
  const dx = p.x - pupil.x;
  const dy = p.y - pupil.y;
  const maxR = pupil.r * 0.92;
  const mag = Math.hypot(dx, dy);
  if (mag <= maxR || mag === 0) return p;
  return { x: pupil.x + (dx / mag) * maxR, y: pupil.y + (dy / mag) * maxR };
}

function clampToRoi(roi: { x: number; y: number; w: number; h: number }, p: Pt): Pt {
  return {
    x: Math.min(roi.x + roi.w, Math.max(roi.x, p.x)),
    y: Math.min(roi.y + roi.h, Math.max(roi.y, p.y)),
  };
}

type Pt = { x: number; y: number };

interface LiveCheck {
  samples: number;
  pupil: number;
  p1: number;
  p4: number;
  last: { pupil?: Pt; p1?: Pt; p4?: Pt };
}

type P4CalibPlan =
  | { kind: "calibrate"; pupil: Pt; p1: Pt; p4: Pt; radius: number }
  | { kind: "skip"; reason: string }
  | null;

interface AutoSnapshot {
  settings: Record<string, string>;
  roi: string;
  /** Detector values that were saved before auto-tune ran (key -> value). */
  saved: Record<string, string>;
  gain?: number;
  cameraKey?: string;
}

export function attachTuningPanel(
  conn: Connection,
  panel: HTMLElement,
  toggle: HTMLButtonElement,
  body: HTMLElement,
  hooks?: {
    setPaused?: (paused: boolean) => void;
    setPendingRoi?: (roi: RoiBox) => void;
    beginTuneProposal?: (proposal: TuneProposal) => void;
    endTuneProposal?: () => void;
    getTuneProposal?: () => TuneProposal | null;
  },
): TuningPanel {
  const rows = new Map<string, NumRow>();
  const segButtons: HTMLButtonElement[] = [];
  let mode = "";

  // ---- collapse ----
  const showOpen = (open: boolean) => {
    panel.classList.toggle("collapsed", !open);
    toggle.setAttribute("aria-expanded", String(open));
    toggle.title = open ? "Hide settings" : "Show tracking settings";
  };
  showOpen(settings.flag("ui.tuningOpen", true));
  settings.on("ui.tuningOpen", () => showOpen(settings.flag("ui.tuningOpen", true)));
  toggle.addEventListener("click", () => {
    const open = panel.classList.contains("collapsed");
    showOpen(open);
    settings.set("ui.tuningOpen", open);
  });

  // ---- build ----
  const modeBlock = el("div", "tuning-mode-block");
  const seg = el("div", "tuning-segmented");
  seg.setAttribute("role", "radiogroup");
  seg.setAttribute("aria-label", "Detection type");
  for (const m of MODES) {
    const b = el("button", "tuning-seg", m.label) as HTMLButtonElement;
    b.type = "button";
    b.dataset.mode = m.id;
    b.title = m.hint;
    b.setAttribute("role", "radio");
    b.addEventListener("click", () => void onModePick(m.id));
    segButtons.push(b);
    seg.append(b);
  }
  modeBlock.append(seg);
  body.append(modeBlock);

  const auto = el("div", "tuning-auto");
  const autoRow = el("div", "tuning-auto-row");
  const autoBtn = el("button", "tuning-btn tuning-auto-btn", AUTO_LABEL) as HTMLButtonElement;
  autoBtn.type = "button";
  autoBtn.addEventListener("click", () => void autoDetect());
  autoRow.append(autoBtn);
  const autoSummary = el("p", "tuning-auto-summary");
  const qualityList = el("ul", "tuning-quality-list");
  const gainSuggest = el("div", "tuning-gain-suggest");
  const gainSuggestText = el("span", "tuning-gain-suggest-text");
  const applyGainBtn = el(
    "button",
    "tuning-btn tuning-gain-apply",
    "Apply gain and re-tune",
  ) as HTMLButtonElement;
  applyGainBtn.type = "button";
  applyGainBtn.hidden = true;
  applyGainBtn.addEventListener("click", () => void applyGainAndRedetect());
  gainSuggest.append(gainSuggestText, applyGainBtn);
  const autoResult = el("p", "tuning-auto-result");
  const undoBtn = el("button", "tuning-btn tuning-auto-undo", "Undo") as HTMLButtonElement;
  undoBtn.type = "button";
  undoBtn.title = "Restore the settings and ROI from before auto-tune.";
  undoBtn.hidden = true;
  undoBtn.addEventListener("click", () => void undoAutoDetect());
  const acceptBtn = el("button", "tuning-btn tuning-auto-accept", "Accept") as HTMLButtonElement;
  acceptBtn.type = "button";
  acceptBtn.title = ACCEPT_TITLE;
  acceptBtn.hidden = true;
  acceptBtn.addEventListener("click", () => void acceptAutoDetect());
  const autoFoot = el("div", "tuning-auto-foot");
  autoFoot.append(autoResult);
  const autoActions = el("div", "tuning-auto-actions");
  autoActions.hidden = true;
  autoActions.append(undoBtn, acceptBtn);
  auto.append(autoRow, autoSummary, qualityList, gainSuggest, autoFoot, autoActions);
  body.append(auto);

  const { root: camRoot, content: camBody } = section("Camera");
  const resolutionRow = new ReadOnlyRow("Resolution");
  const frameRateRow = new ReadOnlyRow("Frame rate");
  const frameRateSlider = new NumRow(FPS_CONTROL, (v, final) => onFpsEdit(v, final));
  frameRateSlider.attachLive();
  frameRateSlider.el.hidden = true;
  const ptpRow = new ReadOnlyRow("Clock sync");
  ptpRow.el.hidden = true;
  const binningButtons: HTMLButtonElement[] = [];
  const binningWrap = el("div", "tuning-binning");
  binningWrap.hidden = true;
  binningWrap.title = BINNING_HINT;
  binningWrap.append(el("span", "tuning-label-text", "Pixel binning"));
  const binningSeg = el("div", "tuning-segmented");
  binningSeg.setAttribute("role", "radiogroup");
  binningSeg.setAttribute("aria-label", "Pixel binning");
  binningSeg.title = BINNING_HINT;
  for (const m of BINNING) {
    const b = el("button", "tuning-seg", m.label) as HTMLButtonElement;
    b.type = "button";
    b.title = BINNING_HINT;
    b.dataset.h = String(m.h);
    b.dataset.v = String(m.v);
    b.setAttribute("role", "radio");
    b.addEventListener("click", () => void onBinningPick(m.h, m.v));
    binningButtons.push(b);
    binningSeg.append(b);
  }
  binningWrap.append(binningSeg);
  const exposureRow = new NumRow(EXPOSURE_CONTROL, (v, final) => onExposureEdit(v, final));
  exposureRow.attachLive();
  exposureRow.el.hidden = true;
  const gainRow = new NumRow(GAIN_CONTROL, (v, final) => onGainEdit(v, final));
  camBody.append(
    resolutionRow.el,
    frameRateRow.el,
    frameRateSlider.el,
    exposureRow.el,
    binningWrap,
    gainRow.el,
    ptpRow.el,
  );
  body.append(camRoot);
  gainRow.el.hidden = true;

  const { root: roiRoot, content: roiBody } = section("ROI");
  const followSegButtons: HTMLButtonElement[] = [];
  const roiFollowSeg = el("div", "tuning-segmented");
  roiFollowSeg.setAttribute("role", "radiogroup");
  roiFollowSeg.setAttribute("aria-label", "ROI positioning");
  roiFollowSeg.title = AUTO_ROI_TITLE;
  for (const m of ROI_FOLLOW) {
    const b = el("button", "tuning-seg", m.label) as HTMLButtonElement;
    b.type = "button";
    b.dataset.follow = m.follow ? "1" : "0";
    b.setAttribute("role", "radio");
    b.addEventListener("click", () => void setFollowPupil(m.follow));
    followSegButtons.push(b);
    roiFollowSeg.append(b);
  }
  const roiWRow = new NumRow(ROI_W_CONTROL, (v, final) => applyRoiSize(v, undefined, final));
  const roiHRow = new NumRow(ROI_H_CONTROL, (v, final) => applyRoiSize(undefined, v, final));
  roiBody.append(roiFollowSeg, roiWRow.el, roiHRow.el);
  body.append(roiRoot);

  const targetSectionRoots = new Map<TargetSection["id"], HTMLElement>();
  for (const sec of TARGET_SECTIONS) {
    const { root, content } = section(sec.title);
    for (const c of sec.controls) {
      const row = new NumRow(c, (v, final) => onNumEdit(c, v, final));
      rows.set(c.key, row);
      content.append(row.el);
    }
    targetSectionRoots.set(sec.id, root);
    body.append(root);
  }

  const actions = el("div", "tuning-actions");
  const resetDefaultsBtn = el("button", "tuning-btn tuning-btn-block", "Reset to defaults") as HTMLButtonElement;
  resetDefaultsBtn.type = "button";
  resetDefaultsBtn.title = "Restore serve.tcl defaults and forget saved changes.";
  resetDefaultsBtn.addEventListener("click", () => void resetDefaults());
  actions.append(resetDefaultsBtn);
  const status = el("p", "tuning-status");
  const foot = el("div", "tuning-foot");
  foot.append(actions, status);
  body.append(foot);

  // ---- behavior ----
  function setStatus(msg: string, isError = false) {
    status.textContent = msg;
    status.classList.toggle("error", isError);
  }

  /** Change a setting through the server's store; shows the reason in the panel if it is refused. */
  async function putSetting(key: string, value: string | number | boolean, okMsg = ""): Promise<boolean> {
    try {
      await settings.put(key, value);
      if (okMsg) setStatus(okMsg);
      return true;
    } catch (e) {
      setStatus(e instanceof Error ? e.message : String(e), true);
      return false;
    }
  }

  function onNumEdit(c: NumControl, v: number, final: boolean) {
    rows.get(c.key)?.setChanged(true);
    sendThrottled(c, v, final);
  }

  const lastSent = new Map<string, number>();
  const trailing = new Map<string, number>();
  function sendThrottled(c: NumControl, v: number, final: boolean) {
    window.clearTimeout(trailing.get(c.key));
    const now = performance.now();
    const wait = SEND_INTERVAL_MS - (now - (lastSent.get(c.key) ?? 0));
    const send = () => {
      lastSent.set(c.key, performance.now());
      void putSetting(detKey(c.key), formatValue(v, c.step)).then((ok) => {
        if (ok && final) setStatus("");
      });
    };
    if (final || wait <= 0) send();
    else trailing.set(c.key, window.setTimeout(send, wait));
  }

  async function onModePick(id: string) {
    if (id === mode) return;
    const prev = mode;
    setMode(id);
    if (!(await putSetting("det.mode", id))) setMode(prev);
  }

  function setMode(id: string) {
    mode = id;
    for (const b of segButtons) {
      const on = b.dataset.mode === id;
      b.classList.toggle("selected", on);
      b.setAttribute("aria-checked", String(on));
    }
    updateTargetVisibility();
  }

  function updateTargetVisibility() {
    targetSectionRoots.get("p1")!.hidden = mode === "pupil_only";
    targetSectionRoots.get("p4")!.hidden = mode !== "full";
  }

  let frameW = 0;
  let frameH = 0;
  let currentRoi: RoiBox | undefined;

  function roiInside(roi: RoiBox, fw: number, fh: number): boolean {
    return (
      roi.w > 0 &&
      roi.h > 0 &&
      roi.x >= 0 &&
      roi.y >= 0 &&
      roi.x + roi.w <= fw &&
      roi.y + roi.h <= fh
    );
  }

  /** Centered box at about three quarters of the frame. */
  function centeredRoi(fw: number, fh: number): RoiBox {
    const w = Math.min(fw, Math.max(1, Math.round(fw * 0.75)));
    const h = Math.min(fh, Math.max(1, Math.round(fh * 0.75)));
    return {
      x: Math.floor((fw - w) / 2),
      y: Math.floor((fh - h) / 2),
      w,
      h,
    };
  }

  function setRoi(roi: RoiBox | undefined) {
    if (roi && frameW > 0 && frameH > 0 && !roiInside(roi, frameW, frameH)) {
      const next = centeredRoi(frameW, frameH);
      const already =
        currentRoi !== undefined &&
        currentRoi.x === next.x &&
        currentRoi.y === next.y &&
        currentRoi.w === next.w &&
        currentRoi.h === next.h;
      if (!already) {
        const hadRoi = currentRoi !== undefined;
        currentRoi = next;
        if (!hadRoi) refreshAutoRoiUi();
        roiWRow.setValue(next.w);
        roiHRow.setValue(next.h);
        hooks?.setPendingRoi?.(next);
        void putSetting("roi", `${next.x} ${next.y} ${next.w} ${next.h}`);
      }
      return;
    }
    const hadRoi = currentRoi !== undefined;
    currentRoi = roi;
    if (hadRoi !== (roi !== undefined)) refreshAutoRoiUi();
    if (roi?.w !== undefined && !roiWRow.isFocused()) roiWRow.setValue(Math.round(roi.w));
    if (roi?.h !== undefined && !roiHRow.isFocused()) roiHRow.setValue(Math.round(roi.h));
  }

  const roiCommit = { t: 0, timer: 0 };
  function sendRoiThrottled(roi: string, final: boolean) {
    window.clearTimeout(roiCommit.timer);
    const wait = SEND_INTERVAL_MS - (performance.now() - roiCommit.t);
    const send = () => {
      roiCommit.t = performance.now();
      void putSetting("roi", roi);
    };
    if (final || wait <= 0) send();
    else roiCommit.timer = window.setTimeout(send, wait);
  }

  /** Resize about the current center, clamped to the frame. */
  function applyRoiSize(nextW: number | undefined, nextH: number | undefined, final: boolean) {
    const r = currentRoi;
    if (!r) return;
    const maxW = frameW || ROI_W_CONTROL.max;
    const maxH = frameH || ROI_H_CONTROL.max;
    const w = clamp(Math.round(nextW ?? roiWRow.getValue()), ROI_MIN, maxW);
    const h = clamp(Math.round(nextH ?? roiHRow.getValue()), ROI_MIN, maxH);
    roiWRow.setValue(w);
    roiHRow.setValue(h);
    if (w === r.w && h === r.h) return;
    const cx = r.x + r.w / 2;
    const cy = r.y + r.h / 2;
    let x = Math.round(cx - w / 2);
    let y = Math.round(cy - h / 2);
    if (frameW) x = clamp(x, 0, frameW - w);
    if (frameH) y = clamp(y, 0, frameH - h);
    x = Math.max(0, x);
    y = Math.max(0, y);
    const next = { x, y, w, h };
    currentRoi = next;
    hooks?.setPendingRoi?.(next);
    sendRoiThrottled(`${x} ${y} ${w} ${h}`, final);
  }

  function setSourceInfo(info: { width?: number; height?: number; fps?: number }) {
    const { width, height, fps } = info;
    frameW = width ?? 0;
    frameH = height ?? 0;
    if (frameW) roiWRow.setRange(ROI_MIN, frameW);
    if (frameH) roiHRow.setRange(ROI_MIN, frameH);
    resolutionRow.setValue(
      width !== undefined && height !== undefined ? `${width}\u00d7${height}` : "\u2014",
    );
    if (!frameRateRow.el.hidden) {
      frameRateRow.setValue(fps !== undefined && Number.isFinite(fps) ? `${Math.round(fps)} fps` : "\u2014");
    }
  }

  async function readSettings() {
    try {
      const raw = await conn.sendEvalAsync("eyetracking::getSettings");
      const s = parseTclDict(raw);
      for (const [key, row] of rows) {
        const v = Number(s[key]);
        if (key in s && Number.isFinite(v)) row.setValue(v);
        row.setChanged(settings.has(detKey(key)));
      }
      if (s.detection_mode) setMode(s.detection_mode);
    } catch (e) {
      setStatus(e instanceof Error ? e.message : String(e), true);
    }
  }

  async function resetDefaults() {
    try {
      await settings.reset("detector");
      setStatus("Defaults restored.");
      await readSettings();
    } catch (e) {
      setStatus(e instanceof Error ? e.message : String(e), true);
    }
  }

  // Another browser changed (or reset) a tracking setting.
  settings.on("det.*", (key) => {
    if (key === "det.mode") {
      const m = settings.get("det.mode");
      if (m && MODES.some((x) => x.id === m)) setMode(m);
      else void readSettings();
      return;
    }
    const row = rows.get(key.slice(4));
    if (!row) return;
    const v = settings.get(key);
    row.setChanged(v !== undefined);
    if (v === undefined) void readSettings();
    else if (Number.isFinite(Number(v)) && !row.isFocused()) row.setValue(Number(v));
  });

  async function onConnected() {
    setStatus("");
    await settings.load(); // what the server holds; nothing is pushed to it
    await readSettings();
    await syncAutoRoiFromServer();
  }

  function setEnabled(enabled: boolean) {
    connected = enabled;
    panel.classList.toggle("disabled", !enabled);
    refreshAutoBtn();
  }

  // ---- auto-detect ----
  let connected = false;
  let sourceActive = false;
  let sourcePaused = false;
  let autoRunning = false;
  let snapshot: AutoSnapshot | null = null;
  let cameraKey: string | undefined;
  let pendingGainDelta = 0;
  let lastQualityMetrics = "";
  let p4CalibPlan: P4CalibPlan = null;

  async function onGainEdit(v: number, final: boolean) {
    if (!cameraKey) return;
    sendGainThrottled(v, final);
  }

  const gainLastSent = { t: 0, timer: 0 };
  function sendGainThrottled(v: number, final: boolean) {
    const camKey = cameraKey;
    if (!camKey) return;
    window.clearTimeout(gainLastSent.timer);
    const wait = SEND_INTERVAL_MS - (performance.now() - gainLastSent.t);
    const send = () => {
      gainLastSent.t = performance.now();
      void settings
        .put(`cam.${vendorOf(camKey)}.gain`, formatValue(v, GAIN_CONTROL.step))
        .then((actual) => {
          const n = Number(actual);
          if (final && Number.isFinite(n)) gainRow.setValue(n);
        })
        .catch((e) => setStatus(e instanceof Error ? e.message : String(e), true));
    };
    if (final || wait <= 0) send();
    else gainLastSent.timer = window.setTimeout(send, wait);
  }

  function showCameraControls(on: boolean) {
    frameRateRow.el.hidden = on;
    frameRateSlider.el.hidden = !on;
    exposureRow.el.hidden = !on;
    binningWrap.hidden = !on;
    gainRow.el.hidden = !on;
  }

  let exposureMinUs = 1;
  let exposureMaxUs = 30_000_000;

  function exposureUsForDuty(percent: number, fps: number): number {
    if (!(fps > 0) || !Number.isFinite(percent)) return NaN;
    const pct = clamp(percent, EXPOSURE_CONTROL.min, EXPOSURE_CONTROL.max);
    const us = (pct / 100) * (1_000_000 / fps);
    return Math.min(exposureMaxUs, Math.max(exposureMinUs, us));
  }

  function showDuty(exposureUs: number, fps: number, force = false) {
    if (!(fps > 0) || !Number.isFinite(exposureUs)) return;
    const duty = (exposureUs / (1_000_000 / fps)) * 100;
    if (!Number.isFinite(duty)) return;
    if (force || !exposureRow.isFocused()) {
      exposureRow.setValue(clamp(duty, EXPOSURE_CONTROL.min, EXPOSURE_CONTROL.max));
    }
  }

  async function loadExposure() {
    try {
      await conn.sendEvalAsync("camera::node ExposureAuto Off");
    } catch {
      /* configureExposure also forces manual exposure */
    }
    try {
      const info = parseTclDictNested(await conn.sendEvalAsync("camera::nodeInfo ExposureTime"));
      const min = Number(info.min);
      const nodeMax = Number(info.max);
      if (Number.isFinite(min) && min > 0) exposureMinUs = min;
      if (Number.isFinite(nodeMax) && nodeMax > exposureMinUs) exposureMaxUs = nodeMax;
      const cur = Number(info.value);
      const fps = Number(await conn.sendEvalAsync("camera::configureFrameRate"));
      if (Number.isFinite(cur) && Number.isFinite(fps)) showDuty(cur, fps);
    } catch {
      /* leave the default range */
    }
  }

  async function applyDuty(percent: number, fps: number): Promise<number | undefined> {
    const us = exposureUsForDuty(percent, fps);
    if (!Number.isFinite(us) || !cameraKey) return undefined;
    // The server sets it, keeps it, and answers with what the camera ended up with.
    const actual = Number(await settings.put(`cam.${vendorOf(cameraKey)}.exposure_us`, Math.round(us)));
    return Number.isFinite(actual) ? actual : Math.round(us);
  }

  async function loadFrameRate() {
    try {
      const range = parseTclDict(await conn.sendEvalAsync("camera::getFrameRateRange"));
      const min = Number(range.min);
      const max = Number(range.max);
      if (Number.isFinite(min) && Number.isFinite(max) && max > min) {
        frameRateSlider.setRange(min, max);
      }
    } catch {
      frameRateSlider.setRange(FPS_CONTROL.min, FPS_CONTROL.max);
    }
    try {
      const cur = Number(await conn.sendEvalAsync("camera::configureFrameRate"));
      if (Number.isFinite(cur)) frameRateSlider.setValue(cur);
    } catch {
      /* no camera */
    }
  }

  function selectBinning(h: number, v: number) {
    for (const b of binningButtons) {
      const on = Number(b.dataset.h) === h && Number(b.dataset.v) === v;
      b.classList.toggle("selected", on);
      b.setAttribute("aria-checked", String(on));
    }
  }

  async function loadBinning() {
    const raw = await conn.sendEvalAsync("camera::configureBinning");
    const d = parseTclDict(raw);
    selectBinning(Number(d.horizontal), Number(d.vertical));
  }

  const fpsLastSent = { t: 0, timer: 0 };
  function onFpsEdit(v: number, final: boolean) {
    const camKey = cameraKey;
    if (!camKey) return;
    window.clearTimeout(fpsLastSent.timer);
    const send = () => {
      fpsLastSent.t = performance.now();
      void settings
        .put(`cam.${vendorOf(camKey)}.fps`, formatValue(v, FPS_CONTROL.step))
        .then(async (actual) => {
          const n = Number(actual);
          const fps = Number.isFinite(n) ? n : v;
          if (final && Number.isFinite(n)) frameRateSlider.setValue(n);
          const exp = await applyDuty(exposureRow.getValue(), fps);
          if (final && exp !== undefined) showDuty(exp, fps);
          if (final) return loadFrameRate();
        })
        .catch((e) => setStatus(e instanceof Error ? e.message : String(e), true));
    };
    const wait = SEND_INTERVAL_MS - (performance.now() - fpsLastSent.t);
    if (final || wait <= 0) send();
    else fpsLastSent.timer = window.setTimeout(send, wait);
  }

  const exposureLastSent = { t: 0, timer: 0 };
  function onExposureEdit(v: number, final: boolean) {
    if (!cameraKey) return;
    window.clearTimeout(exposureLastSent.timer);
    const send = () => {
      exposureLastSent.t = performance.now();
      const fps = frameRateSlider.getValue();
      void applyDuty(v, fps)
        .then((exp) => {
          if (!final || exp === undefined) return;
          showDuty(exp, fps);
          return loadFrameRate();
        })
        .catch((e) => {
          setStatus(e instanceof Error ? e.message : String(e), true);
          void syncExposureSlider();
        });
    };
    const wait = SEND_INTERVAL_MS - (performance.now() - exposureLastSent.t);
    if (final || wait <= 0) send();
    else exposureLastSent.timer = window.setTimeout(send, wait);
  }

  async function onBinningPick(h: number, v: number) {
    if (!cameraKey) return;
    selectBinning(h, v);
    try {
      await settings.put(`cam.${vendorOf(cameraKey)}.binning`, `${h} ${v}`);
      await loadBinning(); // what the camera ended up with
      await loadFrameRate();
    } catch (e) {
      setStatus(e instanceof Error ? e.message : String(e), true);
      try {
        await loadBinning();
      } catch {
        /* leave the last selection */
      }
    }
  }

  let liveTimer = 0;
  let liveBusy = false;
  const liveFps = cameraFpsReader(conn);

  function showLive(fps: number | null, exposureUs: number | null) {
    frameRateSlider.setLive(fps !== null && Number.isFinite(fps) ? `${Math.round(fps)} fps` : "\u2014");
    exposureRow.setLive(
      exposureUs !== null && Number.isFinite(exposureUs) ? formatExposureUs(exposureUs) : "\u2014",
    );
  }

  function stopCameraLive() {
    window.clearInterval(liveTimer);
    liveTimer = 0;
    window.clearInterval(ptpTimer);
    ptpTimer = 0;
    ptpRow.el.hidden = true;
    showLive(null, null);
  }

  async function syncExposureSlider() {
    try {
      const us = Number(await conn.sendEvalAsync("camera::node ExposureTime"));
      const fps = frameRateSlider.getValue();
      if (Number.isFinite(us)) showDuty(us, fps, true);
    } catch {
      /* leave the slider */
    }
  }

  async function readCameraLive() {
    if (!cameraKey || liveBusy) return;
    liveBusy = true;
    try {
      const fps = await liveFps.read();
      const exposureUs = Number(await conn.sendEvalAsync("camera::node ExposureTime"));
      if (cameraKey && Number.isFinite(fps) && Number.isFinite(exposureUs)) showLive(fps, exposureUs);
      const gain = Number(await conn.sendEvalAsync("camera::node Gain"));
      if (cameraKey && Number.isFinite(gain) && !gainRow.isFocused()) gainRow.setValue(gain);
    } catch {
      /* camera dropped */
    } finally {
      liveBusy = false;
    }
  }

  // IEEE 1588 (PTP) state, read straight from the camera's nodes so it works
  // under any launcher script. The offset is a latched value: PtpDataSetLatch
  // refreshes it first. Cameras without PTP nodes hide the row.
  const PTP_POLL_MS = 2000;
  const PTP_EXPLAIN =
    "When synced, frame timestamps are on the grandmaster's clock and can be compared directly with dserv datapoint times.";
  let ptpTimer = 0;
  let ptpBusy = false;
  let ptpUnsupported = false;

  function showPtp(enabled: boolean, status: string, servo: string, offsetNs: number) {
    const detail = [`status ${status}`, servo && `servo ${servo}`, Number.isFinite(offsetNs) && `offset ${formatOffsetNs(offsetNs)}`]
      .filter(Boolean)
      .join(", ");
    ptpRow.el.hidden = false;
    if (!enabled || status === "Disabled") {
      ptpRow.setValue("PTP off", undefined, `PTP is disabled on the camera, so frame timestamps use its own clock and cannot be compared with dserv times.`);
    } else if (status === "Slave") {
      const off = Number.isFinite(offsetNs) ? ` \u00b7 ${formatOffsetNs(offsetNs)}` : "";
      ptpRow.setValue(`synced${off}`, "ok", `Following the PTP grandmaster (${detail}). ${PTP_EXPLAIN}`);
    } else if (status === "Master") {
      ptpRow.setValue("PTP master", undefined, `This camera is the PTP grandmaster (${detail}).`);
    } else if (status === "Listening") {
      ptpRow.setValue("no master", "warn", `PTP is on but no grandmaster has been heard (${detail}). Is ptp4l running on the host?`);
    } else if (status === "Uncalibrated") {
      ptpRow.setValue("syncing\u2026", "warn", `Locking to the grandmaster (${detail}).`);
    } else {
      ptpRow.setValue(status, "warn", `PTP is not synced (${detail}).`);
    }
  }

  async function readPtp() {
    if (!cameraKey || ptpBusy || ptpUnsupported) return;
    ptpBusy = true;
    try {
      const raw = await conn.sendEvalAsync(
        "catch {camera::node PtpDataSetLatch 1}; " +
          "list [camera::node PtpEnable] [camera::node PtpStatus] [camera::node PtpServoStatus] [camera::node PtpOffsetFromMaster]",
      );
      const [enabled, status, servo, offset] = parseTclList(raw);
      if (cameraKey) showPtp(enabled === "1", status ?? "", servo ?? "", Number(offset));
    } catch (e) {
      if (String(e).includes("no such node")) {
        ptpUnsupported = true;
        ptpRow.el.hidden = true;
      }
      /* otherwise the camera dropped; keep the last reading */
    } finally {
      ptpBusy = false;
    }
  }

  function startCameraLive() {
    stopCameraLive();
    void readCameraLive();
    liveTimer = window.setInterval(() => void readCameraLive(), 500);
    void readPtp();
    ptpTimer = window.setInterval(() => void readPtp(), PTP_POLL_MS);
  }

  async function setCamera(key: string | undefined) {
    cameraKey = key || undefined;
    liveFps.reset();
    ptpUnsupported = false;
    if (!cameraKey) {
      showCameraControls(false);
      stopCameraLive();
      return;
    }
    showCameraControls(true);
    startCameraLive();
    await loadExposure();
    await loadFrameRate();
    try {
      await loadBinning();
    } catch {
      /* binning stays unselected */
    }
    try {
      const info = parseTclDictNested(await conn.sendEvalAsync("camera::nodeInfo Gain"));
      const min = Number(info.min);
      const max = Number(info.max);
      if (Number.isFinite(min) && Number.isFinite(max) && max > min) {
        gainRow.setRange(min, max);
      }
    } catch {
      gainRow.setRange(GAIN_CONTROL.min, GAIN_CONTROL.max);
    }
    try {
      const cur = Number(await conn.sendEvalAsync("camera::node Gain"));
      if (Number.isFinite(cur)) gainRow.setValue(cur);
      else showStoredGain();
    } catch {
      showStoredGain();
    }
  }

  function showStoredGain() {
    if (!cameraKey) return;
    const stored = settings.num(`cam.${vendorOf(cameraKey)}.gain`, NaN);
    if (Number.isFinite(stored)) gainRow.setValue(stored);
  }

  function setSourceState(active: boolean, paused: boolean) {
    sourceActive = active;
    sourcePaused = paused;
    refreshAutoBtn();
  }

  // ROI positioning (follow the pupil or stay put): the server holds it as roi.follow.
  let autoRoiOn = settings.flag("roi.follow", false);

  function refreshAutoRoiUi(): void {
    for (const b of followSegButtons) {
      const on = (b.dataset.follow === "1") === autoRoiOn;
      b.classList.toggle("selected", on);
      b.setAttribute("aria-checked", String(on));
    }
    const off = !connected || !sourceActive || autoRunning;
    for (const b of followSegButtons) b.disabled = off;
    const noRoi = off || !currentRoi;
    roiWRow.setDisabled(noRoi);
    roiHRow.setDisabled(noRoi);
  }

  async function syncAutoRoiFromServer(): Promise<void> {
    autoRoiOn = settings.flag("roi.follow", false);
    refreshAutoRoiUi();
    if (!connected) return;
    try {
      // What the plugin is doing now (it starts from the saved setting).
      const v = (await conn.sendEvalAsync("eyetracking::roiFollow")).trim();
      autoRoiOn = v === "1" || v.toLowerCase() === "true";
      refreshAutoRoiUi();
    } catch {
      /* plugin may not be loaded yet */
    }
  }

  settings.on("roi.follow", () => {
    autoRoiOn = settings.flag("roi.follow", false);
    refreshAutoRoiUi();
  });

  async function setFollowPupil(next: boolean): Promise<void> {
    if (next === autoRoiOn) return;
    if (!connected || !sourceActive || autoRunning) {
      refreshAutoRoiUi();
      return;
    }
    try {
      const v = (await settings.put("roi.follow", next)).trim();
      autoRoiOn = v === "1" || v.toLowerCase() === "true";
      refreshAutoRoiUi();
    } catch (e) {
      refreshAutoRoiUi();
      setStatus(e instanceof Error ? e.message : String(e), true);
    }
  }

  function refreshAutoBtn() {
    const ready = connected && sourceActive && sourcePaused && !autoRunning;
    autoBtn.disabled = !ready;
    autoBtn.textContent = autoRunning ? "Tuning\u2026" : AUTO_LABEL;
    const hint = !connected
      ? "Not connected to VideoStream."
      : !sourceActive
        ? "Choose a source first."
        : !sourcePaused
          ? AUTO_HINT_PLAYING
          : AUTO_HINT_READY;
    // Disabled buttons don't reliably show their own tooltip; the wrapper does.
    autoBtn.title = hint;
    auto.title = hint;
    refreshAutoRoiUi();
  }

  function setAutoMessage(summary: string, result = "", kind: "" | "ok" | "warn" | "error" = "") {
    autoSummary.textContent = summary;
    autoResult.textContent = result;
    autoResult.className = `tuning-auto-result${kind ? ` ${kind}` : ""}`;
  }

  function showAutoActions(show: boolean) {
    autoActions.hidden = !show;
    undoBtn.hidden = !show;
    acceptBtn.hidden = !show;
  }

  function dismissAutoDetectReport() {
    setAutoMessage("", "");
    qualityList.replaceChildren();
    gainSuggestText.textContent = "";
    applyGainBtn.hidden = true;
    pendingGainDelta = 0;
  }

  async function acceptAutoDetect() {
    const plan = proposalPlan(hooks?.getTuneProposal?.() ?? null);
    hooks?.endTuneProposal?.();
    p4CalibPlan = null;
    setAcceptLabel(null);
    dismissAutoDetectReport();
    snapshot = null;
    showAutoActions(false);
    if (plan?.kind === "skip") {
      setAutoMessage("", plan.reason, "warn");
      return;
    }
    if (plan?.kind !== "calibrate") return;
    if (!sourcePaused) {
      setAutoMessage(
        "",
        "P4 not calibrated: playback resumed before Accept. Pause and run auto-tune again.",
        "warn",
      );
      return;
    }
    setAutoMessage("", "Calibrating P4\u2026");
    const n = (v: number) => v.toFixed(2);
    try {
      await conn.sendEvalAsync(
        `eyetracking::calibrateP4FromPoints ${n(plan.pupil.x)} ${n(plan.pupil.y)} ` +
          `${n(plan.p1.x)} ${n(plan.p1.y)} ${n(plan.p4.x)} ${n(plan.p4.y)} ${n(plan.radius)}`,
      );
      const s = parseTclDict(await conn.sendEvalAsync("eyetracking::getP4ModelStatus"));
      const ratio = Number(s.magnitude_ratio);
      const angle = Number(s.angle_offset_deg);
      const detail =
        Number.isFinite(ratio) && Number.isFinite(angle)
          ? ` (ratio ${ratio.toFixed(2)}, angle ${angle.toFixed(0)}\u00b0)`
          : "";
      setAutoMessage("", `P4 model calibrated${detail}.`, "ok");
    } catch (e) {
      setAutoMessage("", `P4 calibration failed: ${e instanceof Error ? e.message : String(e)}`, "error");
    }
  }

  function renderQuality(qualityRaw: string | undefined) {
    qualityList.replaceChildren();
    applyGainBtn.hidden = true;
    gainSuggestText.textContent = "";
    pendingGainDelta = 0;
    if (!qualityRaw) return;
    const q = parseTclDictNested(qualityRaw);
    lastQualityMetrics = Object.entries(q)
      .filter(([k]) => k !== "checks")
      .map(([k, v]) => `${k}=${v}`)
      .join(", ");
    const checks = parseQualityChecks(q.checks ?? "");
    const delta = Number(q.gain_delta_db);
    const gainCheck = checks.find((c) => c.id === "gain");
    // The line under the list is the gain action. Skip the checklist copy of it.
    // A warn here means "do not raise gain" (already noisy), so that stays in the list.
    // "marginal" means the frame is dim and soft: focus and aperture, not gain.
    const showGainAction =
      Number.isFinite(delta) &&
      Math.abs(delta) >= 1.5 &&
      gainCheck?.level !== "warn" &&
      !checks.some((c) => c.id === "marginal");
    for (const c of checks) {
      if (c.id === "gain" && showGainAction) continue;
      const li = el("li", `tuning-quality-item level-${c.level}`, c.message);
      li.title = lastQualityMetrics;
      qualityList.append(li);
    }
    if (!showGainAction) return;
    pendingGainDelta = delta;
    const cur = Number(gainRow.getValue());
    const suggested = Number.isFinite(cur) ? cur + delta : delta;
    const sugText = `Suggested gain: ${formatValue(suggested, 0.1)} dB`;
    if (cameraKey) {
      gainSuggestText.textContent = `${sugText} (now ${formatValue(cur, 0.1)} dB). Keep the subject looking straight ahead.`;
      applyGainBtn.hidden = false;
    } else {
      const dir = delta > 0 ? "raise" : "lower";
      gainSuggestText.textContent = `Recorded video: on the camera, ${dir} gain about ${formatValue(Math.abs(delta), 0.1)} dB.`;
    }
  }

  async function applyGainAndRedetect() {
    if (!cameraKey || autoRunning) return;
    const cur = gainRow.getValue();
    const target = cur + pendingGainDelta;
    autoRunning = true;
    refreshAutoBtn();
    applyGainBtn.disabled = true;
    setAutoMessage("Applying gain\u2026", "Keep the subject looking straight ahead.");
    try {
      await conn.sendEvalAsync("vstream::pause 0");
      await settings.put(`cam.${vendorOf(cameraKey)}.gain`, formatValue(target, GAIN_CONTROL.step));
      gainRow.setValue(target);
      await sleep(500);
      await conn.sendEvalAsync("vstream::pause 1");
      sourcePaused = true;
      hooks?.setPaused?.(true);
      await sleep(200);
      await runAutoDetect(true);
    } catch (e) {
      setAutoMessage("", e instanceof Error ? e.message : String(e), "error");
    } finally {
      applyGainBtn.disabled = false;
      autoRunning = false;
      refreshAutoBtn();
    }
  }

  async function autoDetect() {
    if (autoRunning) return;
    autoRunning = true;
    refreshAutoBtn();
    try {
      await runAutoDetect(false);
    } finally {
      autoRunning = false;
      refreshAutoBtn();
    }
  }

  async function runAutoDetect(fromGainApply: boolean) {
    if (!fromGainApply) setAutoMessage("Analyzing the paused frame\u2026");
    try {
      const [settingsRaw, roiRaw] = await Promise.all([
        conn.sendEvalAsync("eyetracking::getSettings"),
        conn.sendEvalAsync("eyetracking::setROI"),
      ]);
      let gainBefore: number | undefined;
      if (cameraKey) {
        try {
          gainBefore = Number(await conn.sendEvalAsync("camera::configureGain"));
        } catch {
          gainBefore = gainRow.getValue();
        }
      }
      // Which of the values auto-tune is about to change were saved settings
      // already (the rest were defaults), so Undo can put things back exactly.
      const saved: Record<string, string> = {};
      for (const k of [...settings.keys("det."), "roi", ...(cameraKey ? [`cam.${vendorOf(cameraKey)}.gain`] : [])]) {
        const v = settings.get(k);
        if (v !== undefined) saved[k] = v;
      }
      const before: AutoSnapshot = {
        settings: parseTclDict(settingsRaw),
        roi: roiRaw.trim(),
        saved,
        gain: gainBefore,
        cameraKey,
      };
      const d = parseTclDictNested(await conn.sendEvalAsync("eyetracking::autoDetect"));
      renderQuality(d.quality);

      const roi = parseTclList(d.roi ?? "").map(Number);
      if (roi.length !== 4 || roi.some((v) => !Number.isFinite(v))) {
        throw new Error("Auto-tune returned no ROI");
      }
      const p1Found = d.p1_found === "1";
      const p4Found = d.p4_found === "1";
      const writes: [string, string][] = [];
      for (const key of AUTO_KEYS) {
        if (!(key in d)) continue;
        const c = controlFor(key);
        const raw = Number(d[key]);
        if (!c || !Number.isFinite(raw)) continue;
        const v = clamp(snap(raw, c.step), c.min, c.max);
        rows.get(key)?.setValue(v);
        rows.get(key)?.setChanged(true);
        writes.push([detKey(key), formatValue(v, c.step)]);
      }
      writes.push(["roi", roi.join(" ")]);
      for (const [k, v] of writes) await settings.put(k, v);
      await conn.sendEvalAsync("eyetracking::resetTrackingState");
      snapshot = before;
      p4CalibPlan = null;
      setAcceptLabel(null);
      showAutoActions(true);

      const r = Math.round(Number(d.pupil_radius));
      const bits = [`Pupil r ${r}px, threshold ${d.pupil_threshold}`];
      bits.push(
        p1Found
          ? `P1 ${Math.round(Number(d.p1_area))}px\u00b2 (gate ${d.p1_min_area}\u2013${d.p1_max_area}, min ${d.p1_min_intensity})`
          : "P1 not found",
      );
      const proposal = buildTuneProposal(d, roi, p1Found, p4Found);
      bits.push(
        p4Found
          ? `P4 peak ${d.p4_peak} over ${d.p4_background} (min ${d.p4_min_intensity})`
          : "P4 wasn't found. Drag it onto the glint.",
      );
      if (proposal) hooks?.beginTuneProposal?.(proposal);
      p4CalibPlan = proposalPlan(proposal);
      setAcceptLabel(p4CalibPlan);
      setAutoMessage(bits.join(" \u00b7 "), "Verifying\u2026");

      const v = await verifyLive();
      const checkP1 = p1Found && mode !== "pupil_only";
      const checkP4 = p4Found && mode === "full";
      const pct = (x: number) => `${Math.round(x * 100)}%`;
      const seen = [`pupil ${pct(v.pupil)}`];
      if (mode !== "pupil_only") seen.push(`P1 ${pct(v.p1)}`);
      if (p4Found && mode === "full") seen.push(`P4 ${pct(v.p4)}`);
      const ok =
        v.samples > 0 &&
        v.pupil >= 0.95 &&
        (!checkP1 || v.p1 >= 0.95) &&
        (!checkP4 || v.p4 >= 0.8);
      const missing = (!p1Found && mode !== "pupil_only") || (!p4Found && mode === "full");
      const notes = parseTclList(d.notes ?? "").filter(
        (n) => !(!p4Found && n.startsWith("P4 not distinguishable")),
      );
      const suffix = notes.length ? ` ${notes.join(". ")}.` : "";
      setAutoMessage(
        bits.join(" \u00b7 "),
        `${ok ? "Verified" : "Check tracking"}: ${seen.join(", ")}.${suffix}`,
        ok && !missing ? "ok" : "warn",
      );
    } catch (e) {
      hooks?.endTuneProposal?.();
      p4CalibPlan = null;
      setAcceptLabel(null);
      showAutoActions(false);
      dismissAutoDetectReport();
      setAutoMessage("", e instanceof Error ? e.message : String(e), "error");
    }
  }

  /** Proposed marks for the review overlay. A missing P4 gets a placeholder inside the pupil. */
  function buildTuneProposal(
    d: Record<string, string>,
    roi: number[],
    p1Found: boolean,
    p4Found: boolean,
  ): TuneProposal | null {
    const px = Number(d.pupil_x);
    const py = Number(d.pupil_y);
    const pr = Number(d.pupil_radius);
    if (![px, py, pr].every((v) => Number.isFinite(v)) || pr <= 0) return null;
    const pupil = { x: px, y: py, r: pr };
    const box = { x: roi[0]!, y: roi[1]!, w: roi[2]!, h: roi[3]! };
    const p1 = p1Found ? finitePoint(d.p1_x, d.p1_y) : null;
    const foundP4 = p4Found ? finitePoint(d.p4_x, d.p4_y) : null;
    const p4 = clampToPupil(pupil, foundP4 ?? defaultP4(pupil, p1));
    return {
      pupil,
      roi: box,
      p1: p1 ? clampToRoi(box, p1) : null,
      p4,
      p4Placed: !foundP4,
    };
  }

  function proposalPlan(p: TuneProposal | null): P4CalibPlan {
    if (mode !== "full" || !p?.p1) return null;
    const baseline = Math.hypot(p.p1.x - p.pupil.x, p.p1.y - p.pupil.y);
    if (baseline < P4_CALIB_MIN_BASELINE_PX) {
      return {
        kind: "skip",
        reason:
          `P4 not calibrated: P1 is only ${baseline.toFixed(1)}px from the pupil center, ` +
          "too close for a reliable model. Try again with the gaze slightly off-center.",
      };
    }
    return { kind: "calibrate", pupil: p.pupil, p1: p.p1, p4: p.p4, radius: p.pupil.r };
  }

  /**
   * Fraction of live results with each feature, sampled for about a second,
   * plus the last positions seen (the frame is paused, so they don't move).
   */
  async function verifyLive(): Promise<LiveCheck> {
    await sleep(300);
    let samples = 0, pupil = 0, p1 = 0, p4 = 0;
    const last: LiveCheck["last"] = {};
    for (let i = 0; i < 20; i++) {
      const raw = await conn.sendEvalAsync("eyetracking::getResults");
      if (raw.trim() !== "no results") {
        const res = parseTclDictNested(raw);
        samples++;
        for (const key of ["pupil", "p1", "p4"] as const) {
          if (!(key in res)) continue;
          const pt = parseTclDict(res[key]);
          const x = Number(pt.x);
          const y = Number(pt.y);
          if (Number.isFinite(x) && Number.isFinite(y)) last[key] = { x, y };
        }
        if ("pupil" in res) pupil++;
        if ("p1" in res) p1++;
        if ("p4" in res) p4++;
      }
      await sleep(50);
    }
    const f = (n: number) => (samples ? n / samples : 0);
    return { samples, pupil: f(pupil), p1: f(p1), p4: f(p4), last };
  }

  function setAcceptLabel(plan: P4CalibPlan) {
    acceptBtn.title =
      plan?.kind === "calibrate"
        ? "Keep the applied settings and calibrate the P4 model from the proposed pupil, P1, and P4."
        : ACCEPT_TITLE;
  }

  async function undoAutoDetect() {
    const s = snapshot;
    if (!s) return;
    try {
      for (const key of AUTO_KEYS) {
        const c = controlFor(key);
        const v = Number(s.settings[key]);
        if (c && Number.isFinite(v)) await settings.put(detKey(key), formatValue(v, c.step));
      }
      if (/^-?\d+ -?\d+ \d+ \d+$/.test(s.roi)) await settings.put("roi", s.roi);
      if (s.gain !== undefined && s.cameraKey) {
        await settings.put(`cam.${vendorOf(s.cameraKey)}.gain`, formatValue(s.gain, GAIN_CONTROL.step));
      }
      // Whatever was only a default before is not a saved setting now either.
      const touched = [
        ...AUTO_KEYS.map(detKey),
        "roi",
        ...(s.cameraKey ? [`cam.${vendorOf(s.cameraKey)}.gain`] : []),
      ];
      settings.drop(...touched.filter((k) => !(k in s.saved)));
      await conn.sendEvalAsync("eyetracking::resetTrackingState");
    } catch (e) {
      setStatus(e instanceof Error ? e.message : String(e), true);
      return;
    }
    hooks?.endTuneProposal?.();
    if (s.gain !== undefined && s.cameraKey) gainRow.setValue(s.gain);
    snapshot = null;
    p4CalibPlan = null;
    setAcceptLabel(null);
    showAutoActions(false);
    dismissAutoDetectReport();
    setAutoMessage("", "Restored the previous settings.");
    await readSettings();
  }

  setEnabled(false);
  updateTargetVisibility();
  return { onConnected, setEnabled, setSourceState, setCamera, setSourceInfo, setRoi };
}

function controlFor(key: string): NumControl | undefined {
  return ALL_CONTROLS.find((c) => c.key === key);
}

function sleep(ms: number): Promise<void> {
  return new Promise((r) => window.setTimeout(r, ms));
}

class ReadOnlyRow {
  readonly el: HTMLElement;
  private valueEl: HTMLElement;

  constructor(label: string) {
    this.el = el("div", "tuning-row tuning-row-readonly");
    this.el.append(el("span", "tuning-label-text", label));
    this.valueEl = el("span", "tuning-readonly-value", "\u2014");
    this.el.append(this.valueEl);
  }

  setValue(text: string, level?: "ok" | "warn", tooltip?: string) {
    this.valueEl.textContent = text;
    this.valueEl.classList.toggle("level-ok", level === "ok");
    this.valueEl.classList.toggle("level-warn", level === "warn");
    this.el.title = tooltip ?? "";
  }
}

// "+143 ns", "-12.3 \u00b5s", "+1.2 ms"
function formatOffsetNs(ns: number): string {
  const sign = ns < 0 ? "-" : "+";
  const a = Math.abs(ns);
  if (a < 1000) return `${sign}${Math.round(a)} ns`;
  if (a < 1e6) return `${sign}${(a / 1000).toFixed(1)} \u00b5s`;
  return `${sign}${(a / 1e6).toFixed(1)} ms`;
}

function formatExposureUs(us: number): string {
  if (us >= 10000) return `${(us / 1000).toFixed(1)} ms`;
  return `${Math.round(us)} \u00b5s`;
}

class NumRow {
  readonly el: HTMLElement;
  private slider: HTMLInputElement;
  private num: HTMLInputElement;
  private liveEl: HTMLElement | null = null;

  constructor(
    private c: NumControl,
    private onEdit: (v: number, final: boolean) => void,
  ) {
    this.el = el("div", "tuning-row");
    if (c.hint) this.el.title = c.hint;
    const label = el("label", "tuning-label");
    label.append(
      el("span", "tuning-changed"),
      el("span", "tuning-label-text", c.unit ? `${c.label} (${c.unit})` : c.label),
    );
    this.num = document.createElement("input");
    this.num.type = "number";
    this.num.className = "tuning-num";
    this.num.min = String(c.min);
    this.num.max = String(c.max);
    this.num.step = String(c.step);
    this.num.setAttribute("aria-label", c.label);
    this.slider = document.createElement("input");
    this.slider.type = "range";
    this.slider.className = "tuning-slider";
    this.slider.min = String(c.min);
    this.slider.max = String(c.max);
    this.slider.step = String(c.step);
    this.slider.setAttribute("aria-label", c.label);
    this.el.append(label, this.num, this.slider);

    this.slider.addEventListener("input", () => {
      const v = Number(this.slider.value);
      this.num.value = formatValue(v, c.step);
      this.onEdit(v, false);
    });
    this.slider.addEventListener("change", () => this.onEdit(Number(this.slider.value), true));
    this.num.addEventListener("change", () => {
      const raw = Number(this.num.value);
      if (!Number.isFinite(raw)) return;
      const v = clamp(snap(raw, c.step), Number(this.num.min), Number(this.num.max));
      this.setValue(v);
      this.onEdit(v, true);
    });
    this.num.addEventListener("keydown", (ev) => {
      if (ev.key === "Enter") this.num.blur();
    });
  }

  /** Grey measured value, between the label and the number stepper. */
  attachLive() {
    this.el.classList.add("tuning-row-live");
    this.liveEl = el("span", "tuning-readonly-value tuning-live", "\u2014");
    this.el.insertBefore(this.liveEl, this.num);
  }

  setLive(text: string) {
    if (this.liveEl) this.liveEl.textContent = text;
  }

  setValue(v: number) {
    this.slider.value = String(v);
    this.num.value = formatValue(v, this.c.step);
  }

  setChanged(on: boolean) {
    this.el.classList.toggle("changed", on);
  }

  setDisabled(on: boolean, tooltip?: string) {
    this.slider.disabled = on;
    this.num.disabled = on;
    this.el.classList.toggle("disabled", on);
    if (tooltip !== undefined) this.setTooltip(tooltip);
  }

  setTooltip(text: string) {
    this.el.title = text;
  }

  setRange(min: number, max: number) {
    this.slider.min = String(min);
    this.slider.max = String(max);
    this.num.min = String(min);
    this.num.max = String(max);
  }

  getValue(): number {
    return Number(this.slider.value);
  }

  isFocused(): boolean {
    const ae = document.activeElement;
    return ae === this.num || ae === this.slider;
  }
}

function section(title: string): { root: HTMLElement; content: HTMLElement } {
  const root = el("section", "tuning-section");
  const head = el("button", "tuning-section-head") as HTMLButtonElement;
  head.type = "button";
  head.append(el("span", "", title), el("span", "tuning-section-chevron", "\u25BE"));
  const content = el("div", "tuning-section-body");
  head.addEventListener("click", () => {
    const closed = root.classList.toggle("closed");
    head.setAttribute("aria-expanded", String(!closed));
  });
  root.classList.add("closed");
  head.setAttribute("aria-expanded", "false");
  root.append(head, content);
  return { root, content };
}

/** Flat Tcl dict ("k v k v ...") whose values contain no spaces. */
function parseTclDict(raw: string): Record<string, string> {
  const out: Record<string, string> = {};
  const parts = raw.trim().split(/\s+/);
  for (let i = 0; i + 1 < parts.length; i += 2) out[parts[i]] = parts[i + 1];
  return out;
}

function parseQualityChecks(checksRaw: string): QualityCheckRow[] {
  if (!checksRaw.trim()) return [];
  return parseTclList(checksRaw).map((chunk) => {
    const d = parseTclDictNested(chunk);
    return { id: d.id ?? "", level: d.level ?? "info", message: d.message ?? chunk };
  });
}

function decimals(step: number): number {
  const s = String(step);
  const dot = s.indexOf(".");
  return dot < 0 ? 0 : s.length - dot - 1;
}

function formatValue(v: number, step: number): string {
  return v.toFixed(decimals(step));
}

function snap(v: number, step: number): number {
  return Number((Math.round(v / step) * step).toFixed(decimals(step)));
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, v));
}

function el(tag: string, cls: string, text?: string): HTMLElement {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}
