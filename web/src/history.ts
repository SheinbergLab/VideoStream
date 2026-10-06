import type { EyeTrackingOverlay } from "./protocol";
import { LAYERS } from "./overlay/eyetracking";
import { settings } from "./settings";

const WINDOW_OPTIONS = [1, 2, 5, 10] as const;
// Kept by the server (ui.history.open / ui.history.window), the same in every browser.
const OPEN_SETTING = "ui.history.open";
const WINDOW_SETTING = "ui.history.window";

const layerColor = Object.fromEntries(LAYERS.map((l) => [l.id, l.color])) as Record<string, string>;
const MISSING = "#353535";
const BLINK = "rgba(255, 190, 60, 0.22)";
const LOST = "rgba(255, 60, 60, 0.28)";

interface Sample {
  frame: number;
  pupil: boolean;
  p1: boolean;
  p4: boolean;
  radius: number | null;
  blink: boolean;
  lost: boolean;
}

type Lane = { key: "pupil" | "p1" | "p4"; label: string; color: string };

interface PlotGeom {
  plotX: number;
  plotW: number;
  span: number;
  newest: number;
  plotBottom: number;
}

/**
 * Rolling view of tracking: one status lane per tracked feature plus a pupil
 * radius trace. Time is source time (analysis frame / source fps), so the plot
 * freezes while paused and spans the chosen window of video at any speed.
 */
export class HistoryPlot {
  private samples: Sample[] = [];
  private fps = 0;
  private mode: EyeTrackingOverlay["mode"] = "pupil_p1";
  private sourceKey = "";
  private dirty = true;
  private open: boolean;
  private windowS: number;
  private seekEnabled = false;
  private plotGeom: PlotGeom | null = null;

  constructor(
    private root: HTMLElement,
    private toggle: HTMLButtonElement,
    private canvas: HTMLCanvasElement,
    windowSelect: HTMLSelectElement,
    private onSeek: (frame: number, alreadyThere: boolean) => void,
  ) {
    this.open = settings.flag(OPEN_SETTING, true);
    this.windowS = loadWindowS();
    windowSelect.value = String(this.windowS);
    windowSelect.addEventListener("change", () => {
      this.windowS = Number(windowSelect.value);
      settings.set(WINDOW_SETTING, windowSelect.value);
      this.trimToWindow();
      this.dirty = true;
      this.draw();
    });
    this.applyOpen();
    toggle.addEventListener("click", () => {
      this.open = !this.open;
      settings.set(OPEN_SETTING, this.open);
      this.applyOpen();
    });
    settings.on(OPEN_SETTING, () => {
      this.open = settings.flag(OPEN_SETTING, true);
      this.applyOpen();
    });
    settings.on(WINDOW_SETTING, () => {
      this.windowS = loadWindowS();
      windowSelect.value = String(this.windowS);
      this.trimToWindow();
      this.dirty = true;
      this.draw();
    });
    new ResizeObserver(() => {
      this.dirty = true;
      this.draw();
    }).observe(canvas);
    canvas.addEventListener("click", (ev) => this.onClick(ev));
  }

  /** Review playback can jump to a clicked frame. Live view leaves the plot inert. */
  setSeekEnabled(enabled: boolean): void {
    this.seekEnabled = enabled;
    this.canvas.classList.toggle("seekable", enabled);
    this.canvas.title = enabled ? "Click to go to this frame and pause" : "";
  }

  /** Feed every displayed frame; repeated analysis frames (paused) are ignored. */
  push(ov: EyeTrackingOverlay | undefined, srcFps: number, sourceKey: string): void {
    if (sourceKey !== this.sourceKey) {
      this.sourceKey = sourceKey;
      this.clear();
    }
    if (!ov || !ov.valid || ov.frame_id === undefined) return;
    const frameId = ov.frame_id;
    if (srcFps > 0) this.fps = srcFps;
    if (ov.mode !== this.mode) {
      this.mode = ov.mode;
      this.dirty = true;
    }
    const last = this.samples[this.samples.length - 1];
    if (last && frameId === last.frame) return;
    if (last && frameId < last.frame) {
      const oldest = this.samples[0].frame;
      if (frameId < oldest) this.samples = [];
      else this.samples = this.samples.filter((s) => s.frame <= frameId);
    }
    const tail = this.samples[this.samples.length - 1];
    if (tail && frameId === tail.frame) {
      this.trimToWindow(frameId);
      this.dirty = true;
      this.draw();
      return;
    }
    this.samples.push({
      frame: frameId,
      pupil: Boolean(ov.pupil?.detected),
      p1: Boolean(ov.p1?.detected),
      p4: Boolean(ov.p4?.detected),
      radius: ov.pupil?.detected && ov.pupil.r !== undefined ? ov.pupil.r : null,
      blink: Boolean(ov.in_blink),
      lost: Boolean(ov.tracking_lost),
    });
    this.trimToWindow(frameId);
    this.dirty = true;
    this.draw();
  }

  clear(): void {
    this.samples = [];
    this.plotGeom = null;
    this.dirty = true;
    this.draw();
  }

  private onClick(ev: MouseEvent): void {
    if (!this.seekEnabled || !this.open) return;
    const g = this.plotGeom;
    if (!g || g.span <= 0) return;
    const x = ev.offsetX;
    const y = ev.offsetY;
    if (x < g.plotX || x > g.plotX + g.plotW || y < 0 || y > g.plotBottom) return;
    const frame = Math.round(g.newest - (g.span * (g.plotX + g.plotW - x)) / g.plotW);
    this.onSeek(frame, frame === g.newest);
  }

  private spanFrames(): number {
    return (this.fps > 0 ? this.fps : 30) * this.windowS;
  }

  private trimToWindow(newest?: number): void {
    const end = newest ?? this.samples[this.samples.length - 1]?.frame;
    if (end === undefined) return;
    const left = end - this.spanFrames();
    // Keep the sample just left of the window. It anchors the bar and radius
    // segment that are still crossing the left edge.
    while (this.samples.length >= 2 && this.samples[1].frame < left) this.samples.shift();
    if (this.samples.length === 1 && this.samples[0].frame < left) this.samples.shift();
  }

  private applyOpen(): void {
    this.root.classList.toggle("collapsed", !this.open);
    this.toggle.setAttribute("aria-expanded", String(this.open));
    this.dirty = true;
    this.draw();
  }

  private lanes(): Lane[] {
    const lanes: Lane[] = [{ key: "pupil", label: "Pupil", color: layerColor.pupil }];
    if (this.mode !== "pupil_only") lanes.push({ key: "p1", label: "P1", color: layerColor.p1 });
    if (this.mode === "full") lanes.push({ key: "p4", label: "P4", color: layerColor.p4 });
    return lanes;
  }

  private draw(): void {
    if (!this.dirty || !this.open) return;
    this.dirty = false;
    const c = this.canvas;
    const dpr = window.devicePixelRatio || 1;
    const cssW = c.clientWidth;
    const cssH = c.clientHeight;
    if (cssW === 0 || cssH === 0) return;
    const pw = Math.round(cssW * dpr);
    const ph = Math.round(cssH * dpr);
    if (c.width !== pw || c.height !== ph) {
      c.width = pw;
      c.height = ph;
    }
    const ctx = c.getContext("2d")!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cssW, cssH);

    const labelW = 52;
    const valueW = 56;
    const plotX = labelW;
    const plotW = Math.max(1, cssW - labelW - valueW);
    const pad = 6;
    const lanes = this.lanes();
    const laneH = 10;
    const laneGap = 3;
    const lanesH = lanes.length * laneH + (lanes.length - 1) * laneGap;
    const traceTop = pad + lanesH + 8;
    const axisH = 13;
    const traceH = Math.max(12, cssH - traceTop - axisH);

    ctx.font = "11px system-ui, sans-serif";
    ctx.textBaseline = "middle";

    const samples = this.samples;
    const span = this.spanFrames();
    const newest = samples.length ? samples[samples.length - 1].frame : 0;
    const left = newest - span;
    const xOf = (frame: number) => plotX + plotW * (1 - (newest - frame) / span);
    const visible = samples.filter((s) => s.frame >= left);
    const seg = (i: number) => {
      const s = samples[i];
      const x0 = xOf(i > 0 ? samples[i - 1].frame : s.frame - 1);
      const x1 = xOf(s.frame);
      return { s, x0, x1, w: x1 - x0 };
    };

    // Marks draw past the left edge and the plot clips them, so a bar narrows
    // as it leaves instead of vanishing all at once.
    const plotBottom = traceTop + traceH;
    const clipPlot = () => {
      ctx.beginPath();
      ctx.rect(plotX, pad - 2, plotW, plotBottom - (pad - 2));
      ctx.clip();
    };
    ctx.save();
    clipPlot();
    for (let i = 0; i < samples.length; i++) {
      const { s, x0, w } = seg(i);
      if (!s.blink && !s.lost || w <= 0) continue;
      ctx.fillStyle = s.lost ? LOST : BLINK;
      ctx.fillRect(x0, pad - 2, w, plotBottom - (pad - 2));
    }
    ctx.restore();

    lanes.forEach((lane, li) => {
      const y = pad + li * (laneH + laneGap);
      ctx.fillStyle = "#9a9a9a";
      ctx.textAlign = "right";
      ctx.fillText(lane.label, labelW - 8, y + laneH / 2);
      ctx.fillStyle = "#202020";
      ctx.fillRect(plotX, y, plotW, laneH);
    });
    ctx.fillStyle = "#9a9a9a";
    ctx.textAlign = "right";
    ctx.fillText("Radius", labelW - 8, traceTop + traceH / 2);
    ctx.fillStyle = "#181818";
    ctx.fillRect(plotX, traceTop, plotW, traceH);

    ctx.save();
    clipPlot();
    lanes.forEach((lane, li) => {
      const y = pad + li * (laneH + laneGap);
      for (let i = 0; i < samples.length; i++) {
        const { s, x0, w } = seg(i);
        const bar = w - 0.5;
        if (bar <= 0) continue;
        ctx.fillStyle = s[lane.key] ? lane.color : MISSING;
        ctx.fillRect(x0, y, bar, laneH);
      }
    });
    ctx.restore();

    lanes.forEach((lane, li) => {
      const y = pad + li * (laneH + laneGap);
      const pct = visible.length
        ? Math.round((100 * visible.filter((s) => s[lane.key]).length) / visible.length)
        : null;
      ctx.textAlign = "left";
      ctx.fillStyle = pct === null ? "#666" : pct >= 95 ? "#9ad79a" : pct >= 80 ? "#e0c070" : "#e08080";
      ctx.fillText(pct === null ? "\u2013" : `${pct}%`, plotX + plotW + 8, y + laneH / 2);
    });

    // Pupil radius trace; y=0 at the bottom so deflection size is comparable over time.
    const radii = visible.map((s) => s.radius).filter((r): r is number => r !== null);
    if (radii.length) {
      const ymax = Math.max(...radii) * 1.05;
      const yOf = (r: number) => traceTop + traceH - 2 - (r / ymax) * (traceH - 4);
      ctx.save();
      ctx.beginPath();
      ctx.rect(plotX, traceTop, plotW, traceH);
      ctx.clip();
      ctx.strokeStyle = layerColor.pupil;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      let pen = false;
      for (const s of samples) {
        if (s.radius === null) {
          pen = false;
          continue;
        }
        const x = xOf(s.frame);
        const y = yOf(s.radius);
        if (pen) ctx.lineTo(x, y);
        else ctx.moveTo(x, y);
        pen = true;
      }
      ctx.stroke();
      ctx.restore();
      ctx.fillStyle = "#666";
      ctx.font = "10px system-ui, sans-serif";
      ctx.textAlign = "left";
      ctx.fillText(ymax.toFixed(0), plotX + 3, traceTop + 6);
      ctx.fillText("0", plotX + 3, traceTop + traceH - 6);
      ctx.font = "11px system-ui, sans-serif";
      const cur = samples[samples.length - 1].radius;
      ctx.fillStyle = "#c8c8c8";
      ctx.fillText(cur === null ? "\u2013" : `${cur.toFixed(1)}px`, plotX + plotW + 8, traceTop + traceH / 2);
    }

    ctx.fillStyle = "#666";
    ctx.font = "10px system-ui, sans-serif";
    ctx.textAlign = "left";
    const axisY = traceTop + traceH + axisH / 2 + 1;
    ctx.fillText(`\u2212${this.windowS} s`, plotX, axisY);
    ctx.textAlign = "right";
    ctx.fillText("now", plotX + plotW, axisY);

    this.plotGeom = samples.length
      ? { plotX, plotW, span, newest, plotBottom: cssH }
      : null;
  }
}

function loadWindowS(): number {
  const v = settings.num(WINDOW_SETTING, 1);
  return WINDOW_OPTIONS.includes(v as (typeof WINDOW_OPTIONS)[number]) ? v : 1;
}
