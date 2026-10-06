import type { FrameHeader } from "./protocol";
import type { View } from "./renderer";
import { settings } from "./settings";

export type RoiRect = { x: number; y: number; w: number; h: number };

type Handle = "n" | "s" | "e" | "w" | "ne" | "nw" | "se" | "sw";

const MIN = 32;
const HIT_PX = 8;

const CURSORS: Record<Handle, string> = {
  n: "ns-resize",
  s: "ns-resize",
  e: "ew-resize",
  w: "ew-resize",
  ne: "nesw-resize",
  nw: "nwse-resize",
  se: "nwse-resize",
  sw: "nesw-resize",
};

function clampRoi(r: RoiRect, fw: number, fh: number): RoiRect {
  let { x, y, w, h } = r;
  w = Math.max(MIN, Math.min(w, fw));
  h = Math.max(MIN, Math.min(h, fh));
  x = Math.max(0, Math.min(x, fw - w));
  y = Math.max(0, Math.min(y, fh - h));
  return { x: Math.round(x), y: Math.round(y), w: Math.round(w), h: Math.round(h) };
}

function screenToFrame(clientX: number, clientY: number, canvas: HTMLCanvasElement, view: View): Point {
  const rect = canvas.getBoundingClientRect();
  const cssX = clientX - rect.left;
  const cssY = clientY - rect.top;
  return {
    x: (cssX - view.offsetX) / view.scale,
    y: (cssY - view.offsetY) / view.scale,
  };
}

interface Point {
  x: number;
  y: number;
}

function hitHandle(fx: number, fy: number, roi: RoiRect, view: View): Handle | null {
  const t = HIT_PX / view.scale;
  const { x, y, w, h } = roi;
  const onLeft = Math.abs(fx - x) <= t;
  const onRight = Math.abs(fx - (x + w)) <= t;
  const onTop = Math.abs(fy - y) <= t;
  const onBottom = Math.abs(fy - (y + h)) <= t;
  const inX = fx >= x - t && fx <= x + w + t;
  const inY = fy >= y - t && fy <= y + h + t;

  if (onTop && onLeft) return "nw";
  if (onTop && onRight) return "ne";
  if (onBottom && onLeft) return "sw";
  if (onBottom && onRight) return "se";
  if (onTop && inX) return "n";
  if (onBottom && inX) return "s";
  if (onLeft && inY) return "w";
  if (onRight && inY) return "e";
  return null;
}

/** Edge/corner resize for the eyetracking ROI; commits via eyetracking::setROI. */
function roiNearEqual(a: RoiRect, b: RoiRect, eps = 2): boolean {
  return (
    Math.abs(a.x - b.x) < eps &&
    Math.abs(a.y - b.y) < eps &&
    Math.abs(a.w - b.w) < eps &&
    Math.abs(a.h - b.h) < eps
  );
}

/** Edge hit-testing uses the same ROI as the overlay (e.g. after Auto-ROI moves the crop). */
export function attachRoiEditor(
  canvas: HTMLCanvasElement,
  getState: () => { header: FrameHeader | null; view: View },
  getDisplayRoi: () => RoiRect | null,
  onPendingRoi: (roi: RoiRect | null) => void,
): void {
  let dragging: Handle | null = null;
  let startFrame: Point | null = null;
  let startRoi: RoiRect | null = null;
  let pending: RoiRect | null = null;
  let debounceTimer: number | undefined;

  const commit = (r: RoiRect) => {
    // The server applies it and keeps it as the saved ROI.
    settings.put("roi", `${r.x} ${r.y} ${r.w} ${r.h}`).catch((e) => console.warn("ROI not applied:", e));
  };

  const scheduleCommit = (r: RoiRect) => {
    window.clearTimeout(debounceTimer);
    debounceTimer = window.setTimeout(() => commit(r), 150);
  };

  const baseRoi = (): RoiRect | null => {
    if (dragging && pending) {
      return pending;
    }
    const shown = getDisplayRoi();
    if (pending && shown && !roiNearEqual(pending, shown)) {
      pending = null;
    }
    return shown;
  };

  canvas.addEventListener("pointermove", (ev) => {
    const { header, view } = getState();
    if (!header) return;
    const roi = baseRoi();
    if (!roi) {
      canvas.style.cursor = "";
      return;
    }
    const p = screenToFrame(ev.clientX, ev.clientY, canvas, view);
    if (dragging && startFrame && startRoi) {
      const dx = p.x - startFrame.x;
      const dy = p.y - startFrame.y;
      let { x, y, w, h } = startRoi;
      switch (dragging) {
        case "e":
          w = startRoi.w + dx;
          break;
        case "w":
          x = startRoi.x + dx;
          w = startRoi.w - dx;
          break;
        case "s":
          h = startRoi.h + dy;
          break;
        case "n":
          y = startRoi.y + dy;
          h = startRoi.h - dy;
          break;
        case "se":
          w = startRoi.w + dx;
          h = startRoi.h + dy;
          break;
        case "sw":
          x = startRoi.x + dx;
          w = startRoi.w - dx;
          h = startRoi.h + dy;
          break;
        case "ne":
          y = startRoi.y + dy;
          w = startRoi.w + dx;
          h = startRoi.h - dy;
          break;
        case "nw":
          x = startRoi.x + dx;
          y = startRoi.y + dy;
          w = startRoi.w - dx;
          h = startRoi.h - dy;
          break;
      }
      const next = clampRoi({ x, y, w, h }, header.width, header.height);
      pending = next;
      onPendingRoi(next);
      scheduleCommit(next);
      return;
    }
    const h = hitHandle(p.x, p.y, roi, view);
    canvas.style.cursor = h ? CURSORS[h] : "";
  });

  canvas.addEventListener("pointerdown", (ev) => {
    const { header, view } = getState();
    const roi = baseRoi();
    if (!header || !roi) return;
    const p = screenToFrame(ev.clientX, ev.clientY, canvas, view);
    const h = hitHandle(p.x, p.y, roi, view);
    if (!h) return;
    dragging = h;
    startFrame = p;
    startRoi = { ...roi };
    canvas.setPointerCapture(ev.pointerId);
    ev.preventDefault();
  });

  const endDrag = (ev: PointerEvent) => {
    if (!dragging) return;
    dragging = null;
    startFrame = null;
    startRoi = null;
    if (pending) {
      window.clearTimeout(debounceTimer);
      commit(pending);
    }
    pending = null;
    try {
      canvas.releasePointerCapture(ev.pointerId);
    } catch {
      /* ok */
    }
  };

  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);

  canvas.addEventListener("pointerleave", () => {
    if (!dragging) canvas.style.cursor = "";
  });
}
