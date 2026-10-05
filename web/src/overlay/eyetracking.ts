import type { EyeTrackingOverlay } from "../protocol";
import type { View } from "../renderer";

// Live marks each have their own hue. Saved tracking stays one orange ghost.
export type LayerGroup = "live" | "stored" | "general";

export const LAYERS = [
  {
    id: "roi",
    label: "ROI",
    hint: "Search box. The tracker only looks for the eye inside this region.",
    color: "#2dd4bf",
    group: "live" as const,
  },
  {
    id: "pupil",
    label: "Pupil",
    hint: "Circle the tracker follows. The cross is the pupil's center of mass, and the radius is what the P1 and P4 search, and blink detection, are based on.",
    color: "#3ddc6a",
    group: "live" as const,
  },
  {
    id: "pupil_ellipse",
    label: "Pupil ellipse",
    hint: "Outline fitted to the pupil, for measuring its size. The long and short axes stay meaningful when the camera sees the pupil at an angle. Tracking does not follow this shape.",
    color: "#b6f5c8",
    group: "live" as const,
  },
  {
    id: "p1",
    label: "P1",
    hint: "First Purkinje image: the bright reflection on the cornea.",
    color: "#60a5fa",
    group: "live" as const,
  },
  {
    id: "p4",
    label: "P4",
    hint: "Fourth Purkinje image: the reflection from the back of the lens.",
    color: "#c084fc",
    group: "live" as const,
  },
  {
    id: "predicted",
    label: "P4 search",
    hint: "Dashed box where the tracker expects to find P4.",
    color: "#e4c8ff",
    group: "live" as const,
  },
  {
    id: "ref_pupil",
    label: "Pupil",
    hint: "Tracked pupil circle from the reference recording: the center of mass and the radius the search used.",
    color: "#ffa500",
    group: "stored" as const,
  },
  {
    id: "reference_ellipse",
    label: "Pupil ellipse",
    hint: "Pupil-size ellipse from the reference recording. This is the size measurement, not the circle the tracker followed.",
    color: "#ffa500",
    group: "stored" as const,
  },
  {
    id: "ref_p1",
    label: "P1",
    hint: "P1 from the loaded reference recording.",
    color: "#ffa500",
    group: "stored" as const,
  },
  {
    id: "ref_p4",
    label: "P4",
    hint: "P4 from the loaded reference recording.",
    color: "#ffa500",
    group: "stored" as const,
  },
  {
    id: "labels",
    label: "Labels",
    hint: "Names drawn next to the live marks.",
    color: "#ffffff",
    group: "general" as const,
  },
] as const;

export type LayerId = (typeof LAYERS)[number]["id"];
export type LayerState = Record<LayerId, boolean>;

/** Marks shown instead of the live overlay while auto-tune waits for Accept or Undo. */
export interface TuneProposal {
  pupil: { x: number; y: number; r: number };
  roi: { x: number; y: number; w: number; h: number };
  p1: { x: number; y: number } | null;
  p4: { x: number; y: number };
  /** True when auto-tune did not find P4 and this marker is a placeholder. */
  p4Placed: boolean;
}

export const STORED_LAYER_IDS = LAYERS.filter((l) => l.group === "stored").map((l) => l.id);

const color = Object.fromEntries(LAYERS.map((l) => [l.id, l.color])) as Record<LayerId, string>;

/** Contour ellipse fit (semi-axes a,b; angle in degrees for canvas ctx.ellipse). */
export function pupilHasEllipse(pupil: EyeTrackingOverlay["pupil"] | undefined): boolean {
  if (!pupil?.detected || pupil.x === undefined || pupil.y === undefined) return false;
  const a = pupil.a ?? 0;
  const b = pupil.b ?? 0;
  return a > 0 && b > 0 && pupil.angle !== undefined;
}

/** Proposal marks. Labels stay on even though the layer checkboxes are cleared. */
export function drawTuneProposal(ctx: CanvasRenderingContext2D, p: TuneProposal, view: View): void {
  const px = 1 / view.scale;
  ctx.lineWidth = 1.5 * px;
  ctx.font = `${12 * px}px system-ui, sans-serif`;
  ctx.textBaseline = "middle";
  const label = (text: string, x: number, y: number, c: string) => {
    const prevWidth = ctx.lineWidth;
    ctx.textAlign = "left";
    ctx.lineWidth = 3 * px;
    ctx.strokeStyle = "rgba(0,0,0,0.85)";
    ctx.lineJoin = "round";
    ctx.strokeText(text, x, y);
    ctx.fillStyle = c;
    ctx.fillText(text, x, y);
    ctx.lineWidth = prevWidth;
  };
  const { pupil, p1, p4 } = p;
  ctx.strokeStyle = color.pupil;
  ctx.beginPath();
  ctx.arc(pupil.x, pupil.y, pupil.r, 0, 2 * Math.PI);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(pupil.x - 6 * px, pupil.y);
  ctx.lineTo(pupil.x + 6 * px, pupil.y);
  ctx.moveTo(pupil.x, pupil.y - 6 * px);
  ctx.lineTo(pupil.x, pupil.y + 6 * px);
  ctx.stroke();
  label("Pupil", pupil.x + pupil.r + 6 * px, pupil.y, color.pupil);
  if (p1) {
    ctx.strokeStyle = color.p1;
    ctx.beginPath();
    ctx.arc(p1.x, p1.y, 4 * px, 0, 2 * Math.PI);
    ctx.stroke();
    label("P1", p1.x + 8 * px, p1.y, color.p1);
  }
  ctx.strokeStyle = color.p4;
  ctx.beginPath();
  ctx.arc(p4.x, p4.y, 5 * px, 0, 2 * Math.PI);
  ctx.stroke();
  label("P4", p4.x + 8 * px, p4.y, color.p4);
}

// The context is already transformed to frame coordinates; `px` converts a
// size in screen pixels to frame units so strokes stay thin at any zoom.
export function drawEyeTracking(
  ctx: CanvasRenderingContext2D,
  ov: EyeTrackingOverlay,
  layers: LayerState,
  view: View,
  showStored = true,
): void {
  const px = 1 / view.scale;
  ctx.lineWidth = 1.5 * px;
  ctx.font = `${12 * px}px system-ui, sans-serif`;
  ctx.textBaseline = "middle";

  const label = (
    text: string,
    x: number,
    y: number,
    c: string,
    align: CanvasTextAlign = "left",
  ) => {
    if (!layers.labels) return;
    const prevAlign = ctx.textAlign;
    const prevWidth = ctx.lineWidth;
    ctx.textAlign = align;
    ctx.lineWidth = 3 * px;
    ctx.strokeStyle = "rgba(0,0,0,0.85)";
    ctx.lineJoin = "round";
    ctx.strokeText(text, x, y);
    ctx.fillStyle = c;
    ctx.fillText(text, x, y);
    ctx.lineWidth = prevWidth;
    ctx.textAlign = prevAlign;
  };

  const cross = (x: number, y: number, half: number) => {
    ctx.beginPath();
    ctx.moveTo(x - half, y);
    ctx.lineTo(x + half, y);
    ctx.moveTo(x, y - half);
    ctx.lineTo(x, y + half);
    ctx.stroke();
  };

  const roi = ov.roi;
  const roiWarn = ov.roi_violation || pupilViolatesRoi(ov.pupil, roi);
  if (layers.roi && roi) {
    ctx.strokeStyle = roiWarn ? "#ff4fd8" : color.roi;
    ctx.lineWidth = 2 * px;
    ctx.strokeRect(roi.x, roi.y, roi.w, roi.h);
    ctx.lineWidth = 1.5 * px;
    label("ROI", roi.x + 5 * px, roi.y + 12 * px, color.roi);
  }

  if (!ov.valid) return;

  const pupil = ov.pupil;
  if (layers.pupil && pupil?.detected && pupil.x !== undefined && pupil.y !== undefined) {
    ctx.strokeStyle = color.pupil;
    ctx.lineWidth = 2 * px;
    ctx.beginPath();
    ctx.arc(pupil.x, pupil.y, pupil.r ?? 0, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.lineWidth = 1 * px;
    cross(pupil.x, pupil.y, 5 * px);
    ctx.lineWidth = 1.5 * px;
    label("Pupil", pupil.x + (pupil.r ?? 0) + 5 * px, pupil.y, color.pupil);
  }

  if (layers.pupil_ellipse && pupilHasEllipse(pupil)) {
    const cx = pupil!.ex ?? pupil!.x!;
    const cy = pupil!.ey ?? pupil!.y!;
    const rw = pupil!.a!;
    const rh = pupil!.b!;
    // Same parameterization as cv::ellipse(RotatedRect): center, semi width/height,
    // angle in degrees on the width axis (verified vs ellipse2Poly).
    const rot = ((pupil!.angle ?? 0) * Math.PI) / 180;
    ctx.strokeStyle = color.pupil_ellipse;
    ctx.lineWidth = 2 * px;
    ctx.setLineDash([6 * px, 4 * px]);
    ctx.beginPath();
    ctx.ellipse(cx, cy, rw, rh, rot, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.lineWidth = 1.5 * px;
    const extX = Math.hypot(rw * Math.cos(rot), rh * Math.sin(rot));
    label("Pupil ell.", cx - extX - 5 * px, cy, color.pupil_ellipse, "right");
  }

  const p1 = ov.p1;
  if (layers.p1 && p1?.detected && p1.x !== undefined && p1.y !== undefined) {
    ctx.strokeStyle = color.p1;
    ctx.beginPath();
    ctx.arc(p1.x, p1.y, 4 * px, 0, 2 * Math.PI);
    ctx.stroke();
    cross(p1.x, p1.y, 3 * px);
    label("P1", p1.x + 8 * px, p1.y, color.p1);
  }

  const p4 = ov.p4;
  if (layers.p4 && p4?.detected && p4.x !== undefined && p4.y !== undefined) {
    ctx.strokeStyle = color.p4;
    ctx.beginPath();
    ctx.arc(p4.x, p4.y, 5 * px, 0, 2 * Math.PI);  // circle only: a cross hides the spot
    ctx.stroke();
    label("P4", p4.x + 8 * px, p4.y, color.p4);
  }

  const pred = ov.p4_predicted;
  if (layers.predicted && pred && pred.w > 0 && pred.h > 0) {
    ctx.strokeStyle = color.predicted;
    ctx.lineWidth = 1 * px;
    ctx.setLineDash([4 * px, 3 * px]);
    ctx.strokeRect(pred.x - pred.w / 2, pred.y - pred.h / 2, pred.w, pred.h);
    ctx.setLineDash([]);
    ctx.lineWidth = 1.5 * px;
  }

  if (!showStored) return;

  const ref = ov.reference;
  const storedStroke = color.ref_pupil;
  ctx.lineWidth = 1 * px;

  if (layers.ref_pupil && ref?.pupil) {
    ctx.strokeStyle = storedStroke;
    ctx.beginPath();
    ctx.arc(ref.pupil.x, ref.pupil.y, ref.pupil.r, 0, 2 * Math.PI);
    ctx.stroke();
  }

  const h = 5 * px;
  if (layers.ref_p1 && ref?.p1) {
    ctx.strokeStyle = storedStroke;
    ctx.beginPath();
    ctx.moveTo(ref.p1.x, ref.p1.y - h);
    ctx.lineTo(ref.p1.x + h, ref.p1.y);
    ctx.lineTo(ref.p1.x, ref.p1.y + h);
    ctx.lineTo(ref.p1.x - h, ref.p1.y);
    ctx.closePath();
    ctx.stroke();
  }

  if (layers.ref_p4 && ref?.p4) {
    ctx.strokeStyle = storedStroke;
    ctx.strokeRect(ref.p4.x - h, ref.p4.y - h, 2 * h, 2 * h);
  }

  // Stored ellipse has no fit center, so it is drawn on the stored centroid.
  // pupil_angle is the minor-axis direction: rx = minor (b), ry = major (a).
  const rp = ref?.pupil;
  if (layers.reference_ellipse && rp && rp.a && rp.b && rp.angle !== undefined) {
    ctx.strokeStyle = color.reference_ellipse;
    ctx.lineWidth = 1.5 * px;
    ctx.setLineDash([6 * px, 4 * px]);
    ctx.beginPath();
    ctx.ellipse(rp.x, rp.y, rp.b, rp.a, (rp.angle * Math.PI) / 180, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.setLineDash([]);
  }
}

export interface Badge {
  text: string;
  kind: "ok" | "warn" | "bad" | "info";
  title?: string;
}

export type RoiBox = { x: number; y: number; w: number; h: number };

/** Pupil center or circle extends outside the ROI (matches server logic). */
export function pupilViolatesRoi(
  pupil: EyeTrackingOverlay["pupil"] | undefined,
  roi: RoiBox | undefined,
): boolean {
  if (!pupil?.detected || pupil.x === undefined || pupil.y === undefined || !roi) return false;
  const px = pupil.x;
  const py = pupil.y;
  const r = pupil.r ?? 0;
  if (px < roi.x || px >= roi.x + roi.w || py < roi.y || py >= roi.y + roi.h) return true;
  if (r > 0) {
    if (px - r < roi.x || px + r > roi.x + roi.w || py - r < roi.y || py + r > roi.y + roi.h) {
      return true;
    }
  }
  return false;
}

export function eyeTrackingBadges(
  ov: EyeTrackingOverlay | undefined,
  roiOverride?: RoiBox,
): Badge[] {
  if (!ov) return [{ text: "no eyetracking", kind: "warn" }];
  const badges: Badge[] = [];
  if (!ov.valid) badges.push({ text: "no results", kind: "warn" });
  if (ov.tracking_lost) badges.push({ text: "TRACKING LOST", kind: "bad" });
  if (ov.in_blink) badges.push({ text: "BLINK", kind: "warn" });
  const roi = roiOverride ?? ov.roi;
  if (ov.roi_violation || pupilViolatesRoi(ov.pupil, roi)) {
    badges.push({
      text: "ROI VIOLATION",
      kind: "warn",
      title: "Pupil center or edge is outside the analysis ROI.",
    });
  }
  return badges;
}
