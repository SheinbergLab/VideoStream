import type { FrameHeader, FrameMessage } from "./protocol";

// Maps frame pixels to CSS pixels inside the stage.
export interface View {
  scale: number;
  offsetX: number;
  offsetY: number;
  dpr: number;
}

export type OverlayPainter = (ctx: CanvasRenderingContext2D, header: FrameHeader, view: View) => void;

// Diameter of the paused-feed loupe, in CSS pixels. Source pixels inside it
// are drawn at LOUPE_MAG times their current on-screen size.
const LOUPE_DIAMETER_CSS = 240;
const LOUPE_MAG = 6;

// Two stacked canvases: the decoded JPEG, and an overlay drawn in frame
// coordinates. Decoding is latest-wins: while one frame decodes, newer
// arrivals replace each other in a single pending slot.
export class Renderer {
  private videoCtx: CanvasRenderingContext2D;
  private overlayCtx: CanvasRenderingContext2D;
  private bitmap: ImageBitmap | null = null;
  private header: FrameHeader | null = null;
  private pending: FrameMessage | null = null;
  private decoding = false;
  private dirty = false;
  private fresh = false;
  private view: View = { scale: 1, offsetX: 0, offsetY: 0, dpr: 1 };
  // Frame-space point under the pointer while the paused loupe is showing.
  private loupe: { x: number; y: number } | null = null;
  // Short text shown above the loupe, or null for none.
  private loupeHint: string | null = null;

  // Frames that were replaced before they could be decoded.
  skipped = 0;
  onDisplayed: ((header: FrameHeader) => void) | null = null;

  constructor(
    private stage: HTMLElement,
    private videoCanvas: HTMLCanvasElement,
    private overlayCanvas: HTMLCanvasElement,
    private paintOverlay: OverlayPainter,
  ) {
    this.videoCtx = videoCanvas.getContext("2d", { alpha: false })!;
    this.overlayCtx = overlayCanvas.getContext("2d")!;
    new ResizeObserver(() => this.resize()).observe(stage);
    matchMedia(`(resolution: ${devicePixelRatio}dppx)`).addEventListener("change", () => this.resize());
    this.resize();
    requestAnimationFrame(this.tick);
  }

  submit(msg: FrameMessage): void {
    if (this.decoding) {
      if (this.pending) this.skipped++;
      this.pending = msg;
      return;
    }
    this.decode(msg);
  }

  // Repaint the overlay only (e.g. a layer was toggled).
  invalidate(): void {
    this.dirty = true;
  }

  getState(): { header: FrameHeader | null; view: View } {
    return { header: this.header, view: this.view };
  }

  // Frame coordinates of the pointer, or null to hide the loupe. `hint` is
  // drawn above it.
  setLoupe(frame: { x: number; y: number } | null, hint: string | null = null): void {
    if (frame === null) {
      if (this.loupe === null) return;
      this.loupe = null;
      this.loupeHint = null;
    } else if (
      this.loupe &&
      this.loupe.x === frame.x &&
      this.loupe.y === frame.y &&
      this.loupeHint === hint
    ) {
      return;
    } else {
      this.loupe = { x: frame.x, y: frame.y };
      this.loupeHint = hint;
    }
    this.dirty = true;
  }

  private async decode(msg: FrameMessage): Promise<void> {
    this.decoding = true;
    try {
      const bitmap = await createImageBitmap(new Blob([msg.jpeg as BlobPart], { type: "image/jpeg" }));
      this.bitmap?.close();
      this.bitmap = bitmap;
      this.header = msg.header;
      this.dirty = true;
      this.fresh = true;
    } catch {
      // corrupt frame: drop it
    }
    this.decoding = false;
    const next = this.pending;
    this.pending = null;
    if (next) this.decode(next);
  }

  private resize(): void {
    const dpr = window.devicePixelRatio || 1;
    const w = this.stage.clientWidth;
    const h = this.stage.clientHeight;
    for (const c of [this.videoCanvas, this.overlayCanvas]) {
      c.width = Math.max(1, Math.round(w * dpr));
      c.height = Math.max(1, Math.round(h * dpr));
    }
    this.view.dpr = dpr;
    this.dirty = true;
  }

  private fit(frameW: number, frameH: number): void {
    const w = this.stage.clientWidth;
    const h = this.stage.clientHeight;
    const scale = Math.min(w / frameW, h / frameH);
    this.view.scale = scale;
    this.view.offsetX = (w - frameW * scale) / 2;
    this.view.offsetY = (h - frameH * scale) / 2;
  }

  private tick = (): void => {
    requestAnimationFrame(this.tick);
    if (!this.dirty || !this.bitmap || !this.header) return;
    this.dirty = false;

    const { bitmap, header } = this;
    this.fit(bitmap.width, bitmap.height);
    const { scale, offsetX, offsetY, dpr } = this.view;

    const v = this.videoCtx;
    v.setTransform(1, 0, 0, 1, 0, 0);
    v.fillStyle = "#000";
    v.fillRect(0, 0, this.videoCanvas.width, this.videoCanvas.height);
    v.imageSmoothingEnabled = scale < 1;
    v.drawImage(bitmap, offsetX * dpr, offsetY * dpr, bitmap.width * scale * dpr, bitmap.height * scale * dpr);

    const o = this.overlayCtx;
    o.setTransform(1, 0, 0, 1, 0, 0);
    o.clearRect(0, 0, this.overlayCanvas.width, this.overlayCanvas.height);
    o.setTransform(scale * dpr, 0, 0, scale * dpr, offsetX * dpr, offsetY * dpr);
    this.paintOverlay(o, header, this.view);
    this.drawLoupe(bitmap);

    if (this.fresh) {
      this.fresh = false;
      this.onDisplayed?.(header);
    }
  };

  // Circular sample of the source bitmap, centered on the pointer, with the
  // pixel grid locked to source pixels.
  private drawLoupe(bitmap: ImageBitmap): void {
    const loupe = this.loupe;
    if (!loupe) return;
    const { scale, offsetX, offsetY, dpr } = this.view;
    const { x: fx, y: fy } = loupe;
    const radius = LOUPE_DIAMETER_CSS / 2;
    const magScale = LOUPE_MAG * scale;
    if (magScale <= 0) return;
    const cx = offsetX + fx * scale;
    const cy = offsetY + fy * scale;
    const o = this.overlayCtx;

    o.save();
    o.setTransform(dpr, 0, 0, dpr, 0, 0);
    o.beginPath();
    o.arc(cx, cy, radius, 0, Math.PI * 2);
    o.clip();
    o.fillStyle = "#000";
    o.fillRect(cx - radius, cy - radius, radius * 2, radius * 2);
    o.imageSmoothingEnabled = false;
    o.setTransform(
      magScale * dpr,
      0,
      0,
      magScale * dpr,
      (cx - fx * magScale) * dpr,
      (cy - fy * magScale) * dpr,
    );
    o.drawImage(bitmap, 0, 0);

    // Marks are drawn in frame coordinates. Screen-constant strokes use
    // 1/view.scale, so pass the magnified scale and the P1/P4 circles stay
    // their normal size while landing on the zoomed pixels. Lengths that are
    // real frame pixels (the pupil circle) grow with the glass.
    if (this.header) {
      const loupeView: View = { ...this.view, scale: this.view.scale * LOUPE_MAG };
      this.paintOverlay(o, this.header, loupeView);
    }

    o.restore();

    o.save();
    o.setTransform(dpr, 0, 0, dpr, 0, 0);
    o.beginPath();
    o.arc(cx, cy, radius - 1, 0, Math.PI * 2);
    o.strokeStyle = "rgba(255,255,255,0.92)";
    o.lineWidth = 2;
    o.stroke();
    if (this.loupeHint) this.drawLoupeHint(o, this.loupeHint, cx, cy, radius);
    o.restore();
  }

  // Label centered above the loupe (below it when there is no room above).
  private drawLoupeHint(
    o: CanvasRenderingContext2D,
    text: string,
    cx: number,
    cy: number,
    radius: number,
  ): void {
    o.font = "12px system-ui, sans-serif";
    o.textAlign = "center";
    o.textBaseline = "middle";
    const w = o.measureText(text).width + 14;
    const h = 20;
    const stageW = this.overlayCanvas.width / this.view.dpr;
    const above = cy - radius - 6 - h >= 0;
    const y = above ? cy - radius - 6 - h : cy + radius + 6;
    const x = Math.min(Math.max(cx - w / 2, 2), Math.max(2, stageW - w - 2));
    o.fillStyle = "rgba(0,0,0,0.75)";
    o.beginPath();
    o.roundRect(x, y, w, h, 4);
    o.fill();
    o.fillStyle = "#fff";
    o.fillText(text, x + w / 2, y + h / 2 + 0.5);
  }
}
