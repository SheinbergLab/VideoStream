import type { FrameHeader, FrameMessage } from "./protocol";

// Rolling one-second figures for the status bar.
export class Stats {
  private windowStart = performance.now();
  private received = 0;
  private displayed = 0;
  private bytes = 0;
  private lastSeq = -1;
  private trackSumUs = 0;
  private trackCount = 0;
  private trackMaxUs = 0;

  // Totals since the page loaded.
  serverGaps = 0;

  // Latest values, refreshed once a second.
  rxFps = 0;
  displayFps = 0;
  kbps = 0;
  last: FrameHeader | null = null;
  lag: number | null = null;
  latencyMs: number | null = null;
  // Time to find pupil, P1 and P4 per frame (ms), mean and worst over the last second.
  trackMs: number | null = null;
  trackMaxMs: number | null = null;

  frameReceived(msg: FrameMessage): void {
    this.received++;
    this.bytes += msg.bytes;
    const seq = msg.header.seq;
    // seq counts every encoded frame; a jump means the server skipped us
    // (backpressure) or encoded for a faster client.
    if (this.lastSeq >= 0 && seq > this.lastSeq + 1) this.serverGaps += seq - this.lastSeq - 1;
    this.lastSeq = seq;
  }

  frameDisplayed(h: FrameHeader): void {
    this.displayed++;
    this.last = h;
    const et = h.overlay.eye_tracking;
    if (et?.valid && et.process_us !== undefined) {
      this.trackSumUs += et.process_us;
      this.trackCount++;
      this.trackMaxUs = Math.max(this.trackMaxUs, et.process_us);
    }
    if (et?.valid && et.frame_id !== undefined && et.frame_id >= 0) {
      this.lag = h.frame_id - et.frame_id;
    } else if (et?.valid && et.analysis_frame !== undefined && h.ring_size > 0) {
      // Older servers: ring slots only, so lag beyond ring_size wraps.
      this.lag = (((h.ring_index - et.analysis_frame) % h.ring_size) + h.ring_size) % h.ring_size;
    } else {
      this.lag = null;
    }
    // Only meaningful when server and browser clocks agree (same machine).
    this.latencyMs = Date.now() - h.ts_us / 1000;
  }

  // Returns true once per second when the rates have been recomputed.
  tick(): boolean {
    const now = performance.now();
    const dt = (now - this.windowStart) / 1000;
    if (dt < 1) return false;
    this.rxFps = this.received / dt;
    this.displayFps = this.displayed / dt;
    this.kbps = this.bytes / 1024 / dt;
    if (this.trackCount > 0) {
      this.trackMs = this.trackSumUs / this.trackCount / 1000;
      this.trackMaxMs = this.trackMaxUs / 1000;
    }
    this.trackSumUs = this.trackCount = this.trackMaxUs = 0;
    this.received = this.displayed = this.bytes = 0;
    this.windowStart = now;
    return true;
  }

  resetSeq(): void {
    this.lastSeq = -1;
  }
}
