import type { Connection } from "./connection";
import { parseTclList, tclDoubleQuoted } from "./tcl";

// The viewer's settings live on the server (tcl/viewer_settings.tcl), in one
// file, and every browser that connects adopts them. This is the browser's
// view of them: a cache of what the server holds, plus the calls that change it.
//
// Nothing is kept in the browser itself. The one exception is the one-time
// import of values older versions left in localStorage (see importLegacy).

type Pattern = string; // an exact key, or a prefix ending in "*"

const LEGACY_KEYS = [
  "vs.viewer.tuning",
  "vs.viewer.tuningOpen",
  "vs.viewer.autoRoi",
  "vs.viewer.cameraGain",
  "vs.viewer.layers",
  "vs.viewer.layersOpen",
  "vs.viewer.historyOpen",
  "vs.viewer.historyWindow",
  "vs.viewer.pickerSort",
  "vs.viewer.playbackSpeed",
];

const DETECTOR_KEYS = [
  "pupil_threshold",
  "p1_min_intensity",
  "p1_max_jump",
  "p1_min_area",
  "p1_max_area",
  "p1_pupil_radius_max",
  "p4_min_intensity",
  "p4_max_jump",
  "p4_max_prediction_error",
];

export class Settings {
  private conn: Connection | null = null;
  private values = new Map<string, string>();
  private subs: { pattern: Pattern; fn: (key: string) => void }[] = [];
  // Puts sent and not yet answered, per key. The server also announces our own
  // changes to us; those echoes are ignored so a stale one cannot undo a newer edit.
  private pending = new Map<string, number>();
  private loading: Promise<void> | null = null;
  private warnedNoStore = false;

  /** Give the settings the connection to the server. */
  bind(conn: Connection): void {
    this.conn = conn;
  }

  // ---- reading ----------------------------------------------------------------

  has(key: string): boolean {
    return this.values.has(key);
  }

  get(key: string): string | undefined {
    return this.values.get(key);
  }

  num(key: string, fallback: number): number {
    const v = Number(this.values.get(key));
    return this.values.has(key) && Number.isFinite(v) ? v : fallback;
  }

  flag(key: string, fallback: boolean): boolean {
    const v = this.values.get(key);
    return v === undefined ? fallback : v === "1";
  }

  json<T>(key: string, fallback: T): T {
    const v = this.values.get(key);
    if (v === undefined) return fallback;
    try {
      return JSON.parse(v) as T;
    } catch {
      return fallback;
    }
  }

  /** Saved keys that start with `prefix`. */
  keys(prefix: string): string[] {
    return [...this.values.keys()].filter((k) => k.startsWith(prefix));
  }

  // ---- changing ---------------------------------------------------------------

  /**
   * Change a setting on the server. Resolves with the value the server kept
   * (the camera may clamp it); rejects with the server's reason if it refuses.
   * The cache and subscribers see the new value at once.
   */
  async put(key: string, value: string | number | boolean): Promise<string> {
    const text = typeof value === "boolean" ? (value ? "1" : "0") : String(value);
    this.apply(key, text);
    const conn = this.conn;
    if (!conn) return text;
    this.pending.set(key, (this.pending.get(key) ?? 0) + 1);
    try {
      const kept = (await conn.sendEvalAsync(`::vs::put ${key} ${tclDoubleQuoted(text)}`)).trim();
      this.settle(key);
      this.apply(key, kept);
      return kept;
    } catch (e) {
      this.settle(key);
      const msg = e instanceof Error ? e.message : String(e);
      if (/invalid command name/.test(msg)) {
        // Started from a script that predates the store: nothing would apply this.
        this.noStore();
        throw new Error("This VideoStream was started before the settings update. Restart it to change settings.");
      }
      void this.load(); // show what the server really has
      throw e instanceof Error ? e : new Error(msg);
    }
  }

  /** Fire-and-forget put for preferences; a refusal is only logged. */
  set(key: string, value: string | number | boolean): void {
    this.put(key, value).catch((e) => console.warn(`could not save ${key}:`, e));
  }

  /** Forget saved keys on the server. */
  drop(...keys: string[]): void {
    for (const k of keys) this.remove(k);
    if (!this.conn || !keys.length) return;
    void this.conn.sendEvalAsync(`::vs::drop ${keys.join(" ")}`).catch(() => {});
  }

  /** Forget a group of saved values and put the script's own defaults back. */
  async reset(group: "detector" | "roi" | "camera" | "ui" | "all"): Promise<void> {
    if (!this.conn) return;
    await this.conn.sendEvalAsync(`::vs::reset ${group}`);
    await this.load();
  }

  // ---- notification -----------------------------------------------------------

  /** Call `fn(key)` after a matching key changes or is removed. Pattern: a key, or a prefix + "*". */
  on(pattern: Pattern, fn: (key: string) => void): void {
    this.subs.push({ pattern, fn });
  }

  // ---- from the server --------------------------------------------------------

  /** Read everything the server holds (on every connect). Calls that arrive while one is running share it. */
  load(): Promise<void> {
    if (this.loading) return this.loading;
    const conn = this.conn;
    if (!conn) return Promise.resolve();
    this.loading = (async () => {
      try {
        const raw = await conn.sendEvalAsync("::vs::all");
        const list = parseTclList(raw);
        const next = new Map<string, string>();
        for (let i = 0; i + 1 < list.length; i += 2) next.set(list[i], list[i + 1]);
        const changed = new Set<string>();
        for (const [k, v] of next) if (this.values.get(k) !== v) changed.add(k);
        for (const k of this.values.keys()) if (!next.has(k)) changed.add(k);
        this.values = next;
        for (const k of changed) this.notify(k);
        await this.importLegacy();
      } catch (e) {
        const msg = e instanceof Error ? e.message : String(e);
        if (/invalid command name/.test(msg)) this.noStore();
        else console.warn("could not read the server's settings:", e);
      } finally {
        this.loading = null;
      }
    })();
    return this.loading;
  }

  /** A server event; settings events change the cache. */
  handleEvent(event: string, data: unknown): void {
    if (typeof data !== "string") return;
    if (event === "vstream/settings") {
      const [key, value] = parseTclList(data);
      if (key !== undefined && value !== undefined && !this.pending.get(key)) this.apply(key, value);
    } else if (event === "vstream/settings_unset") {
      for (const key of parseTclList(data)) if (!this.pending.get(key)) this.remove(key);
    }
  }

  // ---- internals --------------------------------------------------------------

  private settle(key: string): void {
    const n = (this.pending.get(key) ?? 1) - 1;
    if (n > 0) this.pending.set(key, n);
    else this.pending.delete(key);
  }

  private apply(key: string, value: string): void {
    if (this.values.get(key) === value) return;
    this.values.set(key, value);
    this.notify(key);
  }

  private remove(key: string): void {
    if (this.values.delete(key)) this.notify(key);
  }

  private notify(key: string): void {
    for (const { pattern, fn } of this.subs) {
      if (pattern === key || (pattern.endsWith("*") && key.startsWith(pattern.slice(0, -1)))) fn(key);
    }
  }

  private noStore(): void {
    if (this.warnedNoStore) return;
    this.warnedNoStore = true;
    console.warn("This VideoStream's startup script has no settings store (it was started before the update); restart it.");
  }

  /**
   * Older versions kept settings in this browser. Hand any that are left to the
   * server (only for keys it does not have yet, so the first browser wins), then
   * forget them here.
   */
  private async importLegacy(): Promise<void> {
    let ls: Storage;
    try {
      ls = localStorage;
    } catch {
      return;
    }
    const read = (k: string): string | null => {
      try {
        return ls.getItem(k);
      } catch {
        return null;
      }
    };
    const offer = (key: string, value: unknown): void => {
      if (value === undefined || value === null || this.has(key)) return;
      this.set(key, value as string | number);
    };
    const parse = (k: string): Record<string, unknown> => {
      try {
        const o = JSON.parse(read(k) ?? "{}");
        return o && typeof o === "object" ? (o as Record<string, unknown>) : {};
      } catch {
        return {};
      }
    };

    if (!LEGACY_KEYS.some((k) => read(k) !== null)) return;

    const tuning = parse("vs.viewer.tuning");
    for (const name of DETECTOR_KEYS) if (typeof tuning[name] === "number") offer(`det.${name}`, tuning[name]);
    if (typeof tuning.detection_mode === "string") offer("det.mode", tuning.detection_mode);

    const gains = parse("vs.viewer.cameraGain"); // { "lucid:serial": dB }
    const seen = new Set<string>();
    for (const [camera, db] of Object.entries(gains)) {
      const vendor = camera.split(":")[0];
      if (typeof db === "number" && !seen.has(vendor)) {
        seen.add(vendor);
        offer(`cam.${vendor}.gain`, db);
      }
    }

    const flag = (legacy: string, key: string) => {
      const v = read(legacy);
      if (v === "0" || v === "1") offer(key, v);
    };
    flag("vs.viewer.autoRoi", "roi.follow");
    flag("vs.viewer.tuningOpen", "ui.tuningOpen");
    flag("vs.viewer.layersOpen", "ui.layersOpen");
    flag("vs.viewer.historyOpen", "ui.history.open");

    const layers = read("vs.viewer.layers");
    if (layers) offer("ui.layers", layers);
    const win = Number(read("vs.viewer.historyWindow"));
    if (Number.isInteger(win) && win > 0) offer("ui.history.window", win);
    const sort = read("vs.viewer.pickerSort");
    if (sort) offer("ui.pickerSort", sort);
    const speed = Number(read("vs.viewer.playbackSpeed"));
    if (Number.isFinite(speed) && speed > 0) offer("playback.speed", speed);

    for (const k of LEGACY_KEYS) {
      try {
        ls.removeItem(k);
      } catch {
        /* ignore */
      }
    }
  }
}

export const settings = new Settings();
