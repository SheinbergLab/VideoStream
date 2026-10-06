import {
  parseFrameMessage,
  type BrowseResponse,
  type FrameMessage,
  type PreviewCommand,
  type SourcesResponse,
} from "./protocol";

/** One line of the server's console output. */
export interface LogLine {
  seq: number;
  err?: boolean;
  text: string;
}

export interface ConnectionOptions {
  fps: number;
  quality?: number;
  onFrame: (msg: FrameMessage) => void;
  onStatus: (connected: boolean, url: string) => void;
  /** Welcome from the server: true when this browser is on the server machine. */
  onWelcome?: (local: boolean) => void;
  /** A server event (`vstream/*`): its name and data, which is a string for the settings events. */
  onEvent?: (event: string, data: unknown) => void;
  /** Server console lines, while setLogs(true) is in effect. */
  onLog?: (lines: LogLine[]) => void;
  /** The server restarted (its line numbers started over): drop what was shown. */
  onLogReset?: () => void;
  onEvalError?: (message: string) => void;
  onEvalOk?: () => void;
}

type JsonMsg = Record<string, unknown>;

type Pending = {
  resolve: (v: JsonMsg) => void;
  reject: (e: Error) => void;
};

// ws://<page host>/ws, or ?ws=host:port to point at another VideoStream.
export function resolveWsUrl(): string {
  const params = new URLSearchParams(location.search);
  const override = params.get("ws");
  if (override) {
    return /^wss?:\/\//.test(override) ? override : `ws://${override}/ws`;
  }
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  return `${scheme}://${location.host}/ws`;
}

export class Connection {
  private ws: WebSocket | null = null;
  private retryMs = 500;
  private retryTimer: number | undefined;
  private reqId = 0;
  private pending = new Map<string, Pending>();
  readonly url = resolveWsUrl();
  // Console stream state, kept so a reconnect resumes where it left off.
  private logsWanted = false;
  private logsLast = 0;

  constructor(private opts: ConnectionOptions) {}

  start(): void {
    this.connect();
  }

  /** Runs Tcl on the host (blocks the WS thread until the main loop answers). */
  sendEval(script: string): void {
    this.sendRaw({ cmd: "eval", script });
  }

  /** Eval with response (for source switch). */
  sendEvalAsync(script: string): Promise<string> {
    return this.request({ cmd: "eval", script }).then((j) => {
      if (j.status === "error") throw new Error(String(j.error ?? "eval failed"));
      return String(j.result ?? "");
    });
  }

  /** Change this client's preview rate; `null` restores the configured rate. */
  setPreviewFps(fps: number | null): void {
    const cmd: PreviewCommand = { cmd: "preview", enable: true, fps: fps ?? this.opts.fps };
    if (this.opts.quality) cmd.quality = this.opts.quality;
    this.sendRaw(cmd);
  }

  fetchSources(): Promise<SourcesResponse> {
    return this.request({ cmd: "sources" }).then((j) => j as unknown as SourcesResponse);
  }

  browsePath(path?: string): Promise<BrowseResponse> {
    const p = path ? { cmd: "browse", path } : { cmd: "browse" };
    return this.request(p).then((j) => j as unknown as BrowseResponse);
  }

  /** Start or stop the server console stream; it resumes after a reconnect. */
  setLogs(enable: boolean): void {
    this.logsWanted = enable;
    this.sendRaw({ cmd: "logs", enable, since: this.logsLast });
  }

  private sendRaw(obj: object): void {
    const ws = this.ws;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify(obj));
  }

  private request(payload: object): Promise<JsonMsg> {
    const ws = this.ws;
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      return Promise.reject(new Error("not connected"));
    }
    const requestId = `r${++this.reqId}`;
    return new Promise((resolve, reject) => {
      const timer = window.setTimeout(() => {
        this.pending.delete(requestId);
        reject(new Error("request timeout"));
      }, 30000);
      this.pending.set(requestId, {
        resolve: (v) => {
          window.clearTimeout(timer);
          resolve(v);
        },
        reject: (e) => {
          window.clearTimeout(timer);
          reject(e);
        },
      });
      ws.send(JSON.stringify({ ...payload, requestId }));
    });
  }

  private connect(): void {
    const ws = new WebSocket(this.url);
    ws.binaryType = "arraybuffer";
    this.ws = ws;

    ws.onopen = () => {
      this.retryMs = 500;
      this.opts.onStatus(true, this.url);
      const cmd: PreviewCommand = { cmd: "preview", enable: true, fps: this.opts.fps };
      if (this.opts.quality) cmd.quality = this.opts.quality;
      ws.send(JSON.stringify(cmd));
    };

    ws.onmessage = (ev) => {
      if (ev.data instanceof ArrayBuffer) {
        const msg = parseFrameMessage(ev.data);
        if (msg) this.opts.onFrame(msg);
        return;
      }
      if (typeof ev.data !== "string") return;
      try {
        const j = JSON.parse(ev.data) as JsonMsg;
        if (j.type === "welcome") {
          this.opts.onWelcome?.(j.local === true);
          const latest = typeof j.log_latest === "number" ? j.log_latest : 0;
          if (latest < this.logsLast) {
            this.logsLast = 0;
            this.opts.onLogReset?.();
          }
          if (this.logsWanted) this.sendRaw({ cmd: "logs", enable: true, since: this.logsLast });
          return;
        }
        if (j.type === "event" && typeof j.event === "string") {
          this.opts.onEvent?.(j.event, j.data);
          return;
        }
        if (j.type === "log" && Array.isArray(j.lines)) {
          const lines = j.lines as LogLine[];
          if (lines.length) this.logsLast = lines[lines.length - 1].seq;
          this.opts.onLog?.(lines);
          return;
        }
        const rid = j.requestId;
        if (typeof rid === "string" && this.pending.has(rid)) {
          const p = this.pending.get(rid)!;
          this.pending.delete(rid);
          if (j.status === "error") p.reject(new Error(String(j.error ?? "error")));
          else p.resolve(j);
          return;
        }
        if (j.status === "error" && j.error) this.opts.onEvalError?.(String(j.error));
        else if (j.status === "ok" && j.type !== "preview") this.opts.onEvalOk?.();
      } catch {
        // ignore non-JSON text
      }
    };

    ws.onclose = () => {
      if (this.ws !== ws) return;
      this.ws = null;
      for (const [, p] of this.pending) p.reject(new Error("disconnected"));
      this.pending.clear();
      this.opts.onStatus(false, this.url);
      this.retryTimer = window.setTimeout(() => this.connect(), this.retryMs);
      this.retryMs = Math.min(this.retryMs * 2, 5000);
    };

    ws.onerror = () => ws.close();
  }

  stop(): void {
    window.clearTimeout(this.retryTimer);
    const ws = this.ws;
    this.ws = null;
    ws?.close();
  }
}
