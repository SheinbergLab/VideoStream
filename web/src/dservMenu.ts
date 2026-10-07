import type { Connection } from "./connection";
import type { Settings } from "./settings";
import { parseTclDictNested, parseTclList } from "./tcl";

// The "dserv" control in the top bar: which data server VideoStream forwards
// results to and takes ess/in_obs + ess/datafile from. The server finds
// dservs on the local network over mDNS (`_dserv._tcp`); connecting saves the
// `dserv` setting ("host port"), which the server applies (dserv_connect in
// tcl/dserv_link.tcl) and restores on the next start.

interface FoundDserv {
  name: string;
  host: string; // .local name
  ip: string;
  dp: number; // datapoint port to connect to
  wg: string;
  ver: string;
}

interface DservStatus {
  host: string;
  port: string;
  forwarding: boolean;
  subscribed: boolean;
  error: string; // why the last subscribe failed
  discovery: string;
  found: FoundDserv[];
}

export interface DservMenuOptions {
  conn: Connection;
  settings: Settings;
  wrap: HTMLElement;
  button: HTMLButtonElement;
  label: HTMLElement;
  menu: HTMLElement;
  foundList: HTMLElement;
  refreshBtn: HTMLButtonElement;
  manualForm: HTMLFormElement;
  hostInput: HTMLInputElement;
  portInput: HTMLInputElement;
  currentSection: HTMLElement;
  statusEl: HTMLElement;
  disconnectBtn: HTMLButtonElement;
  errorEl: HTMLElement;
}

const POLL_MS = 3000;

function parseStatus(raw: string): DservStatus {
  const d = parseTclDictNested(raw);
  const found = parseTclList(d.found ?? "").map((e) => {
    const f = parseTclDictNested(e);
    return {
      name: f.name ?? "",
      host: f.host ?? "",
      ip: f.ip ?? "",
      dp: Number(f.dp) || 4620,
      wg: f.wg ?? "",
      ver: f.ver ?? "",
    };
  });
  return {
    host: d.host ?? "",
    port: d.port ?? "",
    forwarding: d.forwarding === "1",
    subscribed: d.subscribed === "1",
    error: d.error ?? "",
    discovery: d.discovery ?? "",
    found,
  };
}

export function attachDservMenu(opts: DservMenuOptions): {
  refresh: () => Promise<void>;
  handleEvent: (event: string) => void;
  setEnabled: (up: boolean) => void;
} {
  let status: DservStatus | null = null;
  let timer: number | undefined;
  let busy = false;

  const showError = (msg: string) => {
    opts.errorEl.textContent = msg;
    opts.errorEl.hidden = !msg;
  };

  const close = () => {
    opts.menu.hidden = true;
    opts.wrap.classList.remove("open");
    opts.button.setAttribute("aria-expanded", "false");
    showError("");
  };

  const open = () => {
    opts.menu.hidden = false;
    opts.wrap.classList.add("open");
    opts.button.setAttribute("aria-expanded", "true");
    void refresh();
  };

  // The found entry the current connection points at, if any.
  const currentEntry = (s: DservStatus): FoundDserv | undefined =>
    s.host ? s.found.find((f) => f.ip === s.host || f.host === s.host || f.name === s.host) : undefined;

  const render = () => {
    const s = status;
    opts.button.classList.remove("dserv-none", "dserv-up", "dserv-partial");
    if (!s || !s.host) {
      opts.button.classList.add("dserv-none");
      opts.label.textContent = "dserv: none";
      opts.button.title = "Not connected to a data server; results are not forwarded";
    } else {
      const both = s.forwarding && s.subscribed;
      opts.button.classList.add(both ? "dserv-up" : "dserv-partial");
      const shown = currentEntry(s)?.name || s.host;
      opts.label.textContent = `dserv: ${shown}`;
      opts.button.title =
        `${s.host}:${s.port}\n` +
        `results forwarded: ${s.forwarding ? "yes" : "connecting…"}\n` +
        `ess/in_obs, ess/datafile: ${s.subscribed ? "subscribed" : "registering…"}` +
        (s.error && !s.subscribed ? `\n${s.error}` : "");
    }

    // Found on the network
    opts.foundList.replaceChildren();
    const found = s?.found ?? [];
    if (!found.length) {
      const p = document.createElement("p");
      p.className = "dserv-empty";
      p.textContent =
        s && s.discovery !== "browsing"
          ? `Discovery is off: ${s.discovery.replace(/^off: /, "")}`
          : "No dservs found on this network (they advertise as _dserv._tcp).";
      opts.foundList.append(p);
    }
    const cur = s ? currentEntry(s) : undefined;
    for (const f of found) {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "source-menu-item dserv-item";
      btn.classList.toggle("selected", f === cur);
      const name = document.createElement("span");
      name.textContent = f.name;
      const meta = document.createElement("span");
      meta.className = "dserv-item-meta";
      meta.textContent = [f.ip || f.host, f.wg, f.ver && `v${f.ver}`].filter(Boolean).join(" · ");
      btn.append(name, meta);
      btn.title = `${f.host} (${f.ip || "no IPv4 address"}), datapoint port ${f.dp}`;
      btn.disabled = busy;
      btn.addEventListener("click", () => void connect(f.ip || f.host, f.dp));
      opts.foundList.append(btn);
    }

    // Current connection
    opts.currentSection.hidden = !s?.host;
    if (s?.host) {
      opts.statusEl.textContent =
        `${s.host}:${s.port} — results ${s.forwarding ? "forwarded" : "connecting…"}, ` +
        `obs/datafile ${s.subscribed ? "subscribed" : "registering…"}` +
        (s.error && !s.subscribed ? ` (${s.error})` : "");
    }
    opts.disconnectBtn.disabled = busy;
    opts.refreshBtn.disabled = busy;
  };

  async function refresh(): Promise<void> {
    try {
      status = parseStatus(await opts.conn.sendEvalAsync("dserv_status"));
      opts.wrap.hidden = false;
    } catch (e) {
      const msg = e instanceof Error ? e.message : String(e);
      // A script without dserv_link.tcl (eyetracker.tcl, an old serve.tcl): no control.
      if (/invalid command name/.test(msg)) opts.wrap.hidden = true;
      status = null;
    }
    render();
  }

  async function connect(host: string, port: number): Promise<void> {
    busy = true;
    showError("");
    render();
    try {
      await opts.settings.put("dserv", host ? `${host} ${port}` : "");
      close();
    } catch (e) {
      showError(e instanceof Error ? e.message : String(e));
    } finally {
      busy = false;
      await refresh();
    }
  }

  opts.button.addEventListener("click", () => (opts.menu.hidden ? open() : close()));
  opts.refreshBtn.addEventListener("click", () => void refresh());
  opts.disconnectBtn.addEventListener("click", () => void connect("", 0));
  opts.manualForm.addEventListener("submit", (ev) => {
    ev.preventDefault();
    const host = opts.hostInput.value.trim();
    const port = Number(opts.portInput.value.trim() || "4620");
    if (!host) {
      showError("Enter a host name or IP address");
      return;
    }
    if (!Number.isInteger(port) || port < 1 || port > 65535) {
      showError("Port must be 1-65535");
      return;
    }
    void connect(host, port);
  });
  opts.wrap.addEventListener("keydown", (ev) => {
    if (ev.key === "Escape" && !opts.menu.hidden) {
      ev.stopPropagation();
      close();
      opts.button.focus();
    }
  });
  document.addEventListener("click", (ev) => {
    if (!opts.menu.hidden && !opts.wrap.contains(ev.target as Node)) close();
  });

  return {
    refresh,
    // The server announces connect/disconnect/re-register to every browser.
    handleEvent: (event) => {
      if (event === "vstream/dserv") void refresh();
    },
    setEnabled: (up) => {
      opts.button.disabled = !up;
      if (timer !== undefined) window.clearInterval(timer);
      timer = undefined;
      if (up) {
        void refresh();
        // Forwarding and subscription come up in the background; poll for that.
        timer = window.setInterval(() => void refresh(), POLL_MS);
      } else {
        close();
        status = null;
        render();
      }
    },
  };
}
